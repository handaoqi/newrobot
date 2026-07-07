import uuid

from django.test import TestCase
from django.utils import timezone

from .message_handlers import handle_mqtt_message
from .models import (
    InspectionEvent,
    MapData,
    PatrolRoute,
    PatrolTask,
    RemoteCommand,
    Robot,
    TaskExecution,
    TrajectoryPoint,
)
from .services.command_service import CommandService
from .services.task_service import TaskExecutionService


class MessageHandlerTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="rx-001", name="RX", location="site", area="site")
        map_data = MapData.objects.create(name="map", robot=self.robot)
        route = PatrolRoute.objects.create(
            name="route", map_data=map_data, robot=self.robot, waypoints=[[1, 2], [2, 3]]
        )
        now = timezone.now()
        task = PatrolTask.objects.create(
            name="task",
            robot=self.robot,
            route=route,
            route_name=route.name,
            scheduled_start=now,
            scheduled_end=now + timezone.timedelta(hours=1),
        )
        self.execution = TaskExecutionService.create_execution(task)
        self.command = CommandService.create(self.execution, "task.start")
        self.session_id = str(uuid.uuid4())

    def envelope(self, message_type, payload, message_id=None, sequence=1):
        return {
            "protocol_version": "1.0",
            "message_id": str(message_id or uuid.uuid4()),
            "message_type": message_type,
            "robot_id": self.robot.code,
            "session_id": self.session_id,
            "sent_at": timezone.now().isoformat(),
            "trace_id": str(uuid.uuid4()),
            "sequence": sequence,
            "payload": payload,
        }

    def test_ack_and_result_are_separate(self):
        ack = self.envelope(
            "command.ack",
            {
                "command_id": str(self.command.id),
                "task_execution_id": str(self.execution.id),
                "ack": "accepted",
                "acknowledged_at": timezone.now().isoformat(),
                "reason_code": None,
                "reason_message": None,
                "duplicate": False,
                "edge_state_version": 2,
            },
        )
        handle_mqtt_message("robots/rx-001/commands/x/ack", ack)
        self.command.refresh_from_db()
        self.assertEqual(self.command.status, "accepted")

        result = self.envelope(
            "command.result",
            {
                "command_id": str(self.command.id),
                "task_execution_id": str(self.execution.id),
                "status": "succeeded",
                "started_at": timezone.now().isoformat(),
                "finished_at": timezone.now().isoformat(),
                "error_code": None,
                "error_message": None,
                "result": {"final_task_state": "completed", "state_version": 3},
            },
            sequence=2,
        )
        handle_mqtt_message("robots/rx-001/commands/x/result", result)
        self.command.refresh_from_db()
        self.execution.refresh_from_db()
        self.assertEqual(self.command.status, "succeeded")
        self.assertEqual(self.execution.state, "completed")

    def test_duplicate_and_stale_messages_do_not_regress(self):
        message_id = uuid.uuid4()
        progress = self.envelope(
            "task.progress",
            {
                "task_execution_id": str(self.execution.id),
                "state": "running",
                "state_version": 4,
                "current_waypoint_index": 1,
                "completed_waypoints": 1,
                "total_waypoints": 2,
                "reported_at": timezone.now().isoformat(),
            },
            message_id=message_id,
        )
        first = handle_mqtt_message("robots/rx-001/events/task", progress)
        second = handle_mqtt_message("robots/rx-001/events/task", progress)
        self.assertFalse(first.get("duplicate", False))
        self.assertTrue(second["duplicate"])
        stale = self.envelope(
            "task.progress",
            {
                **progress["payload"],
                "state_version": 2,
                "current_waypoint_index": 0,
            },
            sequence=2,
        )
        handle_mqtt_message("robots/rx-001/events/task", stale)
        self.execution.refresh_from_db()
        self.assertEqual(self.execution.current_waypoint_index, 1)

    def test_trajectory_batch_is_idempotent(self):
        payload = {
            "task_execution_id": str(self.execution.id),
            "map_id": "1",
            "map_version": "legacy-mapdata-1",
            "frame_id": "map",
            "batch_id": str(uuid.uuid4()),
            "first_seq": 0,
            "last_seq": 1,
            "points": [
                {"seq": 0, "sampled_at": timezone.now().isoformat(), "x": 1.0, "y": 2.0, "yaw": 0.0, "speed_mps": 0.2, "localization_status": "normal"},
                {"seq": 1, "sampled_at": timezone.now().isoformat(), "x": 1.1, "y": 2.1, "yaw": 0.1, "speed_mps": 0.2, "localization_status": "normal"},
            ],
        }
        handle_mqtt_message("robots/rx-001/telemetry/trajectory", self.envelope("trajectory.batch", payload))
        duplicate = self.envelope("trajectory.batch", payload, sequence=2)
        handle_mqtt_message("robots/rx-001/telemetry/trajectory", duplicate)
        self.assertEqual(TrajectoryPoint.objects.count(), 2)

    def test_alert_event_id_is_idempotent(self):
        event_id = str(uuid.uuid4())
        payload = {
            "event_id": event_id,
            "event_type": "localization_lost",
            "severity": "high",
            "occurred_at": timezone.now().isoformat(),
            "task_execution_id": str(self.execution.id),
            "map_id": "1",
            "map_version": "v1",
            "pose": {"frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.0},
            "source": {"component": "localization", "code": "LOCALIZATION_LOST"},
            "attributes": {},
        }
        handle_mqtt_message("robots/rx-001/events/alert", self.envelope("alert.event", payload))
        handle_mqtt_message("robots/rx-001/events/alert", self.envelope("alert.event", payload, sequence=2))
        self.assertEqual(InspectionEvent.objects.filter(event_id=event_id).count(), 1)
