import uuid

from django.test import TestCase
from django.utils import timezone

from .message_handlers import handle_mqtt_message
from .models import MapData, PatrolRoute, PatrolTask, Robot, TaskExecutionEvent
from .services.task_service import TaskExecutionService


class ActiveTaskStateSyncTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="sync-dog", name="Sync Dog", location="yard", area="yard")
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
        self.execution = TaskExecutionService.transition(
            self.execution, "dispatching", event_type="test.dispatching", state_version=1
        )
        self.execution = TaskExecutionService.transition(
            self.execution, "accepted", event_type="test.accepted", state_version=2
        )
        self.execution = TaskExecutionService.transition(
            self.execution, "running", event_type="test.running", state_version=3
        )

    def envelope(self, state, version, **reason):
        return {
            "protocol_version": "1.0",
            "message_id": str(uuid.uuid4()),
            "message_type": "sync.request",
            "robot_id": self.robot.code,
            "session_id": str(uuid.uuid4()),
            "sent_at": timezone.now().isoformat(),
            "trace_id": str(uuid.uuid4()),
            "payload": {
                "current_task_execution_id": str(self.execution.id),
                "local_task_state": state,
                "local_task_state_version": version,
                "outbox_pending": 0,
                **reason,
            },
        }

    def test_newer_edge_pause_repairs_cloud_state_and_preserves_reason(self):
        result = handle_mqtt_message(
            f"robots/{self.robot.code}/sync/state",
            self.envelope(
                "paused",
                4,
                local_task_reason_code="ARRIVAL_CONFIRMATION_UNSTABLE",
                local_task_reason_message="到点位姿未连续满足要求",
            ),
        )

        self.execution.refresh_from_db()
        self.assertEqual(self.execution.state, "paused")
        self.assertEqual(self.execution.state_version, 4)
        self.assertEqual(self.execution.failure_code, "ARRIVAL_CONFIRMATION_UNSTABLE")
        self.assertEqual(result["action"], "report_only")
        self.assertIs(result["state_reconciled"], True)
        event = TaskExecutionEvent.objects.get(
            task_execution=self.execution,
            event_type="task.sync_state_reconciled",
        )
        self.assertEqual(event.reason_message, "到点位姿未连续满足要求")

    def test_stale_edge_state_cannot_overwrite_newer_cloud_state(self):
        result = handle_mqtt_message(
            f"robots/{self.robot.code}/sync/state",
            self.envelope("paused", 2),
        )

        self.execution.refresh_from_db()
        self.assertEqual(self.execution.state, "running")
        self.assertEqual(self.execution.state_version, 3)
        self.assertEqual(result["action"], "hold")
        self.assertIs(result["state_reconciled"], False)
