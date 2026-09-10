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

    def test_active_edge_repairs_legacy_center_only_timeout(self):
        self.execution = TaskExecutionService.transition(
            self.execution,
            "timed_out",
            event_type="task.start.timeout",
            state_version=27,
            reason_code="COMMAND_TIMED_OUT",
            reason_message="Edge Agent 未在超时时间内确认或完成指令",
        )

        result = handle_mqtt_message(
            f"robots/{self.robot.code}/sync/state",
            self.envelope(
                "paused",
                26,
                local_task_reason_code="ABSOLUTE_LOCALIZATION_REQUIRED",
                local_task_reason_message="等待 NDT 重定位",
            ),
        )

        self.execution.refresh_from_db()
        self.assertEqual(self.execution.state, "paused")
        self.assertEqual(self.execution.state_version, 28)
        self.assertIsNone(self.execution.finished_at)
        self.assertEqual(self.execution.failure_code, "ABSOLUTE_LOCALIZATION_REQUIRED")
        self.assertEqual(result["action"], "report_only")
        self.assertIs(result["state_reconciled"], True)
        self.assertIs(result["center_timeout_recovered"], True)
        self.assertEqual(result["expected_task_state"], "paused")
        self.assertEqual(result["expected_state_version"], 28)
        event = TaskExecutionEvent.objects.get(
            task_execution=self.execution,
            event_type="task.sync_center_timeout_reconciled",
        )
        self.assertEqual(event.payload["previous_cloud_state"], "timed_out")
        self.assertEqual(event.payload["edge_state_version"], 26)

    def test_real_edge_timeout_is_not_reopened_by_stale_active_state(self):
        self.execution = TaskExecutionService.transition(
            self.execution,
            "timed_out",
            event_type="task.timed_out",
            state_version=4,
            reason_code="TASK_MAX_DURATION_EXCEEDED",
            reason_message="设备确认任务已超时",
        )

        result = handle_mqtt_message(
            f"robots/{self.robot.code}/sync/state",
            self.envelope("paused", 3),
        )

        self.execution.refresh_from_db()
        self.assertEqual(self.execution.state, "timed_out")
        self.assertEqual(result["action"], "hold")
        self.assertIs(result["center_timeout_recovered"], False)

    def test_matching_interrupted_state_does_not_increment_on_every_sync(self):
        self.execution = TaskExecutionService.transition(
            self.execution,
            "interrupted",
            event_type="task.start.timeout_pending_edge",
            state_version=4,
            reason_code="COMMAND_TIMED_OUT",
            reason_message="等待 Edge 状态对账",
        )

        result = handle_mqtt_message(
            f"robots/{self.robot.code}/sync/state",
            self.envelope("interrupted", 4),
        )

        self.execution.refresh_from_db()
        self.assertEqual(self.execution.state_version, 4)
        self.assertEqual(result["action"], "continue")
        self.assertIs(result["center_timeout_recovered"], False)
