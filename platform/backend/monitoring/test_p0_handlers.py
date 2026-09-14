import uuid
from decimal import Decimal
from unittest.mock import patch

from django.test import TestCase
from django.utils import timezone

from .message_handlers import handle_mqtt_message
from .models import (
    InboundMessage,
    InspectionEvent,
    MapData,
    PatrolLoopSession,
    PatrolRoute,
    PatrolTask,
    RemoteCommand,
    Robot,
    RobotCommand,
    SpeechCategory,
    SpeechTemplate,
    SystemLog,
    TaskExecution,
    TaskExecutionEvent,
    TrajectoryBatchReceipt,
    TrajectoryPoint,
)
from .protocol import ProtocolError
from .serializers import PatrolLoopSessionSerializer
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

    def test_command_progress_updates_executing_payload_without_finishing(self):
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
        progress = self.envelope(
            "command.progress",
            {
                "command_id": str(self.command.id),
                "task_execution_id": str(self.execution.id),
                "status": "executing",
                "started_at": timezone.now().isoformat(),
                "result": {
                    "localization_attempts": {
                        "state": "running",
                        "attempts": [
                            {"index": 1, "status": "verifying", "x": 1.0, "y": 2.0, "yaw": 0.1},
                        ],
                    }
                },
            },
        )
        handle_mqtt_message("robots/rx-001/commands/x/progress", progress)
        self.command.refresh_from_db()
        self.assertEqual(self.command.status, "executing")
        self.assertEqual(
            self.command.result_payload["localization_attempts"]["attempts"][0]["status"],
            "verifying",
        )
        self.assertIsNone(self.command.finished_at)

    def test_localization_terminal_error_preserves_last_candidate_metrics(self):
        command = CommandService.create_robot_command(
            robot=self.robot,
            command_type="nav.relocalize",
            payload={"seed_source": "quick_then_global"},
        )
        progress = self.envelope(
            "command.progress",
            {
                "command_id": str(command.id),
                "task_execution_id": None,
                "status": "executing",
                "started_at": timezone.now().isoformat(),
                "result": {
                    "localization_attempts": {
                        "state": "running",
                        "attempts": [{
                            "index": 1,
                            "status": "rejected",
                            "matching_error": 1.65,
                            "inlier_fraction": 0.0,
                            "reject_reason": "ndt_not_converged",
                        }],
                    },
                },
            },
        )
        handle_mqtt_message("robots/rx-001/commands/x/progress", progress)
        result = self.envelope(
            "command.result",
            {
                "command_id": str(command.id),
                "task_execution_id": None,
                "status": "failed",
                "started_at": timezone.now().isoformat(),
                "finished_at": timezone.now().isoformat(),
                "error_code": "INITIAL_POSE_NOT_ACCEPTED",
                "error_message": "initial pose rejected",
                "result": {"localization_status": "global_search_required"},
            },
            sequence=2,
        )

        handle_mqtt_message("robots/rx-001/commands/x/result", result)

        command.refresh_from_db()
        attempt = command.result_payload["localization_attempts"]["attempts"][0]
        self.assertEqual(attempt["matching_error"], 1.65)
        self.assertEqual(attempt["inlier_fraction"], 0.0)
        self.assertEqual(command.result_payload["localization_status"], "global_search_required")

    def test_sync_reconciles_edge_terminal_state_and_releases_robot(self):
        result = handle_mqtt_message(
            "robots/rx-001/sync/state",
            self.envelope(
                "sync.request",
                {
                    "current_task_execution_id": str(self.execution.id),
                    "local_task_state": "failed",
                    "local_task_state_version": 4,
                    "last_processed_command_id": str(self.command.id),
                    "last_trajectory_seq": -1,
                    "outbox_pending": 3056,
                },
            ),
        )

        self.execution.refresh_from_db()
        self.assertEqual(self.execution.state, "failed")
        self.assertEqual(self.execution.state_version, 4)
        self.assertEqual(self.execution.failure_code, "EDGE_SYNC_TERMINAL")
        self.assertEqual(result["action"], "report_only")
        self.assertIs(result["terminal_reconciled"], True)
        self.assertFalse(
            TaskExecution.objects.filter(
                robot=self.robot,
                state__in=TaskExecution.ACTIVE_STATES,
            ).exists()
        )
        event = TaskExecutionEvent.objects.get(event_type="task.sync_terminal_reconciled")
        self.assertEqual(event.payload["previous_cloud_state"], "dispatching")
        self.assertEqual(event.payload["outbox_pending"], 3056)

    @patch("monitoring.message_handlers.realtime_publisher.publish_task_event")
    def test_arrival_stage_events_are_informational(self, publish_task_event):
        self.execution.refresh_from_db()
        initial_state = self.execution.state
        initial_version = self.execution.state_version

        for sequence, message_type in enumerate(
            (
                "task.arrival_check",
                "task.arrival_confirmed",
                "task.arrival_heading_aligning",
                "task.arrival_heading_aligned",
                "task.waypoint_postprocess_completed",
            ),
            start=10,
        ):
            result = handle_mqtt_message(
                "robots/rx-001/events/task",
                self.envelope(
                    message_type,
                    {
                        "task_execution_id": str(self.execution.id),
                        "state": initial_state,
                        "state_version": initial_version,
                        "waypoint_index": 0,
                        "reason_message": message_type,
                    },
                    sequence=sequence,
                ),
            )
            self.assertEqual(result, {"state": initial_state, "state_version": initial_version})

        self.execution.refresh_from_db()
        self.assertEqual(self.execution.state, initial_state)
        self.assertEqual(self.execution.state_version, initial_version)
        self.assertEqual(
            set(
                SystemLog.objects.filter(
                    task_execution=self.execution,
                    event_code__in={
                        "task.arrival_check",
                        "task.arrival_confirmed",
                        "task.arrival_heading_aligning",
                        "task.arrival_heading_aligned",
                        "task.waypoint_postprocess_completed",
                    },
                ).values_list("event_code", flat=True)
            ),
            {
                "task.arrival_check",
                "task.arrival_confirmed",
                "task.arrival_heading_aligning",
                "task.arrival_heading_aligned",
                "task.waypoint_postprocess_completed",
            },
        )
        self.assertEqual(
            InboundMessage.objects.filter(
                message_type__in={
                    "task.arrival_check",
                    "task.arrival_confirmed",
                    "task.arrival_heading_aligning",
                    "task.arrival_heading_aligned",
                    "task.waypoint_postprocess_completed",
                },
                process_status="processed",
            ).count(),
            5,
        )
        self.assertEqual(publish_task_event.call_count, 5)

    def test_late_pause_failure_does_not_overwrite_resume(self):
        TaskExecutionService.transition(
            self.execution,
            "accepted",
            event_type="test.accepted",
            state_version=2,
        )
        TaskExecutionService.transition(
            self.execution,
            "running",
            event_type="test.running",
            state_version=3,
        )
        self.execution.refresh_from_db()
        pause_command = CommandService.create(self.execution, "task.pause")
        self.execution.refresh_from_db()
        CommandService.create(self.execution, "task.resume")
        result = self.envelope(
            "command.result",
            {
                "command_id": str(pause_command.id),
                "task_execution_id": str(self.execution.id),
                "status": "failed",
                "started_at": timezone.now().isoformat(),
                "finished_at": timezone.now().isoformat(),
                "error_code": "ROBOT_NOT_STOPPED",
                "error_message": "robot speed did not reach stop threshold",
                "result": {},
            },
            sequence=3,
        )
        handle_mqtt_message("robots/rx-001/commands/x/result", result)
        self.execution.refresh_from_db()
        self.assertEqual(self.execution.state, "resuming")

    def test_failed_recovery_does_not_terminalize_paused_execution(self):
        TaskExecutionService.transition(
            self.execution,
            "accepted",
            event_type="test.accepted",
            state_version=2,
        )
        TaskExecutionService.transition(
            self.execution,
            "running",
            event_type="test.running",
            state_version=3,
        )
        TaskExecutionService.transition(
            self.execution,
            "paused",
            event_type="test.paused",
            state_version=4,
        )
        self.execution.refresh_from_db()
        recovery_command = CommandService.create_task_recovery(
            self.execution,
            episode_id=uuid.uuid4(),
            attempt=1,
            reason_code="LOCALIZATION_LOST",
            reason_message="定位恢复中",
        )

        result = self.envelope(
            "command.result",
            {
                "command_id": str(recovery_command.id),
                "task_execution_id": str(self.execution.id),
                "status": "failed",
                "started_at": timezone.now().isoformat(),
                "finished_at": timezone.now().isoformat(),
                "error_code": "ROBOT_NOT_STOPPED",
                "error_message": "recovery requires a confirmed stop",
                "result": {},
            },
            sequence=4,
        )
        handle_mqtt_message("robots/rx-001/commands/x/result", result)

        self.execution.refresh_from_db()
        recovery_command.refresh_from_db()
        self.assertEqual(recovery_command.status, "failed")
        self.assertEqual(self.execution.state, "paused")
        self.assertEqual(self.execution.state_version, 4)
        self.assertFalse(
            TaskExecutionEvent.objects.filter(
                task_execution=self.execution,
                event_type="command.result",
            ).exists()
        )

    def test_blocked_resume_persists_edge_pause_reason(self):
        TaskExecutionService.transition(
            self.execution,
            "accepted",
            event_type="test.accepted",
            state_version=2,
        )
        TaskExecutionService.transition(
            self.execution,
            "running",
            event_type="test.running",
            state_version=3,
        )
        TaskExecutionService.transition(
            self.execution,
            "paused",
            event_type="test.paused",
            state_version=4,
        )
        self.execution.refresh_from_db()
        resume_command = CommandService.create(self.execution, "task.resume")
        reason_message = "FAST-LIO 已到达航点，NDT 校正源尚未就绪"
        result = self.envelope(
            "command.result",
            {
                "command_id": str(resume_command.id),
                "task_execution_id": str(self.execution.id),
                "status": "succeeded",
                "started_at": timezone.now().isoformat(),
                "finished_at": timezone.now().isoformat(),
                "error_code": None,
                "error_message": None,
                "result": {
                    "final_task_state": "paused",
                    "state_version": 5,
                    "resume_blocked": True,
                    "reason_code": "ABSOLUTE_LOCALIZATION_REQUIRED",
                    "reason_message": reason_message,
                },
            },
            sequence=3,
        )

        handle_mqtt_message("robots/rx-001/commands/x/result", result)

        self.execution.refresh_from_db()
        event = TaskExecutionEvent.objects.get(
            task_execution=self.execution,
            event_type="command.result",
        )
        self.assertEqual(self.execution.state, "paused")
        self.assertEqual(self.execution.failure_code, "ABSOLUTE_LOCALIZATION_REQUIRED")
        self.assertEqual(self.execution.failure_message, reason_message)
        self.assertEqual(event.reason_code, "ABSOLUTE_LOCALIZATION_REQUIRED")
        self.assertEqual(event.reason_message, reason_message)

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

    def test_waypoint_milestone_is_persisted_for_guard_duty_timeline(self):
        reported_at = timezone.now().isoformat()
        progress = self.envelope(
            "task.progress",
            {
                "task_execution_id": str(self.execution.id),
                "state": "running",
                "state_version": 4,
                "current_waypoint_index": 0,
                "current_waypoint_id": "wp-1",
                "completed_waypoints": 0,
                "total_waypoints": 2,
                "reported_at": reported_at,
                "milestone": "target_dispatched",
                "waypoint": {"waypoint_id": "wp-1", "map_point_number": 1, "x": 1.0, "y": 2.0, "yaw": 0.2},
                "robot_pose": {"x": 0.8, "y": 1.9, "yaw": 0.1, "sampled_at": reported_at},
            },
        )

        handle_mqtt_message("robots/rx-001/events/task", progress)

        event = TaskExecutionEvent.objects.get(task_execution=self.execution, state_version=4)
        self.assertEqual(event.event_type, "task.target_dispatched")
        self.assertEqual(event.payload["waypoint"]["map_point_number"], 1)
        self.assertEqual(event.payload["robot_pose"]["x"], 0.8)

    @patch("monitoring.message_handlers.tts_service.synthesize_speech", return_value=("tts-audio/waypoint.mp3", True))
    def test_completed_waypoint_queues_selected_speech(self, synthesize_speech):
        category, _ = SpeechCategory.objects.get_or_create(name="巡检智能播报")
        template = SpeechTemplate.objects.create(name="到点播报", text="已到达巡检点", category=category)
        snapshot = self.execution.route_snapshot
        snapshot["waypoints"][0].update(
            {
                "speech_template_id": template.id,
                "speech_template_name": template.name,
                "speech_text": template.text,
                "speech_mode": "blocking",
            }
        )
        self.execution.route_snapshot = snapshot
        self.execution.save(update_fields=["route_snapshot", "updated_at"])
        progress = self.envelope(
            "task.progress",
            {
                "task_execution_id": str(self.execution.id),
                "state": "running",
                "state_version": 4,
                "current_waypoint_index": 0,
                "current_waypoint_id": "wp-1",
                "completed_waypoints": 1,
                "total_waypoints": 2,
                "reported_at": timezone.now().isoformat(),
                "milestone": "waypoint_reached",
                "execution_waypoint_index": 0,
                "waypoint": {"waypoint_id": "wp-1", "map_point_number": 1, "x": 1.0, "y": 2.0, "yaw": 0.2},
            },
        )
        handle_mqtt_message("robots/rx-001/events/task", progress)
        command = RobotCommand.objects.get(payload__source="patrol_waypoint_speech")
        self.assertEqual(command.payload["audio_name"], "到点播报")
        self.assertEqual(command.payload["waypoint_index"], 0)
        synthesize_speech.assert_called_once_with("已到达巡检点")

    @patch("monitoring.message_handlers.tts_service.synthesize_speech", return_value=("tts-audio/obstacle.mp3", True))
    def test_obstacle_speech_uses_fixed_title_and_allows_three_recovery_attempts(self, synthesize_speech):
        category, _ = SpeechCategory.objects.get_or_create(name="巡检智能播报")
        for name in ("发现障碍物", "后退尝试避障", "劝阻离开线路"):
            SpeechTemplate.objects.update_or_create(
                name=name,
                defaults={"text": f"{name}文案", "category": category},
            )
        for sequence, stage, attempt in (
            (1, "obstacle_detected", 0),
            (2, "recovery_attempt", 1),
            (3, "recovery_attempt", 2),
            (4, "recovery_attempt", 3),
            (5, "leave_route", 3),
        ):
            handle_mqtt_message(
                "robots/rx-001/events/task",
                self.envelope(
                    "task.obstacle_speech",
                    {
                        "task_execution_id": str(self.execution.id),
                        "obstacle_episode_id": "episode-1",
                        "speech_stage": stage,
                        "recovery_attempt": attempt,
                        "reported_at": timezone.now().isoformat(),
                    },
                    sequence=sequence,
                ),
            )
        for sequence, episode_id in ((6, "episode-2"), (7, "episode-2")):
            handle_mqtt_message(
                "robots/rx-001/events/task",
                self.envelope(
                    "task.obstacle_speech",
                    {
                        "task_execution_id": str(self.execution.id),
                        "obstacle_episode_id": episode_id,
                        "speech_stage": "obstacle_detected",
                        "recovery_attempt": 0,
                        "reported_at": timezone.now().isoformat(),
                    },
                    sequence=sequence,
                ),
            )
        commands = RobotCommand.objects.filter(payload__source="patrol_obstacle_speech").order_by("id")
        self.assertEqual([item.payload["audio_name"] for item in commands], [
            "发现障碍物", "后退尝试避障", "后退尝试避障", "后退尝试避障", "劝阻离开线路", "发现障碍物",
        ])
        self.assertEqual(synthesize_speech.call_count, 6)
        self.assertEqual(
            [item.payload["alert_skill"] for item in commands],
            ["obstacle_detected", "avoidance", "avoidance", "avoidance", "dissuasion", "obstacle_detected"],
        )
        self.assertTrue(all(item.payload["dual_output"] for item in commands))

    @patch("monitoring.message_handlers.tts_service.synthesize_speech", return_value=("tts-audio/final.mp3", True))
    def test_task_completion_queues_final_waypoint_speech(self, synthesize_speech):
        category, _ = SpeechCategory.objects.get_or_create(name="巡检智能播报")
        template = SpeechTemplate.objects.create(name="终点播报", text="已到达巡检终点", category=category)
        snapshot = self.execution.route_snapshot
        snapshot["waypoints"][1].update(
            {
                "speech_template_id": template.id,
                "speech_template_name": template.name,
                "speech_text": template.text,
                "speech_mode": "blocking",
            }
        )
        self.execution.route_snapshot = snapshot
        self.execution.save(update_fields=["route_snapshot", "updated_at"])
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
        progress = self.envelope(
            "task.progress",
            {
                "task_execution_id": str(self.execution.id),
                "state": "running",
                "state_version": 3,
                "current_waypoint_index": 1,
                "current_waypoint_id": "wp-2",
                "completed_waypoints": 2,
                "total_waypoints": 2,
                "reported_at": timezone.now().isoformat(),
                "milestone": "waypoint_reached",
                "execution_waypoint_index": 1,
                "waypoint": {"waypoint_id": "wp-2", "map_point_number": 2, "x": 3.0, "y": 4.0, "yaw": 0.4},
            },
            sequence=2,
        )
        handle_mqtt_message("robots/rx-001/events/task", progress)
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
                "result": {"final_task_state": "completed", "state_version": 4, "completed_waypoints": 2, "total_waypoints": 2},
            },
            sequence=3,
        )
        handle_mqtt_message("robots/rx-001/commands/x/result", result)
        command = RobotCommand.objects.get(payload__source="patrol_waypoint_speech")
        self.assertEqual(command.payload["audio_name"], "终点播报")
        self.assertEqual(command.payload["waypoint_index"], 1)
        synthesize_speech.assert_called_once_with("已到达巡检终点")

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

    def test_loop_trajectory_batches_accumulate_once_across_rounds(self):
        loop_id = uuid.uuid4()
        self.execution.loop_session_id = loop_id
        self.execution.save(update_fields=["loop_session_id", "updated_at"])
        loop = PatrolLoopSession.objects.create(
            id=loop_id,
            robot=self.robot,
            task=self.execution.task,
            route_snapshot={},
            state="running",
            duration_seconds=600,
            ends_at=timezone.now() + timezone.timedelta(minutes=10),
            current_round=1,
            current_execution=self.execution,
        )

        tail_payload = {
            "task_execution_id": str(self.execution.id),
            "map_id": "1",
            "map_version": "v1",
            "frame_id": "map",
            "batch_id": str(uuid.uuid4()),
            "first_seq": 2,
            "last_seq": 2,
            "points": [
                {"seq": 2, "sampled_at": timezone.now().isoformat(), "x": 6.0, "y": 8.0, "yaw": 0.0, "speed_mps": 0.2, "localization_status": "normal"},
            ],
        }
        handle_mqtt_message(
            "robots/rx-001/telemetry/trajectory",
            self.envelope("trajectory.batch", tail_payload),
        )

        first_batch_id = uuid.uuid4()
        first_payload = {
            "task_execution_id": str(self.execution.id),
            "map_id": "1",
            "map_version": "v1",
            "frame_id": "map",
            "batch_id": str(first_batch_id),
            "first_seq": 0,
            "last_seq": 1,
            "points": [
                {"seq": 0, "sampled_at": timezone.now().isoformat(), "x": 0.0, "y": 0.0, "yaw": 0.0, "speed_mps": 0.2, "localization_status": "normal"},
                {"seq": 1, "sampled_at": timezone.now().isoformat(), "x": 3.0, "y": 4.0, "yaw": 0.0, "speed_mps": 0.2, "localization_status": "normal"},
            ],
        }
        handle_mqtt_message(
            "robots/rx-001/telemetry/trajectory",
            self.envelope("trajectory.batch", first_payload),
        )
        handle_mqtt_message(
            "robots/rx-001/telemetry/trajectory",
            self.envelope("trajectory.batch", first_payload, sequence=2),
        )

        self.execution.state = "completed"
        self.execution.finished_at = timezone.now()
        self.execution.save(update_fields=["state", "finished_at", "updated_at"])
        second_execution = TaskExecution.objects.create(
            task=self.execution.task,
            robot=self.robot,
            route=self.execution.route,
            map_data=self.execution.map_data,
            route_snapshot=self.execution.route_snapshot,
            loop_session_id=loop_id,
            round_number=2,
        )
        loop.current_round = 2
        loop.current_execution = second_execution
        loop.save(update_fields=["current_round", "current_execution", "updated_at"])
        second_payload = {
            "task_execution_id": str(second_execution.id),
            "map_id": "1",
            "map_version": "v1",
            "frame_id": "map",
            "batch_id": str(uuid.uuid4()),
            "first_seq": 0,
            "last_seq": 1,
            "points": [
                {"seq": 0, "sampled_at": timezone.now().isoformat(), "x": 0.0, "y": 0.0, "yaw": 0.0, "speed_mps": 0.2, "localization_status": "normal"},
                {"seq": 1, "sampled_at": timezone.now().isoformat(), "x": 0.0, "y": 6.0, "yaw": 0.0, "speed_mps": 0.2, "localization_status": "normal"},
            ],
        }
        handle_mqtt_message(
            "robots/rx-001/telemetry/trajectory",
            self.envelope("trajectory.batch", second_payload, sequence=3),
        )

        loop.refresh_from_db()
        self.assertEqual(Decimal(loop.metadata["total_distance_m"]), Decimal("16.000000"))
        loop.metadata = {}
        self.assertEqual(
            PatrolLoopSessionSerializer(loop).data["total_distance_m"],
            "16.000000",
        )

    def test_trajectory_ack_is_published_after_commit(self):
        payload = {
            "task_execution_id": str(self.execution.id),
            "map_id": "1",
            "map_version": "v1",
            "frame_id": "map",
            "batch_id": str(uuid.uuid4()),
            "first_seq": 0,
            "last_seq": 0,
            "points": [
                {
                    "seq": 0,
                    "sampled_at": timezone.now().isoformat(),
                    "x": 1.0,
                    "y": 2.0,
                    "yaw": 0.0,
                    "speed_mps": 0.2,
                    "localization_status": "normal",
                },
            ],
        }
        published = []
        with self.captureOnCommitCallbacks(execute=True):
            handle_mqtt_message(
                "robots/rx-001/telemetry/trajectory",
                self.envelope("trajectory.batch", payload),
                lambda topic, message, qos, retain: published.append((topic, message)),
            )

        self.assertEqual(len(published), 1)
        self.assertEqual(published[0][0], "robots/rx-001/sync/state")
        self.assertTrue(published[0][1]["payload"]["accepted"])

    def test_trajectory_batch_skips_inbound_audit_table(self):
        payload = {
            "task_execution_id": str(self.execution.id),
            "map_id": "1",
            "map_version": "v1",
            "frame_id": "map",
            "batch_id": str(uuid.uuid4()),
            "first_seq": 0,
            "last_seq": 0,
            "points": [
                {
                    "seq": 0,
                    "sampled_at": timezone.now().isoformat(),
                    "x": 1.0,
                    "y": 2.0,
                    "yaw": 0.0,
                    "speed_mps": 0.2,
                    "localization_status": "normal",
                },
            ],
        }
        handle_mqtt_message("robots/rx-001/telemetry/trajectory", self.envelope("trajectory.batch", payload))
        self.assertEqual(TrajectoryPoint.objects.count(), 1)
        self.assertEqual(TrajectoryBatchReceipt.objects.count(), 1)
        self.assertFalse(InboundMessage.objects.filter(message_type="trajectory.batch").exists())

    def test_unknown_task_execution_does_not_record_inbound_or_points(self):
        payload = {
            "task_execution_id": str(uuid.uuid4()),
            "map_id": "1",
            "map_version": "v1",
            "frame_id": "map",
            "batch_id": str(uuid.uuid4()),
            "first_seq": 0,
            "last_seq": 0,
            "points": [
                {
                    "seq": 0,
                    "sampled_at": timezone.now().isoformat(),
                    "x": 1.0,
                    "y": 2.0,
                    "yaw": 0.0,
                    "speed_mps": 0.2,
                    "localization_status": "normal",
                },
            ],
        }
        with self.assertRaises(ProtocolError) as ctx:
            handle_mqtt_message("robots/rx-001/telemetry/trajectory", self.envelope("trajectory.batch", payload))
        self.assertEqual(ctx.exception.code, "UNKNOWN_TASK_EXECUTION")
        self.assertEqual(TrajectoryPoint.objects.count(), 0)
        self.assertEqual(TrajectoryBatchReceipt.objects.count(), 0)
        self.assertEqual(InboundMessage.objects.count(), 0)

    def test_unknown_command_persists_failed_inbound_after_rollback(self):
        ack = self.envelope(
            "command.ack",
            {
                "command_id": str(uuid.uuid4()),
                "task_execution_id": str(self.execution.id),
                "ack": "accepted",
                "acknowledged_at": timezone.now().isoformat(),
                "reason_code": None,
                "reason_message": None,
                "duplicate": False,
                "edge_state_version": 2,
            },
        )
        with self.assertRaises(ProtocolError) as ctx:
            handle_mqtt_message("robots/rx-001/commands/x/ack", ack)
        self.assertEqual(ctx.exception.code, "UNKNOWN_COMMAND")
        inbound = InboundMessage.objects.get()
        self.assertEqual(inbound.process_status, "failed")
        self.assertIn("command not found", inbound.error_message)

    def test_processed_inbound_integrity_error_is_treated_as_duplicate(self):
        from django.db import IntegrityError

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
        with patch(
            "monitoring.message_handlers.InboundMessage.objects.get_or_create",
            side_effect=IntegrityError("UNIQUE constraint failed"),
        ):
            result = handle_mqtt_message("robots/rx-001/commands/x/ack", ack)
        self.assertEqual(result["duplicate"], True)

    def test_bicycle_alert_event_id_is_idempotent(self):
        event_id = str(uuid.uuid4())
        payload = {
            "event_id": event_id,
            "event_type": "vehicle_illegal_parking",
            "severity": "medium",
            "occurred_at": timezone.now().isoformat(),
            "task_execution_id": str(self.execution.id),
            "map_id": "1",
            "map_version": "v1",
            "pose": {"frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.0},
            "source": {"component": "bike_bot", "code": "BICYCLE_ALERT"},
            "detection": {"label": "自行车违停", "class": "bicycle", "confidence": 0.9},
            "attributes": {},
        }
        handle_mqtt_message("robots/rx-001/events/alert", self.envelope("alert.event", payload))
        handle_mqtt_message("robots/rx-001/events/alert", self.envelope("alert.event", payload, sequence=2))
        self.assertEqual(InspectionEvent.objects.filter(event_id=event_id).count(), 1)

    def test_non_bicycle_edge_alert_is_not_registered_in_event_center(self):
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
            "detection": {"label": "定位丢失", "class": "localization_lost", "confidence": 1},
            "attributes": {},
        }

        result = handle_mqtt_message(
            "robots/rx-001/events/alert",
            self.envelope("alert.event", payload),
        )

        self.assertFalse(result["created"])
        self.assertFalse(result["registered"])
        self.assertEqual(result["event_id"], event_id)
        self.assertFalse(InspectionEvent.objects.filter(event_id=event_id).exists())

    @patch(
        "monitoring.message_handlers.tts_service.synthesize_speech",
        return_value=("tts-audio/low-battery-alert.mp3", True),
    )
    def test_low_battery_alert_never_dispatches_docking_or_map_switch(self, synthesize_speech):
        route = self.execution.task.route
        self.assertEqual(len(route.waypoints), 2)
        self.execution.state = "completed"
        self.execution.save(update_fields=["state", "updated_at"])
        self.robot.charging_map = route.map_data
        self.robot.charging_route = route
        self.robot.connection_status = "online"
        self.robot.localization_status = "normal"
        self.robot.nav_ready = True
        self.robot.battery_level = 19
        self.robot.save(
            update_fields=[
                "charging_map",
                "charging_route",
                "connection_status",
                "localization_status",
                "nav_ready",
                "battery_level",
                "updated_at",
            ]
        )
        execution_count = TaskExecution.objects.count()
        remote_command_count = RemoteCommand.objects.count()

        formats = (
            ("low_battery_alert", "LOW_BATTERY_ALERT"),
            ("low_battery_return_charge", "LOW_BATTERY_RETURN_CHARGE"),
        )
        for sequence, (event_type, source_code) in enumerate(formats, start=10):
            event_id = str(uuid.uuid4())
            payload = {
                "event_id": event_id,
                "event_type": event_type,
                "severity": "high",
                "occurred_at": timezone.now().isoformat(),
                "task_execution_id": str(self.execution.id),
                "map_id": str(route.map_data_id),
                "map_version": "v1",
                "pose": {"frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.0},
                "source": {"component": "charge_control_adapter", "code": source_code},
                "detection": {
                    "label": "低电量停车告警",
                    "class": "low_battery",
                    "confidence": 1,
                },
                "attributes": {
                    "low_battery_episode_id": event_id,
                    "battery_percent": 19,
                    "automatic_docking": False,
                },
            }

            result = handle_mqtt_message(
                "robots/rx-001/events/alert",
                self.envelope("alert.event", payload, sequence=sequence),
            )

            self.assertIs(result["automatic_docking"], False)
            self.assertTrue(result["registered"])
            event = InspectionEvent.objects.get(event_id=event_id)
            self.assertEqual(event.title, "低电量停车告警")
            self.assertEqual(event.object_class, "low_battery")

        self.assertEqual(TaskExecution.objects.count(), execution_count)
        self.assertEqual(RemoteCommand.objects.count(), remote_command_count)
        self.assertFalse(RemoteCommand.objects.filter(command_type="map.activate").exists())
        self.assertFalse(
            RemoteCommand.objects.filter(payload__docking__enabled=True).exists()
        )
        self.assertEqual(
            RobotCommand.objects.filter(
                payload__source="low_battery_alert_speech"
            ).count(),
            2,
        )
        self.assertEqual(synthesize_speech.call_count, 2)

    @patch("monitoring.message_handlers.tts_service.synthesize_speech", return_value=("tts-audio/slam-diverged.mp3", True))
    def test_slam_diverged_alert_queues_operator_speech(self, synthesize_speech):
        event_id = str(uuid.uuid4())
        payload = {
            "event_id": event_id,
            "event_type": "slam_diverged",
            "severity": "high",
            "occurred_at": timezone.now().isoformat(),
            "task_execution_id": None,
            "map_id": "1",
            "map_version": "v1",
            "pose": {"frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.0},
            "source": {"component": "mapping", "code": "SLAM_DIVERGED"},
            "detection": {"label": "建图定位已发散", "class": "slam_diverged", "confidence": 1},
            "attributes": {"mapping_session_id": "session-1", "message": "pose anomaly"},
        }
        handle_mqtt_message("robots/rx-001/events/alert", self.envelope("alert.event", payload))
        handle_mqtt_message("robots/rx-001/events/alert", self.envelope("alert.event", payload, sequence=2))
        self.assertFalse(InspectionEvent.objects.filter(event_id=event_id).exists())
        commands = RobotCommand.objects.filter(payload__source="mapping_divergence_speech")
        self.assertEqual(commands.count(), 1)
        self.assertEqual(commands.get().payload["mapping_session_id"], "session-1")
        self.assertTrue(commands.get().payload["dual_output"])
        synthesize_speech.assert_called_once()
