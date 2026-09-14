import uuid

from django.contrib.auth import get_user_model
from django.test import TestCase
from django.utils import timezone
from rest_framework.test import APIClient
from rest_framework.exceptions import PermissionDenied

from .models import (
    MapData,
    PatrolLoopSession,
    PatrolRoute,
    PatrolTask,
    RemoteCommand,
    Robot,
    RobotStatusLatest,
)
from .services.patrol_loop_service import PatrolLoopError, PatrolLoopService
from .services.command_service import CommandService
from .services.task_service import TaskExecutionService


class PatrolLoopServiceTests(TestCase):
    def setUp(self):
        self.user = get_user_model().objects.create_user("loop-user", password="secret")
        self.robot = Robot.objects.create(
            code="loop-rx-001",
            name="Loop RX",
            location="site",
            area="site",
            connection_status="online",
            status="online",
            last_seen_at=timezone.now(),
            localization_status="normal",
            ros_ready=True,
            nav_ready=True,
            capabilities=["task.recover.v1"],
        )
        self.map = MapData.objects.create(name="loop-map", robot=self.robot)
        self.route = PatrolRoute.objects.create(
            name="loop-route",
            map_data=self.map,
            robot=self.robot,
            waypoints=[[1, 2, 0], [2, 3, 0.5]],
            waypoint_names=["A", "B"],
        )
        now = timezone.now()
        self.task = PatrolTask.objects.create(
            name="loop-task",
            robot=self.robot,
            route=self.route,
            route_name=self.route.name,
            scheduled_start=now,
            scheduled_end=now + timezone.timedelta(hours=1),
        )
        self.status = RobotStatusLatest.objects.create(
            robot=self.robot,
            state_version=1,
            sampled_at=now,
            received_at=now,
            speed_mps=0,
            localization_status="normal",
            power_available=True,
            battery_percent=80,
            ros_ready=True,
            nav_ready=True,
            emergency_stop=False,
            control_mode="autonomous",
        )

    def create_running_loop(self):
        session, created = PatrolLoopService.create_session(
            task=self.task,
            duration_seconds=600,
            rest_seconds=1,
            operator=self.user,
        )
        self.assertTrue(created)
        return PatrolLoopService.process(session.id)

    def test_api_creates_durable_loop_and_dispatches_first_round(self):
        client = APIClient()
        client.force_authenticate(self.user)
        session_id = uuid.uuid4()

        response = client.post(
            "/api/patrol-loop-sessions/",
            {
                "task_id": self.task.id,
                "duration_seconds": 600,
                "rest_seconds": 5,
                "session_id": str(session_id),
            },
            format="json",
        )

        self.assertEqual(response.status_code, 201)
        session = PatrolLoopSession.objects.get(pk=session_id)
        self.assertEqual(session.state, "running")
        self.assertEqual(session.current_round, 1)
        self.assertEqual(response.data["total_distance_m"], "0.000000")
        self.assertEqual(session.current_execution.loop_session_id, session.id)
        self.assertTrue(session.current_execution.commands.filter(command_type="task.start").exists())

    def test_route_save_to_task_list_to_guard_loop_keeps_continuous_recording(self):
        client = APIClient()
        client.force_authenticate(self.user)

        saved = client.put(
            f"/api/routes/{self.route.id}/",
            {"record_rosbag": True},
            format="json",
        )
        tasks = client.get("/api/patrol-tasks/")
        created = client.post(
            "/api/patrol-loop-sessions/",
            {
                "task_id": self.task.id,
                "duration_seconds": 600,
                "rest_seconds": 5,
            },
            format="json",
        )

        self.assertEqual(saved.status_code, 200)
        self.assertIs(saved.data["record_rosbag"], True)
        self.assertEqual(tasks.status_code, 200)
        listed = next(item for item in tasks.data if item["id"] == self.task.id)
        self.assertIs(listed["route_record_rosbag"], True)
        self.assertIs(listed["effective_record_rosbag"], True)
        self.assertEqual(created.status_code, 201)
        session = PatrolLoopSession.objects.get(pk=created.data["id"])
        command = session.current_execution.commands.get(command_type="task.start")
        self.assertIs(created.data["record_rosbag"], True)
        self.assertIs(command.payload["record_rosbag"], True)
        self.assertIs(command.payload["continuous_rosbag"], True)

    def test_loop_freezes_route_priority_recording_and_dispatches_continuous_bag(self):
        self.route.record_rosbag = True
        self.route.save(update_fields=["record_rosbag", "updated_at"])
        session, _ = PatrolLoopService.create_session(
            task=self.task,
            duration_seconds=600,
            rest_seconds=1,
            operator=self.user,
        )

        session = PatrolLoopService.process(session.id)
        first = session.current_execution.commands.get(command_type="task.start")

        self.assertIs(session.metadata["record_rosbag"], True)
        self.assertEqual(session.metadata["record_rosbag_source"], "route")
        self.assertIs(first.payload["record_rosbag"], True)
        self.assertIs(first.payload["continuous_rosbag"], True)
        self.assertEqual(first.payload["loop_session_id"], str(session.id))

        TaskExecutionService.transition(
            session.current_execution,
            "completed",
            event_type="task.completed",
        )
        session = PatrolLoopService.process(session.id)
        self.route.record_rosbag = False
        self.route.save(update_fields=["record_rosbag", "updated_at"])
        session.next_action_at = timezone.now() - timezone.timedelta(seconds=1)
        session.save(update_fields=["next_action_at", "updated_at"])

        session = PatrolLoopService.process(session.id)
        second = session.current_execution.commands.get(command_type="task.start")

        self.assertIs(second.payload["record_rosbag"], True)
        self.assertIs(second.payload["continuous_rosbag"], True)

    def test_terminal_loop_dispatches_one_idempotent_recording_stop(self):
        self.route.record_rosbag = True
        self.route.save(update_fields=["record_rosbag", "updated_at"])
        session = self.create_running_loop()

        session = PatrolLoopService.stop(session, self.user)
        session = PatrolLoopService.stop(session, self.user)

        stop_commands = RemoteCommand.objects.filter(
            command_type="diagnostics.nav_rosbag_stop",
            payload__loop_session_id=str(session.id),
        )
        self.assertEqual(stop_commands.count(), 1)
        self.assertEqual(
            session.metadata["recording_stop_command_id"],
            str(stop_commands.get().id),
        )

    def test_completed_round_rests_then_dispatches_next_round(self):
        session = self.create_running_loop()
        TaskExecutionService.transition(
            session.current_execution,
            "completed",
            event_type="task.completed",
        )
        session = PatrolLoopService.process(session.id)
        self.assertEqual(session.state, "resting")
        session.next_action_at = timezone.now() - timezone.timedelta(seconds=1)
        session.save(update_fields=["next_action_at", "updated_at"])

        session = PatrolLoopService.process(session.id)

        self.assertEqual(session.state, "running")
        self.assertEqual(session.current_round, 2)
        self.assertEqual(session.current_execution.round_number, 2)

    def test_system_pause_observes_five_seconds_then_dispatches_recovery(self):
        session = self.create_running_loop()
        TaskExecutionService.transition(
            session.current_execution,
            "accepted",
            event_type="command.ack",
        )
        TaskExecutionService.transition(
            session.current_execution,
            "running",
            event_type="task.started",
        )
        TaskExecutionService.transition(
            session.current_execution,
            "paused",
            event_type="task.safe_hold",
            reason_code="NAV_STACK_NOT_READY",
            reason_message="Nav2 unavailable",
        )
        session = PatrolLoopService.process(session.id)
        self.assertEqual(session.state, "observing")
        session = PatrolLoopService.process(session.id)
        self.assertIsNotNone(session.observation_started_at)
        session.observation_started_at = timezone.now() - timezone.timedelta(seconds=6)
        session.next_action_at = timezone.now() - timezone.timedelta(seconds=1)
        session.save(update_fields=["observation_started_at", "next_action_at", "updated_at"])

        session = PatrolLoopService.process(session.id)

        self.assertEqual(session.state, "recovering")
        self.assertEqual(session.recovery_attempt, 1)
        command = RemoteCommand.objects.get(command_type="task.recover.v1")
        self.assertEqual(command.payload["observation_seconds"], 5)

    def test_interlock_during_observation_resets_the_five_second_window(self):
        session = self.create_running_loop()
        TaskExecutionService.transition(session.current_execution, "accepted", event_type="command.ack")
        TaskExecutionService.transition(session.current_execution, "running", event_type="task.started")
        TaskExecutionService.transition(
            session.current_execution,
            "paused",
            event_type="task.safe_hold",
            reason_code="NAV_STACK_NOT_READY",
        )
        session = PatrolLoopService.process(session.id)
        session = PatrolLoopService.process(session.id)
        self.assertIsNotNone(session.observation_started_at)
        self.status.emergency_stop = True
        self.status.state_version += 1
        self.status.save(update_fields=["emergency_stop", "state_version"])

        session = PatrolLoopService.process(session.id)

        self.assertIsNone(session.observation_started_at)
        self.assertEqual(session.recovery_reason_code, "NAV_STACK_NOT_READY")
        self.assertEqual(session.metadata["observation_blocker"]["code"], "EMERGENCY_STOP")
        self.assertEqual(session.recovery_attempt, 0)

    def test_observation_uses_collision_monitor_not_localization_pose_speed(self):
        self.status.speed_mps = 0.20  # Scan matching jitter must not block a stopped robot.
        self.status.raw_payload = {
            "navigation": {
                "actual_planar_speed_mps": 0.0,
                "actual_turn_speed_rps": 0.0,
                "actual_velocity_sample_age_seconds": 8.0,
            },
            "localization": {"fresh": True, "sample_age_seconds": 0.1},
        }
        self.status.save(update_fields=["speed_mps", "raw_payload"])

        safe, code, message = PatrolLoopService._observation_safety(
            PatrolLoopSession(robot=self.robot)
        )

        self.assertTrue(safe)
        self.assertEqual((code, message), ("", ""))

    def test_observation_reports_stale_edge_ros_data_instead_of_false_motion(self):
        self.status.speed_mps = 0.0737
        self.status.raw_payload = {
            "navigation": {
                "actual_planar_speed_mps": 0.0,
                "actual_turn_speed_rps": 0.0,
                "actual_velocity_sample_age_seconds": 60.0,
            },
            "localization": {"fresh": False, "sample_age_seconds": 60.0},
        }
        self.status.save(update_fields=["speed_mps", "raw_payload"])

        safe, code, message = PatrolLoopService._observation_safety(
            PatrolLoopSession(robot=self.robot)
        )

        self.assertFalse(safe)
        self.assertEqual(code, "EDGE_ROS_DATA_STALE")
        self.assertIn("ROS", message)

    def test_manual_pause_never_auto_resumes(self):
        session = self.create_running_loop()
        session = PatrolLoopService.pause(session, self.user)
        self.assertTrue(session.manual_paused)

        session.next_action_at = timezone.now() - timezone.timedelta(minutes=1)
        session.save(update_fields=["next_action_at", "updated_at"])
        session = PatrolLoopService.process(session.id)

        self.assertEqual(session.state, "paused")
        self.assertFalse(RemoteCommand.objects.filter(command_type="task.recover.v1").exists())

    def test_continue_creates_a_new_recovery_episode_after_safety_observation(self):
        session = self.create_running_loop()
        TaskExecutionService.transition(session.current_execution, "accepted", event_type="command.ack")
        TaskExecutionService.transition(session.current_execution, "running", event_type="task.started")
        TaskExecutionService.transition(
            session.current_execution,
            "paused",
            event_type="task.safe_hold",
            reason_code="NAV_STACK_NOT_READY",
        )
        session = PatrolLoopService.pause(session, self.user)

        session = PatrolLoopService.continue_recovery(session)

        self.assertEqual(session.state, "observing")
        self.assertEqual(session.recovery_reason_code, "OPERATOR_CONTINUE")
        self.assertEqual(session.recovery_attempt, 0)
        episode_id = session.recovery_episode_id
        session = PatrolLoopService.process(session.id)
        session.observation_started_at = timezone.now() - timezone.timedelta(seconds=6)
        session.save(update_fields=["observation_started_at", "updated_at"])

        session = PatrolLoopService.process(session.id)

        self.assertEqual(session.state, "recovering")
        command = RemoteCommand.objects.get(command_type="task.recover.v1")
        self.assertEqual(command.payload["recovery_episode_id"], str(episode_id))
        self.assertEqual(command.payload["attempt"], 1)

    def test_continue_blocks_manual_takeover_until_operator_releases_it(self):
        session = self.create_running_loop()
        TaskExecutionService.transition(session.current_execution, "accepted", event_type="command.ack")
        TaskExecutionService.transition(session.current_execution, "running", event_type="task.started")
        TaskExecutionService.transition(session.current_execution, "paused", event_type="task.safe_hold")
        session = PatrolLoopService.pause(session, self.user)
        self.status.control_mode = "manual_takeover"
        self.status.state_version += 1
        self.status.save(update_fields=["control_mode", "state_version"])

        with self.assertRaisesMessage(PatrolLoopError, "先退出人工接管"):
            PatrolLoopService.continue_recovery(session)

    def test_low_battery_immediately_terminates_loop_and_execution(self):
        session = self.create_running_loop()
        start_command = session.current_execution.commands.get(command_type="task.start")

        PatrolLoopService.terminate_low_battery(
            robot=self.robot,
            episode_key="edge-low-battery-1",
            battery_percent=19,
            source="edge_alert",
            task_execution_id=session.current_execution_id,
        )

        session.refresh_from_db()
        session.current_execution.refresh_from_db()
        start_command.refresh_from_db()
        self.assertEqual(session.state, "low_battery_stopped")
        self.assertEqual(session.current_execution.state, "cancelled")
        self.assertEqual(start_command.status, "cancelled")
        self.assertFalse(RemoteCommand.objects.filter(command_type="task.recover.v1").exists())

    def test_low_battery_does_not_force_exit_active_docking_task(self):
        execution = TaskExecutionService.create_execution(self.task, self.user)
        CommandService.create(
            execution,
            "task.start",
            self.user,
            command_options={"docking": {"enabled": True}},
        )

        PatrolLoopService.terminate_low_battery(
            robot=self.robot,
            episode_key="edge-low-battery-docking",
            battery_percent=19,
            source="edge_alert",
        )

        execution.refresh_from_db()
        self.assertEqual(execution.state, "dispatching")
        self.assertFalse(execution.commands.filter(command_type="task.force_exit").exists())

    def test_low_battery_latch_blocks_patrol_until_rearm_percent(self):
        PatrolLoopService.terminate_low_battery(
            robot=self.robot,
            episode_key="edge-low-battery-latch",
            battery_percent=19,
            source="edge_alert",
        )
        self.status.battery_percent = 24
        self.status.state_version += 1
        self.status.save(update_fields=["battery_percent", "state_version"])
        with self.assertRaises(PermissionDenied):
            CommandService.ensure_task_start_allowed(self.robot)

        self.status.battery_percent = 25
        self.status.state_version += 1
        self.status.save(update_fields=["battery_percent", "state_version"])
        PatrolLoopService.observe_battery(self.status)

        CommandService.ensure_task_start_allowed(self.robot)

    def test_tenth_failed_recovery_holds_round_for_operator_continue(self):
        session = self.create_running_loop()
        TaskExecutionService.transition(session.current_execution, "accepted", event_type="command.ack")
        TaskExecutionService.transition(session.current_execution, "running", event_type="task.started")
        TaskExecutionService.transition(
            session.current_execution,
            "paused",
            event_type="task.safe_hold",
            reason_code="NAV_STACK_NOT_READY",
        )
        session = PatrolLoopService.process(session.id)
        session.recovery_attempt = 10
        session.observation_started_at = timezone.now() - timezone.timedelta(seconds=6)
        session.next_action_at = timezone.now() - timezone.timedelta(seconds=1)
        session.save()

        session = PatrolLoopService.process(session.id)

        self.assertEqual(session.state, "paused")
        self.assertTrue(session.manual_paused)
        self.assertTrue(session.metadata["continuation_required"])
        session.current_execution.refresh_from_db()
        self.assertEqual(session.current_execution.state, "paused")
        self.assertFalse(RemoteCommand.objects.filter(command_type="task.recover.v1").exists())
        self.assertFalse(session.current_execution.commands.filter(command_type="task.force_exit").exists())

    def test_async_recovery_does_not_consume_another_attempt_while_in_progress(self):
        session = self.create_running_loop()
        TaskExecutionService.transition(session.current_execution, "accepted", event_type="command.ack")
        TaskExecutionService.transition(session.current_execution, "running", event_type="task.started")
        TaskExecutionService.transition(
            session.current_execution,
            "paused",
            event_type="task.safe_hold",
            reason_code="LOCALIZATION_LOST",
        )
        session = PatrolLoopService.process(session.id)
        session.observation_started_at = timezone.now() - timezone.timedelta(seconds=6)
        session.save(update_fields=["observation_started_at", "updated_at"])
        session = PatrolLoopService.process(session.id)
        command = RemoteCommand.objects.get(command_type="task.recover.v1")
        command.status = "succeeded"
        command.result_payload = {
            "final_task_state": "paused",
            "recovery_status": "in_progress",
            "reason_code": "LOCALIZATION_RECOVERY_IN_PROGRESS",
        }
        command.save(update_fields=["status", "result_payload", "updated_at"])

        session = PatrolLoopService.process(session.id)
        self.assertEqual(session.state, "recovering")
        self.assertEqual(session.recovery_attempt, 1)
        self.assertEqual(session.metadata["recovery_in_progress_timeout_seconds"], 70)
        session.next_action_at = timezone.now() - timezone.timedelta(seconds=1)
        session.save(update_fields=["next_action_at", "updated_at"])
        session = PatrolLoopService.process(session.id)

        self.assertEqual(session.state, "recovering")
        self.assertEqual(session.recovery_attempt, 1)
        self.assertEqual(RemoteCommand.objects.filter(command_type="task.recover.v1").count(), 1)

    def test_nav2_reapproach_async_recovery_uses_short_action_deadline(self):
        session = self.create_running_loop()
        TaskExecutionService.transition(session.current_execution, "accepted", event_type="command.ack")
        TaskExecutionService.transition(session.current_execution, "running", event_type="task.started")
        TaskExecutionService.transition(
            session.current_execution,
            "paused",
            event_type="task.safe_hold",
            reason_code="ARRIVAL_POSE_CONVERGENCE_FAILED",
        )
        session = PatrolLoopService.process(session.id)
        session.observation_started_at = timezone.now() - timezone.timedelta(seconds=6)
        session.save(update_fields=["observation_started_at", "updated_at"])
        session = PatrolLoopService.process(session.id)
        command = RemoteCommand.objects.get(command_type="task.recover.v1")
        command.status = "succeeded"
        command.result_payload = {
            "final_task_state": "paused",
            "recovery_status": "in_progress",
            "recovery_action": "nav2_reapproach",
            "reason_code": "ARRIVAL_POSE_CONVERGENCE_FAILED",
        }
        command.save(update_fields=["status", "result_payload", "updated_at"])

        session = PatrolLoopService.process(session.id)
        self.assertEqual(session.metadata["recovery_in_progress_action"], "nav2_reapproach")
        self.assertEqual(session.metadata["recovery_in_progress_timeout_seconds"], 35)

        session.metadata["recovery_in_progress_started_at"] = (
            timezone.now() - timezone.timedelta(seconds=36)
        ).isoformat()
        session.next_action_at = timezone.now() - timezone.timedelta(seconds=1)
        session.save(update_fields=["metadata", "next_action_at", "updated_at"])
        session = PatrolLoopService.process(session.id)

        self.assertEqual(session.state, "observing")
        timeout_event = session.events.get(event_type="loop.recovery_in_progress_timeout")
        self.assertEqual(timeout_event.reason_code, "RECOVERY_IN_PROGRESS_TIMEOUT")
        self.assertIn("到点重接近超时", timeout_event.reason_message)
        self.assertNotIn("recovery_in_progress_started_at", session.metadata)

    def test_successful_recovery_clears_episode_before_the_next_fault(self):
        session = self.create_running_loop()
        session.recovery_episode_id = uuid.uuid4()
        session.recovery_attempt = 1
        session.recovery_reason_code = "NAV_STACK_NOT_READY"
        session.state = "recovering"
        session.save()
        TaskExecutionService.transition(session.current_execution, "accepted", event_type="command.ack")
        TaskExecutionService.transition(session.current_execution, "running", event_type="task.started")
        command = RemoteCommand.objects.create(
            robot=self.robot,
            task_execution=session.current_execution,
            command_type="task.recover.v1",
            status="succeeded",
            expires_at=timezone.now() + timezone.timedelta(minutes=5),
            payload={
                "recovery_episode_id": str(session.recovery_episode_id),
                "attempt": 1,
            },
        )
        command.result_payload = {"final_task_state": "running", "recovery_status": "recovered"}
        command.save(update_fields=["result_payload", "updated_at"])

        session = PatrolLoopService.process(session.id)

        self.assertEqual(session.state, "running")
        self.assertIsNone(session.recovery_episode_id)
        self.assertEqual(session.recovery_attempt, 0)
