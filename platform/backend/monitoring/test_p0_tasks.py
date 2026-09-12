from django.contrib.auth import get_user_model
from django.db import OperationalError
from django.test import TestCase, override_settings
from django.utils import timezone
from rest_framework.test import APIClient
import uuid
from unittest.mock import patch

from .models import (
    MapData,
    PatrolRoute,
    PatrolSchedule,
    PatrolTask,
    RemoteCommand,
    Robot,
    SpeechCategory,
    SpeechTemplate,
    TaskExecution,
)
from .services.command_service import CommandService
from .services.docking_service import dispatch_docking_task
from .services.schedule_service import ScheduleService
from .services.task_service import TaskExecutionService, TaskStateError, assert_transition_allowed
from .serializers import PatrolRouteSerializer
from .management.commands.run_device_worker import Command as DeviceWorkerCommand


class TaskExecutionTests(TestCase):
    def setUp(self):
        self.user = get_user_model().objects.create_user("p0-user", password="secret")
        self.robot = Robot.objects.create(
            code="rx-001",
            name="RX",
            location="site",
            area="site",
            connection_status="online",
            status="online",
            last_seen_at=timezone.now(),
            localization_status="normal",
            ros_ready=True,
            nav_ready=True,
        )
        self.map = MapData.objects.create(name="map", robot=self.robot)
        self.route = PatrolRoute.objects.create(
            name="route",
            map_data=self.map,
            robot=self.robot,
            waypoints=[[1, 2, 0], [2, 3, 0.5], [3, 4, 1.0]],
            waypoint_names=["A", "B", "C"],
        )
        now = timezone.now()
        self.task = PatrolTask.objects.create(
            name="task",
            robot=self.robot,
            route=self.route,
            route_name=self.route.name,
            scheduled_start=now,
            scheduled_end=now + timezone.timedelta(hours=1),
        )

    def test_route_snapshot_is_immutable_copy(self):
        execution = TaskExecutionService.create_execution(self.task, self.user)
        self.route.waypoints[0][0] = 99
        self.route.save()
        execution.refresh_from_db()
        self.assertEqual(execution.route_snapshot["waypoints"][0]["x"], 1.0)

    def test_route_snapshot_preserves_waypoint_speech_template(self):
        category, _ = SpeechCategory.objects.get_or_create(name="巡检智能播报")
        template = SpeechTemplate.objects.create(name="到点提醒", text="已到达巡检点", category=category)
        self.route.waypoints = [
            {
                "x": 1,
                "y": 2,
                "yaw": 0,
                "speech_template_id": template.id,
                "speech_template_name": template.name,
                "speech_text": template.text,
            }
        ]
        self.route.save(update_fields=["waypoints", "updated_at"])
        execution = TaskExecutionService.create_execution(self.task, self.user)
        waypoint = execution.route_snapshot["waypoints"][0]
        self.assertEqual(waypoint["speech_template_id"], template.id)
        self.assertEqual(waypoint["speech_template_name"], "到点提醒")
        self.assertEqual(waypoint["speech_mode"], "non_blocking")

    def test_route_snapshot_preserves_heading_and_segment_avoidance(self):
        self.route.waypoints = [{
            "x": 1,
            "y": 2,
            "yaw": 1.25,
            "require_yaw": True,
            "avoidance_to_next": False,
        }]
        self.route.save(update_fields=["waypoints", "updated_at"])
        execution = TaskExecutionService.create_execution(self.task, self.user)
        waypoint = execution.route_snapshot["waypoints"][0]
        self.assertEqual(waypoint["yaw"], 1.25)
        self.assertIs(waypoint["require_yaw"], True)
        self.assertIs(waypoint["avoidance_to_next"], False)


    def test_route_snapshot_preserves_controller_settings(self):
        self.route.global_controller = "navfn"
        self.route.waypoints = [
            {
                "x": 1,
                "y": 2,
                "yaw": 0,
                "local_controller": "mppi",
                "global_controller": "theta_star",
            },
            {"x": 2, "y": 3, "yaw": 0},
        ]
        self.route.save(update_fields=["global_controller", "waypoints", "updated_at"])
        execution = TaskExecutionService.create_execution(self.task, self.user)
        self.assertEqual(execution.route_snapshot["global_controller"], "navfn")
        self.assertEqual(execution.route_snapshot["waypoints"][0]["local_controller"], "mppi")
        self.assertEqual(execution.route_snapshot["waypoints"][0]["global_controller"], "theta_star")
        self.assertEqual(execution.route_snapshot["waypoints"][1]["global_controller"], "navfn")

    def test_route_snapshot_preserves_registered_rpp(self):
        self.route.waypoints = [{"x": 1, "y": 2, "yaw": 0, "local_controller": "rpp"}]
        self.route.save(update_fields=["waypoints", "updated_at"])
        execution = TaskExecutionService.create_execution(self.task, self.user)
        self.assertEqual(execution.route_snapshot["waypoints"][0]["local_controller"], "rpp")

    def test_route_snapshot_preserves_smac_hybrid_and_ilqr(self):
        self.route.global_controller = "smac_hybrid"
        self.route.waypoints = [{
            "x": 1, "y": 2, "yaw": 0,
            "global_controller": "smac_hybrid",
            "local_controller": "ilqr",
        }]
        self.route.save(update_fields=["global_controller", "waypoints", "updated_at"])
        execution = TaskExecutionService.create_execution(self.task, self.user)
        waypoint = execution.route_snapshot["waypoints"][0]
        self.assertEqual(execution.route_snapshot["global_controller"], "smac_hybrid")
        self.assertEqual(waypoint["global_controller"], "smac_hybrid")
        self.assertEqual(waypoint["local_controller"], "ilqr")

    def test_route_representation_exposes_registered_controllers(self):
        self.route.global_controller = "navfn"
        self.route.waypoints = [[1, 2], {"x": 3, "y": 4, "local_controller": "rpp"}]
        data = PatrolRouteSerializer(self.route).data
        self.assertEqual([point["local_controller"] for point in data["waypoints"]], ["mppi", "rpp"])
        self.assertEqual([point["global_controller"] for point in data["waypoints"]], ["navfn", "navfn"])

    def test_route_rejects_invalid_waypoint_global_controller(self):
        serializer = PatrolRouteSerializer(
            self.route,
            data={"waypoints": [{"x": 1, "y": 2, "yaw": 0, "global_controller": "invalid"}]},
            partial=True,
        )
        self.assertIs(serializer.is_valid(), False)
        self.assertIn("waypoints", serializer.errors)

    def test_route_snapshot_preserves_fractional_dwell_seconds(self):
        self.route.waypoints = [{
            "x": 1,
            "y": 2,
            "yaw": 0,
            "dwell_seconds": 1.5,
        }]
        self.route.save(update_fields=["waypoints", "updated_at"])
        execution = TaskExecutionService.create_execution(self.task, self.user)
        self.assertEqual(execution.route_snapshot["waypoints"][0]["dwell_seconds"], 1.5)

    def test_one_active_execution_per_robot(self):
        TaskExecutionService.create_execution(self.task, self.user)
        with self.assertRaises(TaskStateError):
            TaskExecutionService.create_execution(self.task, self.user)

    def test_invalid_terminal_resume_transition(self):
        with self.assertRaises(TaskStateError):
            assert_transition_allowed("completed", "resuming")

    def test_command_lifecycle_starts_created(self):
        execution = TaskExecutionService.create_execution(self.task, self.user)
        command = CommandService.create(execution, "task.start", self.user)
        self.assertEqual(command.status, "created")
        self.assertEqual(command.events.get().event_type, "created")
        execution.refresh_from_db()
        self.assertEqual(execution.state, "dispatching")

    @override_settings(TASK_START_ACK_TIMEOUT_SECONDS=60)
    def test_unacknowledged_task_start_waits_for_edge_reconciliation(self):
        execution = TaskExecutionService.create_execution(self.task, self.user)
        command = CommandService.create(execution, "task.start", self.user)
        RemoteCommand.objects.filter(pk=command.pk).update(
            status="published",
            issued_at=timezone.now() - timezone.timedelta(seconds=61),
        )

        DeviceWorkerCommand._expire_commands()

        command.refresh_from_db()
        execution.refresh_from_db()
        self.assertEqual(command.status, "timed_out")
        self.assertEqual(command.error_code, "COMMAND_TIMED_OUT")
        self.assertIn("等待 Edge 状态对账", command.error_message)
        self.assertEqual(execution.state, "interrupted")
        self.assertEqual(execution.failure_code, "COMMAND_TIMED_OUT")
        with self.assertRaises(TaskStateError):
            TaskExecutionService.create_execution(self.task, self.user)

    def test_low_battery_docking_episode_creates_exactly_one_return_task(self):
        self.route.waypoints = [[1, 2, 0], [2, 3, 0.5]]
        self.route.save(update_fields=["waypoints", "updated_at"])
        self.robot.charging_map = self.map
        self.robot.charging_route = self.route
        self.robot.battery_level = 19
        self.robot.save(
            update_fields=["charging_map", "charging_route", "battery_level", "updated_at"]
        )
        episode_id = str(uuid.uuid4())

        first = dispatch_docking_task(robot=self.robot, low_battery_episode_id=episode_id)
        second = dispatch_docking_task(robot=self.robot, low_battery_episode_id=episode_id)

        self.assertTrue(first.created)
        self.assertFalse(second.created)
        self.assertEqual(first.execution.id, second.execution.id)
        self.assertEqual(first.command.id, second.command.id)
        self.assertEqual(str(first.execution.loop_session_id), episode_id)
        self.assertTrue(first.command.payload["docking"]["enabled"])
        self.assertEqual(
            first.command.payload["docking"]["low_battery_episode_id"], episode_id
        )

    def test_force_exit_clears_active_execution_and_unblocks_next_task(self):
        execution = TaskExecutionService.create_execution(self.task, self.user)
        start_command = CommandService.create(execution, "task.start", self.user)
        command = CommandService.create(execution, "task.force_exit", self.user)
        execution.refresh_from_db()
        start_command.refresh_from_db()
        self.assertEqual(command.command_type, "task.force_exit")
        self.assertEqual(execution.state, "cancelled")
        self.assertEqual(start_command.status, "cancelled")
        self.assertEqual(start_command.result_payload["final_task_state"], "cancelled")
        self.assertTrue(start_command.events.filter(event_type="execution_terminal").exists())
        RemoteCommand.objects.filter(pk=start_command.pk).update(
            issued_at=timezone.now() - timezone.timedelta(minutes=2),
            expires_at=timezone.now() - timezone.timedelta(seconds=1)
        )
        DeviceWorkerCommand._expire_commands()
        start_command.refresh_from_db()
        self.assertEqual(start_command.status, "cancelled")
        next_execution = TaskExecutionService.create_execution(self.task, self.user)
        self.assertEqual(next_execution.state, "created")

    def test_late_terminal_execution_repairs_historical_start_timeout(self):
        execution = TaskExecutionService.create_execution(self.task, self.user)
        start_command = CommandService.create(execution, "task.start", self.user)
        TaskExecutionService.transition(execution, "accepted", event_type="command.ack")
        RemoteCommand.objects.filter(pk=start_command.pk).update(
            status="timed_out",
            error_code="COMMAND_TIMED_OUT",
            error_message="legacy false timeout",
        )

        TaskExecutionService.transition(
            TaskExecution.objects.get(pk=execution.pk),
            "cancelled",
            event_type="task.sync_terminal_reconciled",
            reason_code="FORCE_EXIT",
            reason_message="设备端已退出",
        )

        start_command.refresh_from_db()
        self.assertEqual(start_command.status, "cancelled")
        self.assertEqual(start_command.error_code, "FORCE_EXIT")
        self.assertEqual(start_command.error_message, "设备端已退出")

    def test_task_api_execute_and_busy_conflict(self):
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(f"/api/patrol-tasks/{self.task.id}/execute/", {}, format="json")
        self.assertEqual(response.status_code, 201)
        second = client.post(f"/api/patrol-tasks/{self.task.id}/execute/", {}, format="json")
        self.assertEqual(second.status_code, 409)
        self.assertEqual(RemoteCommand.objects.count(), 1)

    def test_task_execute_can_request_navigation_rosbag(self):
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(
            f"/api/patrol-tasks/{self.task.id}/execute/",
            {"record_rosbag": True},
            format="json",
        )
        self.assertEqual(response.status_code, 201)
        self.assertIs(RemoteCommand.objects.get().payload["record_rosbag"], True)

    def test_task_execute_uses_persisted_navigation_rosbag_setting(self):
        self.task.record_rosbag = True
        self.task.save(update_fields=["record_rosbag", "updated_at"])
        client = APIClient()
        client.force_authenticate(self.user)

        response = client.post(f"/api/patrol-tasks/{self.task.id}/execute/", {}, format="json")

        self.assertEqual(response.status_code, 201)
        self.assertIs(RemoteCommand.objects.get().payload["record_rosbag"], True)

    def test_task_execute_can_override_persisted_navigation_rosbag_setting(self):
        self.task.record_rosbag = True
        self.task.save(update_fields=["record_rosbag", "updated_at"])
        client = APIClient()
        client.force_authenticate(self.user)

        response = client.post(
            f"/api/patrol-tasks/{self.task.id}/execute/",
            {"record_rosbag": False},
            format="json",
        )

        self.assertEqual(response.status_code, 201)
        self.assertIs(RemoteCommand.objects.get().payload["record_rosbag"], False)

    def test_task_api_persists_navigation_rosbag_setting(self):
        client = APIClient()
        client.force_authenticate(self.user)

        response = client.put(
            f"/api/patrol-tasks/{self.task.id}/",
            {"record_rosbag": True},
            format="json",
        )

        self.assertEqual(response.status_code, 200)
        self.task.refresh_from_db()
        self.assertIs(self.task.record_rosbag, True)
        self.assertIs(response.data["record_rosbag"], True)

    def test_scheduled_task_uses_persisted_navigation_rosbag_setting(self):
        self.task.record_rosbag = True
        self.task.save(update_fields=["record_rosbag", "updated_at"])
        schedule = PatrolSchedule.objects.create(
            name="daily patrol",
            robot=self.robot,
            task_template=self.task,
            route=self.route,
            map_data=self.map,
            schedule_type="daily",
            time_of_day=timezone.localtime().time().replace(second=0, microsecond=0),
        )

        run = ScheduleService.trigger_now(schedule, operator=self.user)

        self.assertEqual(run.status, "dispatched")
        self.assertIs(run.remote_command.payload["record_rosbag"], True)

    def test_task_execute_marks_loop_execution(self):
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(
            f"/api/patrol-tasks/{self.task.id}/execute/",
            {"loop_execution": True},
            format="json",
        )
        self.assertEqual(response.status_code, 201)
        self.assertIs(RemoteCommand.objects.get().payload["loop_execution"], True)

    def test_task_execute_persists_loop_session_and_round(self):
        client = APIClient()
        client.force_authenticate(self.user)
        session_id = uuid.uuid4()
        response = client.post(
            f"/api/patrol-tasks/{self.task.id}/execute/",
            {
                "loop_execution": True,
                "loop_session_id": str(session_id),
                "round_number": 4,
            },
            format="json",
        )

        self.assertEqual(response.status_code, 201)
        execution = TaskExecution.objects.get()
        self.assertEqual(execution.loop_session_id, session_id)
        self.assertEqual(execution.round_number, 4)
        self.assertEqual(response.data["round_number"], 4)
        self.assertEqual(RemoteCommand.objects.get().payload["round_number"], 4)

    def test_repeated_loop_round_returns_same_execution_without_second_command(self):
        client = APIClient()
        client.force_authenticate(self.user)
        session_id = uuid.uuid4()
        payload = {
            "loop_execution": True,
            "loop_session_id": str(session_id),
            "round_number": 4,
        }

        first = client.post(f"/api/patrol-tasks/{self.task.id}/execute/", payload, format="json")
        second = client.post(f"/api/patrol-tasks/{self.task.id}/execute/", payload, format="json")

        self.assertEqual(first.status_code, 201)
        self.assertEqual(second.status_code, 200)
        self.assertEqual(second["X-Idempotent-Replay"], "true")
        self.assertEqual(second.data["id"], first.data["id"])
        self.assertEqual(TaskExecution.objects.count(), 1)
        self.assertEqual(RemoteCommand.objects.count(), 1)
        self.assertEqual(
            TaskExecution.objects.get().loop_dispatch_key,
            f"{session_id}:4",
        )

    def test_loop_round_retries_one_transient_sqlite_writer_lock(self):
        client = APIClient()
        client.force_authenticate(self.user)
        session_id = uuid.uuid4()
        original_create = TaskExecutionService.create_execution
        attempts = 0

        def create_after_transient_lock(*args, **kwargs):
            nonlocal attempts
            attempts += 1
            if attempts == 1:
                raise OperationalError("database is locked")
            return original_create(*args, **kwargs)

        with (
            patch.object(TaskExecutionService, "create_execution", side_effect=create_after_transient_lock),
            patch("monitoring.views.close_old_connections"),
            patch("monitoring.views.time.sleep"),
        ):
            response = client.post(
                f"/api/patrol-tasks/{self.task.id}/execute/",
                {
                    "loop_execution": True,
                    "loop_session_id": str(session_id),
                    "round_number": 5,
                },
                format="json",
            )

        self.assertEqual(response.status_code, 201)
        self.assertEqual(attempts, 2)
        self.assertEqual(TaskExecution.objects.count(), 1)
        self.assertEqual(RemoteCommand.objects.count(), 1)

    def test_task_execute_rejects_invalid_loop_round(self):
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(
            f"/api/patrol-tasks/{self.task.id}/execute/",
            {"loop_execution": True, "round_number": 0},
            format="json",
        )

        self.assertEqual(response.status_code, 400)
        self.assertEqual(TaskExecution.objects.count(), 0)

    def test_low_battery_task_execute_does_not_orphan_created_execution(self):
        self.robot.battery_level = 19
        self.robot.save(update_fields=["battery_level", "updated_at"])
        client = APIClient()
        client.force_authenticate(self.user)

        response = client.post(f"/api/patrol-tasks/{self.task.id}/execute/", {}, format="json")

        self.assertEqual(response.status_code, 403)
        self.assertEqual(TaskExecution.objects.count(), 0)
        self.assertEqual(RemoteCommand.objects.count(), 0)

    def test_charging_dock_saves_default_and_dispatches_two_point_task(self):
        self.route.waypoints = [[1, 2, 0], [2, 3, 0.5]]
        self.route.save(update_fields=["waypoints", "updated_at"])
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(
            f"/api/robots/{self.robot.id}/charging-dock/",
            {"map_id": self.map.id, "route_id": self.route.id},
            format="json",
        )
        self.assertEqual(response.status_code, 201)
        self.robot.refresh_from_db()
        self.assertEqual(self.robot.charging_map_id, self.map.id)
        self.assertEqual(self.robot.charging_route_id, self.route.id)
        command = RemoteCommand.objects.get(task_execution__isnull=False)
        self.assertTrue(command.payload["docking"]["enabled"])

    def test_charging_dock_rejects_non_two_point_route(self):
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(
            f"/api/robots/{self.robot.id}/charging-dock/",
            {"map_id": self.map.id, "route_id": self.route.id},
            format="json",
        )
        self.assertEqual(response.status_code, 400)

    def test_task_execute_rejects_invalid_navigation_rosbag_flag(self):
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(
            f"/api/patrol-tasks/{self.task.id}/execute/",
            {"record_rosbag": "yes"},
            format="json",
        )
        self.assertEqual(response.status_code, 400)

    def test_route_api_execute_creates_quick_task_and_command(self):
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(f"/api/routes/{self.route.id}/execute/", {}, format="json")
        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.data["route"], self.route.id)
        self.assertEqual(RemoteCommand.objects.count(), 1)
        self.assertTrue(PatrolTask.objects.filter(route=self.route, name=f"路线快速执行 - {self.route.name}").exists())

    def test_low_battery_route_execute_is_rejected_before_quick_task_creation(self):
        self.robot.battery_level = 19
        self.robot.save(update_fields=["battery_level", "updated_at"])
        client = APIClient()
        client.force_authenticate(self.user)

        response = client.post(f"/api/routes/{self.route.id}/execute/", {}, format="json")

        self.assertEqual(response.status_code, 403)
        self.assertFalse(PatrolTask.objects.filter(name=f"路线快速执行 - {self.route.name}").exists())
        self.assertEqual(TaskExecution.objects.count(), 0)
        self.assertEqual(RemoteCommand.objects.count(), 0)

    def test_disabled_task_cannot_execute(self):
        self.task.enabled = False
        self.task.save(update_fields=["enabled", "updated_at"])
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(f"/api/patrol-tasks/{self.task.id}/execute/", {}, format="json")
        self.assertEqual(response.status_code, 409)
        self.assertIn("TASK_DISABLED", response.data["detail"])
