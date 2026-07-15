from django.contrib.auth import get_user_model
from django.test import TestCase
from django.utils import timezone
from rest_framework.test import APIClient

from .models import MapData, PatrolRoute, PatrolTask, RemoteCommand, Robot, TaskExecution
from .services.command_service import CommandService
from .services.task_service import TaskExecutionService, TaskStateError, assert_transition_allowed


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

    def test_task_api_execute_and_busy_conflict(self):
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(f"/api/patrol-tasks/{self.task.id}/execute/", {}, format="json")
        self.assertEqual(response.status_code, 201)
        second = client.post(f"/api/patrol-tasks/{self.task.id}/execute/", {}, format="json")
        self.assertEqual(second.status_code, 409)
        self.assertEqual(RemoteCommand.objects.count(), 1)

    def test_route_api_execute_creates_quick_task_and_command(self):
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(f"/api/routes/{self.route.id}/execute/", {}, format="json")
        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.data["route"], self.route.id)
        self.assertEqual(RemoteCommand.objects.count(), 1)
        self.assertTrue(PatrolTask.objects.filter(route=self.route, name=f"路线快速执行 - {self.route.name}").exists())

    def test_disabled_task_cannot_execute(self):
        self.task.enabled = False
        self.task.save(update_fields=["enabled", "updated_at"])
        client = APIClient()
        client.force_authenticate(self.user)
        response = client.post(f"/api/patrol-tasks/{self.task.id}/execute/", {}, format="json")
        self.assertEqual(response.status_code, 409)
        self.assertIn("TASK_DISABLED", response.data["detail"])
