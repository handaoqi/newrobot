from django.test import TestCase
from django.utils import timezone
from rest_framework.test import APIClient

from .models import RemoteCommand, Robot


class NavigationSingleGoalTests(TestCase):
    def setUp(self):
        self.client = APIClient()
        self.robot = Robot.objects.create(
            code="RX-SINGLE-GOAL-01",
            name="Single Goal Robot",
            connection_status="online",
            last_seen_at=timezone.now(),
        )

    def test_navigation_single_goal_creates_remote_command(self):
        response = self.client.post(
            f"/api/robots/{self.robot.id}/navigation/single-goal/",
            {
                "frame_id": "map",
                "x": 1.25,
                "y": -0.5,
                "yaw": 0.75,
                "require_yaw": True,
                "map_id": "7",
                "map_version": "v3",
            },
            format="json",
        )

        self.assertEqual(response.status_code, 202, response.data)
        command = RemoteCommand.objects.get(pk=response.data["id"])
        self.assertEqual(command.command_type, "nav.single_goal")
        self.assertEqual(
            command.payload,
            {
                "reason": "route_planner_single_goal",
                "frame_id": "map",
                "map_id": "7",
                "map_version": "v3",
                "require_yaw": True,
                "x": 1.25,
                "y": -0.5,
                "yaw": 0.75,
            },
        )
