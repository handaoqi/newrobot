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
                "global_controller": "navfn",
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
                "global_controller": "navfn",
                "x": 1.25,
                "y": -0.5,
                "yaw": 0.75,
            },
        )

    def test_navigation_single_goal_rejects_invalid_global_controller(self):
        response = self.client.post(
            f"/api/robots/{self.robot.id}/navigation/single-goal/",
            {
                "x": 1.25,
                "y": -0.5,
                "yaw": 0.75,
                "global_controller": "invalid",
            },
            format="json",
        )

        self.assertEqual(response.status_code, 400)
        self.assertEqual(RemoteCommand.objects.count(), 0)

    def test_progressive_relocalization_forwards_ordered_route_waypoints(self):
        response = self.client.post(
            f"/api/robots/{self.robot.id}/navigation/relocalize/",
            {
                "seed_source": "progressive",
                "map_id": "7",
                "map_version": "v3",
                "wait_seconds": 180,
                "waypoints": [
                    {"x": 1, "y": 2, "yaw": 0.1, "name": "ignored"},
                    {"x": 3, "y": 4, "yaw": -0.2},
                ],
            },
            format="json",
        )

        self.assertEqual(response.status_code, 202, response.data)
        command = RemoteCommand.objects.get(pk=response.data["id"])
        self.assertEqual(command.command_type, "nav.relocalize")
        self.assertEqual(command.payload["seed_source"], "progressive")
        self.assertEqual(command.payload["wait_seconds"], 180.0)
        self.assertEqual(command.payload["waypoints"], [
            {"x": 1.0, "y": 2.0, "yaw": 0.1},
            {"x": 3.0, "y": 4.0, "yaw": -0.2},
        ])

    def test_trusted_pose_relocalization_reserves_time_for_all_bounded_candidates(self):
        response = self.client.post(
            f"/api/robots/{self.robot.id}/navigation/relocalize/",
            {"seed_source": "last_trusted", "map_id": "7", "map_version": "v3"},
            format="json",
        )

        self.assertEqual(response.status_code, 202, response.data)
        command = RemoteCommand.objects.get(pk=response.data["id"])
        self.assertEqual(command.payload["wait_seconds"], 180.0)

    def test_rtk_initial_pose_forwards_validation_budget_and_navigation_start(self):
        response = self.client.post(
            f"/api/robots/{self.robot.id}/navigation/initial-pose/",
            {
                "seed_source": "rtk",
                "map_id": "7",
                "map_version": "v3",
                "wait_seconds": 30,
                "start_navigation": True,
            },
            format="json",
        )

        self.assertEqual(response.status_code, 202, response.data)
        command = RemoteCommand.objects.get(pk=response.data["id"])
        self.assertEqual(command.command_type, "nav.initial_pose")
        self.assertEqual(command.payload["reason"], "outdoor_rtk_initialization")
        self.assertEqual(command.payload["seed_source"], "rtk")
        self.assertEqual(command.payload["wait_seconds"], 30.0)
        self.assertIs(command.payload["start_navigation"], True)
