from django.test import TestCase

from .models import MapData, PatrolRoute, Robot
from .serializers import PatrolRouteSerializer
from .services.map_coordinate import MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN
from .services.task_service import TaskExecutionService, TaskStateError


class MapCoordinateTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="rx-map", name="RX Map", location="park", area="park")
        self.local_map = MapData.objects.create(
            name="indoor-local",
            robot=self.robot,
            coordinate_mode="local_only",
            scene_scope="indoor",
            localization_mode="ndt",
            origin_status="local_only",
            map_completeness="complete",
        )
        self.task_route = PatrolRoute.objects.create(
            name="indoor-route",
            map_data=self.local_map,
            robot=self.robot,
            scene_scope="indoor",
            waypoints=[{"x": 1.0, "y": 2.0, "yaw": 0.0, "localization_mode": "ndt"}],
        )

    def test_serializer_rejects_outdoor_binding_for_local_only_map(self):
        serializer = PatrolRouteSerializer(
            data={
                "name": "outdoor",
                "map_data": self.local_map.id,
                "robot": self.robot.id,
                "scene_scope": "outdoor",
                "waypoints": [{"x": 1.0, "y": 2.0, "yaw": 0.0, "localization_mode": "ndt"}],
            }
        )
        self.assertFalse(serializer.is_valid())
        self.assertIn("local_only", str(serializer.errors).lower())

    def test_serializer_rejects_rtk_waypoints_for_local_only_map(self):
        serializer = PatrolRouteSerializer(
            data={
                "name": "rtk-route",
                "map_data": self.local_map.id,
                "robot": self.robot.id,
                "scene_scope": "indoor",
                "waypoints": [{"x": 1.0, "y": 2.0, "yaw": 0.0, "localization_mode": "rtk"}],
            }
        )
        self.assertFalse(serializer.is_valid())

    def test_serializer_accepts_ukf_waypoints_for_local_only_map(self):
        serializer = PatrolRouteSerializer(
            data={
                "name": "corridor-ukf",
                "map_data": self.local_map.id,
                "robot": self.robot.id,
                "scene_scope": "indoor",
                "waypoints": [{"x": 1.0, "y": 2.0, "yaw": 0.0, "localization_mode": "ukf"}],
            }
        )
        self.assertTrue(serializer.is_valid(), serializer.errors)

    def test_task_start_rejects_local_only_outdoor_route(self):
        from .models import PatrolTask
        from django.utils import timezone

        self.task_route.scene_scope = "outdoor"
        self.task_route.save(update_fields=["scene_scope", "updated_at"])
        task = PatrolTask.objects.create(
            name="outdoor-task",
            robot=self.robot,
            route=self.task_route,
            route_name=self.task_route.name,
            scheduled_start=timezone.now(),
            scheduled_end=timezone.now(),
        )
        with self.assertRaises(TaskStateError) as error:
            TaskExecutionService.create_execution(task)
        self.assertEqual(str(error.exception), MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN)

    def test_legacy_map_still_allows_rtk_waypoints(self):
        legacy = MapData.objects.create(name="legacy", robot=self.robot)
        serializer = PatrolRouteSerializer(
            data={
                "name": "legacy-rtk",
                "map_data": legacy.id,
                "robot": self.robot.id,
                "scene_scope": "outdoor",
                "waypoints": [{"x": 1.0, "y": 2.0, "yaw": 0.0, "localization_mode": "rtk"}],
            }
        )
        self.assertTrue(serializer.is_valid(), serializer.errors)
