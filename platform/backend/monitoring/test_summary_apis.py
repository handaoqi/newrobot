from datetime import timedelta

from django.test import TestCase
from django.contrib.auth import get_user_model
from django.utils import timezone
from rest_framework.test import APIClient

from .models import MapData, MapSet, MapSetMember, PatrolRoute, PatrolTask, Robot, TaskExecution, Track


class SummaryListApiTests(TestCase):
    def setUp(self):
        self.client = APIClient()
        self.client.force_authenticate(get_user_model().objects.create_user("summary-user"))
        self.robot = Robot.objects.create(code="summary-dog", name="Summary Dog", location="yard", area="yard")
        self.map = MapData.objects.create(
            name="summary map", robot=self.robot, description="large description",
            edit_metadata={"strokes": list(range(30))}, mapping_metrics={"coverage": 0.91},
        )
        self.map_set = MapSet.objects.create(
            name="summary set", robot=self.robot, manifest={"total_distance_m": 12.5}
        )
        MapSetMember.objects.create(map_set=self.map_set, map_data=self.map, sequence=1, submap_id="sub-1")
        self.route = PatrolRoute.objects.create(
            name="summary route", map_data=self.map, robot=self.robot,
            waypoints=[{"x": index, "y": index + 1} for index in range(4)],
            waypoint_names=["one", "two", "three", "four"],
        )
        self.task = PatrolTask.objects.create(
            name="summary task", robot=self.robot, route=self.route, route_name=self.route.name,
            scheduled_start=timezone.now(), scheduled_end=timezone.now() + timedelta(hours=1),
        )
        self.track = Track.objects.create(
            robot=self.robot, map_data=self.map, route=self.route, task=self.task,
            path=[[1, 2, 3], [2, 3, 4], [3, 4, 5]], start_time=timezone.now(), duration=9,
        )

    def test_summary_payloads_are_compact(self):
        maps = self.client.get("/api/maps/?view=summary")
        self.assertEqual(maps.status_code, 200)
        self.assertEqual(len(maps.data), 1)
        self.assertNotIn("description", maps.data[0])
        self.assertNotIn("edit_metadata", maps.data[0])
        self.assertNotIn("mapping_metrics", maps.data[0])
        self.assertIn("thumbnail_url", maps.data[0])

        map_sets = self.client.get("/api/map-sets/?view=summary")
        self.assertEqual(map_sets.data[0]["member_count"], 1)
        self.assertEqual(map_sets.data[0]["submap_ids"], ["sub-1"])
        self.assertNotIn("members", map_sets.data[0])

        routes = self.client.get("/api/routes/?view=summary")
        self.assertEqual(routes.data[0]["waypoint_count"], 4)
        self.assertIsNone(routes.data[0]["latest_execution"])
        self.assertNotIn("waypoints", routes.data[0])

        tracks = self.client.get("/api/tracks/?view=summary")
        self.assertEqual(tracks.data[0]["point_count"], 3)
        self.assertEqual(tracks.data[0]["duration"], 9)
        self.assertNotIn("path", tracks.data[0])

    def test_lists_without_summary_remain_full_payloads(self):
        self.assertIn("description", self.client.get("/api/maps/").data[0])
        self.assertIn("edit_metadata", self.client.get("/api/maps/").data[0])
        self.assertIn("members", self.client.get("/api/map-sets/").data[0])
        self.assertIn("waypoints", self.client.get("/api/routes/").data[0])
        full_tracks = self.client.get("/api/tracks/")
        self.assertIn("path", full_tracks.data[0])
        self.assertIn("path", self.client.get(f"/api/tracks/{self.track.id}/").data)

    def test_route_summary_includes_latest_execution_time_and_state(self):
        execution = TaskExecution.objects.create(
            task=self.task,
            robot=self.robot,
            route=self.route,
            map_data=self.map,
            route_snapshot={},
            state="completed",
        )

        route = self.client.get("/api/routes/?view=summary").data[0]

        self.assertEqual(route["latest_execution"]["id"], str(execution.id))
        self.assertEqual(route["latest_execution"]["state"], "completed")
        self.assertIsNotNone(route["latest_execution"]["created_at"])
