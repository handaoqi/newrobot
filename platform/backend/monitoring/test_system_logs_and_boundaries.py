from __future__ import annotations

from django.contrib.auth import get_user_model
from django.test import TestCase
from django.utils import timezone
from rest_framework.exceptions import ValidationError
from rest_framework.test import APIClient

from .models import DebugLogSession, MapData, MapNavigationBoundary, RemoteCommand, Robot, SystemLog
from .services.navigation_boundary_service import validate_waypoints_against_boundary
from .services.system_log_service import ingest_batch


class SystemLogApiTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="RX-LOG-01", name="Log Robot")
        self.user = get_user_model().objects.create_user(username="operator", password="test")
        self.staff = get_user_model().objects.create_user(username="admin", password="test", is_staff=True)
        self.client = APIClient()

    def test_ingest_redacts_deduplicates_and_filters_logs(self):
        payload = {
            "entries": [{
                "occurred_at": timezone.now().isoformat(),
                "level": "WARNING",
                "module": "avoidance",
                "event_code": "avoidance.blocked",
                "message": "前方持续受阻",
                "data": {"front_distance_m": 0.18, "token": "must-not-leak"},
                "pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            }],
        }
        self.assertEqual(ingest_batch(self.robot, payload)["accepted"], 1)
        self.assertEqual(ingest_batch(self.robot, payload)["accepted"], 1)
        row = SystemLog.objects.get()
        self.assertEqual(row.repeat_count, 2)
        self.assertEqual(row.data["token"], "<redacted>")

        self.client.force_authenticate(self.user)
        response = self.client.get(
            f"/api/robots/{self.robot.id}/system-logs/",
            {"levels": "WARNING,ERROR", "modules": "avoidance", "q": "blocked"},
        )
        self.assertEqual(response.status_code, 200, response.data)
        self.assertEqual(len(response.data["results"]), 1)
        self.assertEqual(response.data["results"][0]["x"], 1.0)

    def test_only_staff_can_start_bounded_debug_session(self):
        self.client.force_authenticate(self.user)
        denied = self.client.post(
            f"/api/robots/{self.robot.id}/debug-log-sessions/",
            {"duration_seconds": 300, "modules": ["planner"]},
            format="json",
        )
        self.assertEqual(denied.status_code, 403)

        self.client.force_authenticate(self.staff)
        response = self.client.post(
            f"/api/robots/{self.robot.id}/debug-log-sessions/",
            {"duration_seconds": 300, "sample_hz": 2, "modules": ["planner", "localization"]},
            format="json",
        )
        self.assertEqual(response.status_code, 201, response.data)
        session = DebugLogSession.objects.get(pk=response.data["id"])
        command = RemoteCommand.objects.get(pk=response.data["command_id"])
        self.assertEqual(session.status, "starting")
        self.assertEqual(command.command_type, "diagnostics.log_config")
        self.assertEqual(command.payload["sample_hz"], 2.0)


class NavigationBoundaryApiTests(TestCase):
    def setUp(self):
        self.client = APIClient()
        self.user = get_user_model().objects.create_user(username="boundary-operator", password="test")
        self.client.force_authenticate(self.user)
        self.robot = Robot.objects.create(
            code="RX-BOUNDARY-01", name="Boundary Robot", connection_status="online",
            last_seen_at=timezone.now(), current_map_version="map-v1",
        )
        self.map_data = MapData.objects.create(
            name="Boundary Map", robot=self.robot, resolution=0.05,
            width=200, height=200, origin=[0, 0, 0],
        )
        self.robot.current_map_id = str(self.map_data.id)
        self.robot.save(update_fields=["current_map_id", "updated_at"])
        self.draft = {
            "revision": 0,
            "outer_polygon": [[0, 0], [10, 0], [10, 10], [0, 10]],
            "safety_margin_m": 0.2,
            "zones": [{
                "name": "花坛", "zone_type": "forbidden", "active": True,
                "polygon": [[4, 4], [6, 4], [6, 6], [4, 6]],
                "warning_distance_m": 0.5,
            }],
        }

    def test_save_publish_and_preserve_active_snapshot_across_next_draft(self):
        saved = self.client.put(
            f"/api/maps/{self.map_data.id}/navigation-boundaries/", self.draft, format="json",
        )
        self.assertEqual(saved.status_code, 200, saved.data)
        self.assertEqual(saved.data["revision"], 1)
        published = self.client.post(
            f"/api/maps/{self.map_data.id}/navigation-boundaries/publish/",
            {"robot_id": self.robot.id}, format="json",
        )
        self.assertEqual(published.status_code, 202, published.data)
        command = RemoteCommand.objects.get(pk=published.data["command_id"])
        self.assertEqual(command.command_type, "map.boundary_apply")
        self.assertEqual(command.payload["boundary"]["revision"], 1)

        boundary = MapNavigationBoundary.objects.get(map_data=self.map_data)
        boundary.active_revision = 1
        boundary.active_payload = command.payload["boundary"]
        boundary.apply_status = "active"
        boundary.save()
        changed = {**self.draft, "revision": 1, "outer_polygon": [[0, 0], [8, 0], [8, 8], [0, 8]]}
        saved_again = self.client.put(
            f"/api/maps/{self.map_data.id}/navigation-boundaries/", changed, format="json",
        )
        self.assertEqual(saved_again.status_code, 200, saved_again.data)
        boundary.refresh_from_db()
        self.assertEqual(boundary.revision, 2)
        self.assertEqual(boundary.active_payload["outer_polygon"][1], [10.0, 0.0])

    def test_active_boundary_rejects_unsafe_single_goal_and_crossing_route(self):
        saved = self.client.put(
            f"/api/maps/{self.map_data.id}/navigation-boundaries/", self.draft, format="json",
        )
        boundary = MapNavigationBoundary.objects.get(map_data=self.map_data)
        boundary.active_revision = saved.data["revision"]
        boundary.active_payload = saved.data
        boundary.apply_status = "active"
        boundary.save()

        outside = self.client.post(
            f"/api/robots/{self.robot.id}/navigation/single-goal/",
            {"map_id": self.map_data.id, "x": 0.1, "y": 2, "yaw": 0}, format="json",
        )
        self.assertEqual(outside.status_code, 400, outside.data)
        forbidden = self.client.post(
            f"/api/robots/{self.robot.id}/navigation/single-goal/",
            {"map_id": self.map_data.id, "x": 4.1, "y": 5, "yaw": 0}, format="json",
        )
        self.assertEqual(forbidden.status_code, 400, forbidden.data)
        accepted = self.client.post(
            f"/api/robots/{self.robot.id}/navigation/single-goal/",
            {"map_id": self.map_data.id, "x": 2, "y": 2, "yaw": 0}, format="json",
        )
        self.assertEqual(accepted.status_code, 202, accepted.data)
        self.assertEqual(RemoteCommand.objects.get(pk=accepted.data["id"]).payload["boundary_revision"], 1)

        with self.assertRaisesMessage(ValidationError, "连线穿越硬边界"):
            validate_waypoints_against_boundary(
                self.map_data,
                [{"x": 2, "y": 5}, {"x": 8, "y": 5}],
            )

    def test_rejects_self_intersection_and_stale_revision(self):
        invalid = {**self.draft, "outer_polygon": [[0, 0], [10, 10], [0, 10], [10, 0]]}
        response = self.client.put(
            f"/api/maps/{self.map_data.id}/navigation-boundaries/", invalid, format="json",
        )
        self.assertEqual(response.status_code, 400)
        saved = self.client.put(
            f"/api/maps/{self.map_data.id}/navigation-boundaries/", self.draft, format="json",
        )
        self.assertEqual(saved.status_code, 200)
        stale = self.client.put(
            f"/api/maps/{self.map_data.id}/navigation-boundaries/", self.draft, format="json",
        )
        self.assertEqual(stale.status_code, 409)
