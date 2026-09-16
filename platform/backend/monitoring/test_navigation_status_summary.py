import json
from datetime import timedelta

from django.test import TestCase
from django.utils import timezone
from rest_framework.test import APIClient

from .models import RemoteCommand, Robot, RobotStatusLatest


class NavigationStatusSummaryTests(TestCase):
    def setUp(self):
        self.client = APIClient()
        now = timezone.now()
        self.robot = Robot.objects.create(
            code="summary-nav-dog",
            name="Summary Nav Dog",
            location="yard",
            area="yard",
            connection_status="online",
            localization_status="normal",
            ros_ready=True,
            nav_ready=True,
            current_map_id="151",
            current_map_version="legacy-mapdata-151",
            last_seen_at=now,
        )
        RobotStatusLatest.objects.create(
            robot=self.robot,
            state_version=42,
            sampled_at=now,
            received_at=now,
            map_id="151",
            map_version="legacy-mapdata-151",
            x="1.2500",
            y="-2.5000",
            yaw="0.75000",
            speed_mps="0.30000",
            localization_status="normal",
            localization_source_status=2,
            localization_quality={
                "localization_fresh": True,
                "localization_sample_age_seconds": 0.05,
                "has_converged": True,
                "matching_error": 0.01,
                "decision": {"detail": "large-internal-detail" * 100},
            },
            ros_ready=True,
            nav_ready=True,
            control_mode="autonomous",
            raw_payload={
                "navigation": {
                    "requested_planar_speed_mps": 0.4,
                    "actual_planar_speed_mps": 0.3,
                    "actual_velocity_sample_age_seconds": 0.04,
                    "internal_debug": "large-navigation-detail" * 100,
                },
                "sensors": {"snapshot": "sensor-detail" * 100},
                "power": {"services": "power-detail" * 100},
            },
        )
        RemoteCommand.objects.create(
            robot=self.robot,
            command_type="nav.initial_pose",
            status="succeeded",
            expires_at=now + timedelta(hours=1),
            payload={
                "map_id": "151",
                "map_version": "legacy-mapdata-151",
                "unrelated_operator_data": "must-not-be-exposed",
            },
            result_payload={"attempts": "x" * 40_000},
        )

    def test_summary_keeps_readiness_fields_and_omits_large_details(self):
        full = self.client.get(f"/api/robots/{self.robot.id}/navigation/status/")
        summary = self.client.get(
            f"/api/robots/{self.robot.id}/navigation/status/?view=summary"
        )

        self.assertEqual(full.status_code, 200)
        self.assertEqual(summary.status_code, 200)
        self.assertIn("result_payload", full.data["localization_command"])
        self.assertEqual(
            full.data["localization_command"]["payload"],
            {"map_id": "151", "map_version": "legacy-mapdata-151"},
        )
        self.assertNotIn(
            "unrelated_operator_data", full.data["localization_command"]["payload"]
        )
        self.assertIsNone(summary.data["localization_command"])

        status = summary.data["status"]
        self.assertEqual(status["map_id"], "151")
        self.assertEqual(status["map_version"], "legacy-mapdata-151")
        self.assertEqual(status["current_map"]["map_id"], "151")
        self.assertEqual(status["localization_status"], "normal")
        self.assertTrue(status["nav_ready"])
        self.assertTrue(status["localization_quality"]["localization_fresh"])
        self.assertEqual(status["navigation"]["actual_planar_speed_mps"], 0.3)
        self.assertNotIn("decision", status["localization_quality"])
        self.assertNotIn("internal_debug", status["navigation"])
        self.assertNotIn("sensors", status)
        self.assertNotIn("power", status)

        full_size = len(json.dumps(full.data, default=str).encode())
        summary_size = len(json.dumps(summary.data, default=str).encode())
        self.assertLess(summary_size, 3_000)
        self.assertGreater(full_size, summary_size * 5)
