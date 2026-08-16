from django.test import TestCase
from django.utils import timezone

from .models import Robot
from .serializers import RobotStatusSerializer
from .services.telemetry_service import TelemetryService


class TelemetryServiceTests(TestCase):
    def test_accepts_advanced_mapping_progress_with_stale_pose_sample(self):
        robot = Robot.objects.create(code="mapping-robot", name="Mapping Robot")
        sampled_at = timezone.now().isoformat()
        base = {
            "sampled_at": sampled_at,
            "state_version": 7,
            "mapping": {"save_progress": {"updated_at_unix": 100}},
        }
        TelemetryService.apply_status(robot, base)

        latest, _ = TelemetryService.apply_status(
            robot,
            {
                **base,
                "mapping": {
                    "save_progress": {
                        "updated_at_unix": 102,
                        "stage": "mapping",
                        "keyframe_count": 12,
                    }
                },
            },
        )

        self.assertEqual(latest.raw_payload["mapping"]["save_progress"]["keyframe_count"], 12)

    def test_rejects_unchanged_stale_status(self):
        robot = Robot.objects.create(code="stale-robot", name="Stale Robot")
        payload = {
            "sampled_at": timezone.now().isoformat(),
            "state_version": 7,
            "mapping": {"save_progress": {"updated_at_unix": 100}},
        }
        TelemetryService.apply_status(robot, payload)

        _, changed = TelemetryService.apply_status(robot, payload)

        self.assertFalse(changed)

    def test_preserves_localization_source_decision_in_quality_details(self):
        robot = Robot.objects.create(code="fusion-robot", name="Fusion Robot")
        latest, _ = TelemetryService.apply_status(
            robot,
            {
                "sampled_at": timezone.now().isoformat(),
                "state_version": 1,
                "localization": {
                    "status": "normal",
                    "quality": {"matching_error": 0.12},
                    "decision": {
                        "active_source": "rtk_imu",
                        "rtk_x": 12.3,
                        "rtk_y": 4.5,
                        "rtk_yaw": 0.7,
                    },
                },
            },
        )

        self.assertEqual(latest.localization_quality["decision"]["active_source"], "rtk_imu")
        self.assertEqual(latest.localization_quality["decision"]["rtk_x"], 12.3)

        serialized_quality = RobotStatusSerializer(latest).data["localization_quality"]
        self.assertEqual(serialized_quality["matching_error"], 0.12)
        self.assertEqual(serialized_quality["decision"]["active_source"], "rtk_imu")
        self.assertEqual(serialized_quality["decision"]["rtk_yaw"], 0.7)
