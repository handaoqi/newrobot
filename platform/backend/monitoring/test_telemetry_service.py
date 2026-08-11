from django.test import TestCase
from django.utils import timezone

from .models import Robot
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
