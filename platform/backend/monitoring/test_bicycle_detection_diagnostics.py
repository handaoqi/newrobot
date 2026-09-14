import io
import tempfile
from unittest.mock import patch

from django.contrib.auth import get_user_model
from django.contrib.auth.hashers import make_password
from django.core.files.uploadedfile import SimpleUploadedFile
from django.test import override_settings
from PIL import Image
from rest_framework.test import APIClient, APITestCase
from django.utils import timezone

from .models import (
    BicycleDetectionTestImage,
    BicycleDetectionTestRun,
    InspectionEvent,
    MediaAsset,
    Robot,
    RobotCommand,
    RobotCredential,
)
from .services.alert_service import is_bicycle_detection
from .services.bicycle_detection_test_service import expire_stalled_bicycle_detection_tests


class BicycleDetectionDiagnosticApiTests(APITestCase):
    def setUp(self):
        self.media_root = tempfile.TemporaryDirectory()
        self.settings_override = override_settings(MEDIA_ROOT=self.media_root.name)
        self.settings_override.enable()
        self.addCleanup(self.settings_override.disable)
        self.addCleanup(self.media_root.cleanup)
        self.robot = Robot.objects.create(code="VISION-TEST-01", name="视觉测试", location="测试区", area="测试区")
        self.user = get_user_model().objects.create_user(username="vision-operator", password="secret")
        self.web_client = APIClient()
        self.web_client.force_authenticate(self.user)
        RobotCredential.objects.create(
            robot=self.robot,
            credential_id=self.robot.code,
            secret_hash=make_password("device-secret"),
        )
        self.device_client = APIClient()
        self.device_headers = {
            "HTTP_X_DEVICE_ID": self.robot.code,
            "HTTP_X_DEVICE_CODE": self.robot.code,
            "HTTP_X_DEVICE_KEY": "device-secret",
        }

    @staticmethod
    def image_file(name="bike.jpg"):
        buffer = io.BytesIO()
        Image.new("RGB", (80, 60), "white").save(buffer, format="JPEG")
        return SimpleUploadedFile(name, buffer.getvalue(), content_type="image/jpeg")

    @patch("monitoring.views.event_broker.publish")
    def test_operator_image_test_creates_one_alert_per_qualified_photo(self, publish):
        response = self.web_client.post(
            "/api/events/bicycle-detection-tests/",
            {"robot_id": self.robot.id, "files": [self.image_file(), self.image_file("car.jpg")]},
            format="multipart",
        )
        self.assertEqual(response.status_code, 201)
        self.assertEqual(len(response.data["images"]), 2)
        run_id = response.data["id"]
        self.assertEqual(InspectionEvent.objects.count(), 0)

        response = self.device_client.get(
            "/api/device/bicycle-detection-tests/poll/",
            {"robot_code": self.robot.code},
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["run_id"], run_id)
        image_id = response.data["image_id"]

        response = self.device_client.get(
            f"/api/device/bicycle-detection-tests/{run_id}/images/{image_id}/input/",
            {"robot_code": self.robot.code},
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 200)

        response = self.device_client.post(
            f"/api/device/bicycle-detection-tests/{run_id}/images/{image_id}/report/",
            {
                "status": "finished",
                "result_code": "passed_single_frame",
                "detected_class": "motorcycle",
                "confidence": 0.9,
                "diagnostics": {
                    "bbox": {"x": 1, "y": 2, "width": 40, "height": 40},
                    "bbox_area": 1600,
                    "min_box_area": 1600,
                    "detections": [
                        {
                            "detected_class": "motorcycle", "confidence": 0.9, "bbox_area": 1600,
                            "bbox": {"x": 1, "y": 2, "width": 40, "height": 40},
                            "result_code": "passed_single_frame",
                        },
                        {
                            "detected_class": "bicycle", "confidence": 0.8, "bbox_area": 1800,
                            "bbox": {"x": 20, "y": 2, "width": 45, "height": 40},
                            "result_code": "passed_single_frame",
                        },
                    ],
                },
            },
            format="json",
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 200)
        image = BicycleDetectionTestImage.objects.get(id=image_id)
        self.assertEqual(image.detected_class, "motorcycle")
        self.assertEqual(image.result_code, "passed_single_frame")
        self.assertIsNotNone(image.alert_event_id)
        event = image.alert_event
        self.assertEqual(InspectionEvent.objects.count(), 1)
        self.assertEqual(event.object_class, "motorcycle")
        self.assertEqual(event.source_component, "event_center_photo_test")
        self.assertEqual(len(event.raw_detection["qualified_detections"]), 2)
        self.assertTrue(event.snapshot_asset_id)
        self.assertEqual(MediaAsset.objects.count(), 1)
        self.assertEqual(RobotCommand.objects.count(), 0)
        self.assertEqual(publish.call_args.args[0], "inspection_event_created")

        duplicate = self.device_client.post(
            f"/api/device/bicycle-detection-tests/{run_id}/images/{image_id}/report/",
            {
                "status": "finished",
                "result_code": "passed_single_frame",
                "detected_class": "motorcycle",
                "confidence": 0.9,
                "diagnostics": {"detections": []},
            },
            format="json",
            **self.device_headers,
        )
        self.assertEqual(duplicate.status_code, 200)
        self.assertEqual(InspectionEvent.objects.count(), 1)

        response = self.web_client.get(f"/api/events/bicycle-detection-tests/{run_id}/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["images"][0]["detected_class"], "motorcycle")
        self.assertEqual(response.data["images"][0]["alert_event_id"], event.id)
        self.assertEqual(len(response.data["images"][0]["diagnostics"]["detections"]), 2)

    def test_unqualified_photo_does_not_create_alert(self):
        run = BicycleDetectionTestRun.objects.create(
            robot=self.robot, created_by=self.user, expires_at=timezone.now() + timezone.timedelta(hours=24)
        )
        image = BicycleDetectionTestImage.objects.create(
            run=run, sequence=1, original_name="small-bike.jpg", source_file=self.image_file("small-bike.jpg")
        )
        response = self.device_client.post(
            f"/api/device/bicycle-detection-tests/{run.id}/images/{image.id}/report/",
            {
                "status": "finished",
                "result_code": "below_min_box_area",
                "detected_class": "bicycle",
                "confidence": 0.9,
                "diagnostics": {
                    "detections": [{"detected_class": "bicycle", "result_code": "below_min_box_area"}],
                },
            },
            format="json",
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 200)
        image.refresh_from_db()
        self.assertIsNone(image.alert_event_id)
        self.assertEqual(InspectionEvent.objects.count(), 0)

    def test_invalid_test_image_is_rejected(self):
        invalid = SimpleUploadedFile("not-image.jpg", b"not an image", content_type="image/jpeg")
        response = self.web_client.post(
            "/api/events/bicycle-detection-tests/",
            {"robot_id": self.robot.id, "files": invalid},
            format="multipart",
        )
        self.assertEqual(response.status_code, 400)
        self.assertEqual(BicycleDetectionTestRun.objects.count(), 0)

    def test_all_coco_vehicle_alert_group_classes_are_accepted(self):
        for object_class in ("bicycle", "car", "motorcycle"):
            self.assertTrue(is_bicycle_detection({"object_class": object_class}))

    def test_stalled_test_is_marked_failed_after_120_seconds(self):
        run = BicycleDetectionTestRun.objects.create(
            robot=self.robot, created_by=self.user, expires_at=timezone.now() + timezone.timedelta(hours=24)
        )
        image = BicycleDetectionTestImage.objects.create(
            run=run, sequence=1, original_name="old.jpg", source_file=self.image_file("old.jpg")
        )
        stale_at = timezone.now() - timezone.timedelta(seconds=121)
        BicycleDetectionTestRun.objects.filter(id=run.id).update(created_at=stale_at)

        self.assertEqual(expire_stalled_bicycle_detection_tests(), 1)
        run.refresh_from_db()
        image.refresh_from_db()
        self.assertEqual(run.status, "failed")
        self.assertEqual(image.status, "failed")

    @patch("monitoring.views.event_broker.publish")
    def test_handling_event_publishes_realtime_update(self, publish):
        event = InspectionEvent.objects.create(
            robot=self.robot,
            title="自行车告警",
            event_type="vehicle_illegal_parking",
            location="测试区",
            confidence=0.9,
        )
        response = self.web_client.post(
            f"/api/events/{event.id}/handle/",
            {"status": "resolved", "review_result": "confirmed", "handling_notes": "已处理"},
            format="json",
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(publish.call_args.args[0], "inspection_event_updated")
