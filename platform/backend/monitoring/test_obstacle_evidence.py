import io
import tempfile
import uuid

from django.contrib.auth.hashers import make_password
from django.core.files.uploadedfile import SimpleUploadedFile
from django.test import override_settings
from django.utils import timezone
from PIL import Image
from rest_framework.test import APIClient, APITestCase

from .models import MediaAsset, PatrolRoute, PatrolTask, Robot, RobotCredential
from .services.alert_service import AlertService
from .services.task_service import TaskExecutionService
from .models import MapData


class ObstacleEvidenceApiTests(APITestCase):
    def setUp(self):
        self.media_root = tempfile.TemporaryDirectory()
        self.settings_override = override_settings(MEDIA_ROOT=self.media_root.name)
        self.settings_override.enable()
        self.addCleanup(self.settings_override.disable)
        self.addCleanup(self.media_root.cleanup)
        self.robot = Robot.objects.create(code="OBSTACLE-01", name="避障测试", location="测试区", area="测试区")
        RobotCredential.objects.create(
            robot=self.robot,
            credential_id=self.robot.code,
            secret_hash=make_password("device-secret"),
        )
        map_data = MapData.objects.create(name="map", robot=self.robot)
        route = PatrolRoute.objects.create(
            name="route", map_data=map_data, robot=self.robot, waypoints=[[1, 2], [2, 3]]
        )
        now = timezone.now()
        task = PatrolTask.objects.create(
            name="task",
            robot=self.robot,
            route=route,
            route_name=route.name,
            scheduled_start=now,
            scheduled_end=now + timezone.timedelta(hours=1),
        )
        self.execution = TaskExecutionService.create_execution(task)
        self.client = APIClient()
        self.headers = {
            "HTTP_X_DEVICE_ID": self.robot.code,
            "HTTP_X_DEVICE_CODE": self.robot.code,
            "HTTP_X_DEVICE_KEY": "device-secret",
        }

    @staticmethod
    def image_file(name="obstacle.jpg"):
        buffer = io.BytesIO()
        Image.new("RGB", (80, 60), "gray").save(buffer, format="JPEG")
        return SimpleUploadedFile(name, buffer.getvalue(), content_type="image/jpeg")

    def upload(self, event_id):
        return self.client.post(
            "/api/device/media/upload/",
            {
                "robot_code": self.robot.code,
                "camera_id": "front",
                "media_type": "snapshot",
                "sequence_id": "episode-one",
                "event_id": str(event_id),
                "task_execution_id": str(self.execution.id),
                "event_time": timezone.now().isoformat(),
                "file": self.image_file(),
            },
            format="multipart",
            **self.headers,
        )

    def obstacle_payload(self, episode_id):
        return {
            "task_execution_id": str(self.execution.id),
            "obstacle_episode_id": episode_id,
            "stage": "DETECTED_STOP",
            "alert_description": "发现障碍物，已停车",
            "reported_at": timezone.now().isoformat(),
        }

    def test_event_first_is_linked_when_snapshot_upload_arrives(self):
        event, _ = AlertService.ingest_obstacle_stage(
            self.robot,
            self.execution,
            self.obstacle_payload("episode-one"),
            occurred_at=timezone.now(),
        )

        response = self.upload(event.event_id)

        self.assertEqual(response.status_code, 201)
        event.refresh_from_db()
        self.assertIsNotNone(event.snapshot_asset_id)
        self.assertEqual(event.snapshot_url, response.data["url"])

        duplicate = self.upload(event.event_id)
        self.assertEqual(duplicate.status_code, 200)
        self.assertTrue(duplicate.data["already_exists"])
        self.assertEqual(MediaAsset.objects.filter(event_id=event.event_id).count(), 1)

    def test_snapshot_first_is_linked_when_obstacle_event_arrives(self):
        episode_id = "episode-image-first"
        event_id = uuid.uuid5(
            uuid.NAMESPACE_URL,
            f"roamerx:obstacle:{self.execution.pk}:{episode_id}",
        )
        response = self.upload(event_id)
        self.assertEqual(response.status_code, 201)

        event, _ = AlertService.ingest_obstacle_stage(
            self.robot,
            self.execution,
            self.obstacle_payload(episode_id),
            occurred_at=timezone.now(),
        )

        self.assertEqual(event.event_id, event_id)
        self.assertIsNotNone(event.snapshot_asset_id)
        self.assertEqual(event.snapshot_url, response.data["url"])
