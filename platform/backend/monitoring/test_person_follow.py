from django.contrib.auth import get_user_model
from django.contrib.auth.hashers import make_password
from django.utils import timezone
from rest_framework.test import APIClient, APITestCase

from .models import RemoteCommand, Robot, RobotCredential, RobotPersonDetectionState


class PersonFollowApiTests(APITestCase):
    def setUp(self):
        self.robot = Robot.objects.create(
            code="FOLLOW-TEST-01",
            name="跟随测试机器人",
            location="测试区",
            area="测试区",
            connection_status="online",
            last_seen_at=timezone.now(),
        )
        RobotCredential.objects.create(
            robot=self.robot,
            credential_id=self.robot.code,
            secret_hash=make_password("device-secret"),
        )
        self.device_headers = {
            "HTTP_X_DEVICE_ID": self.robot.code,
            "HTTP_X_DEVICE_CODE": self.robot.code,
            "HTTP_X_DEVICE_KEY": "device-secret",
        }
        self.web_client = APIClient()
        self.web_client.force_authenticate(
            get_user_model().objects.create_user(username="follow-operator", password="secret")
        )

    def test_device_can_update_latest_person_frame(self):
        response = APIClient().post(
            "/api/device/person-detections/",
            {
                "robot_code": self.robot.code,
                "camera_id": "front",
                "frame_width": 1280,
                "frame_height": 720,
                "captured_at": timezone.now().isoformat(),
                "detections": [
                    {
                        "track_id": "person-7",
                        "confidence": 0.91,
                        "bbox": {"x": 320, "y": 80, "width": 240, "height": 520},
                    }
                ],
            },
            format="json",
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 200)
        state = RobotPersonDetectionState.objects.get(robot=self.robot)
        self.assertEqual(state.detections[0]["track_id"], "person-7")

        response = self.web_client.get(f"/api/robots/{self.robot.id}/person-detections/")
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.data["available"])
        self.assertEqual(response.data["detections"][0]["bbox"]["height"], 520)

    def test_follow_velocity_is_limited_before_mqtt_command_is_created(self):
        response = self.web_client.post(
            f"/api/robots/{self.robot.id}/commands/",
            {
                "action": "move_velocity",
                "payload": {"vx": 9, "vy": -9, "yaw_rate": 9, "source": "person_follow"},
            },
            format="json",
        )
        self.assertEqual(response.status_code, 202)
        command = RemoteCommand.objects.get(id=response.data["id"])
        self.assertEqual(command.command_type, "teleop.move_velocity")
        self.assertEqual(command.payload["vx"], 0.2)
        self.assertEqual(command.payload["vy"], -0.15)
        self.assertEqual(command.payload["yaw_rate"], 0.35)

