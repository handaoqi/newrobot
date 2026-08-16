from django.contrib.auth import get_user_model
from django.utils import timezone
from rest_framework.test import APITestCase, APIClient

from .models import RemoteCommand, Robot


class McpTeleopApiTests(APITestCase):
    def setUp(self):
        self.robot = Robot.objects.create(
            code="MCP-TEST-01", name="MCP 测试机器人", location="测试区", area="测试区",
            connection_status="online", last_seen_at=timezone.now(),
        )
        self.client = APIClient()
        self.client.force_authenticate(get_user_model().objects.create_user("mcp-user", password="secret"))

    def test_direction_creates_same_remote_command_path(self):
        response = self.client.post(
            f"/api/mcp/robots/{self.robot.id}/direction/",
            {"direction": "forward", "command": {"vx": 0.4}}, format="json",
        )
        self.assertEqual(response.status_code, 202)
        command = RemoteCommand.objects.get(id=response.data["id"])
        self.assertEqual(command.command_type, "teleop.move_forward")
        self.assertEqual(command.payload["vx"], 0.4)
        self.assertEqual(command.payload["source"], "mcp")

    def test_skill_creates_async_remote_command(self):
        response = self.client.post(
            f"/api/mcp/robots/{self.robot.id}/skill/",
            {"preset": "prone_forward_5s", "description": "匍匐前进五秒"}, format="json",
        )
        self.assertEqual(response.status_code, 202)
        command = RemoteCommand.objects.get(id=response.data["id"])
        self.assertEqual(command.command_type, "teleop.skill")
        self.assertEqual(command.payload["preset"], "prone_forward_5s")
        self.assertEqual(command.expires_at - command.issued_at >= timezone.timedelta(seconds=179), True)

    def test_invalid_direction_is_rejected(self):
        response = self.client.post(
            f"/api/mcp/robots/{self.robot.id}/direction/", {"direction": "fly"}, format="json"
        )
        self.assertEqual(response.status_code, 400)
