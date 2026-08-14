from django.contrib.auth import get_user_model
from django.utils import timezone
from rest_framework.test import APIClient, APITestCase

from .models import Robot, RobotStatusLatest


class LowBatteryPolicyTests(APITestCase):
    def setUp(self):
        self.robot = Robot.objects.create(
            code="LOW-BATTERY-01", name="低电量测试", location="测试区", area="测试区",
            connection_status="online", last_seen_at=timezone.now(), battery_level=19,
        )
        RobotStatusLatest.objects.create(
            robot=self.robot, sampled_at=timezone.now(), power_available=True, battery_percent=19,
        )
        self.client = APIClient()
        self.client.force_authenticate(get_user_model().objects.create_user("battery-policy", password="secret"))

    def test_keeps_manual_motion_available_below_twenty_percent(self):
        response = self.client.post(
            f"/api/robots/{self.robot.id}/commands/", {"action": "move_forward"}, format="json"
        )
        self.assertEqual(response.status_code, 202)

    def test_allows_charge_start_below_twenty_percent(self):
        response = self.client.post(
            f"/api/robots/{self.robot.id}/commands/", {"action": "charge_start"}, format="json"
        )
        self.assertEqual(response.status_code, 202)
