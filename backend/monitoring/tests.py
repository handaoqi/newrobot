from django.test import TestCase
from rest_framework.test import APIClient

from .models import InspectionEvent, Robot
from .views import ensure_demo_seed


class MonitoringApiTests(TestCase):
    def setUp(self):
        ensure_demo_seed()
        self.client = APIClient()

    def authenticate(self):
        response = self.client.post(
            "/api/auth/login/",
            {"username": "operator", "password": "admin123456"},
            format="json",
        )
        self.assertEqual(response.status_code, 200)
        self.client.credentials(HTTP_AUTHORIZATION=f"Token {response.data['token']}")

    def test_login_and_overview(self):
        self.authenticate()
        response = self.client.get("/api/dashboard/overview/")
        self.assertEqual(response.status_code, 200)
        self.assertIn("summary", response.data)

    def test_dashboard_analytics(self):
        self.authenticate()
        response = self.client.get("/api/dashboard/analytics/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(response.data["trends"]), 4)
        self.assertEqual(len(response.data["trends"][0]["series"]), 7)

    def test_telemetry_ingest(self):
        robot = Robot.objects.first()
        before_count = InspectionEvent.objects.count()
        response = self.client.post(
            "/api/telemetry/ingest/",
            {
                "sequence_id": "test-0001",
                "robot_code": robot.code,
                "robot_name": robot.name,
                "reported_at": "2026-05-02T14:35:18+08:00",
                "position": {
                    "name": "太阳宫公园南入口",
                    "latitude": 39.983521,
                    "longitude": 116.447153,
                },
                "motion": {
                    "speed": 1.26,
                    "heading": 83.5,
                },
                "power": {
                    "battery_level": 78,
                    "charging": False,
                },
                "network": {
                    "signal_strength": 92,
                    "network_type": "5G",
                },
                "runtime": {
                    "mode": "auto",
                    "status": "online",
                },
                "detections": [
                    {
                        "type": "vehicle_illegal_parking",
                        "label": "自行车违停",
                        "confidence": 0.92,
                        "risk_level": "medium",
                        "snapshot_url": "https://example.com/demo.jpg",
                        "event_time": "2026-05-02T14:35:16+08:00",
                    }
                ],
            },
            format="json",
        )
        self.assertEqual(response.status_code, 201)
        self.assertEqual(InspectionEvent.objects.count(), before_count + 1)

# Create your tests here.
