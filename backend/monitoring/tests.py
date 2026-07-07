from datetime import datetime

from django.test import TestCase
from django.utils import timezone
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
        self.assertEqual(response.data["cards"][0]["value"], "13 次")
        self.assertEqual(response.data["cards"][1]["value"], "41 次")
        self.assertEqual(response.data["cards"][3]["value"], "30 分钟")
        self.assertEqual(response.data["trends"][2]["summary"]["latest"], 30)
        self.assertEqual(response.data["trends"][3]["summary"]["latest"], 1.3)
        self.assertEqual(response.data["trends"][3]["summary"]["total"], 1.3)

    def test_robot_list_uses_seeded_primary_device_only(self):
        self.authenticate()
        response = self.client.get("/api/robots/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(response.data), 1)
        robot_codes = {robot["code"] for robot in response.data}
        self.assertIn("ZSL-1A-07", robot_codes)

    def test_telemetry_ingest(self):
        self.authenticate()
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

    def test_event_list_pagination_and_status_filter(self):
        self.authenticate()
        robot = Robot.objects.first()
        for index in range(12):
            InspectionEvent.objects.create(
                robot=robot,
                title=f"测试事件 {index}",
                event_type="vehicle_illegal_parking",
                location=robot.location,
                status="pending" if index < 11 else "resolved",
            )

        response = self.client.get("/api/events/?status=pending&search=测试事件&page=1&page_size=5")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["count"], 11)
        self.assertEqual(len(response.data["results"]), 5)
        self.assertTrue(response.data["has_next"])

        second_page = self.client.get("/api/events/?status=pending&search=测试事件&page=3&page_size=5")
        self.assertEqual(second_page.status_code, 200)
        self.assertEqual(len(second_page.data["results"]), 1)
        self.assertFalse(second_page.data["has_next"])

    def test_event_list_filters_by_detected_time_range(self):
        self.authenticate()
        robot = Robot.objects.first()
        current_timezone = timezone.get_current_timezone()
        morning = datetime(2026, 5, 2, 8, 30, tzinfo=current_timezone)
        noon = datetime(2026, 5, 2, 12, 15, tzinfo=current_timezone)
        evening = datetime(2026, 5, 2, 18, 45, tzinfo=current_timezone)
        InspectionEvent.objects.create(
            robot=robot,
            title="早间事件",
            event_type="vehicle_illegal_parking",
            location=robot.location,
            detected_at=morning,
        )
        matched_event = InspectionEvent.objects.create(
            robot=robot,
            title="午间事件",
            event_type="vehicle_illegal_parking",
            location=robot.location,
            detected_at=noon,
        )
        InspectionEvent.objects.create(
            robot=robot,
            title="晚间事件",
            event_type="vehicle_illegal_parking",
            location=robot.location,
            detected_at=evening,
        )

        response = self.client.get(
            "/api/events/?detected_from=2026-05-02T12:00:00%2B08:00&detected_to=2026-05-02T13:00:00%2B08:00"
        )
        self.assertEqual(response.status_code, 200)
        result_ids = [event["id"] for event in response.data["results"]]
        self.assertIn(matched_event.id, result_ids)
        self.assertNotIn("早间事件", [event["title"] for event in response.data["results"]])
        self.assertNotIn("晚间事件", [event["title"] for event in response.data["results"]])

    def test_event_list_rejects_invalid_detected_time(self):
        self.authenticate()
        response = self.client.get("/api/events/?detected_from=not-a-time")
        self.assertEqual(response.status_code, 400)

    def test_event_handle_saves_archive_notes(self):
        self.authenticate()
        robot = Robot.objects.first()
        event = InspectionEvent.objects.create(
            robot=robot,
            title="自行车违停",
            event_type="vehicle_illegal_parking",
            location=robot.location,
            status="pending",
        )

        response = self.client.post(
            f"/api/events/{event.id}/handle/",
            {
                "status": "resolved",
                "handling_notes": "现场复核无新增风险，已归档。",
                "review_result": "false_alarm",
            },
            format="json",
        )
        self.assertEqual(response.status_code, 200)
        event.refresh_from_db()
        self.assertEqual(event.status, "resolved")
        self.assertEqual(event.review_result, "false_alarm")
        self.assertEqual(event.handling_notes, "现场复核无新增风险，已归档。")

# Create your tests here.
