from datetime import datetime
from unittest.mock import patch

from django.test import TestCase
from django.utils import timezone
from rest_framework.test import APIClient

from .models import (
    AlertSkillBinding,
    InspectionEvent,
    Robot,
    RobotCommand,
    RobotStatusLatest,
    SpeechTemplate,
)
from .views import ensure_demo_seed


class MonitoringApiTests(TestCase):
    def setUp(self):
        ensure_demo_seed(force=True)
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

    def test_alert_skill_binding_can_be_updated(self):
        self.authenticate()
        template = SpeechTemplate.objects.create(name="自行车测试播报", text="自行车测试文案")
        response = self.client.patch(
            "/api/alert-skills/bicycle_alert/",
            {"template_id": template.id, "enabled": True},
            format="json",
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["template"]["id"], template.id)
        binding = AlertSkillBinding.objects.get(skill_key="bicycle_alert")
        self.assertEqual(binding.template_id, template.id)

    @patch("monitoring.views.tts_service.synthesize_speech", return_value=("tts-audio/alert-preview.mp3", True))
    def test_disabled_alert_skill_can_be_previewed_on_both_speakers(self, synthesize_speech):
        self.authenticate()
        robot = Robot.objects.first()
        robot.connection_status = "online"
        robot.save(update_fields=["connection_status", "updated_at"])
        template = SpeechTemplate.objects.create(name="试播模板", text="双音响试播内容")
        binding = AlertSkillBinding.objects.get(skill_key="avoidance")
        binding.template = template
        binding.enabled = False
        binding.save(update_fields=["template", "enabled", "updated_at"])

        response = self.client.post(
            "/api/alert-skills/avoidance/preview/",
            {"robot_id": robot.id},
            format="json",
        )

        self.assertEqual(response.status_code, 201)
        command = RobotCommand.objects.get(id=response.data["command"]["id"])
        self.assertEqual(command.payload["source"], "dashboard_alert_skill_preview")
        self.assertEqual(command.payload["alert_skill"], "avoidance")
        self.assertEqual(command.payload["audio_name"], "试播模板")
        self.assertTrue(command.payload["dual_output"])
        self.assertTrue(command.payload["allow_single_fallback"])
        self.assertTrue(command.payload["preview"])
        synthesize_speech.assert_called_once_with("双音响试播内容")

    def test_alert_skill_preview_rejects_offline_robot(self):
        self.authenticate()
        robot = Robot.objects.first()
        robot.connection_status = "offline"
        robot.save(update_fields=["connection_status", "updated_at"])

        response = self.client.post(
            "/api/alert-skills/bicycle_alert/preview/",
            {"robot_id": robot.id},
            format="json",
        )

        self.assertEqual(response.status_code, 409)
        self.assertEqual(RobotCommand.objects.filter(payload__preview=True).count(), 0)

    def test_dashboard_analytics(self):
        self.authenticate()
        response = self.client.get("/api/dashboard/analytics/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(response.data["trends"]), 4)
        self.assertEqual(len(response.data["trends"][0]["series"]), 7)
        self.assertEqual(response.data["cards"][0]["value"], "0 次")
        self.assertEqual(response.data["cards"][1]["value"], "0 次")
        self.assertEqual(response.data["cards"][3]["value"], "无相关数据")
        self.assertEqual(response.data["trends"][2]["summary"]["latest"], 0)
        self.assertEqual(response.data["trends"][3]["summary"]["latest"], 0)
        self.assertEqual(response.data["trends"][3]["summary"]["total"], 0)

    def test_robot_list_uses_seeded_primary_device_only(self):
        self.authenticate()
        response = self.client.get("/api/robots/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(response.data), 1)
        robot_codes = {robot["code"] for robot in response.data}
        self.assertIn("ZSL-1A-07", robot_codes)

    def test_robot_list_prefers_latest_realtime_battery_over_demo_seed(self):
        robot = Robot.objects.get(code="ZSL-1A-07")
        self.assertEqual(robot.battery_level, 78)
        RobotStatusLatest.objects.create(
            robot=robot,
            state_version=1,
            sampled_at=timezone.now(),
            power_available=True,
            battery_percent=14,
        )

        response = self.client.get("/api/robots/")

        self.assertEqual(response.status_code, 200)
        payload = next(item for item in response.data if item["code"] == robot.code)
        self.assertEqual(payload["battery_level"], 14)
        robot.refresh_from_db()
        self.assertEqual(robot.battery_level, 78)

    def test_robot_list_falls_back_when_realtime_power_is_unavailable(self):
        robot = Robot.objects.get(code="ZSL-1A-07")
        RobotStatusLatest.objects.create(
            robot=robot,
            state_version=1,
            sampled_at=timezone.now(),
            power_available=False,
            battery_percent=14,
        )

        response = self.client.get("/api/robots/")

        payload = next(item for item in response.data if item["code"] == robot.code)
        self.assertEqual(payload["battery_level"], 78)

    @patch("monitoring.views.tts_service.synthesize_speech", return_value=("tts-audio/bicycle-reminder.mp3", True))
    def test_telemetry_ingest(self, synthesize_speech):
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
        self.assertEqual(len(response.data["audio_commands_queued"]), 1)
        command = RobotCommand.objects.get(id=response.data["audio_commands_queued"][0])
        self.assertEqual(command.action, "play_audio")
        self.assertEqual(command.payload["source"], "vision_bicycle_auto")
        self.assertEqual(command.payload["audio_name"], "驶离提醒")
        self.assertEqual(command.payload["alert_skill"], "bicycle_alert")
        self.assertTrue(command.payload["dual_output"])
        self.assertIn("请尽快驶离指定区域", command.payload["text"])
        synthesize_speech.assert_called_once()

        second_response = self.client.post(
            "/api/telemetry/ingest/",
            {
                "sequence_id": "test-0002",
                "robot_code": robot.code,
                "robot_name": robot.name,
                "reported_at": "2026-05-02T14:35:19+08:00",
                "position": {"name": "太阳宫公园南入口", "latitude": 39.983521, "longitude": 116.447153},
                "motion": {"speed": 1.26, "heading": 83.5},
                "power": {"battery_level": 78, "charging": False},
                "network": {"signal_strength": 92, "network_type": "5G"},
                "runtime": {"mode": "auto", "status": "online"},
                "detections": [
                    {
                        "type": "vehicle_illegal_parking",
                        "label": "自行车违停",
                        "object_class": "bicycle",
                        "confidence": 0.93,
                        "risk_level": "medium",
                        "event_time": "2026-05-02T14:35:17+08:00",
                    }
                ],
            },
            format="json",
        )
        self.assertEqual(second_response.status_code, 201)
        self.assertEqual(second_response.data["audio_commands_queued"], [])
        self.assertEqual(RobotCommand.objects.filter(payload__source="vision_bicycle_auto").count(), 1)
        synthesize_speech.assert_called_once()

    def test_telemetry_ingest_does_not_register_non_bicycle_detection(self):
        self.authenticate()
        robot = Robot.objects.first()
        before_events = InspectionEvent.objects.count()
        before_alerts = robot.today_alerts

        response = self.client.post(
            "/api/telemetry/ingest/",
            {
                "sequence_id": "test-non-bicycle-0001",
                "robot_code": robot.code,
                "robot_name": robot.name,
                "reported_at": "2026-05-02T14:36:18+08:00",
                "position": {"name": "太阳宫公园南入口"},
                "motion": {"speed": 0, "heading": 0},
                "power": {"battery_level": 78, "charging": False},
                "network": {"signal_strength": 92, "network_type": "5G"},
                "runtime": {"mode": "auto", "status": "online"},
                "detections": [
                    {
                        "type": "fire_detected",
                        "label": "烟火识别",
                        "object_class": "fire",
                        "confidence": 0.96,
                        "risk_level": "high",
                    }
                ],
            },
            format="json",
        )

        self.assertEqual(response.status_code, 201)
        self.assertEqual(InspectionEvent.objects.count(), before_events)
        self.assertEqual(response.data["audio_commands_queued"], [])
        robot.refresh_from_db()
        self.assertEqual(robot.today_alerts, before_alerts)

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
