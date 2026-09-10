import shutil
import tempfile
from datetime import timedelta
from unittest.mock import patch

from django.contrib.auth import get_user_model
from django.contrib.auth.hashers import make_password
from django.core.files.uploadedfile import SimpleUploadedFile
from django.test import override_settings
from django.utils import timezone
from rest_framework.test import APIClient, APITestCase

from .models import RecordedAudio, RemoteCommand, Robot, RobotCommand, RobotCredential, SpeechCategory, SpeechTemplate


class AudioCommandChainTests(APITestCase):
    def setUp(self):
        self.robot = Robot.objects.create(
            code="AUDIO-TEST-01",
            name="音频测试机器人",
            location="测试区",
            area="测试区",
        )
        self.user = get_user_model().objects.create_user(username="audio-operator", password="secret")
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

    def test_preset_audio_command_can_be_polled_and_reported(self):
        response = self.web_client.post(
            f"/api/robots/{self.robot.id}/commands/",
            {
                "action": "play_audio",
                "payload": {
                    "audio_url": "https://platform.example/audio/notice.wav",
                    "audio_name": "通知",
                },
            },
            format="json",
        )
        self.assertEqual(response.status_code, 201)
        command_id = response.data["id"]
        self.assertEqual(response.data["status"], "queued")

        response = self.device_client.get(
            "/api/device/commands/poll/",
            {"robot_code": self.robot.code},
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["id"], command_id)
        self.assertEqual(response.data["action"], "play_audio")
        self.assertEqual(response.data["status"], "sent")

        response = self.device_client.post(
            f"/api/device/commands/{command_id}/report/",
            {"status": "running", "response_payload": {"player": "pending"}},
            format="json",
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["status"], "running")

        response = self.device_client.post(
            f"/api/device/commands/{command_id}/report/",
            {"status": "finished", "response_payload": {"player": "ffmpeg|aplay:plughw:2,0"}},
            format="json",
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["status"], "finished")
        self.assertEqual(RobotCommand.objects.get(id=command_id).status, "finished")

        response = self.device_client.get(
            "/api/device/commands/poll/",
            {"robot_code": self.robot.code},
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 204)

    def test_invalid_audio_url_is_rejected(self):
        response = self.web_client.post(
            f"/api/robots/{self.robot.id}/commands/",
            {"action": "play_audio", "payload": {"audio_url": "/audio/local-only.wav"}},
            format="json",
        )
        self.assertEqual(response.status_code, 400)
        self.assertEqual(RobotCommand.objects.count(), 0)

    def test_operator_can_poll_remote_audio_volume_result(self):
        command = RemoteCommand.objects.create(
            robot=self.robot,
            command_type="audio.volume",
            payload={"target": "speaker_nx", "volume": 64},
            status="succeeded",
            expires_at=timezone.now() + timedelta(seconds=10),
            result_payload={"target": "speaker_nx", "volume": 64},
        )

        response = self.web_client.get(f"/api/robots/{self.robot.id}/commands/{command.id}/")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["status"], "succeeded")
        self.assertEqual(response.data["result_payload"]["volume"], 64)

    def test_operator_can_poll_audio_playback_result(self):
        command = RobotCommand.objects.create(
            robot=self.robot,
            action="play_audio",
            status="finished",
            payload={"audio_url": "https://platform.example/audio/notice.wav"},
            response_payload={
                "playback_mode": "single_nx",
                "active_outputs": ["nx"],
                "unavailable_outputs": {"3588": "USB audio sink is unavailable"},
            },
        )

        response = self.web_client.get(f"/api/robots/{self.robot.id}/commands/{command.id}/")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["status"], "finished")
        self.assertEqual(response.data["response_payload"]["playback_mode"], "single_nx")

    def test_audio_playback_result_is_scoped_to_robot(self):
        other_robot = Robot.objects.create(code="AUDIO-TEST-02", name="另一台机器人")
        command = RobotCommand.objects.create(
            robot=other_robot,
            action="play_audio",
            payload={"audio_url": "https://platform.example/audio/notice.wav"},
        )

        response = self.web_client.get(f"/api/robots/{self.robot.id}/commands/{command.id}/")

        self.assertEqual(response.status_code, 404)

    def test_poll_returns_newest_audio_and_supersedes_older_queue(self):
        older = RobotCommand.objects.create(
            robot=self.robot,
            action="play_audio",
            payload={"audio_url": "https://platform.example/audio/older.wav"},
        )
        newest = RobotCommand.objects.create(
            robot=self.robot,
            action="play_audio",
            payload={"audio_url": "https://platform.example/audio/newest.wav"},
        )
        response = self.device_client.get(
            "/api/device/commands/poll/",
            {"robot_code": self.robot.code},
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["id"], newest.id)
        older.refresh_from_db()
        self.assertEqual(older.status, "superseded")

    def test_device_can_report_audio_as_superseded(self):
        command = RobotCommand.objects.create(
            robot=self.robot,
            action="play_audio",
            payload={"audio_url": "https://platform.example/audio/old.wav"},
            status="running",
        )
        response = self.device_client.post(
            f"/api/device/commands/{command.id}/report/",
            {"status": "superseded", "error_message": "replaced by newer announcement"},
            format="json",
            **self.device_headers,
        )
        self.assertEqual(response.status_code, 200)
        command.refresh_from_db()
        self.assertEqual(command.status, "superseded")

    def test_device_poll_requires_valid_credential(self):
        response = APIClient().get(
            "/api/device/commands/poll/",
            {"robot_code": self.robot.code},
        )
        self.assertIn(response.status_code, {401, 403})

    def test_debug_fallback_only_allows_robot_without_active_credentials(self):
        robot = Robot.objects.create(
            code="AUDIO-DEV-01",
            name="开发模式机器人",
            location="测试区",
            area="测试区",
        )
        RobotCommand.objects.create(
            robot=robot,
            action="play_audio",
            payload={"audio_url": "http://localhost/audio/notice.wav"},
        )
        with override_settings(DEBUG=True):
            response = APIClient().get(
                "/api/device/commands/poll/",
                {"robot_code": robot.code},
                HTTP_X_DEVICE_CODE=robot.code,
                HTTP_X_DEVICE_ID=robot.code,
            )
        self.assertEqual(response.status_code, 200)

        with override_settings(DEBUG=False):
            response = APIClient().get(
                "/api/device/commands/poll/",
                {"robot_code": robot.code},
                HTTP_X_DEVICE_CODE=robot.code,
                HTTP_X_DEVICE_ID=robot.code,
            )
        self.assertIn(response.status_code, {401, 403})


class RecordedAudioCommandTests(APITestCase):
    def setUp(self):
        self.media_root = tempfile.mkdtemp(prefix="audio-command-tests-")
        self.override = override_settings(MEDIA_ROOT=self.media_root, PUBLIC_BASE_URL="https://platform.example")
        self.override.enable()
        self.addCleanup(self.override.disable)
        self.addCleanup(shutil.rmtree, self.media_root, True)
        self.robot = Robot.objects.create(
            code="AUDIO-UPLOAD-01",
            name="录音测试机器人",
            location="测试区",
            area="测试区",
        )
        user = get_user_model().objects.create_user(username="audio-uploader", password="secret")
        self.client.force_authenticate(user)
        self.asr_patcher = patch("monitoring.views.asr_service.process_recording", side_effect=self._complete_asr)
        self.asr_patcher.start()
        self.addCleanup(self.asr_patcher.stop)

    @staticmethod
    def _complete_asr(recording):
        recording.transcript = "请尽快驶离此区域"
        recording.asr_status = "completed"
        recording.asr_error = ""
        recording.save(update_fields=["transcript", "asr_status", "asr_error", "updated_at"])

    def test_recording_upload_creates_downloadable_audio_command(self):
        audio = SimpleUploadedFile("recording.webm", b"test-audio-content", content_type="audio/webm;codecs=opus")
        response = self.client.post(
            f"/api/robots/{self.robot.id}/commands/audio-recording/",
            {"file": audio, "audio_name": "现场录音"},
            format="multipart",
        )
        self.assertEqual(response.status_code, 201)
        self.assertTrue(response.data["audio_url"].startswith("https://platform.example/media/recorded-audio/"))
        self.assertEqual(response.data["recording"]["title"], "现场录音")
        self.assertEqual(response.data["recording"]["transcript"], "请尽快驶离此区域")
        self.assertEqual(response.data["recording"]["source_type"], "recording")
        command = RobotCommand.objects.get(id=response.data["command"]["id"])
        self.assertEqual(command.action, "play_audio")
        self.assertEqual(command.status, "queued")
        self.assertEqual(command.payload["content_type"], "audio/webm")

    def test_recording_can_be_saved_with_title_and_category_without_playing(self):
        category = SpeechCategory.objects.create(name="现场处置")
        audio = SimpleUploadedFile("recording.webm", b"saved-audio", content_type="audio/webm")
        response = self.client.post(
            f"/api/robots/{self.robot.id}/commands/audio-recording/",
            {"file": audio, "title": "南门劝导", "category": category.id, "play_now": "false"},
            format="multipart",
        )
        self.assertEqual(response.status_code, 201)
        self.assertIsNone(response.data["command"])
        recording = RecordedAudio.objects.get(id=response.data["recording"]["id"])
        self.assertEqual(recording.title, "南门劝导")
        self.assertEqual(recording.category, category)
        self.assertFalse(RobotCommand.objects.exists())

    def test_saved_recording_can_be_replayed_and_deleted(self):
        recording = RecordedAudio.objects.create(
            title="重复播放录音",
            file=SimpleUploadedFile("saved.webm", b"saved-audio", content_type="audio/webm"),
            content_type="audio/webm",
            file_size=11,
        )
        response = self.client.post(
            f"/api/robots/{self.robot.id}/commands/recorded-audio/{recording.id}/",
            {},
            format="json",
        )
        self.assertEqual(response.status_code, 201)
        command = RobotCommand.objects.get(id=response.data["id"])
        self.assertEqual(command.payload["source"], "dashboard_recording_library")
        self.assertEqual(command.payload["audio_name"], "重复播放录音")

        stored_path = recording.file.path
        response = self.client.delete(f"/api/recorded-audio/{recording.id}/")
        self.assertEqual(response.status_code, 204)
        self.assertFalse(RecordedAudio.objects.filter(id=recording.id).exists())
        self.assertFalse(__import__("os").path.exists(stored_path))


class SpeechTemplateAndTtsTests(APITestCase):
    def setUp(self):
        self.robot = Robot.objects.create(
            code="TTS-TEST-01",
            name="TTS 测试机器人",
            location="测试区",
            area="测试区",
        )
        user = get_user_model().objects.create_user(username="tts-operator", password="secret")
        self.client.force_authenticate(user)
        self.public_url_override = override_settings(PUBLIC_BASE_URL="https://platform.example")
        self.public_url_override.enable()
        self.addCleanup(self.public_url_override.disable)

    def test_speech_template_can_be_created_updated_and_deleted(self):
        response = self.client.post(
            "/api/speech-templates/",
            {"name": "临时提醒", "text": "请注意安全。"},
            format="json",
        )
        self.assertEqual(response.status_code, 201)
        template_id = response.data["id"]

        response = self.client.patch(
            f"/api/speech-templates/{template_id}/",
            {"text": "请减速慢行。"},
            format="json",
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["text"], "请减速慢行。")

        response = self.client.delete(f"/api/speech-templates/{template_id}/")
        self.assertEqual(response.status_code, 204)
        self.assertFalse(SpeechTemplate.objects.filter(id=template_id).exists())

    def test_speech_category_can_be_created_and_assigned(self):
        category_response = self.client.post(
            "/api/speech-categories/",
            {"name": "临时管制"},
            format="json",
        )
        self.assertEqual(category_response.status_code, 201)
        category_id = category_response.data["id"]

        template_response = self.client.post(
            "/api/speech-templates/",
            {"name": "施工提醒", "text": "前方施工，请绕行。", "category": category_id},
            format="json",
        )
        self.assertEqual(template_response.status_code, 201)
        self.assertEqual(template_response.data["category"], category_id)
        self.assertEqual(template_response.data["category_name"], "临时管制")

        list_response = self.client.get("/api/speech-categories/")
        category_data = next(item for item in list_response.data if item["id"] == category_id)
        self.assertEqual(category_data["template_count"], 1)

        delete_response = self.client.delete(f"/api/speech-categories/{category_id}/")
        self.assertEqual(delete_response.status_code, 204)
        self.assertFalse(SpeechCategory.objects.filter(id=category_id).exists())
        self.assertIsNone(SpeechTemplate.objects.get(name="施工提醒").category_id)

    @patch("monitoring.views.tts_service.synthesize_speech", return_value=("tts-audio/example.mp3", False))
    def test_current_text_is_synthesized_and_queued_for_robot(self, synthesize):
        text = "前方是重点路段，请勿占道停留。"
        response = self.client.post(
            f"/api/robots/{self.robot.id}/commands/tts/",
            {"text": text, "audio_name": "重点路段"},
            format="json",
        )
        self.assertEqual(response.status_code, 201)
        synthesize.assert_called_once_with(text)
        command = RobotCommand.objects.get(id=response.data["command"]["id"])
        self.assertEqual(command.status, "queued")
        self.assertEqual(command.payload["text"], text)
        self.assertEqual(command.payload["audio_url"], "https://platform.example/media/tts-audio/example.mp3")

    @patch("monitoring.views.tts_service.synthesize_speech", return_value=("tts-audio/example.mp3", True))
    def test_speech_preview_returns_cached_audio_url(self, synthesize):
        response = self.client.post(
            "/api/speech/synthesize/",
            {"text": "测试语音预览。"},
            format="json",
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.data["cache_hit"])
        self.assertEqual(response.data["audio_url"], "https://platform.example/media/tts-audio/example.mp3")
