import shutil
import tempfile
from unittest.mock import patch

from django.contrib.auth import get_user_model
from django.contrib.auth.hashers import make_password
from django.core.files.uploadedfile import SimpleUploadedFile
from django.test import override_settings
from rest_framework.test import APIClient, APITestCase

from .models import Robot, RobotCommand, RobotCredential, SpeechTemplate


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

    def test_recording_upload_creates_downloadable_audio_command(self):
        audio = SimpleUploadedFile("recording.webm", b"test-audio-content", content_type="audio/webm;codecs=opus")
        response = self.client.post(
            f"/api/robots/{self.robot.id}/commands/audio-recording/",
            {"file": audio, "audio_name": "现场录音"},
            format="multipart",
        )
        self.assertEqual(response.status_code, 201)
        self.assertTrue(response.data["audio_url"].startswith("https://platform.example/media/command-audio/"))
        command = RobotCommand.objects.get(id=response.data["command"]["id"])
        self.assertEqual(command.action, "play_audio")
        self.assertEqual(command.status, "queued")
        self.assertEqual(command.payload["content_type"], "audio/webm")


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
