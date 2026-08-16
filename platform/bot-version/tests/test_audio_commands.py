import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from bike_bot.audio_commands import AudioCommandClient


class AudioCommandClientTests(unittest.TestCase):
    def make_client(self):
        client = AudioCommandClient.__new__(AudioCommandClient)
        client.config = SimpleNamespace(
            robot=SimpleNamespace(code="AUDIO-TEST-01"),
            telemetry=SimpleNamespace(device_key="device-secret"),
        )
        return client

    def test_device_headers_match_backend_credential_contract(self):
        headers = self.make_client()._headers()
        self.assertEqual(headers["X-Device-Code"], "AUDIO-TEST-01")
        self.assertEqual(headers["X-Device-Id"], "AUDIO-TEST-01")
        self.assertEqual(headers["X-Device-Key"], "device-secret")

    @patch("bike_bot.audio_commands.subprocess.run")
    def test_detect_usb_audio_device_uses_stable_card_name(self, run):
        run.return_value = SimpleNamespace(
            stdout=(
                "card 0: rockchipdp0 [rockchip-dp0], device 0: output\n"
                "card 2: Device [Audio Device], device 0: USB Audio [USB Audio]\n"
            )
        )

        device = self.make_client()._detect_usb_audio_device()

        self.assertEqual(device, "hw:CARD=Device,DEV=0")

    @patch("bike_bot.audio_commands.shutil.which", return_value="/usr/bin/tool")
    @patch.object(AudioCommandClient, "_detect_usb_audio_device", return_value="")
    def test_play_audio_fails_instead_of_falling_back_when_usb_is_missing(self, _detect, _which):
        with self.assertRaisesRegex(RuntimeError, "USB audio device is not available"):
            self.make_client()._play_audio(Path("cached.mp3"))

    @patch("bike_bot.audio_commands.subprocess.getoutput", return_value="2026-07-14T12:00:00+08:00")
    def test_play_audio_command_reports_running_and_finished(self, _getoutput):
        client = self.make_client()
        client.report = Mock()
        client._download_audio = Mock(return_value=Path("cached.wav"))
        client._play_audio = Mock(return_value="ffmpeg|aplay:plughw:2,0")

        client.handle_command(
            {
                "id": 42,
                "action": "play_audio",
                "payload": {"audio_url": "https://platform.example/media/notice.wav"},
            }
        )

        self.assertEqual(client.report.call_count, 2)
        self.assertEqual(client.report.call_args_list[0].args[1], "running")
        self.assertEqual(client.report.call_args_list[1].args[1], "finished")
        self.assertEqual(client.report.call_args_list[1].args[2]["player"], "ffmpeg|aplay:plughw:2,0")


if __name__ == "__main__":
    unittest.main()
