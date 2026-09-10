from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from bike_bot.audio_commands import AudioCommandClient, PlaybackOutcome  # noqa: E402


class FakeRemoteClient:
    def close(self):
        return None


class AudioFallbackTests(unittest.TestCase):
    def make_client(self) -> AudioCommandClient:
        client = AudioCommandClient.__new__(AudioCommandClient)
        client.config = SimpleNamespace(
            audio_playback=SimpleNamespace(sync_enabled=False, remote_host="3588")
        )
        client._connect_remote = Mock(return_value=FakeRemoteClient())
        client._remote_audio_endpoints = Mock(return_value=("hw:CARD=Audio,DEV=0", "usb-sink"))
        client._local_audio_endpoint = Mock(return_value="hw:CARD=Device,DEV=0")
        client._play_audio_local = Mock(return_value="local-player")
        client._play_audio_remote = Mock(return_value="remote-player")
        return client

    def test_uses_nx_when_3588_is_offline(self):
        client = self.make_client()
        client._remote_audio_endpoints.side_effect = RuntimeError("3588 sink unavailable")

        result = client._play_audio_dual(Path("alert.wav"), threading.Event(), allow_single_fallback=True)

        self.assertEqual(result.playback_mode, "single_nx")
        self.assertEqual(result.active_outputs, ("nx",))
        self.assertIn("3588", result.unavailable_outputs)
        client._play_audio_local.assert_called_once()

    def test_uses_3588_when_nx_is_offline(self):
        client = self.make_client()
        client._local_audio_endpoint.side_effect = RuntimeError("NX device unavailable")

        result = client._play_audio_dual(Path("alert.wav"), threading.Event(), allow_single_fallback=True)

        self.assertEqual(result.playback_mode, "single_3588")
        self.assertEqual(result.active_outputs, ("3588",))
        self.assertIn("nx", result.unavailable_outputs)
        client._play_audio_remote.assert_called_once()

    def test_scheduled_mode_uses_nx_when_3588_is_offline(self):
        client = self.make_client()
        client.config.audio_playback.sync_enabled = True
        client.config.audio_playback.remote_temp_dir = "/tmp"
        client._remote_audio_endpoints.side_effect = RuntimeError("3588 sink unavailable")

        result = client._play_audio_dual(Path("alert.wav"), threading.Event(), allow_single_fallback=True)

        self.assertEqual(result.playback_mode, "single_nx")
        client._play_audio_local.assert_called_once()

    def test_scheduled_mode_uses_3588_when_nx_is_offline(self):
        client = self.make_client()
        client.config.audio_playback.sync_enabled = True
        client.config.audio_playback.remote_temp_dir = "/tmp"
        client._local_audio_endpoint.side_effect = RuntimeError("NX device unavailable")

        result = client._play_audio_dual(Path("alert.wav"), threading.Event(), allow_single_fallback=True)

        self.assertEqual(result.playback_mode, "single_3588")
        client._play_audio_remote.assert_called_once()

    def test_fails_with_both_reasons_when_both_outputs_are_offline(self):
        client = self.make_client()
        client._local_audio_endpoint.side_effect = RuntimeError("NX device unavailable")
        client._remote_audio_endpoints.side_effect = RuntimeError("3588 sink unavailable")

        with self.assertRaisesRegex(RuntimeError, "nx=NX device unavailable; 3588=3588 sink unavailable"):
            client._play_audio_dual(Path("alert.wav"), threading.Event(), allow_single_fallback=True)

    def test_uses_dual_output_when_both_outputs_are_online(self):
        client = self.make_client()

        def remote_player(*_args, ready_event=None, start_event=None, **_kwargs):
            ready_event.set()
            start_event.wait(timeout=1)
            return "remote-player"

        client._play_audio_remote.side_effect = remote_player

        result = client._play_audio_dual(Path("alert.wav"), threading.Event(), allow_single_fallback=True)

        self.assertEqual(result.playback_mode, "dual")
        self.assertEqual(result.active_outputs, ("nx", "3588"))

    def test_preview_commands_allow_single_output_fallback(self):
        client = self.make_client()
        client._write_waypoint_status = Mock()
        client.report = Mock()
        client._play_audio = Mock(
            return_value=PlaybackOutcome("local-player", "single_nx", ("nx",), {"3588": "offline"})
        )
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as audio_file:
            audio_path = Path(audio_file.name)
        client._download_audio = Mock(return_value=audio_path)

        client.handle_command(
            {
                "id": 123,
                "action": "play_audio",
                "payload": {
                    "audio_url": "https://platform.example/preview.wav",
                    "preview": True,
                    "alert_skill": "avoidance",
                    "dual_output": True,
                },
            }
        )

        self.assertTrue(client._play_audio.call_args.kwargs["allow_single_fallback"])
        final_report = client.report.call_args_list[-1]
        self.assertEqual(final_report.args[1], "finished")
        self.assertEqual(final_report.args[2]["playback_mode"], "single_nx")


if __name__ == "__main__":
    unittest.main()
