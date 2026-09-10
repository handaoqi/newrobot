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

    def test_normal_waypoint_playback_fails_fast_when_both_outputs_are_offline(self):
        client = self.make_client()
        client._play_audio_remote.side_effect = RuntimeError("3588 sink unavailable")
        client._local_audio_endpoint.side_effect = RuntimeError("NX device unavailable")

        with self.assertRaisesRegex(
            RuntimeError,
            "all audio outputs unavailable: 3588=3588 sink unavailable; nx=NX device unavailable",
        ):
            client._play_audio(Path("waypoint.wav"), threading.Event())

        client._play_audio_local.assert_not_called()

    def test_both_outputs_offline_reports_failure_without_raising(self):
        client = self.make_client()
        client._write_waypoint_status = Mock()
        client.report = Mock()
        client._play_audio = Mock(
            side_effect=RuntimeError(
                "dual-speaker playback failed: nx=NX device unavailable; 3588=3588 sink unavailable"
            )
        )
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as audio_file:
            audio_path = Path(audio_file.name)
        client._download_audio = Mock(return_value=audio_path)
        payload = {
            "audio_url": "https://platform.example/waypoint.wav",
            "source": "patrol_waypoint_speech",
            "task_execution_id": "task-1",
            "waypoint_id": "wp-1",
            "dual_output": True,
            "blocking_fifo": True,
        }

        client.handle_command({"id": 124, "action": "play_audio", "payload": payload})

        self.assertEqual(client._write_waypoint_status.call_args_list[-1].args[2], "failed")
        self.assertEqual(client.report.call_args_list[-1].args[1], "failed")

    def test_missing_audio_url_writes_failed_waypoint_status(self):
        client = self.make_client()
        client._write_waypoint_status = Mock()
        client.report = Mock()
        payload = {
            "source": "patrol_waypoint_speech",
            "task_execution_id": "task-1",
            "waypoint_id": "wp-1",
        }

        client.handle_command({"id": 125, "action": "play_audio", "payload": payload})

        client._write_waypoint_status.assert_called_once_with(
            125,
            payload,
            "failed",
            "audio_url is required",
        )
        self.assertEqual(client.report.call_args.args[1], "failed")

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
