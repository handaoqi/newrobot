from __future__ import annotations

import subprocess
import shlex

from .config import AudioControlConfig
from .protocol import ProtocolError


class AudioControlAdapter:
    def __init__(self, config: AudioControlConfig) -> None:
        self.config = config

    def set_volume(self, target: str, volume: int) -> dict:
        volume = max(0, min(100, int(volume)))
        if target == "speaker_3588":
            configured_sink = shlex.quote(self.config.remote_sink)
            command = (
                f"configured={configured_sink}; "
                "if pactl get-sink-volume \"$configured\" >/dev/null 2>&1; then sink=$configured; "
                "else sink=$(pactl list short sinks | awk '$2 ~ /usb-/ {print $2; exit}'); fi; "
                "test -n \"$sink\" && "
                "pactl set-sink-mute \"$sink\" 0 && "
                f"pactl set-sink-volume \"$sink\" {volume}%"
            )
            completed = subprocess.run(
                ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", self.config.remote_host, command],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=self.config.command_timeout_seconds,
            )
        elif target == "speaker_nx":
            command = (
                "card=$(awk '/USB-Audio/{gsub(/[\\[\\]]/,\"\",$2); print $2; exit}' /proc/asound/cards); "
                "test -n \"$card\"; controls=$(amixer -c \"$card\" scontrols); "
                "control=$(printf '%s\\n' \"$controls\" | sed -n \"s/^Simple mixer control '\\([^']*\\)'.*/\\1/p\" "
                "| grep -E '^(PCM|Playback Feature Unit)$' | head -n1); "
                f"test -n \"$control\" && amixer -c \"$card\" sset \"$control\" {volume}% unmute"
            )
            completed = subprocess.run(
                ["bash", "-lc", command],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=self.config.command_timeout_seconds,
            )
        else:
            raise ProtocolError("INVALID_AUDIO_TARGET", target)
        result = {
            "target": target,
            "volume": volume,
            "returncode": completed.returncode,
            "stdout": completed.stdout[-2000:],
            "stderr": completed.stderr[-2000:],
        }
        if completed.returncode != 0:
            raise ProtocolError("AUDIO_CONTROL_FAILED", result["stderr"] or result["stdout"])
        return result
