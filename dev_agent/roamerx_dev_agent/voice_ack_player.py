from __future__ import annotations

import logging
import shlex
import subprocess
import threading

from .config import VoiceConfig

LOGGER = logging.getLogger(__name__)


class VoiceAckPlayer:
    """Minimal always-on acknowledgement player, independent of video detection."""

    def __init__(self, config: VoiceConfig) -> None:
        self.config = config
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None

    def play(self, audio_url: str) -> None:
        if not audio_url.startswith(("http://", "https://")):
            raise ValueError("invalid acknowledgement audio URL")
        with self._lock:
            if self._process and self._process.poll() is None:
                self._process.terminate()
            command = (
                f"PULSE_SERVER={shlex.quote(self.config.pulse_server)}; export PULSE_SERVER; "
                "sink=$(pactl list short sinks | awk '$2 ~ /usb-/ {print $2; exit}'); "
                "test -n \"$sink\"; export PULSE_SINK=\"$sink\" SDL_AUDIODRIVER=pulseaudio; "
                f"exec /usr/bin/ffplay -nodisp -autoexit -loglevel warning {shlex.quote(audio_url)}"
            )
            self._process = subprocess.Popen([
                "ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-p", str(self.config.port),
                "-i", self.config.identity_file, f"{self.config.user}@{self.config.host}", command,
            ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        threading.Thread(target=self._wait, daemon=True, name="dev-voice-ack").start()

    def _wait(self) -> None:
        with self._lock:
            process = self._process
        if not process:
            return
        stderr = process.communicate(timeout=90)[1]
        if process.returncode:
            LOGGER.warning("voice acknowledgement playback failed: %s", stderr[-500:])
        else:
            LOGGER.info("voice acknowledgement playback finished")
