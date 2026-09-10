from __future__ import annotations

import logging
import fcntl
import os
import shlex
import subprocess
import threading
from pathlib import Path

from .config import VoiceConfig

LOGGER = logging.getLogger(__name__)

NX_AUDIO_LOCK_PATH = Path("/tmp/roamerx-nx-speaker.lock")
VOICE_ACK_PID_PATH = Path("/tmp/roamerx-dev-voice-ack.pid")


class VoiceAckPlayer:
    """Minimal always-on acknowledgement player, independent of video detection."""

    def __init__(self, config: VoiceConfig) -> None:
        self.config = config
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._audio_lease = None

    def play(self, audio_url: str) -> None:
        if not audio_url.startswith(("http://", "https://")):
            raise ValueError("invalid acknowledgement audio URL")
        with self._lock:
            if self._process and self._process.poll() is None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait(timeout=2)
            self._release_audio_lease()
            lease = None
            if self.config.capture_mode == "local_alsa":
                lease = NX_AUDIO_LOCK_PATH.open("a+")
                try:
                    fcntl.flock(lease.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    lease.close()
                    LOGGER.info("voice acknowledgement skipped because alert audio owns the NX speaker")
                    return
            command, options = self._command(audio_url)
            try:
                process = subprocess.Popen(command, **options)
            except Exception:
                if lease is not None:
                    fcntl.flock(lease.fileno(), fcntl.LOCK_UN)
                    lease.close()
                raise
            self._process = process
            self._audio_lease = lease
            if lease is not None:
                VOICE_ACK_PID_PATH.write_text(str(process.pid), encoding="utf-8")
        threading.Thread(target=self._wait, args=(process,), daemon=True, name="dev-voice-ack").start()

    def _command(self, audio_url: str) -> tuple[list[str], dict]:
        options = {"stdout": subprocess.DEVNULL, "stderr": subprocess.PIPE, "text": True}
        if self.config.capture_mode == "local_alsa":
            # ``alsa_device`` may be a dsnoop capture endpoint.  It cannot
            # play sound; acknowledgement playback needs the USB speaker PCM.
            device = self.config.playback_device
            if device.startswith("hw:"):
                device = f"plughw:{device.removeprefix('hw:')}"
            environment = os.environ.copy()
            environment.update({"SDL_AUDIODRIVER": "alsa", "AUDIODEV": device})
            options["env"] = environment
            return ["/usr/bin/ffplay", "-nodisp", "-autoexit", "-loglevel", "warning", audio_url], options
        command = (
            f"PULSE_SERVER={shlex.quote(self.config.pulse_server)}; export PULSE_SERVER; "
            "sink=$(pactl list short sinks | awk '$2 ~ /usb-/ {print $2; exit}'); "
            "test -n \"$sink\"; export PULSE_SINK=\"$sink\" SDL_AUDIODRIVER=pulseaudio; "
            f"exec /usr/bin/ffplay -nodisp -autoexit -loglevel warning {shlex.quote(audio_url)}"
        )
        return [
            "ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-p", str(self.config.port),
            "-i", self.config.identity_file, f"{self.config.user}@{self.config.host}", command,
        ], options

    def _wait(self, process: subprocess.Popen) -> None:
        try:
            stderr = process.communicate(timeout=60)[1]
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                stderr = process.communicate(timeout=10)[1]
            except subprocess.TimeoutExpired:
                process.kill()
                stderr = process.communicate()[1]
            LOGGER.warning("voice acknowledgement playback exceeded 60 seconds: %s", stderr[-500:])
        else:
            if process.returncode:
                LOGGER.warning("voice acknowledgement playback failed: %s", stderr[-500:])
            else:
                LOGGER.info("voice acknowledgement playback finished")
        finally:
            with self._lock:
                if self._process is process:
                    self._process = None
                    self._release_audio_lease()

    def _release_audio_lease(self) -> None:
        lease = self._audio_lease
        self._audio_lease = None
        if lease is not None:
            try:
                fcntl.flock(lease.fileno(), fcntl.LOCK_UN)
            finally:
                lease.close()
        try:
            VOICE_ACK_PID_PATH.unlink(missing_ok=True)
        except OSError:
            LOGGER.warning("failed to remove voice acknowledgement pid file", exc_info=True)
