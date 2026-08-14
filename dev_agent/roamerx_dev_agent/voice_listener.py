from __future__ import annotations

import audioop
import base64
import json
import logging
import subprocess
import threading
import time
import uuid

from .config import VoiceConfig
from .local_asr_client import LocalASRClient, LocalASRError

LOGGER = logging.getLogger(__name__)


class VoiceCommandListener:
    """Capture a physical PulseAudio input and publish speech-only PCM segments."""

    _CHUNK_BYTES = 8000  # 250 ms, mono s16le @ 16 kHz
    _LOCAL_CHUNK_BYTES = 48000  # 250 ms, stereo s16le @ 48 kHz

    def __init__(self, config: VoiceConfig, publish) -> None:
        self.config = config
        self.publish = publish
        self._stop = threading.Event()
        self._rate_state = None
        self._suppress_until = 0.0
        self._suppress_lock = threading.Lock()
        self._local_asr = LocalASRClient(config) if config.local_asr_enabled else None

    def start(self) -> None:
        if self.config.enabled:
            threading.Thread(target=self._run, daemon=True, name="dev-voice-listener").start()

    def suppress(self, seconds: float) -> None:
        """Avoid treating the robot's own acknowledgement as a new command."""
        with self._suppress_lock:
            self._suppress_until = max(self._suppress_until, time.monotonic() + max(0.0, seconds))

    def _is_suppressed(self) -> bool:
        with self._suppress_lock:
            return time.monotonic() < self._suppress_until

    def _command(self) -> list[str]:
        if self.config.capture_mode == "local_alsa":
            return ["arecord", "-D", self.config.alsa_device, "-t", "raw", "-f", "S16_LE", "-c", "2", "-r", "48000"]
        remote = (
            f"PULSE_SERVER={self.config.pulse_server} exec parec --raw "
            f"--device={self.config.source} --format=s16le --rate=16000 --channels=1"
        )
        return ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-p", str(self.config.port),
                "-i", self.config.identity_file, f"{self.config.user}@{self.config.host}", remote]

    def _run(self) -> None:
        while not self._stop.is_set():
            process = None
            try:
                process = subprocess.Popen(self._command(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                assert process.stdout is not None
                speech, silence, chunks = bytearray(), 0, 0
                while not self._stop.is_set() and process.poll() is None:
                    raw_chunk = process.stdout.read(
                        self._LOCAL_CHUNK_BYTES if self.config.capture_mode == "local_alsa" else self._CHUNK_BYTES
                    )
                    if not raw_chunk:
                        break
                    if self.config.capture_mode == "local_alsa":
                        mono = audioop.tomono(raw_chunk, 2, 0.5, 0.5)
                        chunk, self._rate_state = audioop.ratecv(mono, 2, 1, 48000, 16000, self._rate_state)
                    else:
                        chunk = raw_chunk
                    if len(chunk) < self._CHUNK_BYTES - 32:
                        continue
                    if self._is_suppressed():
                        speech, silence, chunks = bytearray(), 0, 0
                        continue
                    loud = audioop.rms(chunk, 2) >= self.config.rms_threshold
                    if loud:
                        speech.extend(chunk); chunks += 1; silence = 0
                    elif speech:
                        speech.extend(chunk); chunks += 1; silence += 1
                    if speech and (silence >= self.config.silence_chunks or chunks >= self.config.max_chunks):
                        if chunks >= self.config.min_chunks:
                            self._publish(bytes(speech))
                        speech, silence, chunks = bytearray(), 0, 0
            except Exception:
                LOGGER.exception("voice capture failed; retrying")
            finally:
                if process and process.poll() is None:
                    process.terminate()
            self._stop.wait(2)

    def _publish(self, pcm: bytes) -> None:
        payload = {
            "voice_id": str(uuid.uuid4()), "format": "s16le", "sample_rate": 16000,
            "channels": 1, "audio_b64": base64.b64encode(pcm).decode("ascii"),
        }
        if self._local_asr:
            try:
                transcript = self._local_asr.transcribe(pcm)
            except LocalASRError as exc:
                if not self.config.local_asr_fallback_to_cloud:
                    LOGGER.warning("discarded voice segment because NX local ASR failed: %s", exc)
                    return
                LOGGER.warning("NX local ASR failed; using cloud ASR fallback: %s", exc)
            else:
                if not transcript:
                    LOGGER.info("discarded non-speech voice segment after NX local ASR")
                    return
                payload["transcript"] = transcript
                payload["asr_engine"] = "nx-sensevoice"
        self.publish(payload)
        LOGGER.info("published voice segment id=%s duration=%.2fs", payload["voice_id"], len(pcm) / 32000)
