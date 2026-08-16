from __future__ import annotations

import audioop
import base64
from collections import deque
import json
import logging
from statistics import median
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
    _LOCAL_ASR_CHUNK_BYTES = 16000  # 250 ms, stereo s16le @ 16 kHz
    _EMPTY_TRANSCRIPT_PUBLISH_INTERVAL_SECONDS = 3.0

    def __init__(self, config: VoiceConfig, publish) -> None:
        self.config = config
        self.publish = publish
        self._stop = threading.Event()
        self._rate_state = None
        self._suppress_until = 0.0
        self._suppress_lock = threading.Lock()
        self._noise_rms: deque[int] = deque(maxlen=max(4, int(config.noise_floor_chunks)))
        self._local_asr = LocalASRClient(config) if config.local_asr_enabled else None
        self._last_empty_transcript_published_at = 0.0

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

    def _speech_threshold(self) -> int:
        """Return an absolute-plus-adaptive VAD threshold without retaining audio."""
        floor = float(median(self._noise_rms)) if len(self._noise_rms) >= 4 else 0.0
        adaptive = round(floor * max(1.0, float(self.config.noise_rms_multiplier)))
        return max(int(self.config.rms_threshold), adaptive)

    def _is_loud(self, rms: int) -> tuple[bool, int]:
        """Classify an idle chunk and update only its scalar noise floor."""
        # A short startup sample lets the adaptive threshold settle before a
        # fan or USB background noise can be mistaken for an utterance.
        if len(self._noise_rms) < 4:
            self._noise_rms.append(rms)
            return False, self._speech_threshold()
        threshold = self._speech_threshold()
        loud = rms >= threshold
        if not loud:
            self._noise_rms.append(rms)
        return loud, threshold

    @staticmethod
    def _open_segment(pre_roll: deque[bytes], chunk: bytes) -> tuple[bytearray, int]:
        """Start a segment with the preceding idle audio preserved."""
        return bytearray(b"".join(pre_roll) + chunk), len(pre_roll) + 1

    @staticmethod
    def _select_array_channel(stereo_pcm: bytes) -> tuple[bytes, str]:
        """Select one complete microphone-array channel for mono ASR.

        Fun-ASR-Nano is a mono acoustic model.  Selecting the stronger channel
        for the complete utterance retains the array's directional/SNR benefit
        without the comb filtering that can arise from naïvely averaging two
        spatially separated microphones.
        """
        left = audioop.tomono(stereo_pcm, 2, 1.0, 0.0)
        right = audioop.tomono(stereo_pcm, 2, 0.0, 1.0)
        left_rms = audioop.rms(left, 2)
        right_rms = audioop.rms(right, 2)
        return (left, "left") if left_rms >= right_rms else (right, "right")

    def _run(self) -> None:
        while not self._stop.is_set():
            process = None
            try:
                process = subprocess.Popen(self._command(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                assert process.stdout is not None
                speech, silence, chunks, segment_peak_rms = bytearray(), 0, 0, 0
                pre_roll: deque[bytes] = deque(maxlen=max(0, int(self.config.pre_roll_chunks)))
                while not self._stop.is_set() and process.poll() is None:
                    local_capture = self.config.capture_mode == "local_alsa"
                    raw_chunk = process.stdout.read(
                        self._LOCAL_CHUNK_BYTES if local_capture else self._CHUNK_BYTES
                    )
                    if not raw_chunk:
                        break
                    if local_capture:
                        # Preserve both microphone-array channels for MFCCA.
                        # Only the VAD measurement below is downmixed; the PCM
                        # segment passed to the ASR service remains interleaved
                        # L/R 16 kHz s16le.
                        chunk, self._rate_state = audioop.ratecv(
                            raw_chunk, 2, 2, 48000, 16000, self._rate_state
                        )
                        vad_chunk = audioop.tomono(chunk, 2, 0.5, 0.5)
                        expected_chunk_bytes = self._LOCAL_ASR_CHUNK_BYTES
                        asr_channels = 2
                    else:
                        chunk = raw_chunk
                        vad_chunk = chunk
                        expected_chunk_bytes = self._CHUNK_BYTES
                        asr_channels = 1
                    if len(chunk) < expected_chunk_bytes - 64:
                        continue
                    if self._is_suppressed():
                        speech, silence, chunks, segment_peak_rms = bytearray(), 0, 0, 0
                        pre_roll.clear()
                        continue
                    rms = audioop.rms(vad_chunk, 2)
                    if speech:
                        threshold = self._speech_threshold()
                        loud = rms >= threshold
                    else:
                        loud, threshold = self._is_loud(rms)
                    if loud:
                        if not speech:
                            speech, chunks = self._open_segment(pre_roll, chunk)
                        else:
                            speech.extend(chunk); chunks += 1
                        silence = 0
                        segment_peak_rms = max(segment_peak_rms, rms)
                    elif speech:
                        speech.extend(chunk); chunks += 1; silence += 1
                        segment_peak_rms = max(segment_peak_rms, rms)
                    else:
                        pre_roll.append(chunk)
                    if speech and (silence >= self.config.silence_chunks or chunks >= self.config.max_chunks):
                        if chunks >= self.config.min_chunks:
                            self._publish(
                                bytes(speech),
                                channels=asr_channels,
                                peak_rms=segment_peak_rms,
                                threshold=threshold,
                            )
                        speech, silence, chunks, segment_peak_rms = bytearray(), 0, 0, 0
                        pre_roll.clear()
            except Exception:
                LOGGER.exception("voice capture failed; retrying")
            finally:
                if process and process.poll() is None:
                    process.terminate()
            self._stop.wait(2)

    def _publish(
        self,
        pcm: bytes,
        *,
        channels: int = 1,
        peak_rms: int = 0,
        threshold: int = 0,
    ) -> None:
        if channels not in {1, 2}:
            raise ValueError(f"unsupported voice channel count: {channels}")
        selected_channel = "mono"
        if channels == 2 and self.config.asr_input_mode == "array_best_channel":
            pcm, selected_channel = self._select_array_channel(pcm)
            channels = 1
        duration_seconds = len(pcm) / (16000 * 2 * channels)
        payload = {
            "voice_id": str(uuid.uuid4()), "format": "s16le", "sample_rate": 16000,
            "channels": channels, "audio_b64": base64.b64encode(pcm).decode("ascii"),
        }
        if self._local_asr:
            try:
                transcript = self._local_asr.transcribe(pcm, channels=channels, sample_rate=16000)
            except LocalASRError as exc:
                if not self.config.local_asr_fallback_to_cloud:
                    LOGGER.warning("discarded voice segment because NX local ASR failed: %s", exc)
                    return
                LOGGER.warning("NX local ASR failed; using cloud ASR fallback: %s", exc)
            else:
                if not transcript:
                    # Preserve this as a text-only diagnostic event. Previously
                    # it was discarded locally, which made the cloud history
                    # look frozen whenever the microphone captured speech that
                    # SenseVoice could not transcribe. Rate-limit it because
                    # VAD false positives are still common in a noisy room.
                    now = time.monotonic()
                    if now - self._last_empty_transcript_published_at < self._EMPTY_TRANSCRIPT_PUBLISH_INTERVAL_SECONDS:
                        LOGGER.debug(
                            "rate-limited no-speech segment after NX local ASR id=%s",
                            payload["voice_id"],
                        )
                        return
                    self._last_empty_transcript_published_at = now
                    payload["transcript"] = ""
                    payload["asr_engine"] = "nx-sensevoice"
                    LOGGER.info(
                        "published no-speech segment after NX local ASR id=%s duration=%.2fs peak_rms=%s threshold=%s",
                        payload["voice_id"], duration_seconds, peak_rms, threshold,
                    )
                    self.publish(payload)
                    return
                payload["transcript"] = transcript
                payload["asr_engine"] = "nx-sensevoice"
                LOGGER.info("NX local ASR transcript id=%s text=%r", payload["voice_id"], transcript[:500])
        self.publish(payload)
        LOGGER.info(
            "published voice segment id=%s duration=%.2fs channels=%s selected_channel=%s",
            payload["voice_id"],
            duration_seconds,
            channels,
            selected_channel,
        )
