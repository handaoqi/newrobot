from __future__ import annotations

import io
import json
import logging
import uuid
import wave
from urllib.error import URLError
from urllib.request import ProxyHandler, Request, build_opener

from .config import VoiceConfig

LOGGER = logging.getLogger(__name__)
_LOCAL_ONLY_OPENER = build_opener(ProxyHandler({}))


class LocalASRError(RuntimeError):
    """The locally hosted ASR endpoint could not return a usable response."""


class LocalASRClient:
    """Small dependency-free client for the NX-local FunASR endpoint."""

    def __init__(self, config: VoiceConfig) -> None:
        self.url = config.local_asr_url
        self.model = config.local_asr_model
        self.language = config.local_asr_language
        self.timeout_seconds = config.local_asr_timeout_seconds

    def transcribe(self, pcm: bytes, *, channels: int = 1, sample_rate: int = 16000) -> str:
        if not pcm:
            return ""
        if channels not in {1, 2}:
            raise LocalASRError(f"unsupported local ASR channel count: {channels}")
        if sample_rate != 16000:
            raise LocalASRError(f"unsupported local ASR sample rate: {sample_rate}")
        boundary = f"----roamerx-{uuid.uuid4().hex}"
        body = self._multipart_body(boundary, pcm, channels=channels, sample_rate=sample_rate)
        request = Request(
            self.url,
            data=body,
            headers={"Content-Type": f"multipart/form-data; boundary={boundary}"},
            method="POST",
        )
        try:
            # The dev-agent service has an outbound HTTP proxy. Never send a
            # loopback ASR request through it: proxies commonly reject localhost
            # targets, and the audio must not leave the NX board in this path.
            with _LOCAL_ONLY_OPENER.open(request, timeout=self.timeout_seconds) as response:
                raw_response = response.read()
        except (OSError, URLError) as exc:
            raise LocalASRError(f"NX local ASR unavailable: {exc}") from exc
        try:
            payload = json.loads(raw_response.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise LocalASRError("NX local ASR returned invalid JSON") from exc
        if not isinstance(payload, dict):
            raise LocalASRError("NX local ASR returned an invalid payload")
        transcript = str(payload.get("text") or "").strip()
        if len(transcript) > 1000:
            raise LocalASRError("NX local ASR transcript is unexpectedly long")
        return transcript

    def _multipart_body(self, boundary: str, pcm: bytes, *, channels: int, sample_rate: int) -> bytes:
        audio = io.BytesIO()
        with wave.open(audio, "wb") as output:
            output.setnchannels(channels)
            output.setsampwidth(2)
            output.setframerate(sample_rate)
            output.writeframes(pcm)
        separator = f"--{boundary}\r\n".encode("ascii")
        parts = [
            separator,
            b'Content-Disposition: form-data; name="model"\r\n\r\n',
            self.model.encode("utf-8"),
            b"\r\n",
            separator,
            b'Content-Disposition: form-data; name="language"\r\n\r\n',
            self.language.encode("utf-8"),
            b"\r\n",
            separator,
            b'Content-Disposition: form-data; name="file"; filename="voice.wav"\r\n',
            b"Content-Type: audio/wav\r\n\r\n",
            audio.getvalue(),
            b"\r\n",
            f"--{boundary}--\r\n".encode("ascii"),
        ]
        return b"".join(parts)
