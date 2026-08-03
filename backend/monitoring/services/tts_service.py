from __future__ import annotations

import asyncio
import hashlib
import tempfile
from pathlib import Path

import edge_tts
from django.conf import settings
from django.core.files import File
from django.core.files.storage import default_storage


def synthesize_speech(text: str) -> tuple[str, bool]:
    normalized = " ".join(text.split())
    voice = settings.TTS_VOICE
    rate = settings.TTS_RATE
    volume = settings.TTS_VOLUME
    digest = hashlib.sha256(f"{voice}\n{rate}\n{volume}\n{normalized}".encode("utf-8")).hexdigest()
    cache_path = f"tts-audio/{digest}.mp3"
    if default_storage.exists(cache_path):
        return cache_path, True

    with tempfile.TemporaryDirectory(prefix="roamerx-tts-") as temp_dir:
        output_path = Path(temp_dir) / "speech.mp3"
        communicate = edge_tts.Communicate(
            normalized,
            voice=voice,
            rate=rate,
            volume=volume,
            connect_timeout=10,
            receive_timeout=45,
        )
        asyncio.run(communicate.save(str(output_path)))
        with output_path.open("rb") as audio_file:
            saved_path = default_storage.save(cache_path, File(audio_file, name=f"{digest}.mp3"))
    return saved_path, False
