import logging
import subprocess
import threading
from pathlib import Path

from django.conf import settings

LOGGER = logging.getLogger(__name__)
_MODEL = None
_MODEL_LOCK = threading.Lock()


def _get_model():
    global _MODEL
    if _MODEL is not None:
        return _MODEL
    with _MODEL_LOCK:
        if _MODEL is None:
            from faster_whisper import WhisperModel

            model_path = Path(settings.ASR_MODEL_PATH)
            if not model_path.exists():
                raise RuntimeError(f"ASR 模型不存在: {model_path}")
            _MODEL = WhisperModel(str(model_path), device="cpu", compute_type="int8", cpu_threads=2)
    return _MODEL


def transcribe_audio(audio_path: str) -> str:
    if not settings.ASR_ENABLED:
        raise RuntimeError("ASR 未启用")
    segments, _info = _get_model().transcribe(
        audio_path, language=settings.ASR_LANGUAGE, task="transcribe", beam_size=5,
        vad_filter=True, condition_on_previous_text=False,
    )
    transcript = "".join(segment.text.strip() for segment in segments).strip()
    if not transcript:
        raise RuntimeError("未识别到有效语音")
    return transcript


def _convert_to_mp4(recording):
    source_path = Path(recording.file.path)
    if source_path.suffix.lower() == ".mp4":
        return source_path
    target_path = source_path.with_suffix(".mp4")
    result = subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-i",
            str(source_path),
            "-vn",
            "-c:a",
            "aac",
            "-b:a",
            "128k",
            str(target_path),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=120,
    )
    if result.returncode != 0 or not target_path.exists():
        LOGGER.warning("recording mp4 conversion failed: %s", result.stderr.strip())
        return source_path

    old_name = recording.file.name
    recording.file.name = str(Path(old_name).with_suffix(".mp4"))
    recording.content_type = "audio/mp4"
    recording.file_size = target_path.stat().st_size
    recording.save(update_fields=["file", "content_type", "file_size", "updated_at"])
    source_path.unlink(missing_ok=True)
    return target_path


def process_recording(recording):
    try:
        audio_path = _convert_to_mp4(recording)
        transcript = transcribe_audio(str(audio_path))
        recording.transcript = transcript
        recording.asr_status = "completed"
        recording.asr_error = ""
    except Exception as exc:
        LOGGER.exception("recording ASR failed recording=%s", recording.id)
        recording.asr_status = "failed"
        recording.asr_error = str(exc)[:255]
    recording.save(update_fields=["transcript", "asr_status", "asr_error", "updated_at"])
