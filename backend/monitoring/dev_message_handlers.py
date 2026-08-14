from __future__ import annotations

import base64
import json
import logging
import re
import tempfile
import uuid
import wave
from pathlib import Path

from django.db import transaction
from django.db.models import Max
from django.utils import timezone
from django.utils.dateparse import parse_datetime

from .models import DevelopmentAgentState, DevelopmentTask, DevelopmentTaskEvent, Robot
from .message_handlers import _public_media_url
from .services import asr_service, tts_service

_VOICE_WAKE_UNTIL: dict[str, object] = {}
LOGGER = logging.getLogger(__name__)
VOICE_WAKE_ALIASES = ("小太阳", "小太陽")
VOICE_WAKE_WINDOW_SECONDS = 8
LOCAL_ASR_ENGINE = "nx-sensevoice"


def _timestamp(value):
    parsed = parse_datetime(str(value or ""))
    return parsed or timezone.now()


def handle_dev_mqtt_message(topic: str, raw_payload: bytes | str | dict, publish=None) -> dict:
    if isinstance(raw_payload, dict):
        payload = raw_payload
    else:
        if isinstance(raw_payload, bytes):
            raw_payload = raw_payload.decode("utf-8")
        payload = json.loads(raw_payload)
    parts = topic.split("/")
    if len(parts) < 4 or parts[0] != "robots" or parts[2] != "dev":
        raise ValueError("invalid development topic")
    robot = Robot.objects.get(code=parts[1])
    if parts[3] == "presence":
        state, _ = DevelopmentAgentState.objects.update_or_create(
            robot=robot,
            defaults={
                "status": str(payload.get("status") or "online"),
                "agent_version": str(payload.get("agent_version") or ""),
                "codex_binary": str(payload.get("codex_binary") or ""),
                "workspaces": payload.get("workspaces") or [],
                "last_seen_at": _timestamp(payload.get("timestamp")),
            },
        )
        if payload.get("agent_restarted"):
            DevelopmentTask.objects.filter(
                robot=robot, status__in=DevelopmentTask.ACTIVE_STATES,
            ).update(
                status="failed",
                finished_at=timezone.now(),
                error_message="ROBOT_AGENT_RESTARTED: Codex process was interrupted by Agent restart",
            )
        return {"status": state.status}
    if parts[3] == "voice" and len(parts) == 5 and parts[4] == "audio":
        return _handle_voice_audio(robot, payload, publish)
    if len(parts) < 6 or parts[3] != "tasks":
        raise ValueError("invalid development task topic")
    task_id = uuid.UUID(parts[4])
    task = DevelopmentTask.objects.get(pk=task_id, robot=robot)
    if parts[5] == "events":
        return _handle_event(task, payload)
    if parts[5] == "result":
        return _handle_result(task, payload)
    raise ValueError("unsupported development topic")


def _handle_voice_audio(robot: Robot, payload: dict, publish=None) -> dict:
    transcript = _voice_transcript(payload)
    now = timezone.now()
    wake_match = re.match(
        rf"^\s*({'|'.join(re.escape(item) for item in VOICE_WAKE_ALIASES)})",
        transcript,
    )
    wake_phrase = wake_match.group(1) if wake_match else ""
    command = ""
    armed_until = _VOICE_WAKE_UNTIL.get(robot.code)
    if wake_match:
        command = transcript[wake_match.end():].strip(" ，。,.!！?？")
        _VOICE_WAKE_UNTIL[robot.code] = now + timezone.timedelta(seconds=VOICE_WAKE_WINDOW_SECONDS)
    elif armed_until and armed_until >= now:
        command = transcript
    elif armed_until:
        _VOICE_WAKE_UNTIL.pop(robot.code, None)
    if len(re.sub(r"[\s，。,.!！?？]", "", command)) < 2:
        command = ""
    if not command:
        if wake_match:
            _publish_voice_ack(robot, publish, "我在")
        return {"status": "armed" if wake_match else "ignored", "transcript": transcript}
    _VOICE_WAKE_UNTIL.pop(robot.code, None)
    if DevelopmentTask.objects.filter(robot=robot, status__in=DevelopmentTask.ACTIVE_STATES).exists():
        return {"status": "busy", "transcript": transcript}
    task = DevelopmentTask.objects.create(robot=robot, workspace="robot-main", model="gpt-5.6-terra", prompt=command)
    _publish_voice_ack(robot, publish, "收到", task_id=task.id)
    return {"status": "accepted", "task_id": str(task.id), "transcript": transcript}


def _publish_voice_ack(robot: Robot, publish, text: str, task_id=None) -> None:
    try:
        saved_path, _cache_hit = tts_service.synthesize_speech(text)
        if publish:
            publish(f"robots/{robot.code}/dev/voice/ack", {"audio_url": _public_media_url(saved_path)}, 1, False)
    except Exception:
        LOGGER.exception("voice acknowledgement TTS failed task=%s", task_id or "wake")


def _voice_transcript(payload: dict) -> str:
    """Use a successful NX-local ASR result, otherwise retain the old cloud path."""
    if payload.get("asr_engine") == LOCAL_ASR_ENGINE:
        transcript = str(payload.get("transcript") or "").strip()
        if not transcript or len(transcript) > 1000:
            raise ValueError("invalid NX local ASR transcript")
        return transcript
    try:
        pcm = base64.b64decode(str(payload.get("audio_b64") or ""), validate=True)
    except ValueError as exc:
        raise ValueError("invalid voice audio") from exc
    if not pcm or len(pcm) > 800_000:
        raise ValueError("invalid voice audio size")
    with tempfile.TemporaryDirectory(prefix="roamerx-voice-") as directory:
        audio_path = Path(directory) / "voice.wav"
        with wave.open(str(audio_path), "wb") as output:
            output.setnchannels(1); output.setsampwidth(2); output.setframerate(16000); output.writeframes(pcm)
        return asr_service.transcribe_audio(str(audio_path))


@transaction.atomic
def _handle_event(task: DevelopmentTask, payload: dict) -> dict:
    task = DevelopmentTask.objects.select_for_update().get(pk=task.pk)
    sequence = int(payload.get("sequence") or 0)
    if sequence <= 0:
        raise ValueError("development event sequence must be positive")
    event, created = DevelopmentTaskEvent.objects.get_or_create(
        task=task,
        sequence=sequence,
        defaults={
            "event_type": str(payload.get("type") or "output")[:32],
            "stream": str(payload.get("stream") or "")[:16],
            "text": str(payload.get("text") or ""),
            "payload": payload,
            "occurred_at": _timestamp(payload.get("timestamp")),
        },
    )
    incoming_status = str(payload.get("status") or "")
    codex_thread_id = str(payload.get("codex_thread_id") or "")
    if codex_thread_id and codex_thread_id != task.codex_thread_id:
        task.codex_thread_id = codex_thread_id
        task.save(update_fields=["codex_thread_id", "updated_at"])
    if incoming_status == "running" and task.status in {"created", "published"}:
        task.status = "running"
        task.started_at = task.started_at or event.occurred_at
        task.save(update_fields=["status", "started_at", "updated_at"])
    elif incoming_status == "cancelling" and task.status not in DevelopmentTask.TERMINAL_STATES:
        task.status = "cancelling"
        task.save(update_fields=["status", "updated_at"])
    return {"created": created, "sequence": sequence}


@transaction.atomic
def _handle_result(task: DevelopmentTask, payload: dict) -> dict:
    task = DevelopmentTask.objects.select_for_update().get(pk=task.pk)
    status = str(payload.get("status") or "failed")
    if status not in DevelopmentTask.TERMINAL_STATES:
        status = "failed"
    task.status = status
    task.finished_at = _timestamp(payload.get("timestamp"))
    task.exit_code = payload.get("exit_code")
    task.error_message = str(payload.get("error") or "")
    task.last_message = str(payload.get("last_message") or "")
    task.codex_thread_id = str(payload.get("codex_thread_id") or task.codex_thread_id or "")
    task.save(update_fields=[
        "status", "finished_at", "exit_code", "error_message", "last_message",
        "codex_thread_id", "updated_at"
    ])
    next_sequence = (task.events.aggregate(value=Max("sequence"))["value"] or 0) + 1
    DevelopmentTaskEvent.objects.get_or_create(
        task=task,
        sequence=next_sequence,
        defaults={
            "event_type": "result",
            "text": task.error_message or task.last_message or task.get_status_display(),
            "payload": payload,
            "occurred_at": task.finished_at,
        },
    )
    return {"status": status}
