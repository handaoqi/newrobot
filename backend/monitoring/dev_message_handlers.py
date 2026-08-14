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
from pypinyin import Style, lazy_pinyin

from .models import DevelopmentAgentState, DevelopmentTask, DevelopmentTaskEvent, Robot, VoiceRecognitionEvent
from .message_handlers import _public_media_url
from .services import asr_service, tts_service

_VOICE_WAKE_UNTIL: dict[str, object] = {}
LOGGER = logging.getLogger(__name__)
# Wake matching remains anchored at the start of the utterance, but is based on
# approximate Mandarin syllables instead of an exact ASR spelling.  This is
# needed because the NX ASR has returned variants such as “小菜阳” and “要太阳”
# for the spoken wake word “小太阳”.
VOICE_WAKE_ALIASES = ("小太阳", "小太陽")
VOICE_WAKE_PINYIN = ("xiao", "tai", "yang")
VOICE_WAKE_WINDOW_SECONDS = 8
LOCAL_ASR_ENGINE = "nx-sensevoice"
# A task's final agent message is normally the user-facing Chinese result.
# Preserve it in full for the speaker; the ceiling only prevents a malformed
# event from monopolising the robot's audio output indefinitely.
TASK_SUMMARY_MAX_CHARS = 1800
TASK_SUMMARY_MAX_SUPPRESS_SECONDS = 600


def _timestamp(value):
    parsed = parse_datetime(str(value or ""))
    return parsed or timezone.now()


def _syllable_similarity(expected: str, actual: str) -> int:
    if expected == actual:
        return 2
    # SenseVoice often confuses an initial while retaining the pinyin final,
    # e.g. tai/cai and xiao/yao.  Do not treat unrelated finals as a match.
    if len(expected) >= 2 and len(actual) >= 2 and expected[-2:] == actual[-2:]:
        return 1
    return 0


def _fuzzy_wake_match(transcript: str) -> tuple[int, str]:
    """Return the prefix end and ASR spelling when it sounds like 小太阳."""
    exact = re.match(
        rf"^\s*({'|'.join(re.escape(item) for item in VOICE_WAKE_ALIASES)})",
        transcript,
    )
    if exact:
        return exact.end(), exact.group(1)

    candidate_match = re.match(r"^(\s*)([\u4e00-\u9fff]{3})", transcript)
    if not candidate_match:
        return 0, ""
    candidate = candidate_match.group(2)
    syllables = tuple(lazy_pinyin(candidate, style=Style.NORMAL, errors="default"))
    if len(syllables) != len(VOICE_WAKE_PINYIN):
        return 0, ""
    score = sum(
        _syllable_similarity(expected, actual)
        for expected, actual in zip(VOICE_WAKE_PINYIN, syllables)
    )
    # At least two full syllables plus one near-syllable must agree (5/6).
    # This accepts xiao-cai-yang and yao-tai-yang, but rejects unrelated
    # phrases such as “有太阳” or “呃太阳”.
    if score < 5:
        return 0, ""
    return len(candidate_match.group(1)) + len(candidate), candidate


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
        return _handle_result(task, payload, publish)
    raise ValueError("unsupported development topic")


def _handle_voice_audio(robot: Robot, payload: dict, publish=None) -> dict:
    transcript = _voice_transcript(payload)
    LOGGER.info(
        "received voice transcript robot=%s engine=%s text=%r",
        robot.code,
        payload.get("asr_engine") or "cloud",
        transcript[:500],
    )
    if not transcript:
        _record_voice_recognition(robot, payload, transcript, "no_speech")
        return {"status": "no_speech", "transcript": transcript}
    now = timezone.now()
    wake_end, wake_phrase = _fuzzy_wake_match(transcript)
    wake_matched = bool(wake_phrase)
    command = ""
    armed_until = _VOICE_WAKE_UNTIL.get(robot.code)
    if wake_matched:
        command = transcript[wake_end:].strip(" ，。,.!！?？")
        _VOICE_WAKE_UNTIL[robot.code] = now + timezone.timedelta(seconds=VOICE_WAKE_WINDOW_SECONDS)
    elif armed_until and armed_until >= now:
        command = transcript
    elif armed_until:
        _VOICE_WAKE_UNTIL.pop(robot.code, None)
    if len(re.sub(r"[\s，。,.!！?？]", "", command)) < 2:
        command = ""
    if not command:
        if wake_matched:
            _publish_voice_ack(robot, publish, "我在")
        outcome = "armed" if wake_matched else "ignored"
        _record_voice_recognition(robot, payload, transcript, outcome)
        return {"status": outcome, "transcript": transcript}
    _VOICE_WAKE_UNTIL.pop(robot.code, None)
    if DevelopmentTask.objects.filter(robot=robot, status__in=DevelopmentTask.ACTIVE_STATES).exists():
        _record_voice_recognition(robot, payload, transcript, "busy", command=command)
        return {"status": "busy", "transcript": transcript}
    task = DevelopmentTask.objects.create(robot=robot, workspace="robot-main", model="gpt-5.6-terra", prompt=command)
    _record_voice_recognition(robot, payload, transcript, "accepted", command=command, task=task)
    _publish_voice_ack(robot, publish, "收到", task_id=task.id)
    return {"status": "accepted", "task_id": str(task.id), "transcript": transcript}


def _record_voice_recognition(
    robot: Robot,
    payload: dict,
    transcript: str,
    outcome: str,
    *,
    command: str = "",
    task: DevelopmentTask | None = None,
) -> None:
    """Persist recognised text only; raw microphone audio is never stored here."""
    VoiceRecognitionEvent.objects.create(
        robot=robot,
        task=task,
        transcript=transcript[:1000],
        command=command[:1000],
        asr_engine=str(payload.get("asr_engine") or "cloud")[0:64],
        outcome=outcome,
    )


def _publish_voice_ack(
    robot: Robot,
    publish,
    text: str,
    task_id=None,
    *,
    kind: str = "acknowledgement",
    suppress_seconds: int | None = None,
) -> None:
    try:
        saved_path, _cache_hit = tts_service.synthesize_speech(text)
        if publish:
            payload = {"audio_url": _public_media_url(saved_path), "kind": kind}
            if suppress_seconds is not None:
                payload["suppress_seconds"] = suppress_seconds
            publish(f"robots/{robot.code}/dev/voice/ack", payload, 1, False)
    except Exception:
        LOGGER.exception("voice playback TTS failed task=%s kind=%s", task_id or "wake", kind)


def _latest_agent_message(task: DevelopmentTask) -> str:
    """Return Codex's latest human-facing message, not its JSON bookkeeping."""
    for raw_text in task.events.filter(event_type="output", stream="stdout").order_by("-sequence").values_list("text", flat=True):
        try:
            event = json.loads(raw_text)
        except (TypeError, json.JSONDecodeError):
            continue
        item = event.get("item") if isinstance(event, dict) else None
        if not isinstance(item, dict) or item.get("type") != "agent_message":
            continue
        message = str(item.get("text") or "").strip()
        if message:
            return message
    return ""


def _compact_voice_text(text: str, limit: int = TASK_SUMMARY_MAX_CHARS) -> str:
    text = re.sub(r"[`*_#>]", "", text)
    text = re.sub(r"\s+", " ", text).strip()
    if len(text) <= limit:
        return text
    return f"{text[: limit - 16].rstrip()}。内容过长，剩余部分请在开发页面查看。"


def _task_completion_announcement(task: DevelopmentTask) -> str | None:
    if task.status == "succeeded":
        result = _compact_voice_text(_latest_agent_message(task))
        return (
            f"Codex任务已完成。以下是本次执行结果。{result}"
            if result else "Codex任务已完成。"
        )
    if task.status == "failed":
        detail = _compact_voice_text(task.error_message)
        return f"Codex任务失败。{detail}" if detail else "Codex任务失败。"
    if task.status == "timed_out":
        return "Codex任务超时结束。"
    if task.status == "cancelled":
        return "Codex任务已取消。"
    return None


def _summary_suppress_seconds(text: str) -> int:
    # Mandarin TTS is normally about 3 characters per second; add startup margin
    # so the microphone cannot feed this completion broadcast back as a command.
    return min(TASK_SUMMARY_MAX_SUPPRESS_SECONDS, max(6, 4 + (len(text) + 2) // 3))


def _voice_transcript(payload: dict) -> str:
    """Use a successful NX-local ASR result, otherwise retain the old cloud path."""
    if payload.get("asr_engine") == LOCAL_ASR_ENGINE:
        transcript = str(payload.get("transcript") or "").strip()
        if len(transcript) > 1000:
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
def _handle_result(task: DevelopmentTask, payload: dict, publish=None) -> dict:
    task = DevelopmentTask.objects.select_for_update().get(pk=task.pk)
    if task.status in DevelopmentTask.TERMINAL_STATES:
        return {"status": task.status, "duplicate": True}
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
    announcement = _task_completion_announcement(task)
    if announcement and publish:
        task_id = task.id
        suppress_seconds = _summary_suppress_seconds(announcement)
        transaction.on_commit(
            lambda: _publish_voice_ack(
                task.robot,
                publish,
                announcement,
                task_id=task_id,
                kind="task_summary",
                suppress_seconds=suppress_seconds,
            )
        )
    return {"status": status}
