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

from .models import (
    DevelopmentAgentState,
    DevelopmentConversationState,
    DevelopmentTask,
    DevelopmentTaskEvent,
    Robot,
    VoiceRecognitionEvent,
)
from .message_handlers import _public_media_url
from .services import asr_service, tts_service

_VOICE_WAKE_UNTIL: dict[str, object] = {}
_VOICE_WAKE_COUNT: dict[str, tuple[object, int]] = {}
LOGGER = logging.getLogger(__name__)
# Wake matching remains anchored at the start of the utterance, but is based on
# Mandarin syllables instead of an exact ASR spelling.  The matcher deliberately
# has no list of observed misspellings: any homophone (for example “晓太洋”)
# is handled from its pinyin, and a single near-syllable is allowed without
# permitting an unrelated phrase to create a development task.
# “有太阳” is an intentional alternate wake phrase requested for the NX ASR:
# its syllables are recognised more reliably in the current microphone setup.
# Keep the original wake phrase available as well so existing commands remain
# compatible.
VOICE_WAKE_SHORT_ALIASES = ("太阳", "太陽")
VOICE_WAKE_ALIASES = ("小太阳", "小太陽", "有太阳", *VOICE_WAKE_SHORT_ALIASES)
# Place names spoken by patrol announcements must never arm the development
# wake window.  Match these as prefixes because an announcement normally puts
# the site name at the beginning of a much longer utterance.
VOICE_WAKE_FALSE_POSITIVE_PREFIXES = ("太阳宫",)
# A two-syllable wake alias has no reliable word boundary in unpunctuated
# Chinese ASR text.  Permit it only before an explicit separator or a common
# command lead-in; this keeps place names and nouns such as “太阳宫” and
# “太阳能” out while retaining commands such as “太阳检查导航”.
VOICE_COMMAND_PREFIXES = (
    "请", "帮", "给", "把", "将",
    "查", "看", "检查", "查看", "确认", "分析", "定位",
    "修改", "修复", "解决", "调整", "优化", "更新", "实现",
    "新增", "增加", "删除", "移除", "创建", "生成", "保存",
    "提交", "推送", "部署", "执行", "运行", "测试", "验证",
    "启动", "停止", "重启", "打开", "关闭", "取消", "继续",
    "播放", "设置",
)
VOICE_WAKE_PINYIN = ("xiao", "tai", "yang")
VOICE_WAKE_SHORT_PINYIN = ("tai", "yang")
_PINYIN_INITIALS = (
    "zh", "ch", "sh",
    "b", "p", "m", "f", "d", "t", "n", "l", "g", "k", "h",
    "j", "q", "x", "r", "z", "c", "s", "y", "w",
)
_NEAR_INITIAL_GROUPS = (
    frozenset(("b", "p")),
    frozenset(("d", "t")),
    frozenset(("g", "k")),
    frozenset(("n", "l")),
    frozenset(("f", "h")),
    frozenset(("j", "q", "x")),
    frozenset(("z", "zh")),
    frozenset(("c", "ch")),
    frozenset(("s", "sh")),
)
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


def _split_pinyin_syllable(syllable: str) -> tuple[str, str]:
    """Return Mandarin initial and rime for a tone-free pinyin syllable."""
    normalized = re.sub(r"[^a-zv]", "", syllable.lower().replace("ü", "v"))
    for initial in _PINYIN_INITIALS:
        if normalized.startswith(initial):
            return initial, normalized[len(initial):]
    return "", normalized


def _initial_similarity(expected: str, actual: str) -> float:
    if expected == actual:
        return 1.0
    if any(expected in group and actual in group for group in _NEAR_INITIAL_GROUPS):
        return 0.5
    return 0.0


def _rime_similarity(expected: str, actual: str) -> float:
    if expected == actual:
        return 1.0
    # A dropped glide is a common ASR variation: xiao/lao and xiao/yao keep
    # the same "ao" rime.  Requiring at least two shared trailing letters
    # keeps xiao/you ("ao" versus "ou") from being accepted.
    if min(len(expected), len(actual)) >= 2 and (
        expected.endswith(actual) or actual.endswith(expected)
    ):
        return 0.75
    return 0.0


def _syllable_similarity(expected: str, actual: str) -> float:
    """Score one Mandarin syllable from 0.0 to 1.0 without variant tables."""
    if expected == actual:
        return 1.0
    expected_initial, expected_rime = _split_pinyin_syllable(expected)
    actual_initial, actual_rime = _split_pinyin_syllable(actual)
    rime_score = _rime_similarity(expected_rime, actual_rime)
    if not rime_score:
        return 0.0
    # The rime carries more acoustic information than the initial.  A close
    # initial helps, but an ASR initial substitution alone is not a rejection.
    return (0.75 * rime_score) + (0.25 * _initial_similarity(expected_initial, actual_initial))


def _fuzzy_wake_match(transcript: str) -> tuple[int, str]:
    """Return the prefix end when its first two or three syllables are a wake."""
    leading_space = re.match(r"^\s*", transcript)
    prefix_start = leading_space.end() if leading_space else 0
    for alias in VOICE_WAKE_ALIASES:
        if not transcript.startswith(alias, prefix_start):
            continue
        prefix_end = prefix_start + len(alias)
        if alias not in VOICE_WAKE_SHORT_ALIASES or _has_short_wake_boundary(transcript, prefix_end):
            return prefix_end, alias

    candidate_match = re.match(r"^(\s*)([\u4e00-\u9fff]{2,3})", transcript)
    if not candidate_match:
        return 0, ""
    candidate = candidate_match.group(2)
    syllables = tuple(lazy_pinyin(candidate, style=Style.NORMAL, errors="default"))
    if len(syllables) == 3:
        scores = tuple(
            _syllable_similarity(expected, actual)
            for expected, actual in zip(VOICE_WAKE_PINYIN, syllables)
        )
        if sum(scores) >= 2.5 and sum(score >= 0.95 for score in scores) >= 2:
            return len(candidate_match.group(1)) + len(candidate), candidate

    # SenseVoice may alter or omit the first wake syllable.  For a three-byte
    # Chinese prefix, treat only characters 2 and 3 as the wake core; the
    # first character is deliberately unconstrained ("小太阳"/"有太阳"/…).
    # With only two prefix characters, they themselves are the wake core.
    cores = [(candidate[1:3], len(candidate))] if len(candidate) == 3 else []
    cores.append((candidate[:2], 2))
    for wake_core, consumed in cores:
        wake_syllables = tuple(lazy_pinyin(wake_core, style=Style.NORMAL, errors="default"))
        if len(wake_syllables) != 2:
            continue
        scores = tuple(
            _syllable_similarity(expected, actual)
            for expected, actual in zip(VOICE_WAKE_SHORT_PINYIN, wake_syllables)
        )
        prefix_end = len(candidate_match.group(1)) + consumed
        if (
            sum(scores) >= 1.5
            and sum(score >= 0.95 for score in scores) >= 1
            and _has_short_wake_boundary(transcript, prefix_end)
        ):
            return prefix_end, candidate[:consumed]
    return 0, ""


def _has_short_wake_boundary(transcript: str, prefix_end: int) -> bool:
    """Require a separator, utterance end, or command lead-in after a short wake."""
    remainder = transcript[prefix_end:]
    if not remainder:
        return True
    if re.match(r"^[\s，。,.!！?？：:；;、]", remainder):
        return True
    return remainder.startswith(VOICE_COMMAND_PREFIXES)


def _is_double_wake_phrase(transcript: str) -> bool:
    """Recognise “小太阳小太阳” spoken as one utterance.

    The normal matcher consumes one wake phrase only.  Detecting the repeated
    form first prevents its second half from being treated as executable text.
    """
    compact = re.sub(r"[\s，。,.!！?？]", "", transcript)
    return any(compact.startswith(alias + alias) for alias in VOICE_WAKE_ALIASES)


def _is_false_wake_phrase(transcript: str) -> bool:
    """Reject known non-command prefixes before fuzzy wake matching."""
    compact = re.sub(r"[\s，。,.!！?？：:；;、]", "", transcript)
    return compact.startswith(VOICE_WAKE_FALSE_POSITIVE_PREFIXES)


def _looks_like_short_wake_attempt(transcript: str) -> bool:
    """Identify a garbled, short retry while the wake window is already open.

    A wake-only request opens a short window so that the next utterance can be
    used as the development command.  Without this guard, an ASR retry such as
    “小要大呀” could be mistaken for that command.  It is safer to keep the
    window open and ask for the command than to submit meaningless text to
    Codex.  This deliberately applies only to short utterances beginning with
    the first wake syllable; normal commands remain unchanged.
    """
    compact = re.sub(r"[\s，。,.!！?？]", "", transcript)
    if len(compact) > 4:
        return False
    candidate_match = re.match(r"^[\u4e00-\u9fff]{3}", compact)
    if not candidate_match:
        return False
    syllables = tuple(lazy_pinyin(candidate_match.group(), style=Style.NORMAL, errors="default"))
    return bool(syllables) and _syllable_similarity(VOICE_WAKE_PINYIN[0], syllables[0]) >= 0.95


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
    if _is_false_wake_phrase(transcript):
        # A false positive must also close a previously armed window; otherwise
        # the following unrelated utterance could still become a Codex task.
        _VOICE_WAKE_UNTIL.pop(robot.code, None)
        _VOICE_WAKE_COUNT.pop(robot.code, None)
        _record_voice_recognition(robot, payload, transcript, "ignored")
        return {"status": "ignored", "transcript": transcript}
    now = timezone.now()
    if _is_double_wake_phrase(transcript):
        _VOICE_WAKE_UNTIL[robot.code] = now + timezone.timedelta(seconds=VOICE_WAKE_WINDOW_SECONDS)
        _VOICE_WAKE_COUNT[robot.code] = (now, 2)
        _publish_voice_ack(robot, publish, "在呢")
        _record_voice_recognition(robot, payload, transcript, "armed")
        return {"status": "armed", "transcript": transcript}
    wake_end, wake_phrase = _fuzzy_wake_match(transcript)
    wake_matched = bool(wake_phrase)
    command = ""
    armed_until = _VOICE_WAKE_UNTIL.get(robot.code)
    if wake_matched:
        command = transcript[wake_end:].strip(" ，。,.!！?？：:；;、")
        _VOICE_WAKE_UNTIL[robot.code] = now + timezone.timedelta(seconds=VOICE_WAKE_WINDOW_SECONDS)
        if not command:
            previous = _VOICE_WAKE_COUNT.get(robot.code)
            count = previous[1] + 1 if previous and now - previous[0] <= timezone.timedelta(seconds=VOICE_WAKE_WINDOW_SECONDS) else 1
            _VOICE_WAKE_COUNT[robot.code] = (now, count)
            _publish_voice_ack(robot, publish, "在呢" if count >= 2 else "收到")
            _record_voice_recognition(robot, payload, transcript, "armed")
            return {"status": "armed", "transcript": transcript}
        _VOICE_WAKE_COUNT.pop(robot.code, None)
    elif armed_until and armed_until >= now:
        if _looks_like_short_wake_attempt(transcript):
            _VOICE_WAKE_UNTIL[robot.code] = now + timezone.timedelta(seconds=VOICE_WAKE_WINDOW_SECONDS)
            _publish_voice_ack(robot, publish, "在呢")
            _record_voice_recognition(robot, payload, transcript, "armed")
            return {"status": "armed", "transcript": transcript}
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
    _VOICE_WAKE_COUNT.pop(robot.code, None)
    if DevelopmentTask.objects.filter(robot=robot, status__in=DevelopmentTask.ACTIVE_STATES).exists():
        # A voice command used to disappear silently while Codex was busy.
        # Preserve the single-task safety boundary, but make the outcome
        # audible so the operator knows the microphone and wake word worked.
        _publish_voice_ack(robot, publish, "我正在处理上一条任务，请稍后再说")
        _record_voice_recognition(robot, payload, transcript, "busy", command=command)
        return {"status": "busy", "transcript": transcript}
    conversation_state, _ = DevelopmentConversationState.objects.get_or_create(robot=robot)
    task = DevelopmentTask.objects.create(
        robot=robot,
        workspace=conversation_state.workspace,
        model=conversation_state.model,
        execution_mode=conversation_state.mode,
        prompt=command,
    )
    _record_voice_recognition(robot, payload, transcript, "accepted", command=command, task=task)
    _publish_voice_ack(
        robot,
        publish,
        "收到",
        task_id=task.id,
    )
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
    sample_rate = int(payload.get("sample_rate") or 16000)
    channels = int(payload.get("channels") or 1)
    if sample_rate != 16000 or channels not in {1, 2}:
        raise ValueError("invalid voice audio format")
    if len(pcm) % (2 * channels):
        raise ValueError("invalid voice audio frame alignment")
    with tempfile.TemporaryDirectory(prefix="roamerx-voice-") as directory:
        audio_path = Path(directory) / "voice.wav"
        with wave.open(str(audio_path), "wb") as output:
            output.setnchannels(channels); output.setsampwidth(2); output.setframerate(sample_rate); output.writeframes(pcm)
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
