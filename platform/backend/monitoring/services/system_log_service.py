from __future__ import annotations

import json
import math
import uuid
from datetime import timedelta
from typing import Any

from django.db import transaction
from django.utils import timezone
from django.utils.dateparse import parse_datetime

from ..models import MapData, RemoteCommand, SystemLog, TaskExecution


LEVELS = {choice[0] for choice in SystemLog.LEVEL_CHOICES}
MODULES = {choice[0] for choice in SystemLog.MODULE_CHOICES}
MAX_DATA_BYTES = 16 * 1024
SENSITIVE_KEYS = {"password", "secret", "token", "authorization", "credential", "private_key"}


def _sanitize(value: Any, depth: int = 0) -> Any:
    if depth > 5:
        return "<truncated>"
    if isinstance(value, dict):
        return {
            str(key)[:96]: "<redacted>" if any(secret in str(key).lower() for secret in SENSITIVE_KEYS) else _sanitize(item, depth + 1)
            for key, item in list(value.items())[:100]
        }
    if isinstance(value, list):
        return [_sanitize(item, depth + 1) for item in value[:50]]
    if isinstance(value, str):
        return value[:2000]
    if value is None or isinstance(value, (bool, int, float)):
        return value
    return str(value)[:500]


def sanitize_data(value: Any) -> dict:
    cleaned = _sanitize(value if isinstance(value, dict) else {})
    encoded = json.dumps(cleaned, ensure_ascii=False, default=str).encode("utf-8")
    if len(encoded) <= MAX_DATA_BYTES:
        return cleaned
    return {"truncated": True, "original_bytes": len(encoded)}


def _uuid_or_none(value):
    try:
        return uuid.UUID(str(value)) if value else None
    except (TypeError, ValueError, AttributeError):
        return None


def _repeat_count(value) -> int:
    try:
        return max(1, min(1_000_000, int(value or 1)))
    except (TypeError, ValueError):
        return 1


def _coordinate(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(float(value)) else None


def emit_center_log(*, robot, level: str, module: str, event_code: str, message: str, data=None, command=None, task_execution=None, map_data=None, trace_id=None, waypoint_index=None, pose=None, dedupe_seconds: int = 0):
    pose = pose or {}
    if map_data is None:
        candidate = (getattr(command, "payload", None) or {}).get("map_id") if command else None
        if candidate is None and task_execution:
            candidate = ((task_execution.route_snapshot or {}).get("map") or {}).get("map_id")
        if str(candidate or "").isdigit():
            map_data = MapData.objects.filter(pk=candidate).first()
    if dedupe_seconds > 0:
        previous = SystemLog.objects.filter(
            robot=robot,
            level=level,
            module=module,
            event_code=event_code,
            command=command,
            task_execution=task_execution,
            occurred_at__gte=timezone.now() - timedelta(seconds=dedupe_seconds),
        ).order_by("-occurred_at").first()
        if previous and previous.message == message[:500]:
            previous.repeat_count += 1
            previous.occurred_at = timezone.now()
            previous.data = sanitize_data(data or {})
            previous.save(update_fields=["repeat_count", "occurred_at", "data", "updated_at"])
            return previous
    return SystemLog.objects.create(
        robot=robot,
        level=level,
        module=module,
        event_code=event_code[:96],
        message=message[:500],
        source="center",
        data=sanitize_data(data or {}),
        command=command,
        task_execution=task_execution,
        map_data=map_data,
        trace_id=_uuid_or_none(trace_id or getattr(command, "trace_id", None)),
        waypoint_index=waypoint_index,
        x=pose.get("x"),
        y=pose.get("y"),
        yaw=pose.get("yaw"),
    )


@transaction.atomic
def ingest_batch(robot, payload: dict, *, envelope_trace_id=None) -> dict:
    entries = payload.get("entries") or []
    accepted = 0
    for raw in entries:
        level = str(raw.get("level") or "").upper()
        module = str(raw.get("module") or "")
        if level not in LEVELS or module not in MODULES:
            continue
        occurred_at = parse_datetime(str(raw.get("occurred_at") or "")) or timezone.now()
        if timezone.is_naive(occurred_at):
            occurred_at = timezone.make_aware(occurred_at)
        command_id = _uuid_or_none(raw.get("command_id"))
        task_id = _uuid_or_none(raw.get("task_execution_id"))
        command = RemoteCommand.objects.filter(pk=command_id, robot=robot).first() if command_id else None
        task = TaskExecution.objects.filter(pk=task_id, robot=robot).first() if task_id else None
        map_id = raw.get("map_id")
        map_data = MapData.objects.filter(pk=map_id).first() if str(map_id or "").isdigit() else None
        pose = raw.get("pose") if isinstance(raw.get("pose"), dict) else {}
        event_code = str(raw.get("event_code") or "unknown")[:96]
        source = str(raw.get("source") or "edge")[:64]
        previous = SystemLog.objects.filter(
            robot=robot,
            level=level,
            module=module,
            event_code=event_code,
            source=source,
            occurred_at__gte=occurred_at - timedelta(seconds=5),
        ).order_by("-occurred_at").first()
        if previous and previous.message == str(raw.get("message") or "")[:500] and level != "DEBUG":
            previous.repeat_count += _repeat_count(raw.get("repeat_count"))
            previous.occurred_at = occurred_at
            previous.data = sanitize_data(raw.get("data") or {})
            previous.save(update_fields=["repeat_count", "occurred_at", "data", "updated_at"])
        else:
            SystemLog.objects.create(
                robot=robot,
                occurred_at=occurred_at,
                level=level,
                module=module,
                event_code=event_code,
                message=str(raw.get("message") or event_code)[:500],
                source=source,
                data=sanitize_data(raw.get("data") or {}),
                trace_id=_uuid_or_none(raw.get("trace_id")) or envelope_trace_id,
                command=command,
                task_execution=task,
                map_data=map_data,
                waypoint_index=raw.get("waypoint_index") if isinstance(raw.get("waypoint_index"), int) else None,
                x=_coordinate(pose.get("x")),
                y=_coordinate(pose.get("y")),
                yaw=_coordinate(pose.get("yaw")),
                repeat_count=_repeat_count(raw.get("repeat_count")),
            )
        accepted += 1
    return {"accepted": accepted, "rejected": len(entries) - accepted}
