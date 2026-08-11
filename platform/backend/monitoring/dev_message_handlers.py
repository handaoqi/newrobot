from __future__ import annotations

import json
import uuid

from django.db import transaction
from django.db.models import Max
from django.utils import timezone
from django.utils.dateparse import parse_datetime

from .models import DevelopmentAgentState, DevelopmentTask, DevelopmentTaskEvent, Robot


def _timestamp(value):
    parsed = parse_datetime(str(value or ""))
    return parsed or timezone.now()


def handle_dev_mqtt_message(topic: str, raw_payload: bytes | str | dict) -> dict:
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
        return {"status": state.status}
    if len(parts) < 6 or parts[3] != "tasks":
        raise ValueError("invalid development task topic")
    task_id = uuid.UUID(parts[4])
    task = DevelopmentTask.objects.get(pk=task_id, robot=robot)
    if parts[5] == "events":
        return _handle_event(task, payload)
    if parts[5] == "result":
        return _handle_result(task, payload)
    raise ValueError("unsupported development topic")


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
