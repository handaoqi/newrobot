from __future__ import annotations

import json
from decimal import Decimal
from typing import Any

from django.db import IntegrityError, transaction
from django.utils import timezone

from ..models import PatrolRoute, PatrolTask, Robot, TaskExecution, TaskExecutionEvent


class TaskStateError(ValueError):
    pass


ALLOWED_TRANSITIONS = {
    "created": {"dispatching", "pausing", "resuming", "cancelling", "cancelled"},
    "dispatching": {"accepted", "pausing", "resuming", "cancelling", "cancelled", "rejected", "timed_out"},
    # MQTT delivery can make the final Result overtake task.started.
    # Edge Result is authoritative, so terminal reconciliation is legal here.
    "accepted": {"running", "pausing", "resuming", "cancelling", "completed", "cancelled", "failed", "timed_out", "interrupted"},
    "running": {"pausing", "resuming", "cancelling", "cancelled", "completed", "failed", "timed_out", "interrupted"},
    "pausing": {"accepted", "running", "paused", "resuming", "cancelling", "cancelled", "failed", "interrupted"},
    "paused": {"pausing", "resuming", "cancelling", "cancelled", "interrupted"},
    "resuming": {"accepted", "running", "paused", "pausing", "cancelling", "cancelled", "failed", "interrupted"},
    "cancelling": {"cancelled", "failed", "interrupted"},
    "interrupted": {"paused", "running", "pausing", "resuming", "cancelling", "cancelled", "failed"},
}


def assert_transition_allowed(current: str, target: str) -> None:
    if target not in ALLOWED_TRANSITIONS.get(current, set()):
        raise TaskStateError(f"invalid task state transition: {current} -> {target}")


def normalize_waypoints(route: PatrolRoute) -> list[dict[str, Any]]:
    normalized = []
    names = route.waypoint_names or []
    for index, raw in enumerate(route.waypoints or []):
        if isinstance(raw, dict):
            x = raw.get("x")
            y = raw.get("y")
            yaw = raw.get("yaw", 0.0)
            name = raw.get("name") or (names[index] if index < len(names) else f"航点 {index + 1}")
            waypoint_id = str(raw.get("waypoint_id") or f"wp-{index + 1}")
            map_point_number = int(raw.get("map_point_number") or index + 1)
            dwell_seconds = int(raw.get("dwell_seconds", 0))
            actions = list(raw.get("actions") or [])
            speech_template_id = raw.get("speech_template_id")
            speech_template_name = str(raw.get("speech_template_name") or "")
            speech_text = str(raw.get("speech_text") or "")
            localization_mode = str(raw.get("localization_mode") or "ndt").lower()
            avoidance_to_next = bool(raw.get("avoidance_to_next", True))
            require_yaw = bool(raw.get("require_yaw", False))
        elif isinstance(raw, (list, tuple)) and len(raw) >= 2:
            x, y = raw[0], raw[1]
            yaw = raw[2] if len(raw) >= 3 else 0.0
            name = names[index] if index < len(names) else f"航点 {index + 1}"
            waypoint_id = f"wp-{index + 1}"
            map_point_number = index + 1
            dwell_seconds = 0
            actions = []
            speech_template_id = None
            speech_template_name = ""
            speech_text = ""
            localization_mode = "ndt"
            avoidance_to_next = True
            require_yaw = False
        else:
            raise TaskStateError(f"route waypoint {index} has invalid format")
        waypoint = {
                "waypoint_id": waypoint_id,
                "sequence": index,
                "map_point_number": map_point_number,
                "name": name,
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
                "dwell_seconds": dwell_seconds,
                "actions": actions,
                "localization_mode": localization_mode if localization_mode in {"ndt", "rtk"} else "ndt",
                "avoidance_to_next": avoidance_to_next,
                "require_yaw": require_yaw,
            }
        if speech_template_id not in (None, ""):
            waypoint.update(
                {
                    "speech_template_id": int(speech_template_id),
                    "speech_template_name": speech_template_name,
                    "speech_text": speech_text,
                }
            )
        normalized.append(waypoint)
    if not normalized:
        raise TaskStateError("route must contain at least one waypoint")
    return normalized


def build_route_snapshot(route: PatrolRoute) -> dict[str, Any]:
    map_data = route.map_data
    snapshot = {
        "route_id": str(route.id),
        "route_name": route.name,
        "frame_id": "map",
        "map": {
            "map_id": str(map_data.id),
            "map_version": f"legacy-mapdata-{map_data.id}",
            "sha256": None,
            "map_name": map_data.name,
            "local_map_dir": _map_local_dir(map_data),
        },
        "waypoints": normalize_waypoints(route),
    }
    if route.map_set_id:
        members = list(route.map_set.members.select_related("map_data").all())
        snapshot["map_set"] = {
            "id": str(route.map_set_id),
            "name": route.map_set.name,
            "version": route.map_set.version,
            "manifest": route.map_set.manifest,
            "submaps": [
                {
                    "submap_id": member.submap_id,
                    "sequence": member.sequence,
                    "map_id": str(member.map_data_id),
                    "map_version": f"legacy-mapdata-{member.map_data_id}",
                    "metadata": member.metadata,
                    "local_map_dir": _member_local_map_dir(member),
                }
                for member in members
            ],
        }
    return snapshot


def _member_local_map_dir(member) -> str:
    try:
        description = json.loads(member.map_data.description or "{}")
    except (TypeError, json.JSONDecodeError):
        return ""
    return str(description.get("source_map_dir") or "")


def _map_local_dir(map_data: MapData) -> str:
    try:
        description = json.loads(map_data.description or "{}")
    except (TypeError, json.JSONDecodeError):
        return ""
    return str(description.get("source_map_dir") or "")


class TaskExecutionService:
    @staticmethod
    @transaction.atomic
    def create_execution(task: PatrolTask, operator=None) -> TaskExecution:
        robot = Robot.objects.select_for_update().get(pk=task.robot_id)
        if TaskExecution.objects.filter(robot=robot, state__in=TaskExecution.ACTIVE_STATES).exists():
            raise TaskStateError("ROBOT_BUSY")
        if not task.enabled:
            raise TaskStateError("TASK_DISABLED")
        route = task.route
        if route is None:
            raise TaskStateError("TASK_ROUTE_MISSING")
        snapshot = build_route_snapshot(route)
        try:
            execution = TaskExecution.objects.create(
                task=task,
                robot=robot,
                route=route,
                map_data=route.map_data,
                route_snapshot=snapshot,
                total_waypoints=len(snapshot["waypoints"]),
                created_by=operator,
            )
        except IntegrityError as exc:
            raise TaskStateError("ROBOT_BUSY") from exc
        TaskExecutionEvent.objects.create(
            task_execution=execution,
            state="created",
            state_version=0,
            event_type="task.created",
            occurred_at=timezone.now(),
            payload={"source": "center"},
        )
        return execution

    @staticmethod
    @transaction.atomic
    def transition(
        execution: TaskExecution,
        target: str,
        *,
        event_type: str,
        state_version: int | None = None,
        occurred_at=None,
        message_id=None,
        reason_code: str = "",
        reason_message: str = "",
        payload: dict[str, Any] | None = None,
    ) -> TaskExecution:
        locked = TaskExecution.objects.select_for_update().get(pk=execution.pk)
        assert_transition_allowed(locked.state, target)
        next_version = locked.state_version + 1 if state_version is None else state_version
        if next_version <= locked.state_version:
            raise TaskStateError("STALE_TASK_STATE")
        locked.state = target
        locked.state_version = next_version
        locked.last_edge_event_at = occurred_at or timezone.now()
        if target == "accepted":
            locked.accepted_at = occurred_at or timezone.now()
        if target == "running" and locked.started_at is None:
            locked.started_at = occurred_at or timezone.now()
        if target == "paused":
            locked.paused_at = occurred_at or timezone.now()
        if target in {"completed", "failed", "cancelled", "timed_out", "rejected"}:
            locked.finished_at = occurred_at or timezone.now()
        if reason_code:
            locked.failure_code = reason_code
            locked.failure_message = reason_message
        locked.save()
        TaskExecutionEvent.objects.create(
            task_execution=locked,
            state=target,
            state_version=next_version,
            event_type=event_type,
            occurred_at=occurred_at or timezone.now(),
            message_id=message_id,
            reason_code=reason_code,
            reason_message=reason_message,
            payload=payload or {},
        )
        return locked

    @staticmethod
    def apply_progress(execution: TaskExecution, payload: dict[str, Any]) -> TaskExecution:
        incoming_version = int(payload["state_version"])
        if incoming_version <= execution.state_version:
            return execution
        execution.state_version = incoming_version
        execution.current_waypoint_index = payload.get("current_waypoint_index")
        execution.current_waypoint_id = payload.get("current_waypoint_id", "")
        execution.completed_waypoints = int(payload.get("completed_waypoints", 0))
        execution.total_waypoints = int(payload.get("total_waypoints", execution.total_waypoints))
        distance = payload.get("distance_remaining_m")
        execution.distance_remaining_m = Decimal(str(distance)) if distance is not None else None
        execution.estimated_time_remaining_s = payload.get("estimated_time_remaining_s")
        execution.last_edge_event_at = timezone.now()
        execution.save()
        return execution
