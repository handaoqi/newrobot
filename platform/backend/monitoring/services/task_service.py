from __future__ import annotations

import json
from decimal import Decimal
from typing import Any

from django.db import IntegrityError, transaction
from django.utils import timezone

from ..models import PatrolRoute, PatrolTask, Robot, TaskExecution, TaskExecutionEvent
from .map_coordinate import MapConstraintError, constraints_from_map_data, validate_route_against_map


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

# A reconnecting Edge can be the only side that observed the terminal result.
# Allow that authoritative terminal state to close any cloud-side active state
# instead of leaving the robot blocked behind a stale ROBOT_BUSY execution.
TERMINAL_RECONCILIATION_STATES = {"completed", "failed", "cancelled", "timed_out", "rejected"}
for _active_state in TaskExecution.ACTIVE_STATES:
    ALLOWED_TRANSITIONS[_active_state].update(TERMINAL_RECONCILIATION_STATES)


def assert_transition_allowed(current: str, target: str) -> None:
    if target not in ALLOWED_TRANSITIONS.get(current, set()):
        raise TaskStateError(f"invalid task state transition: {current} -> {target}")


def _normalize_global_controller(value: object | None) -> str:
    normalized = str(value or "theta_star").strip().lower()
    return normalized if normalized in {"theta_star", "navfn"} else "theta_star"


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
            try:
                dwell_seconds = float(raw.get("dwell_seconds", 0) or 0)
            except (TypeError, ValueError) as exc:
                raise TaskStateError(f"route waypoint {index} dwell_seconds must be numeric") from exc
            if not 0.0 <= dwell_seconds <= 3600.0:
                raise TaskStateError(f"route waypoint {index} dwell_seconds must be between 0 and 3600")
            actions = list(raw.get("actions") or [])
            speech_template_id = raw.get("speech_template_id")
            speech_template_name = str(raw.get("speech_template_name") or "")
            speech_text = str(raw.get("speech_text") or "")
            localization_mode = str(raw.get("localization_mode") or "ndt").lower()
            local_controller = str(raw.get("local_controller") or "mppi").lower()
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
            local_controller = "mppi"
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
            "dwell_seconds": round(dwell_seconds, 1),
                "actions": actions,
                "localization_mode": localization_mode if localization_mode in {"ndt", "rtk", "ukf"} else "ndt",
                # RPP is a legacy route value. The deployed navigo stack only
                # registers FollowPath (MPPI), so old routes remain executable.
                "local_controller": "mppi" if local_controller in {"rpp", "mppi"} else "mppi",
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
            "coordinate_mode": map_data.coordinate_mode,
            "scene_scope": map_data.scene_scope,
            "localization_mode": map_data.localization_mode,
            "origin_status": map_data.origin_status,
            "completeness": map_data.map_completeness,
        },
        "scene_scope": getattr(route, "scene_scope", "") or map_data.scene_scope or "indoor",
        "global_controller": _normalize_global_controller(getattr(route, "global_controller", "")),
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
    def loop_dispatch_key(loop_session_id, round_number: int) -> str | None:
        if not loop_session_id:
            return None
        return f"{loop_session_id}:{max(1, int(round_number))}"

    @staticmethod
    def find_loop_execution(loop_session_id, round_number: int) -> TaskExecution | None:
        key = TaskExecutionService.loop_dispatch_key(loop_session_id, round_number)
        if not key:
            return None
        return TaskExecution.objects.filter(loop_dispatch_key=key).first()

    @staticmethod
    @transaction.atomic
    def create_execution(
        task: PatrolTask,
        operator=None,
        *,
        loop_session_id=None,
        round_number: int = 1,
    ) -> TaskExecution:
        robot = Robot.objects.select_for_update().get(pk=task.robot_id)
        active = TaskExecution.objects.filter(robot=robot, state__in=TaskExecution.ACTIVE_STATES).order_by("-created_at").first()
        if active:
            raise TaskStateError(f"ROBOT_BUSY: 执行 {active.id} 仍处于 {active.state}（第 {active.round_number} 轮）")
        if not task.enabled:
            raise TaskStateError("TASK_DISABLED")
        route = task.route
        if route is None:
            raise TaskStateError("TASK_ROUTE_MISSING")
        try:
            validate_route_against_map(
                constraints_from_map_data(route.map_data),
                scene_scope=str(getattr(route, "scene_scope", "") or route.map_data.scene_scope or "indoor"),
                waypoints=route.waypoints or [],
            )
        except MapConstraintError as exc:
            raise TaskStateError(exc.code) from exc
        snapshot = build_route_snapshot(route)
        try:
            execution = TaskExecution.objects.create(
                task=task,
                robot=robot,
                route=route,
                map_data=route.map_data,
                route_snapshot=snapshot,
                loop_session_id=loop_session_id,
                round_number=max(1, int(round_number)),
                loop_dispatch_key=TaskExecutionService.loop_dispatch_key(loop_session_id, round_number),
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
            payload={
                "source": "center",
                "loop_session_id": str(execution.loop_session_id) if execution.loop_session_id else None,
                "round_number": execution.round_number,
            },
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
