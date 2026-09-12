from __future__ import annotations

import json
from decimal import Decimal
from typing import Any

from django.db import IntegrityError, transaction
from django.utils import timezone

from ..models import PatrolRoute, PatrolTask, Robot, TaskExecution, TaskExecutionEvent
from .map_coordinate import MapConstraintError, constraints_from_map_data, validate_route_against_map
from .navigation_boundary_service import boundary_payload


class TaskStateError(ValueError):
    pass


ALLOWED_TRANSITIONS = {
    "created": {"dispatching", "pausing", "resuming", "cancelling", "cancelled", "interrupted"},
    "dispatching": {"accepted", "pausing", "resuming", "cancelling", "cancelled", "rejected", "timed_out", "interrupted"},
    # MQTT delivery can make the final Result overtake task.started.
    # Edge Result is authoritative, so terminal reconciliation is legal here.
    "accepted": {"running", "pausing", "resuming", "cancelling", "completed", "cancelled", "failed", "timed_out", "interrupted"},
    # Safety gates such as absolute-localization and arrival-stability checks
    # stop motion and emit task.paused atomically; they intentionally do not
    # expose an intermediate pausing state.
    "running": {"pausing", "paused", "resuming", "cancelling", "cancelled", "completed", "failed", "timed_out", "interrupted"},
    "pausing": {"accepted", "running", "paused", "resuming", "cancelling", "cancelled", "failed", "interrupted"},
    # A recovery command result can overtake task.resuming/task.resumed events.
    "paused": {"running", "pausing", "resuming", "cancelling", "cancelled", "interrupted"},
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
    return normalized if normalized in {"theta_star", "navfn", "smac_hybrid"} else "theta_star"


def _normalize_local_controller(value: object | None) -> str:
    normalized = str(value or "mppi").strip().lower()
    return normalized if normalized in {"mppi", "rpp", "ilqr"} else "mppi"


def _normalize_arrival_policy(value: object | None, *, dwell_seconds: float = 0.0,
                              require_yaw: bool = False, actions: list | None = None,
                              is_last: bool = False) -> str:
    """Normalize the explicit stop semantics while preserving legacy routes."""
    normalized = str(value or "").strip().lower()
    if normalized in {"pass_through", "stop_and_confirm", "precision", "dock"}:
        return normalized
    if is_last:
        return "stop_and_confirm"
    if dwell_seconds > 0 or require_yaw or actions:
        return "stop_and_confirm"
    # Legacy routes had no explicit semantics; fail closed and preserve the
    # existing stop/settle behavior until operators opt into pass-through.
    return "stop_and_confirm"


def _normalize_arrival_micro_adjust_mode(value: object | None, *, arrival_policy: str) -> str:
    mode = str(value or "cmd_vel").strip().lower()
    if mode not in {"cmd_vel", "nav2_goal"}:
        mode = "cmd_vel"
    if arrival_policy in {"pass_through", "dock"}:
        return "cmd_vel"
    return mode


def _normalize_anchor_preference(value: object | None) -> str:
    value = str(value or "balanced").strip().lower()
    return value if value in {"ndt", "rtk", "balanced"} else "balanced"


def normalize_waypoints(route: PatrolRoute) -> list[dict[str, Any]]:
    normalized = []
    names = route.waypoint_names or []
    route_global_controller = _normalize_global_controller(getattr(route, "global_controller", ""))
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
            local_controller = _normalize_local_controller(raw.get("local_controller"))
            global_controller = _normalize_global_controller(
                raw.get("global_controller") or route_global_controller
            )
            avoidance_to_next = bool(raw.get("avoidance_to_next", True))
            detour_enabled = bool(raw.get("detour_enabled", avoidance_to_next))
            collision_slowdown_enabled = bool(
                raw.get("collision_slowdown_enabled", avoidance_to_next)
            )
            collision_stop_enabled = bool(raw.get("collision_stop_enabled", True))
            require_yaw = bool(raw.get("require_yaw", False))
            arrival_policy = _normalize_arrival_policy(
                raw.get("arrival_policy"), dwell_seconds=dwell_seconds,
                require_yaw=require_yaw, actions=actions,
                is_last=index == len(route.waypoints or []) - 1,
            )
            arrival_micro_adjust_mode = _normalize_arrival_micro_adjust_mode(
                raw.get("arrival_micro_adjust_mode"), arrival_policy=arrival_policy
            )
            localization_anchor_preference = _normalize_anchor_preference(
                raw.get("localization_anchor_preference")
            )
            rtk_primary_allowed = bool(raw.get("rtk_primary_allowed", False))
            speech_mode = str(raw.get("speech_mode") or "").strip().lower()
            if speech_mode not in {"blocking", "non_blocking", "disabled"}:
                # Playback is a patrol side effect.  Missing TTS, command
                # delivery, or both speakers being offline must not gate the
                # next navigation leg.
                speech_mode = "non_blocking" if speech_template_id not in (None, "") else "disabled"
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
            global_controller = route_global_controller
            avoidance_to_next = True
            detour_enabled = True
            collision_slowdown_enabled = True
            collision_stop_enabled = True
            require_yaw = False
            speech_mode = "disabled"
            arrival_policy = _normalize_arrival_policy(
                None, dwell_seconds=0, require_yaw=False, actions=[],
                is_last=index == len(route.waypoints or []) - 1,
            )
            arrival_micro_adjust_mode = "cmd_vel"
            localization_anchor_preference = "balanced"
            rtk_primary_allowed = False
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
                "local_controller": local_controller,
                "global_controller": global_controller,
                "avoidance_to_next": avoidance_to_next,
                "detour_enabled": detour_enabled,
                "collision_slowdown_enabled": collision_slowdown_enabled,
                "collision_stop_enabled": collision_stop_enabled,
                "require_yaw": require_yaw,
                "arrival_policy": arrival_policy,
                "arrival_micro_adjust_mode": arrival_micro_adjust_mode,
                "localization_anchor_preference": localization_anchor_preference,
                "rtk_primary_allowed": rtk_primary_allowed,
                "speech_mode": speech_mode,
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
    try:
        boundary = map_data.navigation_boundary
    except Exception:
        boundary = None
    if boundary and boundary.active_revision > 0:
        snapshot["boundary_revision"] = boundary.active_revision
        snapshot["boundary"] = boundary_payload(boundary, active=True)
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
        route_snapshot: dict | None = None,
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
        snapshot = dict(route_snapshot) if route_snapshot is not None else None
        try:
            validate_route_against_map(
                constraints_from_map_data(route.map_data),
                scene_scope=str(getattr(route, "scene_scope", "") or route.map_data.scene_scope or "indoor"),
                waypoints=(snapshot.get("waypoints") or []) if snapshot is not None else (route.waypoints or []),
            )
        except MapConstraintError as exc:
            raise TaskStateError(exc.code) from exc
        snapshot = snapshot if snapshot is not None else build_route_snapshot(route)
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
        if target in {"completed", "failed", "cancelled", "timed_out", "rejected"}:
            # Local import avoids a module cycle: CommandService delegates task
            # state changes here, while terminal task state owns start-command
            # lifecycle reconciliation.
            from .command_service import CommandService

            CommandService.reconcile_task_start_for_execution(
                locked,
                source=event_type,
            )
        return locked

    @staticmethod
    @transaction.atomic
    def reconcile_edge_active_after_center_timeout(
        execution: TaskExecution,
        target: str,
        *,
        edge_state_version: int,
        reason_code: str = "",
        reason_message: str = "",
        payload: dict[str, Any] | None = None,
    ) -> TaskExecution:
        """Restore the authoritative active Edge state after a center-only timeout.

        COMMAND_TIMED_OUT says only that the center stopped waiting. It does not
        prove the physical task stopped, so a later sync from the same Edge task
        must be allowed to repair both legacy ``timed_out`` rows and the newer
        ``interrupted`` holding state without decreasing the state version.
        """
        locked = TaskExecution.objects.select_for_update().get(pk=execution.pk)
        if (
            locked.state not in {"timed_out", "interrupted"}
            or locked.failure_code != "COMMAND_TIMED_OUT"
            or target not in TaskExecution.ACTIVE_STATES
            or target in {"created", "dispatching"}
        ):
            raise TaskStateError("TASK_TIMEOUT_NOT_RECONCILABLE")
        next_version = max(locked.state_version + 1, int(edge_state_version) + 1)
        recovered_code = reason_code or (
            "EDGE_SYNC_PAUSED" if target == "paused" else "EDGE_SYNC_ACTIVE"
        )
        recovered_message = reason_message or (
            "云边状态对账：设备端任务已暂停"
            if target == "paused"
            else f"云边状态对账：设备端任务仍处于 {target} 状态"
        )
        previous_cloud_state = locked.state
        locked.state = target
        locked.state_version = next_version
        locked.finished_at = None
        locked.last_edge_event_at = timezone.now()
        locked.failure_code = recovered_code
        locked.failure_message = recovered_message
        if target == "paused":
            locked.paused_at = timezone.now()
        locked.save(update_fields=[
            "state",
            "state_version",
            "finished_at",
            "last_edge_event_at",
            "failure_code",
            "failure_message",
            "paused_at",
            "updated_at",
        ])
        TaskExecutionEvent.objects.create(
            task_execution=locked,
            state=target,
            state_version=next_version,
            event_type="task.sync_center_timeout_reconciled",
            occurred_at=timezone.now(),
            reason_code=recovered_code,
            reason_message=recovered_message,
            payload={
                **(payload or {}),
                "previous_cloud_state": previous_cloud_state,
                "edge_state_version": int(edge_state_version),
            },
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
