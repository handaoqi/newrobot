from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from datetime import datetime
from typing import Any

from django.utils import timezone
from django.utils.dateparse import parse_datetime


PROTOCOL_VERSION = "1.0"
COMMAND_TYPES = {
    "task.start",
    "task.pause",
    "task.resume",
    "task.resume_forward",
    "task.cancel",
    "task.force_exit",
    "task.recover.v1",
    "mapping.start",
    "mapping.save",
    "mapping.cancel",
    "mapping.status",
    "mapping.scene_semantics",
    "mapping.origin_start",
    "mapping.origin_cancel",
    "mapping.origin_extract_global",
    "mapping.slam_start",
    "mapping.begin",
    "nav.status",
    "nav.start",
    "nav.restart",
    "nav.recover",
    "nav.stop",
    "nav.initial_pose",
    "nav.single_goal",
    "nav.relocalize",
    "diagnostics.log_config",
    "diagnostics.nav_rosbag_stop",
    "map.activate",
    "map.boundary_apply",
    "map.optimize",
    "sensor.restart",
    "charge.start",
    "charge.stop",
    "motion.start",
    "motion.stop",
    "audio.volume",
    "teleop.takeover_enter",
    "teleop.takeover_exit",
    "teleop.stand_up",
    "teleop.lie_down",
    "teleop.shake_hand",
    "teleop.two_leg_stand",
    "teleop.crawl_forward",
    "teleop.speed_micro",
    "teleop.speed_slow",
    "teleop.speed_normal",
    "teleop.speed_fast",
    "teleop.move_forward",
    "teleop.move_backward",
    "teleop.move_left",
    "teleop.move_right",
    "teleop.turn_left",
    "teleop.turn_right",
    "teleop.move_velocity",
    "teleop.move_stop",
    "teleop.passive",
    "teleop.skill",
    "teleop.skill_list",
    "teleop.skill_status",
    "teleop.skill_cancel",
    "teleop.person_follow_start",
    "teleop.person_follow_stop",
    "teleop.person_follow_status",
}
UPLINK_MESSAGE_TYPES = {
    "presence.online",
    "presence.heartbeat",
    "presence.offline",
    "telemetry.status",
    "telemetry.pose",
    "trajectory.batch",
    "command.ack",
    "command.progress",
    "command.result",
    "task.progress",
    "task.navigation_stage",
    "task.round_started",
    "task.obstacle_speech",
    "task.obstacle_stage",
    "navigation.obstacle_recovery",
    "task.accepted",
    "task.started",
    "task.pausing",
    "task.paused",
    "task.resuming",
    "task.resumed",
    "task.cancelling",
    "task.completed",
    "task.failed",
    "task.cancelled",
    "task.interrupted",
    "task.arrival_pending_settle",
    "task.arrival_nav2_stopping",
    "task.arrival_zero_confirming",
    "task.arrival_zero_confirmed",
    "task.arrival_zero_timeout",
    "task.arrival_check",
    "task.arrival_confirmed",
    "task.arrival_degraded_accepted",
    "task.arrival_correcting",
    "task.arrival_heading_aligning",
    "task.arrival_heading_aligned",
    "task.recovery_active",
    "task.safe_hold",
    "task.waypoint_actions",
    "task.waypoint_degraded",
    "task.waypoint_postprocess_completed",
    "alert.event",
    "system.log.batch",
    "sync.request",
}
CENTER_DOWNLINK_MESSAGE_TYPES = {
    "sync.response",
    "trajectory.ack",
}
ALL_MESSAGE_TYPES = UPLINK_MESSAGE_TYPES | COMMAND_TYPES


class ProtocolError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def _required(mapping: dict[str, Any], key: str) -> Any:
    if key not in mapping or mapping[key] is None:
        raise ProtocolError("INVALID_MESSAGE", f"missing required field: {key}")
    return mapping[key]


def _uuid(value: Any, field: str) -> uuid.UUID:
    try:
        return uuid.UUID(str(value))
    except (ValueError, TypeError, AttributeError) as exc:
        raise ProtocolError("INVALID_MESSAGE", f"{field} must be UUID") from exc


def normalize_timestamp(value: Any) -> datetime:
    if isinstance(value, datetime):
        parsed = value
    else:
        parsed = parse_datetime(str(value))
    if parsed is None:
        raise ProtocolError("INVALID_MESSAGE", f"invalid timestamp: {value}")
    if timezone.is_naive(parsed):
        raise ProtocolError("INVALID_MESSAGE", "timestamp must include timezone")
    return parsed


@dataclass(frozen=True)
class MessageEnvelope:
    protocol_version: str
    message_id: uuid.UUID
    message_type: str
    robot_id: str
    session_id: str
    sent_at: datetime
    trace_id: uuid.UUID
    sequence: int | None
    payload: dict[str, Any]
    raw: dict[str, Any]


def parse_message(raw: bytes | str | dict[str, Any]) -> MessageEnvelope:
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
    if isinstance(raw, str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ProtocolError("INVALID_MESSAGE", "message is not valid JSON") from exc
    elif isinstance(raw, dict):
        data = raw
    else:
        raise ProtocolError("INVALID_MESSAGE", "message must be JSON object")

    version = str(_required(data, "protocol_version"))
    if version != PROTOCOL_VERSION:
        raise ProtocolError("UNSUPPORTED_PROTOCOL_VERSION", f"unsupported protocol version: {version}")

    message_type = str(_required(data, "message_type"))
    if message_type not in ALL_MESSAGE_TYPES:
        raise ProtocolError("INVALID_MESSAGE", f"unsupported message_type: {message_type}")

    robot_id = str(_required(data, "robot_id")).strip()
    if not robot_id:
        raise ProtocolError("INVALID_MESSAGE", "robot_id must not be empty")

    sequence = data.get("sequence")
    if sequence is not None:
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise ProtocolError("INVALID_MESSAGE", "sequence must be a non-negative integer")

    payload = _required(data, "payload")
    if not isinstance(payload, dict):
        raise ProtocolError("INVALID_MESSAGE", "payload must be an object")

    session_id = str(_required(data, "session_id"))
    if message_type in UPLINK_MESSAGE_TYPES:
        session_id = str(_uuid(session_id, "session_id"))

    envelope = MessageEnvelope(
        protocol_version=version,
        message_id=_uuid(_required(data, "message_id"), "message_id"),
        message_type=message_type,
        robot_id=robot_id,
        session_id=session_id,
        sent_at=normalize_timestamp(_required(data, "sent_at")),
        trace_id=_uuid(_required(data, "trace_id"), "trace_id"),
        sequence=sequence,
        payload=payload,
        raw=data,
    )
    validate_payload(envelope)
    return envelope


def validate_payload(envelope: MessageEnvelope) -> None:
    payload = envelope.payload
    if envelope.message_type in COMMAND_TYPES:
        _uuid(_required(payload, "command_id"), "command_id")
        normalize_timestamp(_required(payload, "issued_at"))
        expires_at = normalize_timestamp(_required(payload, "expires_at"))
        if expires_at <= normalize_timestamp(payload["issued_at"]):
            raise ProtocolError("INVALID_MESSAGE", "expires_at must be after issued_at")
        if envelope.message_type.startswith("task."):
            _uuid(_required(payload, "task_execution_id"), "task_execution_id")
        command = _required(payload, "command")
        if not isinstance(command, dict):
            raise ProtocolError("INVALID_MESSAGE", "command must be an object")
        if envelope.message_type == "task.start":
            _validate_task_start(command)
        elif envelope.message_type == "diagnostics.nav_rosbag_stop":
            _uuid(_required(command, "loop_session_id"), "loop_session_id")
        elif envelope.message_type == "nav.single_goal":
            _validate_nav_single_goal(command)
    elif envelope.message_type == "command.ack":
        _uuid(_required(payload, "command_id"), "command_id")
        if _required(payload, "ack") not in {"accepted", "rejected"}:
            raise ProtocolError("INVALID_MESSAGE", "ack must be accepted or rejected")
    elif envelope.message_type == "command.progress":
        _uuid(_required(payload, "command_id"), "command_id")
        result = payload.get("result")
        if result is not None and not isinstance(result, dict):
            raise ProtocolError("INVALID_MESSAGE", "command progress result must be an object")
    elif envelope.message_type == "command.result":
        _uuid(_required(payload, "command_id"), "command_id")
        if _required(payload, "status") not in {"succeeded", "failed", "cancelled", "timed_out"}:
            raise ProtocolError("INVALID_MESSAGE", "invalid command result status")
    elif envelope.message_type == "trajectory.batch":
        _validate_trajectory_batch(payload)
    elif envelope.message_type == "alert.event":
        _uuid(_required(payload, "event_id"), "event_id")
        normalize_timestamp(_required(payload, "occurred_at"))
        _required(payload, "event_type")
        _required(payload, "severity")
    elif envelope.message_type == "system.log.batch":
        entries = _required(payload, "entries")
        if not isinstance(entries, list) or len(entries) > 100:
            raise ProtocolError("INVALID_MESSAGE", "system.log.batch entries must contain at most 100 items")
        for entry in entries:
            if not isinstance(entry, dict):
                raise ProtocolError("INVALID_MESSAGE", "system log entry must be an object")
            if str(_required(entry, "level")).upper() not in {"DEBUG", "INFO", "WARNING", "ERROR"}:
                raise ProtocolError("INVALID_MESSAGE", "invalid system log level")
            if str(_required(entry, "module")) not in {
                "localization", "navigation", "avoidance", "relocalization",
                "waypoint", "planner", "boundary", "system",
            }:
                raise ProtocolError("INVALID_MESSAGE", "invalid system log module")
            _required(entry, "event_code")
            _required(entry, "message")


def _validate_task_start(command: dict[str, Any]) -> None:
    route = _required(command, "route_snapshot")
    if not isinstance(route, dict):
        raise ProtocolError("INVALID_MESSAGE", "route_snapshot must be an object")
    waypoints = _required(route, "waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise ProtocolError("INVALID_MESSAGE", "task.start requires at least one waypoint")
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            raise ProtocolError("INVALID_MESSAGE", "waypoint must be an object")
        if waypoint.get("sequence") != index:
            raise ProtocolError("INVALID_MESSAGE", "waypoint sequence must start at 0 and be contiguous")
        for coordinate in ("x", "y", "yaw"):
            if isinstance(waypoint.get(coordinate), bool) or not isinstance(waypoint.get(coordinate), (int, float)):
                raise ProtocolError("INVALID_MESSAGE", f"waypoint {coordinate} must be numeric")
        for field in ("avoidance_to_next", "require_yaw", "detour_enabled", "collision_slowdown_enabled", "collision_stop_enabled", "force_localization_correction"):
            if field in waypoint and not isinstance(waypoint[field], bool):
                raise ProtocolError("INVALID_MESSAGE", f"waypoint {field} must be boolean")
        if "arrival_policy" in waypoint and str(waypoint["arrival_policy"]).lower() not in {
            "pass_through", "stop_and_confirm", "precision", "dock",
        }:
            raise ProtocolError("INVALID_MESSAGE", "waypoint arrival_policy is invalid")
        arrival_policy = str(waypoint.get("arrival_policy") or "stop_and_confirm").lower()
        if "arrival_micro_adjust_mode" in waypoint:
            mode = str(waypoint["arrival_micro_adjust_mode"]).lower()
            if mode not in {"cmd_vel", "nav2_goal"}:
                raise ProtocolError("INVALID_MESSAGE", "waypoint arrival_micro_adjust_mode is invalid")
            if mode == "nav2_goal" and arrival_policy in {"pass_through", "dock"}:
                raise ProtocolError("INVALID_MESSAGE", "nav2_goal is not allowed for pass_through or dock")
        if "localization_anchor_preference" in waypoint and str(
            waypoint["localization_anchor_preference"]
        ).lower() not in {"ndt", "rtk", "balanced"}:
            raise ProtocolError("INVALID_MESSAGE", "waypoint localization_anchor_preference is invalid")
        if "rtk_primary_allowed" in waypoint and not isinstance(
            waypoint["rtk_primary_allowed"], bool
        ):
            raise ProtocolError("INVALID_MESSAGE", "waypoint rtk_primary_allowed must be boolean")
        if "speech_mode" in waypoint and str(waypoint["speech_mode"]).lower() not in {
            "blocking", "non_blocking", "disabled",
        }:
            raise ProtocolError("INVALID_MESSAGE", "waypoint speech_mode is invalid")
        if "local_controller" in waypoint:
            mode = str(waypoint["local_controller"]).lower()
            if mode not in {"rpp", "mppi", "ilqr"}:
                raise ProtocolError("INVALID_MESSAGE", "waypoint local_controller must be mppi, rpp or ilqr")
        if "navigation_speed_level" in waypoint and str(
            waypoint["navigation_speed_level"]
        ).lower() not in {"micro", "low", "medium", "high"}:
            raise ProtocolError(
                "INVALID_MESSAGE",
                "waypoint navigation_speed_level must be micro, low, medium or high",
            )
        if "global_controller" in waypoint:
            mode = str(waypoint["global_controller"]).lower()
            if mode not in {"theta_star", "navfn", "smac_hybrid"}:
                raise ProtocolError("INVALID_MESSAGE", "waypoint global_controller must be theta_star, navfn or smac_hybrid")
        if "dwell_seconds" in waypoint:
            dwell_seconds = waypoint["dwell_seconds"]
            if isinstance(dwell_seconds, bool) or not isinstance(dwell_seconds, (int, float)) or not 0 <= dwell_seconds <= 3600:
                raise ProtocolError("INVALID_MESSAGE", "waypoint dwell_seconds must be between 0 and 3600")
        actions = waypoint.get("actions", [])
        if not isinstance(actions, list) or any(action != "snapshot" for action in actions):
            raise ProtocolError("INVALID_MESSAGE", "P0 waypoint actions only support snapshot")
    global_controller = str(route.get("global_controller") or "theta_star").lower()
    if global_controller not in {"theta_star", "navfn", "smac_hybrid"}:
        raise ProtocolError("INVALID_MESSAGE", "route_snapshot global_controller must be theta_star, navfn or smac_hybrid")
    record_rosbag = command.get("record_rosbag")
    if record_rosbag is not None and not isinstance(record_rosbag, bool):
        raise ProtocolError("INVALID_MESSAGE", "task.start record_rosbag must be boolean")
    continuous_rosbag = command.get("continuous_rosbag", False)
    if not isinstance(continuous_rosbag, bool):
        raise ProtocolError("INVALID_MESSAGE", "task.start continuous_rosbag must be boolean")
    if continuous_rosbag:
        if not record_rosbag:
            raise ProtocolError("INVALID_MESSAGE", "continuous_rosbag requires record_rosbag")
        _uuid(_required(command, "loop_session_id"), "loop_session_id")


def _validate_nav_single_goal(command: dict[str, Any]) -> None:
    for coordinate in ("x", "y", "yaw"):
        value = command.get(coordinate)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError("INVALID_MESSAGE", f"nav.single_goal {coordinate} must be numeric")
    global_controller = str(command.get("global_controller") or "theta_star").lower()
    if global_controller not in {"theta_star", "navfn", "smac_hybrid"}:
        raise ProtocolError(
            "INVALID_MESSAGE",
            "nav.single_goal global_controller must be theta_star, navfn or smac_hybrid",
        )


def _validate_trajectory_batch(payload: dict[str, Any]) -> None:
    _uuid(_required(payload, "task_execution_id"), "task_execution_id")
    _uuid(_required(payload, "batch_id"), "batch_id")
    points = _required(payload, "points")
    if not isinstance(points, list) or not 1 <= len(points) <= 100:
        raise ProtocolError("INVALID_MESSAGE", "trajectory points must contain 1..100 items")
    sequences = []
    for point in points:
        if not isinstance(point, dict):
            raise ProtocolError("INVALID_MESSAGE", "trajectory point must be an object")
        seq = _required(point, "seq")
        if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
            raise ProtocolError("INVALID_MESSAGE", "trajectory seq must be non-negative integer")
        sequences.append(seq)
        normalize_timestamp(_required(point, "sampled_at"))
        for coordinate in ("x", "y", "yaw"):
            if isinstance(point.get(coordinate), bool) or not isinstance(point.get(coordinate), (int, float)):
                raise ProtocolError("INVALID_MESSAGE", f"trajectory {coordinate} must be numeric")
        keyframe = point.get("keyframe")
        if keyframe is not None and not isinstance(keyframe, dict):
            raise ProtocolError("INVALID_MESSAGE", "trajectory keyframe must be an object")
    if sequences != list(range(sequences[0], sequences[0] + len(sequences))):
        raise ProtocolError("INVALID_MESSAGE", "trajectory seq must be contiguous")
    if payload.get("first_seq") != sequences[0] or payload.get("last_seq") != sequences[-1]:
        raise ProtocolError("INVALID_MESSAGE", "trajectory batch range does not match points")


def build_command_message(command: Any, *, session_id: str = "") -> dict[str, Any]:
    task_execution_id = str(command.task_execution_id) if command.task_execution_id else None
    payload = {
        "command_id": str(command.id),
        "issued_at": command.issued_at.isoformat(timespec="milliseconds"),
        "expires_at": command.expires_at.isoformat(timespec="milliseconds"),
        "operator_id": str(command.operator_id) if command.operator_id else None,
        "task_execution_id": task_execution_id,
        "command": command.payload,
    }
    if task_execution_id:
        payload["expected_robot_state_version"] = command.robot.last_state_version
    return {
        "protocol_version": PROTOCOL_VERSION,
        "message_id": str(uuid.uuid4()),
        "message_type": command.command_type,
        "robot_id": command.robot.code,
        "session_id": session_id or str(uuid.uuid4()),
        "sent_at": timezone.now().isoformat(timespec="milliseconds"),
        "trace_id": str(command.trace_id),
        "payload": payload,
    }
