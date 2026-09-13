from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any


PROTOCOL_VERSION = "1.0"
TASK_COMMAND_TYPES = {
    "task.start", "task.pause", "task.resume", "task.resume_forward",
    "task.cancel", "task.force_exit", "task.recover.v1",
}
MAPPING_COMMAND_TYPES = {
    "mapping.start", "mapping.save", "mapping.cancel", "mapping.status",
    "mapping.origin_start", "mapping.origin_cancel", "mapping.origin_extract_global", "mapping.slam_start", "mapping.begin",
}
NAV_COMMAND_TYPES = {
    "nav.status", "nav.start", "nav.restart", "nav.recover", "nav.stop",
    "nav.initial_pose", "nav.relocalize", "nav.single_goal",
}
MAP_COMMAND_TYPES = {"map.activate", "map.optimize", "map.boundary_apply"}
DIAGNOSTICS_COMMAND_TYPES = {
    "diagnostics.log_config",
    "diagnostics.nav_rosbag_stop",
}
SENSOR_COMMAND_TYPES = {"sensor.restart"}
CHARGE_COMMAND_TYPES = {"charge.start", "charge.stop"}
MOTION_CONTROL_COMMAND_TYPES = {"motion.start", "motion.stop"}
AUDIO_COMMAND_TYPES = {"audio.volume"}
TELEOP_COMMAND_TYPES = {
    "teleop.takeover_enter",
    "teleop.takeover_exit",
    "teleop.stand_up",
    "teleop.lie_down",
    "teleop.shake_hand",
    "teleop.two_leg_stand",
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
COMMAND_TYPES = (
    TASK_COMMAND_TYPES
    | MAPPING_COMMAND_TYPES
    | NAV_COMMAND_TYPES
    | MAP_COMMAND_TYPES
    | DIAGNOSTICS_COMMAND_TYPES
    | SENSOR_COMMAND_TYPES
    | CHARGE_COMMAND_TYPES
    | MOTION_CONTROL_COMMAND_TYPES
    | AUDIO_COMMAND_TYPES
    | TELEOP_COMMAND_TYPES
)


class ProtocolError(ValueError):
    def __init__(self, code: str, message: str, details: Any = None) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.details = details


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def parse_time(value: Any) -> datetime:
    try:
        parsed = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    except ValueError as exc:
        raise ProtocolError("INVALID_MESSAGE", f"invalid timestamp: {value}") from exc
    if parsed.tzinfo is None:
        raise ProtocolError("INVALID_MESSAGE", "timestamp must include timezone")
    return parsed


def parse_uuid(value: Any, field: str) -> str:
    try:
        return str(uuid.UUID(str(value)))
    except (ValueError, TypeError) as exc:
        raise ProtocolError("INVALID_MESSAGE", f"{field} must be UUID") from exc


@dataclass(frozen=True)
class MessageEnvelope:
    protocol_version: str
    message_id: str
    message_type: str
    robot_id: str
    session_id: str
    sent_at: datetime
    trace_id: str
    sequence: int | None
    payload: dict[str, Any]
    raw: dict[str, Any]


def decode_message(raw: bytes | str | dict[str, Any]) -> MessageEnvelope:
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
    if isinstance(raw, str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ProtocolError("INVALID_MESSAGE", "invalid JSON") from exc
    elif isinstance(raw, dict):
        data = raw
    else:
        raise ProtocolError("INVALID_MESSAGE", "message must be JSON object")
    for key in ("protocol_version", "message_id", "message_type", "robot_id", "session_id", "sent_at", "trace_id", "payload"):
        if key not in data or data[key] is None:
            raise ProtocolError("INVALID_MESSAGE", f"missing required field: {key}")
    if data["protocol_version"] != PROTOCOL_VERSION:
        raise ProtocolError("UNSUPPORTED_PROTOCOL_VERSION", str(data["protocol_version"]))
    if data["message_type"] not in COMMAND_TYPES | {"sync.response", "trajectory.ack"}:
        raise ProtocolError("INVALID_MESSAGE", f"unsupported message_type: {data['message_type']}")
    if not isinstance(data["payload"], dict):
        raise ProtocolError("INVALID_MESSAGE", "payload must be object")
    envelope = MessageEnvelope(
        protocol_version=data["protocol_version"],
        message_id=parse_uuid(data["message_id"], "message_id"),
        message_type=data["message_type"],
        robot_id=str(data["robot_id"]),
        session_id=str(data["session_id"]),
        sent_at=parse_time(data["sent_at"]),
        trace_id=parse_uuid(data["trace_id"], "trace_id"),
        sequence=data.get("sequence"),
        payload=data["payload"],
        raw=data,
    )
    if envelope.message_type in COMMAND_TYPES:
        validate_command(envelope)
    return envelope


def validate_command(envelope: MessageEnvelope) -> None:
    payload = envelope.payload
    for key in ("command_id", "issued_at", "expires_at", "command"):
        if key not in payload or payload[key] is None:
            raise ProtocolError("INVALID_MESSAGE", f"missing command field: {key}")
    parse_uuid(payload["command_id"], "command_id")
    if envelope.message_type in TASK_COMMAND_TYPES:
        if not payload.get("task_execution_id"):
            raise ProtocolError("INVALID_MESSAGE", "missing command field: task_execution_id")
        parse_uuid(payload["task_execution_id"], "task_execution_id")
    issued = parse_time(payload["issued_at"])
    expires = parse_time(payload["expires_at"])
    if expires <= issued:
        raise ProtocolError("INVALID_MESSAGE", "expires_at must be after issued_at")
    if not isinstance(payload["command"], dict):
        raise ProtocolError("INVALID_MESSAGE", "command must be object")
    if envelope.message_type == "task.start":
        route = payload["command"].get("route_snapshot")
        waypoints = route.get("waypoints") if isinstance(route, dict) else None
        if not isinstance(waypoints, list) or not waypoints:
            raise ProtocolError("INVALID_MESSAGE", "task.start requires waypoints")
        for index, waypoint in enumerate(waypoints):
            if waypoint.get("sequence") != index:
                raise ProtocolError("INVALID_MESSAGE", "waypoint sequence must be contiguous")
            for field in ("x", "y", "yaw"):
                if isinstance(waypoint.get(field), bool) or not isinstance(waypoint.get(field), (int, float)):
                    raise ProtocolError("INVALID_MESSAGE", f"waypoint {field} must be numeric")
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
                if mode not in {"mppi", "rpp", "ilqr"}:
                    raise ProtocolError(
                        "INVALID_MESSAGE",
                        "waypoint local_controller must be mppi, rpp or ilqr",
                    )
            if "global_controller" in waypoint:
                mode = str(waypoint["global_controller"]).lower()
                if mode not in {"theta_star", "navfn", "smac_hybrid"}:
                    raise ProtocolError(
                        "INVALID_MESSAGE",
                        "waypoint global_controller must be theta_star, navfn or smac_hybrid",
                    )
            if "dwell_seconds" in waypoint:
                dwell_seconds = waypoint["dwell_seconds"]
                if isinstance(dwell_seconds, bool) or not isinstance(dwell_seconds, (int, float)) or not 0 <= dwell_seconds <= 3600:
                    raise ProtocolError("INVALID_MESSAGE", "waypoint dwell_seconds must be between 0 and 3600")
        route_global_controller = str(route.get("global_controller") or "theta_star").lower()
        if route_global_controller not in {"theta_star", "navfn", "smac_hybrid"}:
            raise ProtocolError(
                "INVALID_MESSAGE",
                "route global_controller must be theta_star, navfn or smac_hybrid",
            )
        record_rosbag = payload["command"].get("record_rosbag")
        if record_rosbag is not None and not isinstance(record_rosbag, bool):
            raise ProtocolError("INVALID_MESSAGE", "task.start record_rosbag must be boolean")
        continuous_rosbag = payload["command"].get("continuous_rosbag", False)
        if not isinstance(continuous_rosbag, bool):
            raise ProtocolError("INVALID_MESSAGE", "task.start continuous_rosbag must be boolean")
        if continuous_rosbag:
            if not record_rosbag:
                raise ProtocolError(
                    "INVALID_MESSAGE",
                    "continuous_rosbag requires record_rosbag",
                )
            parse_uuid(payload["command"].get("loop_session_id"), "loop_session_id")
    if envelope.message_type == "diagnostics.nav_rosbag_stop":
        parse_uuid(payload["command"].get("loop_session_id"), "loop_session_id")
    if envelope.message_type in {"mapping.start", "mapping.origin_start", "mapping.slam_start"}:
        map_name = payload["command"].get("map_name")
        if map_name is not None and not isinstance(map_name, str):
            raise ProtocolError("INVALID_MESSAGE", f"{envelope.message_type} map_name must be string")
        record_rosbag = payload["command"].get("record_rosbag")
        if record_rosbag is not None and not isinstance(record_rosbag, bool):
            raise ProtocolError("INVALID_MESSAGE", "mapping.start record_rosbag must be boolean")
        mapping_type = payload["command"].get("mapping_type")
        if mapping_type is not None and mapping_type not in {"indoor", "outdoor"}:
            raise ProtocolError("INVALID_MESSAGE", f"{envelope.message_type} mapping_type must be indoor or outdoor")
        scene_scope = payload["command"].get("scene_scope")
        if scene_scope is not None and scene_scope not in {"indoor", "transition", "outdoor"}:
            raise ProtocolError("INVALID_MESSAGE", f"{envelope.message_type} scene_scope must be indoor, transition, or outdoor")
    if envelope.message_type == "nav.initial_pose":
        command = payload["command"]
        supplied = [field for field in ("x", "y", "yaw") if command.get(field) is not None]
        seed_source = str(command.get("seed_source") or "").strip()
        if not supplied and seed_source not in {"mapping_start", "last_trusted", "rtk"}:
            raise ProtocolError(
                "INVALID_MESSAGE",
                "nav.initial_pose requires x, y and yaw or a supported seed_source",
            )
        if supplied and len(supplied) != 3:
            raise ProtocolError("INVALID_MESSAGE", "nav.initial_pose requires x, y and yaw together")
        for field in supplied:
            value = command.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ProtocolError("INVALID_MESSAGE", f"nav.initial_pose {field} must be numeric")
    if envelope.message_type == "nav.single_goal":
        command = payload["command"]
        for field in ("x", "y", "yaw"):
            value = command.get(field)
            if value is None or isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ProtocolError("INVALID_MESSAGE", f"nav.single_goal {field} must be numeric")
        global_controller = str(command.get("global_controller") or "theta_star").lower()
        if global_controller not in {"theta_star", "navfn", "smac_hybrid"}:
            raise ProtocolError(
                "INVALID_MESSAGE",
                "nav.single_goal global_controller must be theta_star, navfn or smac_hybrid",
            )
    if envelope.message_type == "nav.relocalize":
        command = payload["command"]
        supplied = [field for field in ("x", "y", "yaw") if command.get(field) is not None]
        seed_source = str(command.get("seed_source") or "last_trusted").strip()
        if not supplied and seed_source not in {
            "mapping_start", "last_trusted", "global", "progressive", "quick_then_global",
        }:
            raise ProtocolError(
                "INVALID_MESSAGE",
                "nav.relocalize requires x, y and yaw or a supported seed_source",
            )
        if supplied and len(supplied) != 3:
            raise ProtocolError("INVALID_MESSAGE", "nav.relocalize requires x, y and yaw together")
        for field in supplied:
            value = command.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ProtocolError("INVALID_MESSAGE", f"nav.relocalize {field} must be numeric")
        if seed_source == "progressive":
            waypoints = command.get("waypoints") or []
            if not isinstance(waypoints, list):
                raise ProtocolError("INVALID_MESSAGE", "nav.relocalize waypoints must be a list")
            for index, waypoint in enumerate(waypoints):
                if not isinstance(waypoint, dict):
                    raise ProtocolError(
                        "INVALID_MESSAGE", f"nav.relocalize waypoint {index} must be an object"
                    )
                for field in ("x", "y", "yaw"):
                    value = waypoint.get(field)
                    if isinstance(value, bool) or not isinstance(value, (int, float)):
                        raise ProtocolError(
                            "INVALID_MESSAGE",
                            f"nav.relocalize waypoint {index} {field} must be numeric",
                        )
    if envelope.message_type == "map.activate":
        command = payload["command"]
        for field in ("map_id", "map_version"):
            if not str(command.get(field, "")).strip():
                raise ProtocolError("INVALID_MESSAGE", f"map.activate requires {field}")
    if envelope.message_type == "map.optimize":
        command = payload["command"]
        if not str(command.get("source_map_id") or command.get("map_id") or "").strip():
            raise ProtocolError("INVALID_MESSAGE", "map.optimize requires source_map_id")
        selected = command.get("selected_candidates")
        if not isinstance(selected, list) or not selected:
            raise ProtocolError("INVALID_MESSAGE", "map.optimize requires selected_candidates")
    if envelope.message_type == "map.boundary_apply":
        command = payload["command"]
        boundary = command.get("boundary")
        if not str(command.get("map_id") or "").strip() or not isinstance(boundary, dict):
            raise ProtocolError("INVALID_MESSAGE", "map.boundary_apply requires map_id and boundary")
        if not isinstance(boundary.get("revision"), int) or boundary["revision"] <= 0:
            raise ProtocolError("INVALID_MESSAGE", "map.boundary_apply revision must be positive")
        if not isinstance(boundary.get("outer_polygon"), list) or len(boundary["outer_polygon"]) < 3:
            raise ProtocolError("INVALID_MESSAGE", "map.boundary_apply requires an outer polygon")
    if envelope.message_type == "diagnostics.log_config":
        command = payload["command"]
        if not isinstance(command.get("enabled"), bool):
            raise ProtocolError("INVALID_MESSAGE", "diagnostics.log_config enabled must be boolean")
        if command.get("enabled"):
            modules = command.get("modules")
            if not isinstance(modules, list) or not modules:
                raise ProtocolError("INVALID_MESSAGE", "diagnostics.log_config modules must be a list")
            sample_hz = command.get("sample_hz")
            if isinstance(sample_hz, bool) or not isinstance(sample_hz, (int, float)) or not 0.1 <= sample_hz <= 5:
                raise ProtocolError("INVALID_MESSAGE", "diagnostics.log_config sample_hz must be between 0.1 and 5")
    if envelope.message_type == "sensor.restart":
        sensor = str(payload["command"].get("sensor") or "").strip().lower()
        if sensor not in {"lidar", "imu", "lidar_imu", "rtk"}:
            raise ProtocolError("INVALID_MESSAGE", "sensor.restart requires lidar_imu or rtk")
    if envelope.message_type == "audio.volume":
        command = payload["command"]
        if str(command.get("target") or "") not in {"speaker_3588", "speaker_nx"}:
            raise ProtocolError("INVALID_MESSAGE", "audio.volume requires a valid target")
        volume = command.get("volume")
        if isinstance(volume, bool) or not isinstance(volume, (int, float)) or not 0 <= volume <= 100:
            raise ProtocolError("INVALID_MESSAGE", "audio.volume must be between 0 and 100")


def encode_message(message: dict[str, Any]) -> str:
    return json.dumps(message, ensure_ascii=False, separators=(",", ":"), default=str)


def build_envelope(
    *,
    message_type: str,
    robot_id: str,
    session_id: str,
    trace_id: str | None = None,
    sequence: int | None = None,
    payload: dict[str, Any],
) -> dict[str, Any]:
    result = {
        "protocol_version": PROTOCOL_VERSION,
        "message_id": str(uuid.uuid4()),
        "message_type": message_type,
        "robot_id": robot_id,
        "session_id": session_id,
        "sent_at": now_iso(),
        "trace_id": trace_id or str(uuid.uuid4()),
        "payload": payload,
    }
    if sequence is not None:
        result["sequence"] = sequence
    return result


def build_ack(envelope: MessageEnvelope, *, accepted: bool, edge_state_version: int, code: str = "", message: str = "", duplicate: bool = False) -> dict:
    return build_envelope(
        message_type="command.ack",
        robot_id=envelope.robot_id,
        session_id=envelope.session_id,
        trace_id=envelope.trace_id,
        payload={
            "command_id": envelope.payload["command_id"],
            "task_execution_id": envelope.payload.get("task_execution_id"),
            "ack": "accepted" if accepted else "rejected",
            "acknowledged_at": now_iso(),
            "reason_code": code or None,
            "reason_message": message or None,
            "duplicate": duplicate,
            "edge_state_version": edge_state_version,
        },
    )


def build_result(
    envelope: MessageEnvelope,
    *,
    status: str,
    result: dict,
    started_at: str,
    error_code: str = "",
    error_message: str = "",
) -> dict:
    return build_envelope(
        message_type="command.result",
        robot_id=envelope.robot_id,
        session_id=envelope.session_id,
        trace_id=envelope.trace_id,
        payload={
            "command_id": envelope.payload["command_id"],
            "task_execution_id": envelope.payload.get("task_execution_id"),
            "status": status,
            "started_at": started_at,
            "finished_at": now_iso(),
            "error_code": error_code or None,
            "error_message": error_message or None,
            "result": result,
        },
    )

def build_progress(
    envelope: MessageEnvelope,
    *,
    result: dict,
    started_at: str,
) -> dict:
    return build_envelope(
        message_type="command.progress",
        robot_id=envelope.robot_id,
        session_id=envelope.session_id,
        trace_id=envelope.trace_id,
        payload={
            "command_id": envelope.payload["command_id"],
            "task_execution_id": envelope.payload.get("task_execution_id"),
            "status": "executing",
            "started_at": started_at,
            "result": result,
        },
    )
