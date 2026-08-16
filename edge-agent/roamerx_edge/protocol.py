from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any


PROTOCOL_VERSION = "1.0"
TASK_COMMAND_TYPES = {"task.start", "task.pause", "task.resume", "task.cancel", "task.force_exit"}
MAPPING_COMMAND_TYPES = {"mapping.start", "mapping.save", "mapping.cancel", "mapping.status"}
NAV_COMMAND_TYPES = {
    "nav.status", "nav.start", "nav.restart", "nav.recover", "nav.stop",
    "nav.initial_pose", "nav.relocalize",
}
MAP_COMMAND_TYPES = {"map.activate"}
SENSOR_COMMAND_TYPES = {"sensor.restart"}
CHARGE_COMMAND_TYPES = {"charge.start", "charge.stop"}
MOTION_CONTROL_COMMAND_TYPES = {"motion.start", "motion.stop"}
AUDIO_COMMAND_TYPES = {"audio.volume"}
TELEOP_COMMAND_TYPES = {
    "teleop.takeover_enter",
    "teleop.takeover_exit",
    "teleop.stand_up",
    "teleop.lie_down",
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
    | SENSOR_COMMAND_TYPES
    | CHARGE_COMMAND_TYPES
    | MOTION_CONTROL_COMMAND_TYPES
    | AUDIO_COMMAND_TYPES
    | TELEOP_COMMAND_TYPES
)


class ProtocolError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


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
            for field in ("avoidance_to_next", "require_yaw"):
                if field in waypoint and not isinstance(waypoint[field], bool):
                    raise ProtocolError("INVALID_MESSAGE", f"waypoint {field} must be boolean")
        record_rosbag = payload["command"].get("record_rosbag")
        if record_rosbag is not None and not isinstance(record_rosbag, bool):
            raise ProtocolError("INVALID_MESSAGE", "task.start record_rosbag must be boolean")
    if envelope.message_type == "mapping.start":
        map_name = payload["command"].get("map_name")
        if map_name is not None and not isinstance(map_name, str):
            raise ProtocolError("INVALID_MESSAGE", "mapping.start map_name must be string")
        record_rosbag = payload["command"].get("record_rosbag")
        if record_rosbag is not None and not isinstance(record_rosbag, bool):
            raise ProtocolError("INVALID_MESSAGE", "mapping.start record_rosbag must be boolean")
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
    if envelope.message_type == "nav.relocalize":
        command = payload["command"]
        supplied = [field for field in ("x", "y", "yaw") if command.get(field) is not None]
        if supplied and len(supplied) != 3:
            raise ProtocolError("INVALID_MESSAGE", "nav.relocalize requires x, y and yaw together")
        for field in supplied:
            value = command.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ProtocolError("INVALID_MESSAGE", f"nav.relocalize {field} must be numeric")
    if envelope.message_type == "map.activate":
        command = payload["command"]
        for field in ("map_id", "map_version"):
            if not str(command.get(field, "")).strip():
                raise ProtocolError("INVALID_MESSAGE", f"map.activate requires {field}")
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
