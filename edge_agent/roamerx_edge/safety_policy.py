from __future__ import annotations

from dataclasses import dataclass

from .config import SafetyConfig
from .protocol import MessageEnvelope, ProtocolError


@dataclass
class RuntimeSafetyState:
    localization_status: str = "unknown"
    nav_ready: bool = False
    emergency_stop: bool = False
    control_mode: str = "unknown"
    power_available: bool = False
    battery_percent: int | None = None
    current_map_id: str = ""
    current_map_version: str = ""
    current_map_local_state: str = "applied"
    current_map_error: str = ""


class SafetyPolicy:
    def __init__(self, config: SafetyConfig, state: RuntimeSafetyState) -> None:
        self.config = config
        self.state = state

    def validate_task_start(self, envelope: MessageEnvelope, has_active_task: bool) -> None:
        if has_active_task:
            raise ProtocolError("ROBOT_BUSY", "another motion task is active")
        if self.state.localization_status != "normal":
            raise ProtocolError("LOCALIZATION_NOT_READY", self.state.localization_status)
        if not self.state.nav_ready:
            raise ProtocolError("NAV_STACK_NOT_READY", "FollowWaypoints action server is unavailable")
        if self.state.emergency_stop:
            raise ProtocolError("EMERGENCY_STOP_ACTIVE", "emergency stop is active")
        if self.state.control_mode == "manual_takeover":
            raise ProtocolError("MANUAL_TAKEOVER_ACTIVE", "manual takeover is active")
        if (
            self.state.power_available
            and self.state.battery_percent is not None
            and self.state.battery_percent < self.config.low_battery_percent
        ):
            raise ProtocolError("LOW_BATTERY", f"battery={self.state.battery_percent}")
        required_map = envelope.payload["command"].get("map") or {}
        map_set = (envelope.payload["command"].get("route_snapshot") or {}).get("map_set") or {}
        if self.state.current_map_local_state not in {"applied", ""}:
            raise ProtocolError(
                "MAP_LOCAL_MISMATCH",
                self.state.current_map_error or self.state.current_map_local_state,
            )
        if map_set:
            allowed = {
                (str(item.get("map_id") or ""), str(item.get("map_version") or ""))
                for item in map_set.get("submaps") or []
            }
            if (self.state.current_map_id, self.state.current_map_version) in allowed:
                return
            # The task executor activates the first local submap before it
            # sends any motion goal. The currently active overview map is
            # therefore valid at command admission time.
            return
        if (required_map.get("map_id") != self.state.current_map_id or required_map.get("map_version") != self.state.current_map_version):
            raise ProtocolError("MAP_VERSION_MISMATCH", "current map does not match task")

    @staticmethod
    def validate_pause(state: str) -> None:
        if state != "running":
            raise ProtocolError("INVALID_TASK_STATE", f"cannot pause from {state}")

    @staticmethod
    def validate_resume(state: str) -> None:
        if state != "paused":
            raise ProtocolError("INVALID_TASK_STATE", f"cannot resume from {state}")

    @staticmethod
    def validate_cancel(state: str) -> None:
        if state not in {"running", "paused", "pausing", "resuming", "interrupted"}:
            raise ProtocolError("INVALID_TASK_STATE", f"cannot cancel from {state}")
