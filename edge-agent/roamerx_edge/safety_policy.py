from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from .config import SafetyConfig
from .protocol import MessageEnvelope, ProtocolError

LOGGER = logging.getLogger(__name__)


@dataclass
class RuntimeSafetyState:
    localization_status: str = "unknown"
    localization_normal_since_monotonic: float = 0.0
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

    def wait_until_localization_stable(self) -> None:
        """Wait out a fresh localization-normal window instead of rejecting start.

        Relocalization resets the normal timer. Operators then start the task
        within a second; failing immediately with LOCALIZATION_NOT_STABLE is
        worse than blocking the command thread for the remaining 3s.
        """
        required = float(self.config.localization_stable_seconds)
        if required <= 0.0 or self.state.localization_status != "normal":
            return
        started = self.state.localization_normal_since_monotonic
        if started <= 0.0:
            return
        remaining = required - (time.monotonic() - started)
        if remaining <= 0.0:
            return
        LOGGER.info(
            "waiting %.1fs for localization to stay normal before task start",
            remaining,
        )
        deadline = time.monotonic() + remaining
        while time.monotonic() < deadline:
            if self.state.localization_status != "normal":
                return
            started = self.state.localization_normal_since_monotonic
            if started <= 0.0:
                return
            still_needed = required - (time.monotonic() - started)
            if still_needed <= 0.0:
                return
            time.sleep(min(0.2, still_needed))

    def validate_task_start(self, envelope: MessageEnvelope, has_active_task: bool) -> None:
        if has_active_task:
            raise ProtocolError("ROBOT_BUSY", "another motion task is active")
        smart_initialize = bool((envelope.payload.get("command") or {}).get("smart_initialize", True))
        if self.state.localization_status != "normal" and not smart_initialize:
            raise ProtocolError("LOCALIZATION_NOT_READY", self.state.localization_status)
        stable_for = time.monotonic() - self.state.localization_normal_since_monotonic
        if stable_for < self.config.localization_stable_seconds:
            raise ProtocolError(
                "LOCALIZATION_NOT_STABLE",
                f"normal for {max(0.0, stable_for):.1f}s; "
                f"requires {self.config.localization_stable_seconds:.1f}s",
            )
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
            docking = envelope.payload["command"].get("docking") or {}
            if not bool(docking.get("enabled")):
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

    def validate_manual_assist(self) -> None:
        """Allow bounded assist only while all hard safety interlocks are clear."""
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

    @staticmethod
    def validate_pause(state: str) -> None:
        if state not in {"accepted", "running", "pausing", "paused", "resuming", "interrupted"}:
            raise ProtocolError("INVALID_TASK_STATE", f"cannot pause from {state}")

    @staticmethod
    def validate_resume(state: str) -> None:
        if state not in {"accepted", "running", "pausing", "paused", "resuming", "interrupted"}:
            raise ProtocolError("INVALID_TASK_STATE", f"cannot resume from {state}")

    @staticmethod
    def validate_cancel(state: str) -> None:
        if state not in {
            "running", "paused", "pausing", "resuming", "interrupted",
            "completed", "failed", "cancelled", "timed_out", "rejected",
        }:
            raise ProtocolError("INVALID_TASK_STATE", f"cannot cancel from {state}")
