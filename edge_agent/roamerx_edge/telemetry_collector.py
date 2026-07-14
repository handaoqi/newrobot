from __future__ import annotations

import threading
from dataclasses import dataclass

from .config import RobotConfig
from .protocol import now_iso
from .safety_policy import RuntimeSafetyState


LOCALIZATION_STATUS = {
    0: "initializing",
    1: "relocalizing",
    2: "relocalized",
    3: "normal",
    4: "lost",
}


@dataclass
class PoseSnapshot:
    sampled_at: str
    x: float
    y: float
    z: float
    yaw: float
    speed_mps: float
    localization_status: str
    source_status: int
    coord_type: int


@dataclass
class LocalizationQualitySnapshot:
    sampled_at: str
    has_converged: bool
    matching_error: float | None
    inlier_fraction: float | None
    relative_translation_m: float | None
    prediction_errors: list[dict]


class TelemetryCollector:
    def __init__(self, robot: RobotConfig, safety_state: RuntimeSafetyState) -> None:
        self.robot = robot
        self.safety_state = safety_state
        self._lock = threading.Lock()
        self._pose: PoseSnapshot | None = None
        self._localization_quality: LocalizationQualitySnapshot | None = None
        self._state_version = 0
        self.power_available = False
        self.battery_percent = None
        self.charging = None
        self.signal_percent = None
        self.network_type = ""

    def on_localization(self, msg) -> None:
        status = LOCALIZATION_STATUS.get(int(msg.status), "unknown")
        with self._lock:
            self._pose = PoseSnapshot(
                sampled_at=now_iso(),
                x=float(msg.pos.x),
                y=float(msg.pos.y),
                z=float(msg.pos.z),
                yaw=float(msg.rpy.z),
                speed_mps=float(msg.speed),
                localization_status=status,
                source_status=int(msg.status),
                coord_type=int(msg.coord_type),
            )
            self._state_version += 1
            self.safety_state.localization_status = status

    def on_scan_matching_status(self, msg) -> None:
        translation = getattr(getattr(msg, "relative_pose", None), "translation", None)
        relative_translation_m = None
        if translation is not None:
            relative_translation_m = (
                float(translation.x) ** 2 + float(translation.y) ** 2 + float(translation.z) ** 2
            ) ** 0.5
        labels = list(getattr(msg, "prediction_labels", []) or [])
        errors = list(getattr(msg, "prediction_errors", []) or [])
        prediction_errors = []
        for label, error in zip(labels, errors):
            error_translation = getattr(error, "translation", None)
            norm = None
            if error_translation is not None:
                norm = (
                    float(error_translation.x) ** 2
                    + float(error_translation.y) ** 2
                    + float(error_translation.z) ** 2
                ) ** 0.5
            prediction_errors.append(
                {
                    "label": getattr(label, "data", ""),
                    "translation_m": norm,
                }
            )
        with self._lock:
            self._localization_quality = LocalizationQualitySnapshot(
                sampled_at=now_iso(),
                has_converged=bool(getattr(msg, "has_converged", False)),
                matching_error=float(getattr(msg, "matching_error", 0.0)),
                inlier_fraction=float(getattr(msg, "inlier_fraction", 0.0)),
                relative_translation_m=relative_translation_m,
                prediction_errors=prediction_errors,
            )

    def on_battery(self, percentage: float, charging: bool) -> None:
        with self._lock:
            self.power_available = True
            self.battery_percent = max(0, min(100, round(percentage * 100 if percentage <= 1 else percentage)))
            self.charging = charging
            self.safety_state.power_available = True
            self.safety_state.battery_percent = self.battery_percent

    def latest_pose(self) -> PoseSnapshot | None:
        with self._lock:
            return self._pose

    def build_status_snapshot(self, task_execution_id: str | None = None) -> dict:
        with self._lock:
            pose = self._pose
            quality = self._localization_quality
            return {
                "sampled_at": pose.sampled_at if pose else now_iso(),
                "state_version": self._state_version,
                "pose": {
                    "frame_id": "map",
                    "x": pose.x if pose else None,
                    "y": pose.y if pose else None,
                    "z": pose.z if pose else None,
                    "yaw": pose.yaw if pose else None,
                    "speed_mps": pose.speed_mps if pose else None,
                },
                "localization": {
                    "status": pose.localization_status if pose else "unknown",
                    "source_status": pose.source_status if pose else None,
                    "coord_type": pose.coord_type if pose else None,
                    "quality": {
                        "sampled_at": quality.sampled_at,
                        "has_converged": quality.has_converged,
                        "matching_error": quality.matching_error,
                        "inlier_fraction": quality.inlier_fraction,
                        "relative_translation_m": quality.relative_translation_m,
                        "prediction_errors": quality.prediction_errors,
                    } if quality else None,
                },
                "power": {
                    "available": self.power_available,
                    "percent": self.battery_percent if self.power_available else None,
                    "charging": self.charging if self.power_available else None,
                },
                "network": {"type": self.network_type, "signal_percent": self.signal_percent},
                "runtime": {
                    "ros_ready": True,
                    "nav_ready": self.safety_state.nav_ready,
                    "emergency_stop": self.safety_state.emergency_stop,
                    "control_mode": self.safety_state.control_mode,
                    "task_execution_id": task_execution_id,
                },
                "current_map": {
                    "map_id": self.robot.current_map_id,
                    "map_version": self.robot.current_map_version,
                    "sha256": None,
                },
            }
