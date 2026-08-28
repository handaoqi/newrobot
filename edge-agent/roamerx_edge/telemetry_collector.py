from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone

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

RTK_QUALITY_ALIASES = {
    "fixed": "fixed",
    "rtk_fixed": "fixed",
    "float": "float",
    "rtk_float": "float",
    "standalone": "standalone",
    "single": "standalone",
    "invalid": "invalid",
    "no_fix": "invalid",
}


def normalize_rtk_quality(value) -> str | None:
    if value is None or value == "":
        return None
    return RTK_QUALITY_ALIASES.get(str(value).strip().lower(), "invalid")


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
        self._localization_decision: dict = {}
        # Progress of an in-flight localization recovery. Empty when not recovering.
        self._localization_recovery: dict = {}
        # Latest lidar-IMU vs 3588-IMU comparison. Monitoring only.
        self._imu_cross_check: dict = {}
        self._storage: dict = {}
        self._raw_rtk: dict = {}
        self._state_version = 0
        self.power_available = False
        self.battery_percent = None
        self.charging = None
        self.signal_percent = None
        self.network_type = ""
        self._power_sampled_monotonic = 0.0
        self._network_sampled_monotonic = 0.0
        self._audio_sampled_monotonic = 0.0
        self._power_details: dict = {}
        self._network_details: dict = {}
        self._audio_details: dict = {}
        self._system_probe_stale_seconds = 30.0
        self._sensor_samples: dict[str, deque[float]] = {}
        self._sensor_details: dict[str, dict] = {}

    def on_sensor_message(self, name: str, **details) -> None:
        now = time.time()
        with self._lock:
            samples = self._sensor_samples.setdefault(name, deque(maxlen=1200))
            samples.append(now)
            current = dict(self._sensor_details.get(name) or {})
            current.update(details)
            current["received_at_unix"] = now
            self._sensor_details[name] = current
            if name == "rtk":
                self._raw_rtk = self._build_raw_rtk_locked(current, now)

    def update_sensor_details(self, name: str, **details) -> None:
        """Merge diagnostics without counting a second synthetic sensor sample."""
        if not details:
            return
        with self._lock:
            current = dict(self._sensor_details.get(name) or {})
            current.update(details)
            current["received_at_unix"] = time.time()
            self._sensor_details[name] = current
            if name == "rtk":
                self._raw_rtk = self._build_raw_rtk_locked(current, current["received_at_unix"])

    @staticmethod
    def _build_raw_rtk_locked(details: dict, now: float) -> dict:
        raw = dict(details)
        raw["quality"] = normalize_rtk_quality(details.get("quality"))
        raw["topic"] = details.get("topic") or "/fix"
        raw["sample_age_seconds"] = details.get("sample_age_seconds")
        raw["received_at"] = details.get("received_at_unix", now)
        heading = dict(details.get("heading") or {})
        heading_stamp = heading.get("measurement_stamp")
        if heading_stamp is not None:
            try:
                heading["sample_age_seconds"] = max(0.0, now - float(heading_stamp))
            except (TypeError, ValueError):
                heading["sample_age_seconds"] = None
        raw["heading"] = heading
        return raw

    def _time_diagnostics_locked(self, now: float) -> dict:
        names = ("lidar", "imu", "rtk", "odometry")
        values = {}
        for name in names:
            details = dict(self._sensor_details.get(name) or {})
            received = details.get("received_at_unix")
            stamp = details.get("measurement_stamp")
            age = None
            reported_age = details.get("sample_age_seconds")
            if reported_age is not None:
                try:
                    age = max(0.0, float(reported_age))
                except (TypeError, ValueError):
                    age = None
            if age is None and received is not None:
                try:
                    age = max(0.0, now - float(received))
                except (TypeError, ValueError):
                    age = None
            values[name] = {
                "topic": details.get("topic"),
                "measurement_stamp": stamp,
                "received_at": received,
                "sample_age_seconds": round(age, 3) if age is not None else None,
                "measurement_time_offset_ms": details.get("measurement_time_offset_ms"),
                "measurement_time_valid": details.get("measurement_time_valid"),
            }

        lidar_stamp = values["lidar"].get("measurement_stamp")
        delta_fields = {
            "lidar_to_rtk_delta_ms": (lidar_stamp, values["rtk"].get("measurement_stamp")),
            "lidar_to_imu_delta_ms": (lidar_stamp, values["imu"].get("measurement_stamp")),
            "lidar_to_odom_delta_ms": (lidar_stamp, values["odometry"].get("measurement_stamp")),
        }
        for field, (left, right) in delta_fields.items():
            if left is None or right is None:
                values[field] = None
            else:
                try:
                    values[field] = round((float(right) - float(left)) * 1000.0, 3)
                except (TypeError, ValueError):
                    values[field] = None

        valid_samples = [
            item["measurement_time_valid"]
            for name, item in values.items()
            if name in names and item["measurement_stamp"] is not None
            and item["measurement_time_valid"] is not None
        ]
        warnings = []
        if any(value is False for value in valid_samples):
            warnings.append("measurement_time_invalid")
        rtk_age = values["rtk"].get("sample_age_seconds")
        if rtk_age is not None and rtk_age > 1.5:
            warnings.append("rtk_stale")
        return {
            **values,
            "all_time_valid": bool(valid_samples) and all(valid_samples),
            "warnings": warnings,
            "warning": ",".join(warnings),
        }

    def _sensor_snapshot_locked(self) -> dict:
        now = time.time()
        stale_after = {
            "lidar": 1.0,
            "laser_scan": 1.0,
            "imu": 0.5,
            "odometry": 1.0,
            "rtk": 3.0,
        }
        result = {}
        for name, threshold in stale_after.items():
            samples = self._sensor_samples.get(name)
            recent = [stamp for stamp in (samples or ()) if now - stamp <= 5.0]
            last_seen = samples[-1] if samples else None
            age = now - last_seen if last_seen is not None else None
            hz = 0.0
            if len(recent) >= 2 and recent[-1] > recent[0]:
                hz = (len(recent) - 1) / (recent[-1] - recent[0])
            result[name] = {
                "online": age is not None and age <= threshold,
                "sample_age_seconds": round(age, 3) if age is not None else None,
                "frequency_hz": round(hz, 1),
                "sampled_at": (
                    datetime.fromtimestamp(last_seen, timezone.utc).isoformat(timespec="milliseconds")
                    if last_seen is not None
                    else None
                ),
                **self._sensor_details.get(name, {}),
            }
        return result

    def on_localization(self, msg) -> None:
        status = LOCALIZATION_STATUS.get(int(msg.status), "unknown")
        with self._lock:
            previous_status = self.safety_state.localization_status
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
            if status == "normal":
                if previous_status != "normal":
                    self.safety_state.localization_normal_since_monotonic = time.monotonic()
            else:
                self.safety_state.localization_normal_since_monotonic = 0.0

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

    def on_localization_decision(self, payload: dict) -> None:
        with self._lock:
            self._localization_decision = dict(payload or {})
            self._state_version += 1

    def localization_decision(self) -> dict:
        with self._lock:
            return dict(self._localization_decision)

    def on_localization_recovery(self, payload: dict | None) -> None:
        """Record recovery progress so a stuck recovery is visible from the platform.

        Pass None or an empty dict once recovery ends.
        """
        with self._lock:
            self._localization_recovery = dict(payload or {})
            self._state_version += 1

    def on_imu_cross_check(self, payload: dict | None) -> None:
        """Record the dual-IMU comparison so both gyros are visible on the platform.

        Deliberately not folded into `quality`: that object describes the NDT
        match, while this describes the sensors feeding it.
        """
        with self._lock:
            self._imu_cross_check = dict(payload or {})
            self._state_version += 1

    def localization_diagnostics(self) -> dict:
        with self._lock:
            pose = self._pose
            quality = self._localization_quality
            return {
                "raw_pose": {
                    "x": pose.x,
                    "y": pose.y,
                    "z": pose.z,
                    "yaw": pose.yaw,
                    "sampled_at": pose.sampled_at,
                    "localization_status": pose.localization_status,
                    "frame_id": "map",
                } if pose else None,
                "quality": {
                    "sampled_at": quality.sampled_at,
                    "has_converged": quality.has_converged,
                    "matching_error": quality.matching_error,
                    "inlier_fraction": quality.inlier_fraction,
                    "relative_translation_m": quality.relative_translation_m,
                } if quality else None,
                "decision": dict(self._localization_decision),
                "raw_rtk": dict(self._raw_rtk) if self._raw_rtk else None,
                "time_diagnostics": self._time_diagnostics_locked(time.time()),
            }

    def configure_system_probe_staleness(self, stale_seconds: float) -> None:
        self._system_probe_stale_seconds = max(1.0, float(stale_seconds))

    def on_battery(self, percentage: float, charging: bool, **details) -> None:
        with self._lock:
            self.power_available = True
            self.battery_percent = max(0, min(100, round(percentage * 100 if percentage <= 1 else percentage)))
            self.charging = charging
            self._power_sampled_monotonic = time.monotonic()
            self._power_details = details
            self.safety_state.power_available = True
            self.safety_state.battery_percent = self.battery_percent

    def on_network(self, network_type: str, signal_percent: int | None, **details) -> None:
        with self._lock:
            self.network_type = str(network_type or "")
            self.signal_percent = (
                max(0, min(100, int(signal_percent))) if signal_percent is not None else None
            )
            self._network_sampled_monotonic = time.monotonic()
            self._network_details = details

    def on_audio(self, **details) -> None:
        with self._lock:
            self._audio_sampled_monotonic = time.monotonic()
            self._audio_details = details

    def on_storage(self, payload: dict | None) -> None:
        """Record disk occupancy and the last retention pass.

        Recordings and maps are the only things on this robot that grow
        without bound, and retention now deletes some of them on its own. Both
        halves belong upstream: the occupancy so the trend is visible before
        it becomes an outage, the deletions so nothing disappears silently.
        """
        with self._lock:
            self._storage = dict(payload or {})

    def latest_pose(self) -> PoseSnapshot | None:
        with self._lock:
            return self._pose

    def latest_power(self) -> dict:
        with self._lock:
            fresh = bool(
                self.power_available
                and time.monotonic() - self._power_sampled_monotonic <= self._system_probe_stale_seconds
            )
            return {
                "available": fresh,
                "percent": self.battery_percent if fresh else None,
                "charging": self.charging if fresh else None,
                **(self._power_details if fresh else {}),
            }

    def build_status_snapshot(self, task_execution_id: str | None = None) -> dict:
        with self._lock:
            pose = self._pose
            quality = self._localization_quality
            now_monotonic = time.monotonic()
            power_fresh = bool(
                self.power_available
                and now_monotonic - self._power_sampled_monotonic <= self._system_probe_stale_seconds
            )
            network_fresh = bool(
                self._network_sampled_monotonic
                and now_monotonic - self._network_sampled_monotonic <= self._system_probe_stale_seconds
            )
            audio_fresh = bool(
                self._audio_sampled_monotonic
                and now_monotonic - self._audio_sampled_monotonic <= self._system_probe_stale_seconds
            )
            now = time.time()
            sensors = self._sensor_snapshot_locked()
            return {
                # The status sample is fresh even when localization is stopped
                # during mapping. Reusing the last pose timestamp makes the
                # center reject changing mapping progress as stale telemetry.
                "sampled_at": now_iso(),
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
                    "decision": dict(self._localization_decision),
                    "recovery": dict(self._localization_recovery) or None,
                    "imu_cross_check": dict(self._imu_cross_check) or None,
                    "raw_rtk": dict(self._raw_rtk) if self._raw_rtk else None,
                    "time_diagnostics": self._time_diagnostics_locked(now),
                },
                "power": {
                    "available": power_fresh,
                    "percent": self.battery_percent if power_fresh else None,
                    "charging": self.charging if power_fresh else None,
                    **(self._power_details if power_fresh else {}),
                },
                "network": {
                    "available": network_fresh,
                    "type": self.network_type if network_fresh else "",
                    "signal_percent": self.signal_percent if network_fresh else None,
                    **(self._network_details if network_fresh else {}),
                },
                "audio": {
                    "available": audio_fresh,
                    **(self._audio_details if audio_fresh else {}),
                },
                "sensors": sensors,
                "storage": dict(self._storage) or None,
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
