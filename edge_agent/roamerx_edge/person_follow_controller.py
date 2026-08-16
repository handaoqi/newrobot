"""Local, fail-closed person-following control for the Edge Agent."""
from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .protocol import ProtocolError


@dataclass
class FollowRun:
    track_id: str
    status: str = "idle"
    reason: str = ""
    started_at: float | None = None
    updated_at: float | None = None
    updates: int = 0

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        return {
            "track_id": self.track_id or None,
            "status": self.status,
            "reason": self.reason or None,
            "elapsed_seconds": round(now - self.started_at, 2) if self.started_at else 0.0,
            "updates": self.updates,
        }


class PersonFollowController:
    """Own one local visual-follow session and always stop on unsafe input."""

    def __init__(self, navigation, config) -> None:
        self.navigation = navigation
        self.config = config
        self._lock = threading.Lock()
        self._run = FollowRun(track_id="")
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self, track_id: str) -> dict[str, Any]:
        track_id = str(track_id or "").strip()
        if not track_id:
            raise ProtocolError("PERSON_TRACK_REQUIRED", "track_id is required")
        with self._lock:
            if self._run.status == "running":
                raise ProtocolError("PERSON_FOLLOW_BUSY", f"already following {self._run.track_id}")
            self._target_from_snapshot(track_id)  # Validate before movement is armed.
            self._stop_event = threading.Event()
            self._run = FollowRun(
                track_id=track_id,
                status="running",
                reason="本地视觉跟随已启动",
                started_at=time.monotonic(),
                updated_at=time.monotonic(),
            )
            self._thread = threading.Thread(
                target=self._run_loop,
                args=(track_id, self._stop_event),
                daemon=True,
                name="person-follow",
            )
            self._thread.start()
            return self._run.snapshot()

    def stop(self, reason: str = "operator_stop") -> dict[str, Any]:
        with self._lock:
            active = self._run.status == "running"
            self._stop_event.set()
            if active:
                self._run.status = "stopped"
                self._run.reason = reason
                self._run.updated_at = time.monotonic()
            snapshot = self._run.snapshot()
        # Stop outside the lock so a ROS publisher cannot block a status call.
        self.navigation.teleop_velocity()
        return snapshot

    def status(self) -> dict[str, Any]:
        with self._lock:
            snapshot = self._run.snapshot()
        snapshot["perception"] = self.perception_status()
        return snapshot

    def perception_status(self) -> dict[str, Any]:
        try:
            payload = self._load_snapshot()
            return {
                "available": True,
                "age_seconds": round(self._snapshot_age(payload), 3),
                "detections": len(payload.get("detections") or []),
            }
        except ProtocolError as exc:
            return {"available": False, "reason": exc.code}

    def _run_loop(self, track_id: str, stop_event: threading.Event) -> None:
        terminal_reason = "operator_stop"
        try:
            while not stop_event.is_set():
                target, frame_width, frame_height = self._target_from_snapshot(track_id)
                obstacle = getattr(self.navigation, "obstacle_monitor_snapshot", lambda: {})()
                distance = obstacle.get("front_obstacle_distance_m")
                if distance is not None and float(distance) <= self.config.obstacle_stop_distance_m:
                    raise ProtocolError("PERSON_FOLLOW_OBSTACLE", "front obstacle is too close")
                self.navigation.teleop_velocity(**self._velocity(target, frame_width, frame_height))
                with self._lock:
                    if self._run.status == "running" and self._run.track_id == track_id:
                        self._run.updates += 1
                        self._run.updated_at = time.monotonic()
                stop_event.wait(self.config.control_interval_seconds)
        except ProtocolError as exc:
            terminal_reason = exc.code
        except Exception:  # Defensive device boundary: never retain motion on an unexpected error.
            terminal_reason = "PERSON_FOLLOW_INTERNAL_ERROR"
        finally:
            self.navigation.teleop_velocity()
            with self._lock:
                if self._run.track_id == track_id and self._run.status == "running":
                    self._run.status = "stopped"
                    self._run.reason = terminal_reason
                    self._run.updated_at = time.monotonic()

    def _target_from_snapshot(self, track_id: str) -> tuple[dict[str, Any], float, float]:
        payload = self._load_snapshot()
        if self._snapshot_age(payload) > self.config.detection_stale_seconds:
            raise ProtocolError("PERSON_DETECTION_STALE", "local person detections are stale")
        frame_width = float(payload.get("frame_width") or 0)
        frame_height = float(payload.get("frame_height") or 0)
        target = next(
            (
                item for item in payload.get("detections") or []
                if str(item.get("track_id") or "") == track_id
                and str(item.get("label") or "person").lower() == "person"
            ),
            None,
        )
        if not target or frame_width <= 0 or frame_height <= 0:
            raise ProtocolError("PERSON_TARGET_UNAVAILABLE", "selected person is not available in the latest local frame")
        return target, frame_width, frame_height

    def _load_snapshot(self) -> dict[str, Any]:
        try:
            payload = json.loads(Path(self.config.detection_snapshot_path).read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ProtocolError("PERSON_DETECTION_UNAVAILABLE", "local person detection snapshot is unavailable") from exc
        if not isinstance(payload, dict) or not isinstance(payload.get("captured_monotonic"), (float, int)):
            raise ProtocolError("PERSON_DETECTION_UNAVAILABLE", "local person detection snapshot is invalid")
        return payload

    @staticmethod
    def _snapshot_age(payload: dict[str, Any]) -> float:
        return max(0.0, time.monotonic() - float(payload["captured_monotonic"]))

    @staticmethod
    def _velocity(target: dict[str, Any], frame_width: float, frame_height: float) -> dict[str, float]:
        box = target.get("bbox") or {}
        center_error = (float(box.get("x", 0)) + float(box.get("width", 0)) / 2) / frame_width - 0.5
        height_ratio = float(box.get("height", 0)) / frame_height
        vx = 0.0
        if height_ratio < 0.40:
            vx = min(0.16, (0.40 - height_ratio) * 0.7)
        elif height_ratio > 0.62:
            vx = max(-0.08, (0.62 - height_ratio) * 0.5)
        yaw_rate = 0.0 if abs(center_error) < 0.07 else max(-0.28, min(0.28, -center_error * 0.75))
        if abs(center_error) > 0.32:
            vx = 0.0
        return {"vx": round(vx, 3), "vy": 0.0, "yaw_rate": round(yaw_rate, 3)}
