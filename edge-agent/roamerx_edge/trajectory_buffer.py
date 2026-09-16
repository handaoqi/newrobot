from __future__ import annotations

import time
import threading
import uuid

from .local_store import LocalStore
from .protocol import build_envelope
from .telemetry_collector import PoseSnapshot


class TrajectoryBuffer:
    def __init__(
        self,
        *,
        robot_id: str,
        session_id: str,
        store: LocalStore,
        batch_size: int = 20,
        flush_seconds: float = 5,
    ) -> None:
        self.robot_id = robot_id
        self.session_id = session_id
        self.store = store
        self.batch_size = batch_size
        self.flush_seconds = flush_seconds
        self._points: list[dict] = []
        self._execution_id: str | None = None
        self._map_id = ""
        self._map_version = ""
        self._last_flush = time.monotonic()
        self._lock = threading.RLock()

    def sample(
        self,
        execution_id: str,
        map_id: str,
        map_version: str,
        pose: PoseSnapshot,
        *,
        keyframe: dict | None = None,
    ) -> dict | None:
        with self._lock:
            if self._points and (
                self._execution_id != execution_id
                or self._map_id != map_id
                or self._map_version != map_version
            ):
                self._flush_locked()
            self._execution_id = execution_id
            self._map_id = map_id
            self._map_version = map_version
            point = {
                "seq": self.store.next_trajectory_seq(execution_id),
                "sampled_at": pose.sampled_at,
                "x": pose.x,
                "y": pose.y,
                "yaw": pose.yaw,
                "speed_mps": pose.speed_mps,
                "localization_status": pose.localization_status,
            }
            if isinstance(keyframe, dict):
                point["keyframe"] = keyframe
            self._points.append(point)
            if len(self._points) >= self.batch_size or time.monotonic() - self._last_flush >= self.flush_seconds:
                return self._flush_locked()
            return None

    def flush(self, execution_id: str, map_id: str, map_version: str) -> dict | None:
        with self._lock:
            if self._points and self._execution_id is None:
                self._execution_id = execution_id
                self._map_id = map_id
                self._map_version = map_version
            return self._flush_locked()

    def flush_active(self) -> dict | None:
        """Flush the last task's partial batch after it becomes terminal."""
        with self._lock:
            return self._flush_locked()

    def _flush_locked(self) -> dict | None:
        if not self._points:
            return None
        points = self._points
        execution_id = self._execution_id
        map_id = self._map_id
        map_version = self._map_version
        if not execution_id:
            return None
        self._points = []
        self._execution_id = None
        self._map_id = ""
        self._map_version = ""
        self._last_flush = time.monotonic()
        batch_id = str(uuid.uuid4())
        message = build_envelope(
            message_type="trajectory.batch",
            robot_id=self.robot_id,
            session_id=self.session_id,
            payload={
                "task_execution_id": execution_id,
                "map_id": map_id,
                "map_version": map_version,
                "frame_id": "map",
                "batch_id": batch_id,
                "first_seq": points[0]["seq"],
                "last_seq": points[-1]["seq"],
                "points": points,
            },
        )
        self.store.enqueue_outbox(
            f"robots/{self.robot_id}/telemetry/trajectory",
            message,
            qos=1,
            dedupe_key=batch_id,
        )
        return message

    def handle_ack(self, payload: dict) -> None:
        batch_id = payload.get("batch_id")
        if not batch_id:
            return
        if payload.get("accepted") or payload.get("duplicate"):
            self.store.ack_outbox(dedupe_key=str(batch_id))
            return
        # Only permanently invalid batches may be dropped. Transient center
        # errors must keep replaying; an absent task execution is permanent
        # because task creation precedes edge telemetry for that execution.
        if payload.get("reason_code") in {"INVALID_MESSAGE", "UNKNOWN_TASK_EXECUTION"}:
            self.store.ack_outbox(dedupe_key=str(batch_id))
