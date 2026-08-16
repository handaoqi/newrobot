from __future__ import annotations

import time
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
        self._last_flush = time.monotonic()

    def sample(self, execution_id: str, map_id: str, map_version: str, pose: PoseSnapshot) -> dict | None:
        self._points.append(
            {
                "seq": self.store.next_trajectory_seq(execution_id),
                "sampled_at": pose.sampled_at,
                "x": pose.x,
                "y": pose.y,
                "yaw": pose.yaw,
                "speed_mps": pose.speed_mps,
                "localization_status": pose.localization_status,
            }
        )
        if len(self._points) >= self.batch_size or time.monotonic() - self._last_flush >= self.flush_seconds:
            return self.flush(execution_id, map_id, map_version)
        return None

    def flush(self, execution_id: str, map_id: str, map_version: str) -> dict | None:
        if not self._points:
            return None
        points = self._points
        self._points = []
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
        self.store.ack_outbox(dedupe_key=payload["batch_id"])
