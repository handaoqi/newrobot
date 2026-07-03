from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Protocol

from .local_store import LocalStore
from .protocol import MessageEnvelope, ProtocolError, now_iso


class NavigationAdapter(Protocol):
    def send_waypoints(self, waypoints: list[dict], feedback_cb: Callable, result_cb: Callable) -> bool: ...
    def cancel_navigation(self, timeout_seconds: float = 5.0) -> bool: ...
    def is_robot_stopped(self) -> bool: ...


@dataclass
class TaskContext:
    task_execution_id: str
    state: str
    state_version: int
    route_snapshot: dict
    current_waypoint_index: int
    start_command_id: str


class TaskExecutor:
    def __init__(
        self,
        store: LocalStore,
        navigation: NavigationAdapter,
        *,
        event_callback: Callable[[str, dict, str], None],
        start_result_callback: Callable[[str, str, dict, str, str], None],
    ) -> None:
        self.store = store
        self.navigation = navigation
        self.event_callback = event_callback
        self.start_result_callback = start_result_callback
        self._lock = threading.RLock()
        self._goal_offset = 0
        raw = store.load_active_task_context()
        self.context = TaskContext(**raw) if raw else None
        if self.context:
            self.context.state = "interrupted"
            self.context.state_version += 1
            self._persist()

    def has_active_task(self) -> bool:
        return self.context is not None and self.context.state not in {
            "completed",
            "failed",
            "cancelled",
            "timed_out",
            "rejected",
        }

    def start_task(self, envelope: MessageEnvelope) -> None:
        with self._lock:
            command = envelope.payload["command"]
            route = command["route_snapshot"]
            self.context = TaskContext(
                task_execution_id=envelope.payload["task_execution_id"],
                state="accepted",
                state_version=1,
                route_snapshot=route,
                current_waypoint_index=0,
                start_command_id=envelope.payload["command_id"],
            )
            self._persist()
            self._emit("task.accepted")
            self._send_from(0)

    def _send_from(self, index: int) -> None:
        if not self.context:
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "task context is missing")
        waypoints = self.context.route_snapshot["waypoints"][index:]
        self._goal_offset = index
        accepted = self.navigation.send_waypoints(waypoints, self.on_feedback, self.on_navigation_result)
        if not accepted:
            self._fail("NAV_STACK_NOT_READY", "FollowWaypoints goal was rejected")
            return
        self.context.state = "running"
        self.context.state_version += 1
        self._persist()
        self._emit("task.started")

    def pause_task(self, execution_id: str) -> dict:
        with self._lock:
            self._assert_execution(execution_id)
            if self.context.state != "running":
                raise ProtocolError("INVALID_TASK_STATE", f"cannot pause from {self.context.state}")
            self.context.state = "pausing"
            self.context.state_version += 1
            self._persist()
            self._emit("task.pausing")
            if not self.navigation.cancel_navigation():
                raise ProtocolError("NAVIGATION_CANCEL_FAILED", "Nav2 action cancel failed")
            if not self.navigation.is_robot_stopped():
                raise ProtocolError("ROBOT_NOT_STOPPED", "robot speed did not reach stop threshold")
            self.context.state = "paused"
            self.context.state_version += 1
            self._persist()
            self._emit("task.paused")
            total = len(self.context.route_snapshot["waypoints"])
            return {
                "final_task_state": "paused",
                "state_version": self.context.state_version,
                "paused_at_waypoint_index": self.context.current_waypoint_index,
                "resume_from_waypoint_index": self.context.current_waypoint_index,
                "remaining_waypoints": max(0, total - self.context.current_waypoint_index),
                "robot_stopped": True,
            }

    def resume_task(self, execution_id: str, resume_index: int) -> dict:
        with self._lock:
            self._assert_execution(execution_id)
            if self.context.state != "paused":
                raise ProtocolError("INVALID_TASK_STATE", f"cannot resume from {self.context.state}")
            if resume_index != self.context.current_waypoint_index:
                raise ProtocolError("TASK_CONTEXT_MISMATCH", "resume index does not match persisted context")
            self.context.state = "resuming"
            self.context.state_version += 1
            self._persist()
            self._emit("task.resuming")
            self._send_from(resume_index)
            return {
                "final_task_state": "running",
                "state_version": self.context.state_version,
                "resume_from_waypoint_index": resume_index,
            }

    def cancel_task(self, execution_id: str) -> dict:
        with self._lock:
            self._assert_execution(execution_id)
            if self.context.state not in {"running", "paused", "pausing", "resuming", "interrupted"}:
                raise ProtocolError("INVALID_TASK_STATE", f"cannot cancel from {self.context.state}")
            previous_state = self.context.state
            self.context.state = "cancelling"
            self.context.state_version += 1
            self._persist()
            self._emit("task.cancelling")
            if previous_state != "paused" and not self.navigation.cancel_navigation():
                raise ProtocolError("NAVIGATION_CANCEL_FAILED", "Nav2 action cancel failed")
            if not self.navigation.is_robot_stopped():
                raise ProtocolError("ROBOT_NOT_STOPPED", "robot speed did not reach stop threshold")
            self.context.state = "cancelled"
            self.context.state_version += 1
            self._persist()
            self._emit("task.cancelled")
            self.store.clear_task_context(execution_id, "cancelled")
            return {
                "final_task_state": "cancelled",
                "state_version": self.context.state_version,
                "robot_stopped": True,
            }

    def on_feedback(self, current_waypoint_index: int, distance_remaining_m: float | None = None) -> None:
        with self._lock:
            if not self.context or self.context.state != "running":
                return
            current_waypoint_index += self._goal_offset
            self.context.current_waypoint_index = current_waypoint_index
            self.context.state_version += 1
            self._persist()
            total = len(self.context.route_snapshot["waypoints"])
            self.event_callback(
                "task.progress",
                {
                    "task_execution_id": self.context.task_execution_id,
                    "state": "running",
                    "state_version": self.context.state_version,
                    "current_waypoint_index": current_waypoint_index,
                    "current_waypoint_id": self.context.route_snapshot["waypoints"][current_waypoint_index]["waypoint_id"]
                    if current_waypoint_index < total
                    else "",
                    "completed_waypoints": current_waypoint_index,
                    "total_waypoints": total,
                    "distance_remaining_m": distance_remaining_m,
                    "estimated_time_remaining_s": None,
                    "reported_at": now_iso(),
                },
                "",
            )

    def on_navigation_result(self, status: str, error_message: str = "") -> None:
        with self._lock:
            if not self.context or self.context.state in {"pausing", "cancelling", "paused", "cancelled"}:
                return
            if status == "succeeded":
                self.context.state = "completed"
                self.context.current_waypoint_index = len(self.context.route_snapshot["waypoints"])
                self.context.state_version += 1
                self._persist()
                self._emit("task.completed")
                self.store.clear_task_context(self.context.task_execution_id, "completed")
                self.start_result_callback(
                    self.context.start_command_id,
                    "succeeded",
                    {
                        "final_task_state": "completed",
                        "state_version": self.context.state_version,
                        "completed_waypoints": self.context.current_waypoint_index,
                        "total_waypoints": self.context.current_waypoint_index,
                    },
                    "",
                    "",
                )
            elif status == "cancelled":
                return
            else:
                self._fail("NAVIGATION_FAILED", error_message or status)

    def _fail(self, code: str, message: str) -> None:
        if not self.context:
            return
        self.context.state = "failed"
        self.context.state_version += 1
        self._persist()
        self._emit("task.failed", code=code, message=message)
        self.store.clear_task_context(self.context.task_execution_id, "failed")
        self.start_result_callback(
            self.context.start_command_id,
            "failed",
            {"final_task_state": "failed", "state_version": self.context.state_version},
            code,
            message,
        )

    def _assert_execution(self, execution_id: str) -> None:
        if not self.context or self.context.task_execution_id != execution_id:
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "task execution does not match local context")

    def _persist(self) -> None:
        if self.context:
            self.store.save_task_context(self.context.__dict__)

    def _emit(self, event_type: str, *, code: str = "", message: str = "") -> None:
        if not self.context:
            return
        self.event_callback(
            event_type,
            {
                "task_execution_id": self.context.task_execution_id,
                "state": self.context.state,
                "state_version": self.context.state_version,
                "occurred_at": now_iso(),
                "reason_code": code or None,
                "reason_message": message or None,
            },
            "",
        )
