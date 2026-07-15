from __future__ import annotations

import threading
from dataclasses import dataclass
from math import hypot
from typing import Callable, Protocol

from .local_store import LocalStore
from .protocol import MessageEnvelope, ProtocolError, now_iso


class NavigationAdapter(Protocol):
    def send_waypoints(self, waypoints: list[dict], feedback_cb: Callable, result_cb: Callable) -> bool: ...
    def cancel_navigation(self, timeout_seconds: float = 5.0) -> bool: ...
    def is_robot_stopped(self) -> bool: ...
    def latest_pose(self): ...


@dataclass
class TaskContext:
    task_execution_id: str
    state: str
    state_version: int
    route_snapshot: dict
    current_waypoint_index: int
    start_command_id: str
    current_segment_index: int = 0


class TaskExecutor:
    TERMINAL_STATES = {"completed", "failed", "cancelled", "timed_out", "rejected"}

    def __init__(
        self,
        store: LocalStore,
        navigation: NavigationAdapter,
        *,
        event_callback: Callable[[str, dict, str], None],
        start_result_callback: Callable[[str, str, dict, str, str], None],
        final_waypoint_tolerance_m: float = 0.35,
        map_set_coordinator=None,
    ) -> None:
        self.store = store
        self.navigation = navigation
        self.event_callback = event_callback
        self.start_result_callback = start_result_callback
        self.final_waypoint_tolerance_m = final_waypoint_tolerance_m
        self.map_set_coordinator = map_set_coordinator
        self._segments = []
        self._lock = threading.RLock()
        self._goal_offset = 0
        raw = store.load_active_task_context()
        self.context = TaskContext(**raw) if raw else None
        if self.context:
            self.context.state = "interrupted"
            self.context.state_version += 1
            self._persist()

    def report_startup_interruption(self) -> None:
        """Close the command lifecycle after an active task is recovered."""
        with self._lock:
            if not self.context or self.context.state != "interrupted":
                return
            self._fail("EDGE_RESTARTED", "Edge Agent restarted while the task was active")

    def has_active_task(self) -> bool:
        return self.context is not None and self.context.state not in self.TERMINAL_STATES

    def reconcile_center_state(self, execution_id: str, expected_state: str | None) -> bool:
        with self._lock:
            if (
                not self.context
                or self.context.task_execution_id != execution_id
                or expected_state not in self.TERMINAL_STATES
            ):
                return False
            self.context.state = expected_state
            self.context.state_version += 1
            self._persist()
            self.store.clear_task_context(execution_id, expected_state)
            self.context = None
            return True

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
            self._segments = self.map_set_coordinator.build_segments(route) if self.map_set_coordinator else []
            if self._segments:
                self.map_set_coordinator.activate(self._segments[0])
                self._send_segment(0)
            else:
                self._send_from(0)

    def _send_segment(self, segment_index: int) -> None:
        if not self.context:
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "task context is missing")
        segment = self._segments[segment_index]
        self.context.current_segment_index = segment_index
        self.context.current_waypoint_index = segment.start_index
        self._goal_offset = segment.start_index
        accepted = self.navigation.send_waypoints(segment.waypoints, self.on_feedback, self.on_navigation_result)
        if not accepted:
            self._fail("NAV_STACK_NOT_READY", "FollowWaypoints goal was rejected")
            return
        self.context.state = "running"
        self.context.state_version += 1
        self._persist()
        self._emit("task.started")

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

    def on_navigation_result(self, status: str, error_message: str = "", details: dict | None = None) -> None:
        with self._lock:
            if not self.context or self.context.state in {"pausing", "cancelling", "paused", "cancelled"}:
                return
            if status == "succeeded":
                missed = list((details or {}).get("missed_waypoints") or [])
                if missed:
                    absolute_missed = [index + self._goal_offset for index in missed]
                    self._fail(
                        "NAVIGATION_MISSED_WAYPOINTS",
                        f"Nav2 reported missed waypoints: {absolute_missed}",
                    )
                    return
                if self._segments and self.context.current_segment_index < len(self._segments) - 1:
                    next_index = self.context.current_segment_index + 1
                    next_segment = self._segments[next_index]
                    self.context.state_version += 1
                    self._persist()
                    self._emit("task.map_switching")
                    try:
                        self.map_set_coordinator.activate(next_segment)
                    except Exception as exc:
                        self._fail("MAP_SWITCH_FAILED", str(exc))
                        return
                    self._send_segment(next_index)
                    return
                pose_error = self._final_pose_error()
                if pose_error:
                    self._fail(*pose_error)
                    return
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

    def _final_pose_error(self) -> tuple[str, str] | None:
        if not self.context:
            return ("TASK_CONTEXT_MISMATCH", "task context is missing")
        waypoints = self.context.route_snapshot.get("waypoints") or []
        if not waypoints:
            return None
        pose = self.navigation.latest_pose()
        if not pose:
            return ("FINAL_POSE_UNAVAILABLE", "latest robot pose is unavailable")
        final_waypoint = waypoints[-1]
        distance = hypot(float(pose.x) - float(final_waypoint["x"]), float(pose.y) - float(final_waypoint["y"]))
        if distance > self.final_waypoint_tolerance_m:
            return (
                "FINAL_POSE_OUT_OF_TOLERANCE",
                (
                    f"final pose is {distance:.2f}m from last waypoint "
                    f"(tolerance {self.final_waypoint_tolerance_m:.2f}m)"
                ),
            )
        return None

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
