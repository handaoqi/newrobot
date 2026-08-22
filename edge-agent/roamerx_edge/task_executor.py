from __future__ import annotations

import logging
import threading
import time
import uuid
from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin
from typing import Callable, Protocol

from .local_store import LocalStore
from .map_coordinate import MapConstraintError, constraints_from_manifest, validate_route_against_map
from .map_package_finalize import load_map_manifest
from .protocol import MessageEnvelope, ProtocolError, now_iso


LOGGER = logging.getLogger(__name__)


class NavigationAdapter(Protocol):
    def prepare_for_navigation(self, timeout_seconds: float = 12.0) -> bool: ...
    def send_waypoints(self, waypoints: list[dict], feedback_cb: Callable, result_cb: Callable) -> bool: ...
    def cancel_navigation(self, timeout_seconds: float = 5.0) -> bool: ...
    def stop_motion(self) -> None: ...
    def is_robot_stopped(self) -> bool: ...
    def latest_pose(self): ...
    def latest_trusted_pose(self): ...
    def set_localization_policy(self, source: str, phase: str) -> dict: ...
    def localization_decision(self) -> dict: ...
    def localization_diagnostics(self) -> dict: ...
    def set_goal_precision(self, *, enabled: bool) -> None: ...


@dataclass
class TaskContext:
    task_execution_id: str
    state: str
    state_version: int
    route_snapshot: dict
    current_waypoint_index: int
    start_command_id: str
    current_segment_index: int = 0
    record_rosbag: bool = False
    docking: dict | None = None


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
        docking_goal_tolerance_m: float = 0.08,
        docking_goal_yaw_tolerance_rad: float = 0.0872665,
        standup_confirmation_timeout_seconds: float = 12.0,
        map_set_coordinator=None,
        obstacle_speech=None,
        rosbag_recorder=None,
        docking_arrived_handler=None,
    ) -> None:
        self.store = store
        self.navigation = navigation
        self.event_callback = event_callback
        self.start_result_callback = start_result_callback
        self.final_waypoint_tolerance_m = final_waypoint_tolerance_m
        self.docking_goal_tolerance_m = docking_goal_tolerance_m
        self.docking_goal_yaw_tolerance_rad = docking_goal_yaw_tolerance_rad
        self.standup_confirmation_timeout_seconds = standup_confirmation_timeout_seconds
        self.map_set_coordinator = map_set_coordinator
        self.obstacle_speech = obstacle_speech
        self.rosbag_recorder = rosbag_recorder
        self.docking_arrived_handler = docking_arrived_handler
        self._rosbag_state: dict = {}
        self._segments = []
        self._lock = threading.RLock()
        self._goal_offset = 0
        self._obstacle_monitor_stop = threading.Event()
        self._obstacle_monitor_thread = None
        self._obstacle_progress_anchor = None
        self._obstacle_progress_anchor_at = None
        self._recovery_attempts = 0
        self._leave_route_announced = False
        self._last_obstacle_seen_at = None
        self._obstacle_episode_id = None
        self._task_started_at = None
        self._blocked_retry_timer = None
        self._paused_for_localization = False
        self._navigation_prepared = False
        self._segment_avoidance_enabled = True
        raw = store.load_active_task_context()
        self.context = TaskContext(**raw) if raw else None
        if self.context:
            self.context.state = "interrupted"
            self.context.state_version += 1
            self._persist()

    def stop(self) -> None:
        self._stop_obstacle_monitor()
        self._stop_task_rosbag()
        self._restore_navigation_profile()
        if self._blocked_retry_timer:
            self._blocked_retry_timer.cancel()
            self._blocked_retry_timer = None

    def _obstacle_monitor_enabled(self) -> bool:
        return bool(
            self._segment_avoidance_enabled
            and self.obstacle_speech
            and self.obstacle_speech.enabled
            and callable(getattr(self.navigation, "obstacle_monitor_snapshot", None))
        )

    def _start_obstacle_monitor(self) -> None:
        if not self._obstacle_monitor_enabled():
            return
        self._obstacle_monitor_stop.clear()
        if self._obstacle_monitor_thread and self._obstacle_monitor_thread.is_alive():
            return
        self._obstacle_monitor_thread = threading.Thread(
            target=self._obstacle_monitor_loop, daemon=True, name="obstacle-speech-monitor"
        )
        self._obstacle_monitor_thread.start()

    def _stop_obstacle_monitor(self) -> None:
        self._obstacle_monitor_stop.set()
        self._obstacle_monitor_thread = None
        self._reset_obstacle_episode()

    def _obstacle_monitor_loop(self) -> None:
        while not self._obstacle_monitor_stop.wait(0.5):
            with self._lock:
                self._evaluate_obstacle_progress()

    def _reset_obstacle_episode(self) -> None:
        self._obstacle_progress_anchor = None
        self._obstacle_progress_anchor_at = None
        self._recovery_attempts = 0
        self._leave_route_announced = False
        self._last_obstacle_seen_at = None
        self._obstacle_episode_id = None

    def _evaluate_obstacle_progress(self) -> None:
        if not self.context or self.context.state != "running" or not self._obstacle_monitor_enabled():
            return
        observation = self.navigation.obstacle_monitor_snapshot()
        obstacle_distance = observation.get("front_obstacle_distance_m")
        raw_planar = abs(float(observation.get("requested_planar_speed_mps") or 0.0))
        actual_planar = abs(float(observation.get("actual_planar_speed_mps") or 0.0))
        raw_turn = abs(float(observation.get("requested_turn_speed_rps") or 0.0))
        actual_turn = abs(float(observation.get("actual_turn_speed_rps") or 0.0))
        scan_blocked = (
            obstacle_distance is not None
            and float(obstacle_distance) <= self.obstacle_speech.obstacle_max_distance_m
        )
        # /cmd_vel_raw is the planner request; /cmd_vel is collision-monitor
        # output. A large reduction here catches the monitor's polygon zones,
        # including obstacles outside the narrow front scan corridor.
        requested_motion = raw_planar >= 0.04 or raw_turn >= 0.15
        actual_motion = max(actual_planar, actual_turn * 0.25)
        requested_magnitude = max(raw_planar, raw_turn * 0.25)
        collision_limited = (
            requested_motion
            and actual_motion <= requested_magnitude * self.obstacle_speech.collision_limit_ratio
        )
        blocked = scan_blocked or collision_limited
        now = time.monotonic()
        pose = self.navigation.latest_pose()
        if not blocked or not pose:
            # Require a continuous clear window before opening a new episode.
            # This prevents scan flicker around one obstacle from causing
            # repeated announcements while still allowing a later obstacle to
            # announce again in the same patrol task.
            if (
                self._last_obstacle_seen_at is not None
                and now - self._last_obstacle_seen_at < self.obstacle_speech.obstacle_clear_seconds
            ):
                return
            self._reset_obstacle_episode()
            return
        self._last_obstacle_seen_at = now
        if self._obstacle_progress_anchor is None:
            self._obstacle_episode_id = str(uuid.uuid4())
            self._obstacle_progress_anchor = (float(pose.x), float(pose.y))
            self._obstacle_progress_anchor_at = now
            self._emit_obstacle_speech("obstacle_detected", 0, observation)
            return
        anchor_x, anchor_y = self._obstacle_progress_anchor
        progressed = hypot(float(pose.x) - anchor_x, float(pose.y) - anchor_y)
        if progressed >= self.obstacle_speech.min_progress_m:
            self._obstacle_progress_anchor = (float(pose.x), float(pose.y))
            self._obstacle_progress_anchor_at = now
            # Progress starts a fresh five-second observation window, but the
            # number of level-2 announcements remains cumulative within this
            # obstacle episode. Level 3 must follow exactly three level-2
            # announcements and may only be announced once per episode.
            return
        if now - self._obstacle_progress_anchor_at < self.obstacle_speech.no_progress_seconds:
            return
        if self._leave_route_announced:
            return
        self._recovery_attempts += 1
        self._obstacle_progress_anchor_at = now
        self._emit_obstacle_speech("recovery_attempt", self._recovery_attempts, observation)
        if self._recovery_attempts >= 3:
            self._leave_route_announced = True
            self._emit_obstacle_speech("leave_route", self._recovery_attempts, observation)

    def _emit_obstacle_speech(self, stage: str, attempt: int, observation: dict) -> None:
        titles = {
            "obstacle_detected": "发现障碍物",
            "recovery_attempt": "后退尝试避障",
            "leave_route": "劝阻离开线路",
        }
        self.event_callback(
            "task.obstacle_speech",
            {
                "task_execution_id": self.context.task_execution_id,
                "obstacle_episode_id": self._obstacle_episode_id,
                "speech_stage": stage,
                "template_name": titles[stage],
                "recovery_attempt": attempt,
                "front_obstacle_distance_m": observation.get("front_obstacle_distance_m"),
                "reported_at": now_iso(),
            },
            "",
        )

    def report_docking_charge(self, event_type: str, *, message: str = "", extra: dict | None = None) -> None:
        """Publish charge-contact milestones for the platform docking dialog."""
        with self._lock:
            if self._is_docking_task():
                self._emit(event_type, message=message, extra=extra)
        LOGGER.info(
            "obstacle speech episode=%s stage=%s attempt=%s distance=%s raw_planar=%s actual_planar=%s",
            self._obstacle_episode_id,
            stage,
            attempt,
            observation.get("front_obstacle_distance_m"),
            observation.get("requested_planar_speed_mps"),
            observation.get("actual_planar_speed_mps"),
        )

    def report_startup_interruption(self) -> None:
        """Close the command lifecycle after an active task is recovered."""
        with self._lock:
            if not self.context or self.context.state != "interrupted":
                return
            self._fail("EDGE_RESTARTED", "Edge Agent restarted while the task was active")

    def has_active_task(self) -> bool:
        return self.context is not None and self.context.state not in self.TERMINAL_STATES

    def is_paused_for_localization(self) -> bool:
        with self._lock:
            return bool(
                self.context
                and self.context.state == "paused"
                and self._paused_for_localization
            )

    def on_localization_lost(self) -> None:
        """Pause active navigation when localization is continuously lost."""
        with self._lock:
            if not self.context or self.context.state != "running":
                return
            trusted_pose_getter = getattr(self.navigation, "latest_trusted_pose", None)
            trusted_pose = trusted_pose_getter() if callable(trusted_pose_getter) else None
            diagnostics_getter = getattr(self.navigation, "localization_diagnostics", None)
            diagnostics = diagnostics_getter() if callable(diagnostics_getter) else {}
            route_map = dict(self.context.route_snapshot.get("map") or {})
            waypoints = self.context.route_snapshot.get("waypoints") or []
            target = (
                dict(waypoints[self.context.current_waypoint_index])
                if self.context.current_waypoint_index < len(waypoints)
                else None
            )
            if target is not None:
                target.setdefault("map_point_number", self.context.current_waypoint_index + 1)
            self.context.state = "pausing"
            self.context.state_version += 1
            self._persist()
            self._emit(
                "task.pausing",
                code="LOCALIZATION_LOST",
                message="localization lost; pausing navigation for relocalization",
                extra={
                    "last_trusted_pose": trusted_pose,
                    "raw_pose": diagnostics.get("raw_pose"),
                    "localization_quality": diagnostics.get("quality"),
                    "localization_decision": diagnostics.get("decision"),
                    "map_id": route_map.get("map_id"),
                    "map_version": route_map.get("map_version"),
                    "current_waypoint_index": self.context.current_waypoint_index,
                    "current_waypoint": target,
                    "recovery_policy": {
                        "quick_attempts": 3,
                        "continuous_retry": True,
                    },
                },
            )

            cancelled = self.navigation.cancel_navigation()
            self.navigation.stop_motion()
            if not cancelled:
                # A cancel acknowledgement can time out while Nav2 is already
                # stopping. The explicit zero command is the safety boundary;
                # keep the task recoverable instead of incorrectly failing it.
                LOGGER.warning(
                    "Nav2 cancel was not acknowledged after localization loss; "
                    "holding the task paused after zeroing motion"
                )

            self._paused_for_localization = True
            self.context.state = "paused"
            self.context.state_version += 1
            self._persist()
            self._emit(
                "task.paused",
                code="LOCALIZATION_LOST",
                message="navigation stopped while localization recovers",
            )

    def on_localization_recovered(self) -> None:
        """Resume a safety-paused task after localization is stably normal."""
        with self._lock:
            if (
                not self.context
                or self.context.state != "paused"
                or not self._paused_for_localization
            ):
                return
            resume_index = self._nearest_remaining_waypoint_index(
                self.context.current_waypoint_index
            )
            self._paused_for_localization = False
            self.context.state = "resuming"
            self.context.state_version += 1
            self._persist()
            self._emit(
                "task.resuming",
                code="LOCALIZATION_RECOVERED",
                message="localization is stable; resuming from the pending waypoint",
            )
            self._send_from(resume_index)

    def _nearest_remaining_waypoint_index(self, start_index: int) -> int:
        if not self.context:
            return start_index
        waypoints = self.context.route_snapshot.get("waypoints") or []
        if start_index >= len(waypoints):
            return start_index
        pose = self.navigation.latest_pose()
        if pose is None:
            return start_index
        candidates = range(max(0, start_index), len(waypoints))
        return min(
            candidates,
            key=lambda index: hypot(
                float(pose.x) - float(waypoints[index]["x"]),
                float(pose.y) - float(waypoints[index]["y"]),
            ),
        )

    def reconcile_center_state(self, execution_id: str, expected_state: str | None) -> bool:
        with self._lock:
            if (
                not self.context
                or self.context.task_execution_id != execution_id
                or expected_state not in self.TERMINAL_STATES
            ):
                return False
            self._stop_task_rosbag()
            self.context.state = expected_state
            self.context.state_version += 1
            self._persist()
            self.store.clear_task_context(execution_id, expected_state)
            self.context = None
            return True

    def prepare_task_start(self, envelope: MessageEnvelope) -> None:
        """Persist an accepted task before its command acknowledgement is sent."""
        with self._lock:
            command = envelope.payload["command"]
            route = dict(command["route_snapshot"])
            route.setdefault("map", dict(command.get("map") or {}))
            route["waypoints"] = [dict(waypoint) for waypoint in route.get("waypoints") or []]
            self._assert_map_constraints(route)
            docking = dict(command.get("docking") or {})
            # Docking is deliberately ordered: waypoint 0 establishes the
            # safe approach line and must never be skipped by nearest-point
            # task startup behavior.
            initial_waypoint_index = 0 if docking.get("enabled") else self._nearest_waypoint_index(route)
            reverse_return = bool(
                command.get("loop_execution")
                and not docking.get("enabled")
                and len(route["waypoints"]) > 1
                and initial_waypoint_index == len(route["waypoints"]) - 1
            )
            if reverse_return:
                route["waypoints"].reverse()
                for sequence, waypoint in enumerate(route["waypoints"]):
                    waypoint.setdefault("map_point_number", int(waypoint.get("sequence", 0)) + 1)
                    waypoint["sequence"] = sequence
                route["execution_order"] = "reverse_from_route_end"
                route["initial_map_point_number"] = route["waypoints"][0].get("map_point_number")
                initial_waypoint_index = 0
            self.context = TaskContext(
                task_execution_id=envelope.payload["task_execution_id"],
                state="accepted",
                # The center creates state_version=0 then advances it to 1 for
                # task.start dispatching.  The edge acknowledgement must be
                # strictly newer than that dispatch state.
                state_version=2,
                route_snapshot=route,
                current_waypoint_index=initial_waypoint_index,
                start_command_id=envelope.payload["command_id"],
                record_rosbag=bool(command.get("record_rosbag", False)),
                docking=docking,
            )
            self._persist()
            if reverse_return:
                LOGGER.info(
                    "loop task %s starts at route end and returns through waypoints in reverse order",
                    self.context.task_execution_id,
                )
            else:
                LOGGER.info(
                    "task %s starts from nearest waypoint %d; %d earlier waypoints are complete",
                    self.context.task_execution_id,
                    initial_waypoint_index,
                    initial_waypoint_index,
                )

    def _assert_map_constraints(self, route: dict) -> None:
        map_info = dict(route.get("map") or {})
        local_dir = str(map_info.get("local_map_dir") or "")
        manifest = load_map_manifest(local_dir) if local_dir else {}
        if not manifest:
            manifest = {
                "coordinate_mode": map_info.get("coordinate_mode", ""),
                "scene_scope": map_info.get("scene_scope", ""),
                "localization_mode": map_info.get("localization_mode", ""),
                "origin_status": map_info.get("origin_status", ""),
                "completeness": map_info.get("completeness") or map_info.get("origin_status") or "",
            }
        constraints = constraints_from_manifest(
            manifest,
            requested_scene_scope=str(route.get("scene_scope") or map_info.get("scene_scope") or ""),
        )
        try:
            validate_route_against_map(
                constraints,
                scene_scope=str(route.get("scene_scope") or constraints.get("scene_scope") or ""),
                waypoints=route.get("waypoints") or [],
            )
        except MapConstraintError as exc:
            raise ProtocolError(exc.code, exc.message) from exc

    def _nearest_waypoint_index(self, route: dict) -> int:
        waypoints = route.get("waypoints") or []
        if not waypoints:
            return 0
        pose = self.navigation.latest_pose()
        if pose is None:
            return 0
        try:
            pose_x = float(pose.x)
            pose_y = float(pose.y)
        except (AttributeError, TypeError, ValueError):
            return 0
        if not isfinite(pose_x) or not isfinite(pose_y):
            return 0
        distances = []
        for waypoint in waypoints:
            try:
                distance = hypot(pose_x - float(waypoint["x"]), pose_y - float(waypoint["y"]))
            except (KeyError, TypeError, ValueError):
                distance = float("inf")
            distances.append(distance)
        nearest = min(range(len(distances)), key=distances.__getitem__)
        return nearest if isfinite(distances[nearest]) else 0

    def launch_prepared_task(self) -> None:
        """Start navigation after the accepted command acknowledgement is queued."""
        with self._lock:
            if not self.context or self.context.state != "accepted":
                raise ProtocolError("TASK_CONTEXT_MISMATCH", "accepted task context is missing")
            route = self.context.route_snapshot
            self._task_started_at = time.monotonic()
            self._start_task_rosbag()
            self._segments = self.map_set_coordinator.build_segments(route) if self.map_set_coordinator else []
            if self._segments:
                initial_index = self.context.current_waypoint_index
                segment_index = next(
                    (
                        index
                        for index, segment in enumerate(self._segments)
                        if segment.start_index <= initial_index < segment.end_index
                    ),
                    0,
                )
                self.map_set_coordinator.activate(self._segments[segment_index])
                self._send_segment(segment_index, initial_index)
            else:
                self._send_from(self.context.current_waypoint_index)

    def start_task(self, envelope: MessageEnvelope) -> None:
        """Compatibility entry point used by tests and direct callers."""
        self.prepare_task_start(envelope)
        self.launch_prepared_task()

    def _send_segment(self, segment_index: int, start_index: int | None = None) -> None:
        if not self.context:
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "task context is missing")
        if not self._prepare_robot_for_navigation():
            return
        segment = self._segments[segment_index]
        goal_start_index = max(segment.start_index, start_index if start_index is not None else segment.start_index)
        self.context.current_segment_index = segment_index
        self.context.current_waypoint_index = goal_start_index
        self._goal_offset = goal_start_index
        waypoint = self.context.route_snapshot["waypoints"][goal_start_index]
        self._apply_navigation_profile(goal_start_index)
        self._set_localization_policy(waypoint, "moving")
        waypoints = [waypoint]
        accepted = self.navigation.send_waypoints(waypoints, self.on_feedback, self.on_navigation_result)
        if not accepted:
            self._fail("NAV_STACK_NOT_READY", "FollowWaypoints goal was rejected")
            return
        self.context.state = "running"
        self.context.state_version += 1
        self._persist()
        self._emit(
            "task.started",
            extra={
                "initial_waypoint_index": goal_start_index,
                "skipped_waypoints": goal_start_index,
                "execution_waypoint_order": [
                    point.get("map_point_number", int(point.get("sequence", 0)) + 1)
                    for point in self.context.route_snapshot["waypoints"][goal_start_index:]
                ],
            },
        )
        self.on_feedback(0, milestone="target_dispatched")
        if self._segment_avoidance_enabled:
            self._start_obstacle_monitor()

    def _send_from(self, index: int) -> None:
        if not self.context:
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "task context is missing")
        if not self._prepare_robot_for_navigation():
            return
        self._apply_navigation_profile(index)
        waypoint = self.context.route_snapshot["waypoints"][index]
        self._set_localization_policy(waypoint, "moving")
        waypoints = [waypoint]
        self._goal_offset = index
        accepted = self.navigation.send_waypoints(waypoints, self.on_feedback, self.on_navigation_result)
        if not accepted:
            self._fail("NAV_STACK_NOT_READY", "FollowWaypoints goal was rejected")
            return
        self.context.state = "running"
        self.context.state_version += 1
        self._persist()
        self._emit(
            "task.started",
            extra={
                "initial_waypoint_index": index,
                "skipped_waypoints": index,
                "execution_waypoint_order": [
                    point.get("map_point_number", int(point.get("sequence", 0)) + 1)
                    for point in self.context.route_snapshot["waypoints"][index:]
                ],
            },
        )
        self.on_feedback(0, milestone="target_dispatched")
        if self._segment_avoidance_enabled:
            self._start_obstacle_monitor()

    def _set_localization_policy(self, waypoint: dict, phase: str) -> None:
        setter = getattr(self.navigation, "set_localization_policy", None)
        if callable(setter):
            setter(waypoint.get("localization_mode", "ndt"), phase)

    def _absolute_localization_ready(self, timeout_seconds: float = 5.0) -> bool:
        getter = getattr(self.navigation, "localization_decision", None)
        if not callable(getter):
            return True
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        while time.monotonic() <= deadline:
            decision = getter() or {}
            source = str(decision.get("active_source") or "")
            if source in {"ndt_imu", "rtk_imu"} and bool(decision.get("absolute_stable")):
                return True
            time.sleep(0.1)
        return False

    def _prepare_robot_for_navigation(self) -> bool:
        if self._navigation_prepared:
            return True
        prepare = getattr(self.navigation, "prepare_for_navigation", None)
        if not callable(prepare):
            self._navigation_prepared = True
            return True
        LOGGER.info("standing robot before starting navigation")
        if prepare(timeout_seconds=self.standup_confirmation_timeout_seconds):
            self._navigation_prepared = True
            return True
        self._stop_obstacle_monitor()
        self._fail(
            "ROBOT_STANDUP_FAILED",
            "SDK did not confirm that the robot was standing; navigation was not started",
        )
        return False

    def pause_task(self, execution_id: str) -> dict:
        with self._lock:
            self._paused_for_localization = False
            self._assert_execution(execution_id)
            self._stop_obstacle_monitor()
            if self.context.state == "paused":
                return {"final_task_state": "paused", "state_version": self.context.state_version, "robot_stopped": True}
            if self.context.state not in {"accepted", "running", "pausing", "resuming", "interrupted"}:
                raise ProtocolError("INVALID_TASK_STATE", f"cannot pause from {self.context.state}")
            previous_state = self.context.state
            self.context.state = "pausing"
            self.context.state_version += 1
            self._persist()
            self._emit("task.pausing")
            if previous_state != "accepted" and not self.navigation.cancel_navigation():
                raise ProtocolError("NAVIGATION_CANCEL_FAILED", "Nav2 action cancel failed")
            stop_motion = getattr(self.navigation, "stop_motion", None)
            if callable(stop_motion):
                stop_motion()
            if not self.navigation.is_robot_stopped():
                raise ProtocolError("ROBOT_NOT_STOPPED", "robot speed did not reach stop threshold")
            self._restore_navigation_profile()
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
            self._paused_for_localization = False
            self._assert_execution(execution_id)
            if self.context.state == "running":
                return {"final_task_state": "running", "state_version": self.context.state_version, "resume_from_waypoint_index": self.context.current_waypoint_index}
            if self.context.state not in {"accepted", "paused", "pausing", "resuming", "interrupted"}:
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

    def force_exit(self, execution_id: str) -> dict:
        """Idempotently clear any local motion task, regardless of its state."""
        with self._lock:
            self._stop_obstacle_monitor()
            self._stop_task_rosbag()
            context = self.context
            if context:
                cancelled = self.navigation.cancel_navigation(timeout_seconds=8.0)
                if not cancelled:
                    LOGGER.error("force-exit could not confirm Nav2 goal cancellation")
            # Stop command delivery before any potentially slow parameter
            # restoration.  This prevents a force-exit from leaving Nav2
            # velocity active while collision-monitor services are busy.
            self.navigation.stop_motion()
            self._restore_navigation_profile()
            if context:
                context.state = "cancelled"
                context.state_version += 1
                self._persist()
                self.store.clear_task_context(context.task_execution_id, "cancelled")
                self.context = None
            return {"final_task_state": "cancelled", "state_version": context.state_version if context else 0, "robot_stopped": self.navigation.is_robot_stopped(), "cleared": True}

    def cancel_task(self, execution_id: str) -> dict:
        with self._lock:
            self._assert_execution(execution_id)
            self._stop_obstacle_monitor()
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
            self._stop_task_rosbag()
            self._restore_navigation_profile()
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

    def on_feedback(
        self,
        current_waypoint_index: int,
        distance_remaining_m: float | None = None,
        *,
        milestone: str = "",
        completed_waypoints: int | None = None,
    ) -> None:
        with self._lock:
            if not self.context or self.context.state != "running":
                return
            current_waypoint_index += self._goal_offset
            self.context.current_waypoint_index = current_waypoint_index
            self.context.state_version += 1
            self._persist()
            total = len(self.context.route_snapshot["waypoints"])
            waypoint = self.context.route_snapshot["waypoints"][current_waypoint_index] if current_waypoint_index < total else None
            pose = self.navigation.latest_pose()
            robot_pose = None
            if pose is not None:
                robot_pose = {
                    "x": float(pose.x),
                    "y": float(pose.y),
                    "yaw": float(getattr(pose, "yaw", 0.0)),
                    "sampled_at": getattr(pose, "sampled_at", None),
                }
            self.event_callback(
                "task.progress",
                {
                    "task_execution_id": self.context.task_execution_id,
                    "state": "running",
                    "state_version": self.context.state_version,
                    "current_waypoint_index": current_waypoint_index,
                    "current_waypoint_id": waypoint["waypoint_id"] if waypoint else "",
                    "completed_waypoints": current_waypoint_index if completed_waypoints is None else completed_waypoints,
                    "total_waypoints": total,
                    "distance_remaining_m": distance_remaining_m,
                    "estimated_time_remaining_s": None,
                    "reported_at": now_iso(),
                    "milestone": milestone or None,
                    "execution_waypoint_order": [
                        point.get("map_point_number", int(point.get("sequence", 0)) + 1)
                        for point in self.context.route_snapshot["waypoints"]
                    ],
                    "waypoint": {
                        "waypoint_id": waypoint.get("waypoint_id"),
                        "map_point_number": waypoint.get("map_point_number", int(waypoint.get("sequence", 0)) + 1),
                        "name": waypoint.get("name") or "",
                        "x": float(waypoint["x"]),
                        "y": float(waypoint["y"]),
                        "yaw": float(waypoint["yaw"]),
                    } if waypoint else None,
                    "robot_pose": robot_pose,
                },
                "",
            )

    def on_navigation_result(self, status: str, error_message: str = "", details: dict | None = None) -> None:
        with self._lock:
            if not self.context or self.context.state in {"pausing", "cancelling", "paused", "cancelled"}:
                return
            if status == "succeeded":
                self._stop_obstacle_monitor()
                missed = list((details or {}).get("missed_waypoints") or [])
                if missed:
                    absolute_missed = [index + self._goal_offset for index in missed]
                    self._fail(
                        "NAVIGATION_MISSED_WAYPOINTS",
                        f"Nav2 reported missed waypoints: {absolute_missed}",
                    )
                    return
                reached_index = self._goal_offset
                reached_waypoint = self.context.route_snapshot["waypoints"][reached_index]
                self._set_localization_policy(reached_waypoint, "stationary")
                if not self._absolute_localization_ready():
                    self.navigation.stop_motion()
                    self._restore_navigation_profile()
                    self._paused_for_localization = True
                    self.context.state = "paused"
                    self.context.current_waypoint_index = reached_index
                    self.context.state_version += 1
                    self._persist()
                    self._emit(
                        "task.paused",
                        code="ABSOLUTE_LOCALIZATION_REQUIRED",
                        message="waypoint reached by dead reckoning; waiting for NDT or RTK confirmation",
                    )
                    return

                self.on_feedback(
                    0,
                    0.0,
                    milestone="waypoint_reached",
                    completed_waypoints=reached_index + 1,
                )

                next_waypoint_index = reached_index + 1
                self.context.current_waypoint_index = next_waypoint_index
                self.context.state_version += 1
                self._persist()
                total_waypoints = len(self.context.route_snapshot["waypoints"])
                if next_waypoint_index < total_waypoints:
                    if self._segments:
                        current_segment = self._segments[self.context.current_segment_index]
                        if next_waypoint_index >= current_segment.end_index:
                            next_segment_index = self.context.current_segment_index + 1
                            next_segment = self._segments[next_segment_index]
                            self._emit("task.map_switching")
                            try:
                                self.map_set_coordinator.activate(next_segment)
                            except Exception as exc:
                                self._fail("MAP_SWITCH_FAILED", str(exc))
                                return
                            self._send_segment(next_segment_index, next_waypoint_index)
                            return
                    self._send_from(next_waypoint_index)
                    return
                self._restore_navigation_profile()
                pose_error = self._final_pose_error()
                if pose_error:
                    self._fail(*pose_error)
                    return
                if self._is_docking_task() and callable(self.docking_arrived_handler):
                    try:
                        self.docking_arrived_handler(dict(self.context.docking or {}))
                    except Exception as exc:
                        self._fail("DOCK_CHARGE_START_FAILED", str(exc))
                        return
                self._stop_task_rosbag()
                self._navigation_prepared = False
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
                        "rosbag": self._rosbag_state,
                    },
                    "",
                    "",
                )
            elif status == "cancelled":
                self._stop_obstacle_monitor()
                self._stop_task_rosbag()
                self._restore_navigation_profile()
                return
            else:
                self._restore_navigation_profile()
                if self._hold_blocked_task():
                    return
                self._stop_obstacle_monitor()
                self._fail("NAVIGATION_FAILED", error_message or status)

    def _is_docking_task(self) -> bool:
        return bool(self.context and (self.context.docking or {}).get("enabled"))

    def _apply_docking_profile(self, waypoint_index: int) -> None:
        if not self._is_docking_task():
            return
        final_index = int((self.context.docking or {}).get("final_waypoint_index", 1))
        setter = getattr(self.navigation, "set_docking_profile", None)
        if callable(setter):
            setter(final_approach=waypoint_index >= final_index)
        if waypoint_index >= final_index:
            self._stop_obstacle_monitor()
            self._emit("task.docking_final_approach", message="进入充电桩末段：微速、实时避障关闭")

    def _apply_navigation_profile(self, waypoint_index: int) -> None:
        if not self.context:
            return
        waypoints = self.context.route_snapshot.get("waypoints") or []
        target = waypoints[waypoint_index]
        source = waypoints[waypoint_index - 1] if waypoint_index > 0 else None
        # The robot may start from anywhere before the first waypoint.  Treat
        # the first waypoint's flag as the profile for that initial approach;
        # for all later legs, the source point controls its "to next" leg.
        profile_waypoint = source if source is not None else target
        avoid_obstacles = bool(profile_waypoint.get("avoidance_to_next", True))
        precision_goal = False
        if self._is_docking_task():
            final_index = int((self.context.docking or {}).get("final_waypoint_index", 1))
            if waypoint_index >= final_index:
                avoid_obstacles = False
            precision_goal = waypoint_index == final_index
        self._segment_avoidance_enabled = avoid_obstacles
        setter = getattr(self.navigation, "set_waypoint_profile", None)
        if callable(setter):
            setter(
                avoid_obstacles=avoid_obstacles,
                require_yaw=bool(target.get("require_yaw", False)) or precision_goal,
            )
        precision_setter = getattr(self.navigation, "set_goal_precision", None)
        if callable(precision_setter):
            precision_setter(enabled=precision_goal)
        self._apply_docking_profile(waypoint_index)
        if not avoid_obstacles:
            self._stop_obstacle_monitor()

    def _restore_docking_profile(self) -> None:
        setter = getattr(self.navigation, "set_docking_profile", None)
        if callable(setter):
            setter(final_approach=False)

    def _restore_navigation_profile(self) -> None:
        self._segment_avoidance_enabled = True
        try:
            setter = getattr(self.navigation, "set_waypoint_profile", None)
            if callable(setter):
                setter(avoid_obstacles=True, require_yaw=False)
            precision_setter = getattr(self.navigation, "set_goal_precision", None)
            if callable(precision_setter):
                precision_setter(enabled=False)
            self._restore_docking_profile()
        except Exception:
            LOGGER.exception("failed to restore default navigation profile")

    def _hold_blocked_task(self) -> bool:
        """Keep a genuinely obstructed task alive and retry it for five minutes."""
        if not self._obstacle_monitor_enabled() or self._last_obstacle_seen_at is None:
            return False
        if time.monotonic() - self._last_obstacle_seen_at > 10.0:
            return False
        if self._task_started_at is None:
            self._task_started_at = time.monotonic()
        elapsed = time.monotonic() - self._task_started_at
        minimum = self.obstacle_speech.minimum_blocked_task_seconds
        if elapsed >= minimum:
            return False
        self.context.state_version += 1
        self._persist()
        self.event_callback(
            "task.progress",
            {
                "task_execution_id": self.context.task_execution_id,
                "state": "running",
                "state_version": self.context.state_version,
                "current_waypoint_index": self.context.current_waypoint_index,
                "completed_waypoints": self.context.current_waypoint_index,
                "total_waypoints": len(self.context.route_snapshot["waypoints"]),
                "distance_remaining_m": None,
                "estimated_time_remaining_s": round(minimum - elapsed),
                "reported_at": now_iso(),
            },
            "",
        )
        LOGGER.warning("navigation blocked; keeping task alive for %.0fs more", minimum - elapsed)
        self._blocked_retry_timer = threading.Timer(
            self.obstacle_speech.navigation_retry_seconds,
            self._retry_blocked_navigation,
        )
        self._blocked_retry_timer.daemon = True
        self._blocked_retry_timer.start()
        return True

    def _retry_blocked_navigation(self) -> None:
        with self._lock:
            if not self.context or self.context.state != "running":
                return
            self._send_from(self.context.current_waypoint_index)

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
        tolerance = (
            self.docking_goal_tolerance_m
            if self._is_docking_task()
            else self.final_waypoint_tolerance_m
        )
        if distance > tolerance:
            return (
                "FINAL_POSE_OUT_OF_TOLERANCE",
                (
                    f"final pose is {distance:.2f}m from last waypoint "
                    f"(tolerance {tolerance:.2f}m)"
                ),
            )
        if self._is_docking_task():
            try:
                yaw_error = abs(
                    atan2(
                        sin(float(pose.yaw) - float(final_waypoint["yaw"])),
                        cos(float(pose.yaw) - float(final_waypoint["yaw"])),
                    )
                )
            except (AttributeError, KeyError, TypeError, ValueError):
                return ("FINAL_YAW_UNAVAILABLE", "final docking heading is unavailable")
            if yaw_error > self.docking_goal_yaw_tolerance_rad:
                return (
                    "FINAL_YAW_OUT_OF_TOLERANCE",
                    (
                        f"final heading is {yaw_error:.3f}rad from docking heading "
                        f"(tolerance {self.docking_goal_yaw_tolerance_rad:.3f}rad)"
                    ),
                )
        return None

    def _fail(self, code: str, message: str) -> None:
        if not self.context:
            return
        self._stop_task_rosbag()
        self._navigation_prepared = False
        self._restore_navigation_profile()
        self.context.state = "failed"
        self.context.state_version += 1
        self._persist()
        self._emit("task.failed", code=code, message=message)
        self.store.clear_task_context(self.context.task_execution_id, "failed")
        self.start_result_callback(
            self.context.start_command_id,
            "failed",
            {
                "final_task_state": "failed",
                "state_version": self.context.state_version,
                "rosbag": self._rosbag_state,
            },
            code,
            message,
        )

    def _assert_execution(self, execution_id: str) -> None:
        if not self.context or self.context.task_execution_id != execution_id:
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "task execution does not match local context")

    def _persist(self) -> None:
        if self.context:
            self.store.save_task_context(self.context.__dict__)

    def _emit(
        self,
        event_type: str,
        *,
        code: str = "",
        message: str = "",
        extra: dict | None = None,
    ) -> None:
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
                "rosbag": self._rosbag_state if self.context.record_rosbag else None,
                **(extra or {}),
            },
            "",
        )

    def _start_task_rosbag(self) -> None:
        if not self.context or not self.context.record_rosbag:
            self._rosbag_state = {}
            return
        if not self.rosbag_recorder:
            self._rosbag_state = {"running": False, "available": False, "error": "recorder unavailable"}
            return
        label = f"task_{self.context.task_execution_id[:8]}"
        try:
            self._rosbag_state = self.rosbag_recorder.start(label)
        except Exception as exc:
            LOGGER.exception("failed to start navigation rosbag")
            self._rosbag_state = {"running": False, "available": True, "error": str(exc)}

    def _stop_task_rosbag(self) -> None:
        if not self.context or not self.context.record_rosbag or not self.rosbag_recorder:
            return
        try:
            self._rosbag_state = self.rosbag_recorder.stop()
        except Exception as exc:
            LOGGER.exception("failed to stop navigation rosbag")
            self._rosbag_state = {
                **self._rosbag_state,
                "running": False,
                "error": str(exc),
            }
