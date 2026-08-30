from __future__ import annotations

import logging
import hashlib
import json
import threading
import time
import uuid
from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin
from pathlib import Path
from typing import Callable, Protocol

from .local_store import LocalStore
from .map_coordinate import (
    MapConstraintError,
    constraints_from_manifest,
    validate_route_against_map,
    waypoint_localization_mode,
)
from .map_package_finalize import load_map_manifest
from .protocol import MessageEnvelope, ProtocolError, now_iso


LOGGER = logging.getLogger(__name__)

# Duplicate map clicks for a round-trip (1-2-3-2-1) rarely land on the exact
# same XY.  If the robot is standing on that cluster, start from the earliest
# copy so the outbound legs are not skipped.  After the first copy is marked
# complete, a later resume must also refuse to jump to the return copy.
WAYPOINT_COLOCATION_M = 1.0
# Hand-clicked return-to-start points are often 1-2 m off the original click.
# If the nearest waypoint is the route end and the robot is also that close
# to waypoint 1, start the outbound legs instead of treating 1..N-1 as done.
ROUND_TRIP_START_MARGIN_M = 2.0
# Patrol clicks on a "straight" corridor are rarely colinear. NavigateThroughPoses
# then builds a left-right polyline, and MPPI hugs that heading. Flatten clicks
# whose lateral error is below this threshold; keep real turns.
WAYPOINT_STRAIGHTEN_M = 0.40
# Nav2 feedback arrives around 20 Hz.  UI progress does not need that rate,
# and persisting every sample can monopolize the platform SQLite writer.
TASK_PROGRESS_MIN_INTERVAL_SECONDS = 1.0


def _waypoint_xy(waypoint: dict) -> tuple[float, float] | None:
    try:
        x = float(waypoint["x"])
        y = float(waypoint["y"])
    except (KeyError, TypeError, ValueError):
        return None
    if not isfinite(x) or not isfinite(y):
        return None
    return x, y


def _waypoints_are_colocated(left, right) -> bool:
    left_xy = _waypoint_xy(left) if isinstance(left, dict) else left
    right_xy = _waypoint_xy(right) if isinstance(right, dict) else right
    if left_xy is None or right_xy is None:
        return False
    return hypot(left_xy[0] - right_xy[0], left_xy[1] - right_xy[1]) <= WAYPOINT_COLOCATION_M


def _segment_projection(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> tuple[float, float, float]:
    dx = bx - ax
    dy = by - ay
    length2 = dx * dx + dy * dy
    if length2 < 1e-12:
        return hypot(px - ax, py - ay), ax, ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    qx = ax + t * dx
    qy = ay + t * dy
    return hypot(px - qx, py - qy), qx, qy


def _douglas_peucker_indices(points: list[tuple[float, float]], epsilon_m: float) -> list[int]:
    if len(points) < 3:
        return list(range(len(points)))
    keep = [False] * len(points)
    keep[0] = True
    keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        start, end = stack.pop()
        ax, ay = points[start]
        bx, by = points[end]
        farthest_i = -1
        farthest_d = -1.0
        for index in range(start + 1, end):
            dist, _, _ = _segment_projection(points[index][0], points[index][1], ax, ay, bx, by)
            if dist > farthest_d:
                farthest_d = dist
                farthest_i = index
        if farthest_i >= 0 and farthest_d > epsilon_m:
            keep[farthest_i] = True
            stack.append((start, farthest_i))
            stack.append((farthest_i, end))
    return [index for index, flagged in enumerate(keep) if flagged]


def straighten_pass_through_waypoints(
    waypoints: list[dict],
    epsilon_m: float = WAYPOINT_STRAIGHTEN_M,
) -> list[dict]:
    """Project nearly-colinear via points onto the intended straight segments.

    The cloud route still owns the original clicks for progress and speech.
    Only the Nav2 goal geometry is flattened so a hand-drawn corridor does not
    become a snaking polyline.
    """
    if len(waypoints) < 3:
        return [dict(waypoint) for waypoint in waypoints]
    points: list[tuple[float, float]] = []
    for waypoint in waypoints:
        xy = _waypoint_xy(waypoint)
        if xy is None:
            return [dict(item) for item in waypoints]
        points.append(xy)
    kept = _douglas_peucker_indices(points, epsilon_m)
    kept_set = set(kept)
    if len(kept) == len(points):
        return [dict(waypoint) for waypoint in waypoints]
    straightened = []
    for index, waypoint in enumerate(waypoints):
        item = dict(waypoint)
        if index in kept_set:
            straightened.append(item)
            continue
        prev_keep = max(k for k in kept if k <= index)
        next_keep = min(k for k in kept if k >= index)
        _, qx, qy = _segment_projection(
            points[index][0],
            points[index][1],
            points[prev_keep][0],
            points[prev_keep][1],
            points[next_keep][0],
            points[next_keep][1],
        )
        item["x"] = qx
        item["y"] = qy
        straightened.append(item)
    return straightened


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
        final_waypoint_tolerance_m: float = 0.45,
        docking_goal_tolerance_m: float = 0.08,
        docking_goal_yaw_tolerance_rad: float = 0.0872665,
        standup_confirmation_timeout_seconds: float = 12.0,
        map_set_coordinator=None,
        obstacle_speech=None,
        waypoint_speech=None,
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
        self.waypoint_speech = waypoint_speech
        self.rosbag_recorder = rosbag_recorder
        self.docking_arrived_handler = docking_arrived_handler
        self._rosbag_state: dict = {}
        self._segments = []
        self._lock = threading.RLock()
        self._goal_offset = 0
        self._last_progress_emit_at: float | None = None
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
        self._dispatched_count = 0
        self._patrol_final_approach_applied = False
        self._last_target_index = -1
        self._last_reached_index = -1
        self._last_localization_policy: tuple[str, str] | None = None
        self._speech_waiting_index: int | None = None
        self._speech_wait_finished = False
        self._speech_wait_thread: threading.Thread | None = None
        self._waypoint_localization_ready_index: int | None = None
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
        if self._recovery_attempts <= 3:
            self._perform_obstacle_reverse()
        if self._recovery_attempts >= 3:
            self._leave_route_announced = True
            self._emit_obstacle_speech("leave_route", self._recovery_attempts, observation)

    def _perform_obstacle_reverse(self) -> None:
        """Safely back away a short distance before retrying the current goal."""
        cancel = getattr(self.navigation, "cancel_navigation", None)
        velocity = getattr(self.navigation, "teleop_velocity", None)
        stop = getattr(self.navigation, "stop_motion", None)
        if not callable(velocity):
            LOGGER.warning("obstacle recovery requested but teleop reverse is unavailable")
            return
        if callable(cancel):
            cancel(timeout_seconds=2.0)
        speed = -abs(float(getattr(self.obstacle_speech, "reverse_speed_mps", 0.12)))
        duration = max(0.2, min(float(getattr(self.obstacle_speech, "reverse_duration_seconds", 1.5)), 3.0))
        LOGGER.warning("obstacle recovery reverse start speed=%.2f duration=%.2fs", speed, duration)
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline:
                velocity(vx=speed, vy=0.0, yaw_rate=0.0)
                time.sleep(0.1)
        finally:
            if callable(stop):
                stop()
            else:
                velocity(vx=0.0, vy=0.0, yaw_rate=0.0)
            LOGGER.warning("obstacle recovery reverse finished")

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
        LOGGER.info(
            "obstacle speech episode=%s stage=%s attempt=%s distance=%s raw_planar=%s actual_planar=%s",
            self._obstacle_episode_id,
            stage,
            attempt,
            observation.get("front_obstacle_distance_m"),
            observation.get("requested_planar_speed_mps"),
            observation.get("actual_planar_speed_mps"),
        )

    def report_docking_charge(self, event_type: str, *, message: str = "", extra: dict | None = None) -> None:
        """Publish charge-contact milestones for the platform docking dialog."""
        with self._lock:
            if self._is_docking_task():
                self._emit(event_type, message=message, extra=extra)

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

            # Nav2 has already finished when the task is only waiting for the
            # reached waypoint's speech. Cancelling in that window targets a
            # stale goal handle and, more importantly, recovery must not
            # dispatch the same waypoint for a second time.
            waiting_for_speech = self._speech_waiting_index is not None
            cancelled = waiting_for_speech or self.navigation.cancel_navigation()
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
            # Keep the pending target. Re-picking the nearest remaining point
            # on a round-trip treats the return copy (point 4 of 1-2-3-2-1) as
            # closer and skips the outbound legs.
            resume_index = max(0, self.context.current_waypoint_index)
            self._paused_for_localization = False
            self.context.state = "resuming"
            self.context.state_version += 1
            self._persist()
            self._emit(
                "task.resuming",
                code="LOCALIZATION_RECOVERED",
                message="localization is stable; resuming from the pending waypoint",
            )
            if self._speech_waiting_index is not None:
                reached_index = self._speech_waiting_index
                speech_finished = self._speech_wait_finished
                self._waypoint_localization_ready_index = reached_index
                self.context.state = "running"
                self.context.state_version += 1
                self._persist()
                self._emit("task.resumed")
                if speech_finished:
                    self._speech_waiting_index = None
                    self._speech_wait_finished = False
                    self._waypoint_localization_ready_index = None
                    self._continue_after_waypoint(reached_index)
                return
            self._send_from(resume_index)

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
            self._last_target_index = initial_waypoint_index - 1
            self._last_reached_index = initial_waypoint_index - 1
            self._last_progress_emit_at = None
            self._last_localization_policy = None
            self._speech_waiting_index = None
            self._speech_wait_finished = False
            self._waypoint_localization_ready_index = None
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
        pose = self.navigation.latest_pose()
        index = self._earliest_colocated_nearest_index(waypoints, pose)
        return self._prefer_round_trip_start(waypoints, pose, index)

    def _prefer_round_trip_start(self, waypoints: list, pose, nearest_index: int) -> int:
        if nearest_index != len(waypoints) - 1 or len(waypoints) < 2 or pose is None:
            return nearest_index
        first_xy = _waypoint_xy(waypoints[0])
        last_xy = _waypoint_xy(waypoints[-1])
        if first_xy is None or last_xy is None:
            return nearest_index
        try:
            pose_x = float(pose.x)
            pose_y = float(pose.y)
        except (AttributeError, TypeError, ValueError):
            return nearest_index
        first_dist = hypot(pose_x - first_xy[0], pose_y - first_xy[1])
        last_dist = hypot(pose_x - last_xy[0], pose_y - last_xy[1])
        if first_dist <= last_dist + ROUND_TRIP_START_MARGIN_M:
            LOGGER.info(
                "round-trip start: nearest is route end (index %d, %.2fm) but start is also nearby (%.2fm); using waypoint 0",
                nearest_index,
                last_dist,
                first_dist,
            )
            return 0
        return nearest_index

    def _earliest_colocated_nearest_index(
        self,
        waypoints: list,
        pose,
        *,
        start_index: int = 0,
    ) -> int:
        if not waypoints:
            return 0
        if start_index >= len(waypoints):
            return start_index
        if pose is None:
            return start_index
        try:
            pose_x = float(pose.x)
            pose_y = float(pose.y)
        except (AttributeError, TypeError, ValueError):
            return start_index
        if not isfinite(pose_x) or not isfinite(pose_y):
            return start_index
        distances = []
        for waypoint in waypoints[start_index:]:
            xy = _waypoint_xy(waypoint)
            distances.append(hypot(pose_x - xy[0], pose_y - xy[1]) if xy else float("inf"))
        nearest_offset = min(range(len(distances)), key=distances.__getitem__)
        if not isfinite(distances[nearest_offset]):
            return start_index
        nearest_index = start_index + nearest_offset
        nearest_xy = _waypoint_xy(waypoints[nearest_index])
        if nearest_xy is None:
            return start_index
        chosen_index = nearest_index
        for index in range(start_index, len(waypoints)):
            if _waypoints_are_colocated(waypoints[index], nearest_xy):
                chosen_index = index
                break
        if chosen_index <= start_index:
            return chosen_index
        # A later copy of an earlier waypoint is not "ahead" on the route.
        # After 1 is complete on a 1-2-3-2-1 round trip, the start/end cluster
        # would otherwise jump to 5.  While going to 2, the return copy 4 is
        # often closer and would skip 3.
        for index in range(chosen_index):
            if not _waypoints_are_colocated(waypoints[index], waypoints[chosen_index]):
                continue
            keep_index = max(start_index, index)
            LOGGER.info(
                "keeping pending waypoint %d instead of later colocated copy %d",
                keep_index,
                chosen_index,
            )
            return keep_index
        return chosen_index

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
        segment = self._segments[segment_index]
        goal_start_index = max(segment.start_index, start_index if start_index is not None else segment.start_index)
        self.context.current_segment_index = segment_index
        self._dispatch_navigation(goal_start_index)

    def _send_from(self, index: int) -> None:
        self._dispatch_navigation(index)

    def _batch_end_index(self, start_index: int) -> int:
        """Last exclusive index of one Nav2 goal. Patrol vias share a goal; stops split it."""
        waypoints = self.context.route_snapshot["waypoints"]
        total = len(waypoints)
        if self._is_docking_task():
            return min(start_index + 1, total)
        end = total
        if self._segments:
            end = min(self._segments[self.context.current_segment_index].end_index, total)
        start_wp = waypoints[start_index]
        start_correction_mode = waypoint_localization_mode(start_wp.get("localization_mode"))
        if (
            bool(start_wp.get("require_yaw", False))
            or float(start_wp.get("dwell_seconds") or 0) > 0
            or bool(start_wp.get("speech_template_id"))
        ):
            return start_index + 1
        for index in range(start_index + 1, end):
            # A correction mode belongs to the target waypoint. End the Nav2
            # goal before a mode boundary so the current waypoint can be
            # confirmed with its requested source before dispatching the next.
            if waypoint_localization_mode(
                waypoints[index].get("localization_mode")
            ) != start_correction_mode:
                return index
            if index == end - 1:
                break
            waypoint = waypoints[index]
            if (
                bool(waypoint.get("require_yaw", False))
                or float(waypoint.get("dwell_seconds") or 0) > 0
                or bool(waypoint.get("speech_template_id"))
            ):
                return index + 1
        return self._through_poses_end_index(waypoints, start_index, end)

    def _current_pose_xy(self) -> tuple[float, float] | None:
        pose = self.navigation.latest_pose() if self.navigation else None
        if pose is None:
            return None
        try:
            x = float(pose.x)
            y = float(pose.y)
        except (AttributeError, TypeError, ValueError):
            return None
        if not isfinite(x) or not isfinite(y):
            return None
        return x, y

    def _through_poses_end_index(self, waypoints: list, start_index: int, end: int) -> int:
        """Keep a through-poses goal from ending at the robot's current cluster.

        NavigateThroughPoses is complete once the last pose is within the goal
        checker. A round-trip that returns to start would therefore succeed
        immediately if the robot is already standing on that cluster.
        """
        if end - start_index <= 1:
            return end
        last_xy = _waypoint_xy(waypoints[end - 1])
        origin_xy = self._current_pose_xy() or _waypoint_xy(waypoints[start_index])
        if last_xy is None or origin_xy is None:
            return end
        if hypot(origin_xy[0] - last_xy[0], origin_xy[1] - last_xy[1]) > ROUND_TRIP_START_MARGIN_M:
            return end
        farthest_index = start_index
        farthest_dist = -1.0
        for index in range(start_index, end):
            xy = _waypoint_xy(waypoints[index])
            if xy is None:
                continue
            dist = hypot(origin_xy[0] - xy[0], origin_xy[1] - xy[1])
            if dist >= farthest_dist:
                farthest_index = index
                farthest_dist = dist
        if farthest_index <= start_index or farthest_dist <= WAYPOINT_COLOCATION_M:
            return end
        LOGGER.info(
            "splitting round-trip through-poses at farthest waypoint %d (%.2fm) so the goal is not already complete",
            farthest_index,
            farthest_dist,
        )
        return farthest_index + 1

    def _apply_batch_travel_yaw(self, batch: list[dict], start_index: int) -> None:
        """Point pass-through poses along the Nav2 goal, not the cloud click yaw."""
        for item_index, waypoint in enumerate(batch):
            stop = item_index + 1 == len(batch)
            if stop and bool(waypoint.get("require_yaw", False)):
                waypoint["yaw"] = float(waypoint.get("yaw") or 0.0)
                continue
            if not stop:
                nxt = batch[item_index + 1]
                dx = float(nxt["x"]) - float(waypoint["x"])
                dy = float(nxt["y"]) - float(waypoint["y"])
                if hypot(dx, dy) >= 1e-3:
                    waypoint["yaw"] = atan2(dy, dx)
                    continue
            if item_index > 0:
                previous = batch[item_index - 1]
                dx = float(waypoint["x"]) - float(previous["x"])
                dy = float(waypoint["y"]) - float(previous["y"])
                if hypot(dx, dy) >= 1e-3:
                    waypoint["yaw"] = atan2(dy, dx)
                    continue
            waypoint["yaw"] = self._dispatch_yaw(
                start_index + item_index, waypoint, stop=stop
            )

    def _dispatch_yaw(self, index: int, waypoint: dict, *, stop: bool) -> float:
        if stop and bool(waypoint.get("require_yaw", False)):
            return float(waypoint.get("yaw") or 0.0)
        waypoints = self.context.route_snapshot["waypoints"]
        if not stop and index + 1 < len(waypoints):
            nxt = waypoints[index + 1]
            dx = float(nxt["x"]) - float(waypoint["x"])
            dy = float(nxt["y"]) - float(waypoint["y"])
            if hypot(dx, dy) >= 1e-3:
                return atan2(dy, dx)
        return self._pass_through_yaw(index, waypoint)

    def _dispatch_navigation(self, index: int) -> None:
        if not self.context:
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "task context is missing")
        if not self._prepare_robot_for_navigation():
            return
        waypoints = self.context.route_snapshot["waypoints"]
        if index >= len(waypoints):
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "waypoint index is past the route")
        batch_end = self._batch_end_index(index)
        original_batch = [dict(waypoints[item_index]) for item_index in range(index, batch_end)]
        batch = (
            original_batch
            if self._is_docking_task()
            else straighten_pass_through_waypoints(original_batch)
        )
        self._apply_batch_travel_yaw(batch, index)
        last_index = batch_end - 1
        if batch is not original_batch:
            max_shift = max(
                (
                    hypot(
                        float(straight["x"]) - float(original["x"]),
                        float(straight["y"]) - float(original["y"]),
                    )
                    for straight, original in zip(batch, original_batch)
                ),
                default=0.0,
            )
            if max_shift > 1e-3:
                LOGGER.info(
                    "straightened patrol Nav2 goal by up to %.2fm so colinear clicks do not weave",
                    max_shift,
                )
        single = len(batch) == 1
        # A one-pose batch is a real stop even when it is not the route's
        # overall last point (speech, dwell and localization boundaries split
        # patrols this way).  Cruise speed has a larger turning radius than
        # the 0.35 m goal window and can make the dog orbit such a waypoint.
        initial_final_approach = single and not self._is_docking_task()
        self._patrol_final_approach_applied = initial_final_approach
        self._apply_navigation_profile(
            index,
            force_final=initial_final_approach,
            force_require_yaw=single and bool(waypoints[last_index].get("require_yaw", False)),
        )
        self._set_localization_policy(batch[0], "moving")
        self._goal_offset = index
        self._dispatched_count = len(batch)
        accepted = self.navigation.send_waypoints(batch, self.on_feedback, self.on_navigation_result)
        if not accepted:
            self._fail("NAV_STACK_NOT_READY", "FollowWaypoints goal was rejected")
            return
        self.context.current_waypoint_index = index
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
                    for point in waypoints[index:]
                ],
            },
        )
        self.on_feedback(0, milestone="target_dispatched")
        if self._segment_avoidance_enabled:
            self._start_obstacle_monitor()

    def _pass_through_yaw(self, index: int, waypoint: dict) -> float:
        """Use travel heading for pass-through patrol points (cloud yaw is often 0)."""
        if bool(waypoint.get("require_yaw", False)):
            return float(waypoint.get("yaw") or 0.0)
        target_x = float(waypoint["x"])
        target_y = float(waypoint["y"])
        source_x = None
        source_y = None
        if index > 0:
            previous = self.context.route_snapshot["waypoints"][index - 1]
            source_x = float(previous["x"])
            source_y = float(previous["y"])
        else:
            pose = self.navigation.latest_pose() if self.navigation else None
            if pose is not None:
                source_x = float(getattr(pose, "x", target_x))
                source_y = float(getattr(pose, "y", target_y))
        if source_x is None:
            return float(waypoint.get("yaw") or 0.0)
        dx = target_x - source_x
        dy = target_y - source_y
        if hypot(dx, dy) < 1e-3:
            return float(waypoint.get("yaw") or 0.0)
        return atan2(dy, dx)

    def _set_localization_policy(self, waypoint: dict, phase: str) -> None:
        setter = getattr(self.navigation, "set_localization_policy", None)
        if not callable(setter):
            return
        mode = waypoint_localization_mode(waypoint.get("localization_mode"))
        phase = "moving" if str(phase).lower() == "moving" else "stationary"
        policy = (mode, phase)
        if policy == self._last_localization_policy:
            return
        setter(mode, phase)
        self._last_localization_policy = policy

    def _absolute_localization_ready(self, timeout_seconds: float = 5.0) -> bool:
        getter = getattr(self.navigation, "localization_decision", None)
        if not callable(getter):
            return True
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        while time.monotonic() <= deadline:
            decision = getter() or {}
            source = str(decision.get("active_source") or "")
            policy_source_ready = decision.get("policy_source_ready")
            correction_active = bool(decision.get("correction_smoothing_active", False))
            if correction_active:
                # Keep the controller at the safety boundary while the
                # localization node is moving map->lio_odom.  A single stop
                # command can be overwritten by an active Nav2 controller.
                stop_motion = getattr(self.navigation, "stop_motion", None)
                if callable(stop_motion):
                    stop_motion()
            if (
                source in {"ndt_imu", "rtk_imu", "lio_imu"}
                and bool(decision.get("absolute_stable"))
                and (policy_source_ready is None or policy_source_ready is True)
                and not correction_active
            ):
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
            waiting_for_speech = self._speech_waiting_index is not None
            if (
                previous_state != "accepted"
                and not waiting_for_speech
                and not self.navigation.cancel_navigation()
            ):
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
            if self._speech_waiting_index is not None:
                reached_index = self._speech_waiting_index
                speech_finished = self._speech_wait_finished
                self._waypoint_localization_ready_index = reached_index
                self.context.state = "running"
                self.context.state_version += 1
                self._persist()
                self._emit("task.resumed")
                if speech_finished:
                    self._speech_waiting_index = None
                    self._speech_wait_finished = False
                    self._waypoint_localization_ready_index = None
                    self._continue_after_waypoint(reached_index)
                return {
                    "final_task_state": "running",
                    "state_version": self.context.state_version,
                    "resume_from_waypoint_index": reached_index,
                    "waiting_for_waypoint_speech": not speech_finished,
                }
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
            if self.context.state in self.TERMINAL_STATES:
                # A task can finish between the low-battery active-state check
                # and this locked cancellation. Preserve its real terminal
                # outcome and avoid waiting on a Nav2 goal that no longer
                # exists. Repeated calls intentionally return the same result.
                self._stop_task_rosbag()
                self.navigation.stop_motion()
                return {
                    "final_task_state": self.context.state,
                    "state_version": self.context.state_version,
                    "robot_stopped": True,
                    "already_terminal": True,
                    "cancel_performed": False,
                }
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
                "already_terminal": False,
                "cancel_performed": True,
            }

    def on_feedback(
        self,
        current_waypoint_index: int,
        distance_remaining_m: float | None = None,
        *,
        milestone: str = "",
        completed_waypoints: int | None = None,
    ) -> None:
        apply_final = False
        policy_waypoint = None
        progress_updates: list[dict] = []
        with self._lock:
            if not self.context or self.context.state != "running":
                return
            current_waypoint_index += self._goal_offset
            total = len(self.context.route_snapshot["waypoints"])
            dispatched_final_index = self._goal_offset + max(self._dispatched_count, 1) - 1
            if current_waypoint_index < 0 or current_waypoint_index >= total:
                return
            if milestone != "waypoint_reached":
                policy_waypoint = self.context.route_snapshot["waypoints"][current_waypoint_index]
            if (
                not self._is_docking_task()
                and current_waypoint_index == dispatched_final_index
                and not self._patrol_final_approach_applied
                and milestone != "waypoint_reached"
            ):
                self._patrol_final_approach_applied = True
                apply_final = True
            if milestone == "target_dispatched":
                if current_waypoint_index > self._last_target_index:
                    progress_updates.append(
                        self._build_progress_locked(
                            current_waypoint_index,
                            "target_dispatched",
                            max(current_waypoint_index, self._last_reached_index + 1),
                            distance_remaining_m,
                        )
                    )
                    self._last_target_index = current_waypoint_index
            elif milestone == "waypoint_reached":
                while self._last_reached_index < current_waypoint_index:
                    reached_index = self._last_reached_index + 1
                    if reached_index > self._last_target_index:
                        progress_updates.append(
                            self._build_progress_locked(
                                reached_index,
                                "target_dispatched",
                                reached_index,
                                distance_remaining_m,
                            )
                        )
                        self._last_target_index = reached_index
                    progress_updates.append(
                        self._build_progress_locked(
                            reached_index,
                            "waypoint_reached",
                            reached_index + 1,
                            0.0,
                        )
                    )
                    self._last_reached_index = reached_index
            else:
                if current_waypoint_index > self._last_target_index:
                    while self._last_target_index < current_waypoint_index:
                        if self._last_reached_index < self._last_target_index:
                            self._last_reached_index = self._last_target_index
                            progress_updates.append(
                                self._build_progress_locked(
                                    self._last_reached_index,
                                    "waypoint_reached",
                                    self._last_reached_index + 1,
                                    0.0,
                                )
                            )
                        next_target_index = self._last_target_index + 1
                        progress_updates.append(
                            self._build_progress_locked(
                                next_target_index,
                                "target_dispatched",
                                max(next_target_index, self._last_reached_index + 1),
                                distance_remaining_m,
                            )
                        )
                        self._last_target_index = next_target_index
                elif (
                    self._last_progress_emit_at is None
                    or time.monotonic() - self._last_progress_emit_at
                    >= TASK_PROGRESS_MIN_INTERVAL_SECONDS
                ):
                    progress_updates.append(
                        self._build_progress_locked(
                            current_waypoint_index,
                            "",
                            (
                                completed_waypoints
                                if completed_waypoints is not None
                                else max(current_waypoint_index, self._last_reached_index + 1)
                            ),
                            distance_remaining_m,
                        )
                    )
        if policy_waypoint is not None:
            self._set_localization_policy(policy_waypoint, "moving")
        if apply_final:
            try:
                self._apply_patrol_final_approach()
            except ProtocolError:
                LOGGER.exception(
                    "final-approach navigation profile failed; continuing the current goal"
                )
        with self._lock:
            if not self.context or self.context.state != "running":
                if apply_final:
                    try:
                        self._restore_navigation_profile()
                    except Exception:
                        LOGGER.exception("failed to restore profile after late final approach")
                return
            for progress in progress_updates:
                self.event_callback("task.progress", progress, "")

    def _build_progress_locked(
        self,
        waypoint_index: int,
        milestone: str,
        completed_waypoints: int,
        distance_remaining_m: float | None,
    ) -> dict:
        waypoint = self.context.route_snapshot["waypoints"][waypoint_index]
        self.context.current_waypoint_index = waypoint_index
        self.context.state_version += 1
        self._last_progress_emit_at = time.monotonic()
        self._persist()
        pose = self.navigation.latest_pose()
        robot_pose = None
        if pose is not None:
            robot_pose = {
                "x": float(pose.x),
                "y": float(pose.y),
                "yaw": float(getattr(pose, "yaw", 0.0)),
                "sampled_at": getattr(pose, "sampled_at", None),
            }
        return {
            "task_execution_id": self.context.task_execution_id,
            "state": "running",
            "state_version": self.context.state_version,
            "current_waypoint_index": waypoint_index,
            "current_waypoint_id": waypoint.get("waypoint_id", ""),
            "completed_waypoints": completed_waypoints,
            "total_waypoints": len(self.context.route_snapshot["waypoints"]),
            "distance_remaining_m": distance_remaining_m,
            "estimated_time_remaining_s": None,
            "reported_at": now_iso(),
            "milestone": milestone or None,
            "execution_waypoint_order": [
                point.get("map_point_number", int(point.get("sequence", 0)) + 1)
                for point in self.context.route_snapshot["waypoints"]
            ],
            "execution_waypoint_index": waypoint_index,
            "waypoint": {
                "waypoint_id": waypoint.get("waypoint_id"),
                "map_point_number": waypoint.get(
                    "map_point_number", int(waypoint.get("sequence", 0)) + 1
                ),
                "name": waypoint.get("name") or "",
                "x": float(waypoint["x"]),
                "y": float(waypoint["y"]),
                "yaw": float(waypoint["yaw"]),
            },
            "robot_pose": robot_pose,
        }

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
                reached_index = self._goal_offset + max(self._dispatched_count, 1) - 1
                reached_waypoint = self.context.route_snapshot["waypoints"][reached_index]
                total_waypoints = len(self.context.route_snapshot["waypoints"])
                # A FollowWaypoints success only means Nav2's goal checker
                # accepted the pose; it does not guarantee that the
                # quadruped has finished coasting.  Confirm zero motion before
                # switching to the stationary NDT/RTK policy, otherwise the
                # first correction sample can be taken while the body is
                # still moving and create an avoidable TF correction.
                if not self._hold_final_pose():
                    LOGGER.warning(
                        "waypoint %d reached but zero-motion confirmation timed out; "
                        "continuing with stationary localization policy",
                        reached_index,
                    )
                self._set_localization_policy(reached_waypoint, "stationary")
                speech_required = bool(reached_waypoint.get("speech_template_id"))
                if speech_required:
                    self._clear_waypoint_speech_status(reached_index)
                    # Speech and localization settling run independently; the
                    # next waypoint is gated on both completion conditions.
                    self._waypoint_localization_ready_index = None
                    self._start_waypoint_speech_wait(reached_index)
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
                        message="waypoint reached by FAST-LIO; waiting for the requested correction source",
                    )
                    return
                self._waypoint_localization_ready_index = reached_index
                self.on_feedback(
                    reached_index - self._goal_offset,
                    0.0,
                    milestone="waypoint_reached",
                    completed_waypoints=reached_index + 1,
                )
                if speech_required:
                    if self._speech_wait_finished:
                        self._speech_waiting_index = None
                        self._speech_wait_finished = False
                        self._waypoint_localization_ready_index = None
                        self._continue_after_waypoint(reached_index)
                    return
                self._waypoint_localization_ready_index = None
                self._continue_after_waypoint(reached_index)
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

    def _continue_after_waypoint(self, reached_index: int) -> None:
        with self._lock:
            if not self.context or self.context.state != "running":
                return
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
            pose_error = self._final_pose_error()
            self._restore_navigation_profile()
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
            self.context.current_waypoint_index = total_waypoints
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
                    "completed_waypoints": total_waypoints,
                    "total_waypoints": total_waypoints,
                    "rosbag": self._rosbag_state,
                },
                "",
                "",
            )

    def _waypoint_speech_status_path(self, waypoint_index: int) -> Path | None:
        if not self.context or not self.waypoint_speech:
            return None
        waypoint = self.context.route_snapshot["waypoints"][waypoint_index]
        waypoint_key = hashlib.sha256(
            str(waypoint.get("waypoint_id") or waypoint_index).encode("utf-8")
        ).hexdigest()
        return (
            Path(self.waypoint_speech.status_dir)
            / self.context.task_execution_id
            / f"{waypoint_key}.json"
        )

    def _clear_waypoint_speech_status(self, waypoint_index: int) -> None:
        path = self._waypoint_speech_status_path(waypoint_index)
        if not path:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            path.unlink(missing_ok=True)
        except OSError:
            LOGGER.exception("failed to clear stale waypoint speech status path=%s", path)

    def _start_waypoint_speech_wait(self, waypoint_index: int) -> None:
        self._speech_waiting_index = waypoint_index
        self._speech_wait_finished = False
        self._speech_wait_thread = threading.Thread(
            target=self._wait_for_waypoint_speech,
            args=(self.context.task_execution_id, waypoint_index),
            daemon=True,
            name=f"waypoint-speech-{waypoint_index}",
        )
        self._speech_wait_thread.start()

    def _wait_for_waypoint_speech(self, execution_id: str, waypoint_index: int) -> None:
        path = self._waypoint_speech_status_path(waypoint_index)
        if not path:
            with self._lock:
                self._fail("WAYPOINT_SPEECH_UNAVAILABLE", "waypoint speech status channel is not configured")
            return
        deadline = time.monotonic() + float(self.waypoint_speech.timeout_seconds)
        while time.monotonic() < deadline:
            with self._lock:
                if (
                    not self.context
                    or self.context.task_execution_id != execution_id
                    or self.context.state in self.TERMINAL_STATES | {"cancelling"}
                ):
                    return
            try:
                status = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
            except (OSError, json.JSONDecodeError):
                status = {}
            state = str(status.get("status") or "")
            if state == "finished":
                with self._lock:
                    if not self.context or self.context.task_execution_id != execution_id:
                        return
                    if self.context.state == "paused":
                        self._speech_wait_finished = True
                        return
                    if self.context.state != "running":
                        return
                    self._speech_wait_finished = True
                    if self._waypoint_localization_ready_index != waypoint_index:
                        return
                    self._speech_waiting_index = None
                    self._speech_wait_finished = False
                    self._waypoint_localization_ready_index = None
                    self._continue_after_waypoint(waypoint_index)
                return
            if state in {"failed", "superseded"}:
                with self._lock:
                    if self.context and self.context.task_execution_id == execution_id:
                        self._speech_waiting_index = None
                        self._fail(
                            "WAYPOINT_SPEECH_FAILED",
                            str(status.get("error_message") or f"waypoint speech {state}"),
                        )
                return
            time.sleep(float(self.waypoint_speech.poll_interval_seconds))
        with self._lock:
            if self.context and self.context.task_execution_id == execution_id:
                self._speech_waiting_index = None
                self._fail(
                    "WAYPOINT_SPEECH_TIMEOUT",
                    f"waypoint {waypoint_index} speech did not finish within {self.waypoint_speech.timeout_seconds:.0f}s",
                )

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

    def _apply_navigation_profile(
        self,
        waypoint_index: int,
        *,
        force_final: bool | None = None,
        force_require_yaw: bool | None = None,
    ) -> None:
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
        patrol_final = (not self._is_docking_task()) and waypoint_index == len(waypoints) - 1
        require_yaw = bool(target.get("require_yaw", False)) or precision_goal
        final_approach = patrol_final or precision_goal
        if force_require_yaw is not None:
            require_yaw = bool(force_require_yaw) or precision_goal
        if force_final is not None:
            final_approach = bool(force_final) or precision_goal
        self._segment_avoidance_enabled = avoid_obstacles
        setter = getattr(self.navigation, "set_waypoint_profile", None)
        if callable(setter):
            setter(
                avoid_obstacles=avoid_obstacles,
                require_yaw=require_yaw,
                final_approach=final_approach,
            )
        outdoor_setter = getattr(self.navigation, "apply_outdoor_gps_profile", None)
        if callable(outdoor_setter):
            outdoor_setter()
        if self._is_docking_task():
            precision_setter = getattr(self.navigation, "set_goal_precision", None)
            if callable(precision_setter):
                precision_setter(enabled=precision_goal)
            self._apply_docking_profile(waypoint_index)
        if not avoid_obstacles:
            self._stop_obstacle_monitor()

    def _apply_patrol_final_approach(self) -> None:
        """Slow the live FollowPath goal; do not touch costmaps or docking precision."""
        setter = getattr(self.navigation, "set_waypoint_profile", None)
        if not callable(setter):
            return
        kwargs = {
            "avoid_obstacles": self._segment_avoidance_enabled,
            "require_yaw": False,
            "final_approach": True,
        }
        try:
            setter(**kwargs, live=True)
        except TypeError:
            setter(**kwargs)

    def _restore_docking_profile(self) -> None:
        setter = getattr(self.navigation, "set_docking_profile", None)
        if callable(setter):
            setter(final_approach=False)

    def _restore_navigation_profile(self) -> None:
        self._segment_avoidance_enabled = True
        try:
            setter = getattr(self.navigation, "set_waypoint_profile", None)
            if callable(setter):
                setter(avoid_obstacles=True, require_yaw=False, final_approach=False)
            if self._is_docking_task():
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
            if self._leave_route_announced:
                # After three failed reverse attempts, remain stopped and wait
                # for operator intervention instead of repeatedly re-dispatching
                # the same blocked goal.
                self.navigation.stop_motion()
                LOGGER.error("obstacle recovery exhausted; task remains stopped after dissuasion")
                return
            self._send_from(self.context.current_waypoint_index)

    def _hold_final_pose(self) -> bool:
        """Stop the dog before measuring the last waypoint.

        Nav2's checker does not require zero velocity, and a quadruped still
        coasts after /cmd_vel goes to zero. Measuring immediately lets that
        coast fail a 0.35 m check the controller had already accepted.
        """
        stop_motion = getattr(self.navigation, "stop_motion", None)
        if callable(stop_motion):
            stop_motion()
        is_stopped = getattr(self.navigation, "is_robot_stopped", None)
        if callable(is_stopped):
            try:
                return bool(is_stopped())
            except Exception:
                LOGGER.exception("robot stop confirmation failed")
                return False
        return True

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
