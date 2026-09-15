from __future__ import annotations

import logging
import hashlib
import json
import sqlite3
import threading
import time
import uuid
from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin
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
from .localization_recovery import select_recovery_seed
from .recovery_arbiter import RecoveryArbiter
from .leg_profile import LegProfile
from .navigation_speed import normalize_navigation_speed_level
from .waypoint_actions import WaypointActionRegistry


LOGGER = logging.getLogger(__name__)

# Duplicate map clicks for a round-trip (1-2-3-2-1) rarely land on the exact
# same XY.  If the robot is standing on that cluster, start from the earliest
# copy so the outbound legs are not skipped.  After the first copy is marked
# complete, a later resume must also refuse to jump to the return copy.
WAYPOINT_COLOCATION_M = 1.0
# Outdoor reverse starts often sit 1.0–1.5 m off the end click after RTK
# re-anchor. Use a looser gate so the dog does not cruise back to point N.
REVERSE_START_COLOCATION_M = 1.5
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
# Speech/dwell waypoints are dispatched as a one-pose batch, but the travel
# leg is still a cruise. Switch to the slow DiffDrive profile only inside
# this radius so MPPI can detour on the way.
PATROL_FINAL_APPROACH_M = 1.0
# Physical-obstacle recovery is owned by one Edge episode. Collision Monitor
# remains the final authority for every BehaviorServer velocity command.
OBSTACLE_RECOVERY_MAX_ATTEMPTS = 3
# Graded departure turn: <10° absorb, 10–60° controlled spin, >60° in-place.
DEPARTURE_HEADING_SKIP_RAD = 0.175  # ~10 deg
DEPARTURE_HEADING_ALIGN_RAD = 0.175  # ~10 deg
# A waypoint explicitly marked require_yaw previously let RPP chase the final
# orientation while still following the path.  Keep its original, stricter
# goal-checker tolerance when handing that orientation to the stationary turn.
ARRIVAL_HEADING_ALIGN_RAD = 0.25  # ~14 deg
DEPARTURE_HEADING_INPLACE_RAD = 1.047  # ~60 deg
# Nav2 require_yaw weaves on large heading changes (typical 180deg loop
# turns). Spin with teleop yaw instead, indoors and outdoors. Abort and
# cruise if still misaligned.
DEPARTURE_HEADING_TIMEOUT_SECONDS = 20.0
DEPARTURE_HEADING_TELEOP_YAW_RATE = 0.40
DEPARTURE_HEADING_TELEOP_PERIOD_SECONDS = 0.10
DEPARTURE_HEADING_STABLE_SAMPLES = 3
# Quadruped coast after Nav2 reports goal reached; wait before measuring pose.
HOLD_FINAL_POSE_TIMEOUT_SECONDS = 3.0
# Outdoor patrol still needs to satisfy the 1s continuous-zero confirmation
# gate. Keep a margin for the controller/collision-monitor command handoff.
HOLD_FINAL_POSE_OUTDOOR_TIMEOUT_SECONDS = 2.0
# A no-correction completion still needs a full continuous-zero confirmation.
# Keep the existing 2 s safe window and grow it if configuration requires more.
NO_CORRECTION_STOP_RECHECK_MIN_SECONDS = 2.0
# BT movement recoveries in the deployed trees have a maximum six-second
# action allowance. Leave room for action cancellation and the lease RPC, then
# reclaim only after Edge has confirmed a zero-motion state.
BT_RECOVERY_LEASE_TIMEOUT_SECONDS = 10.0
# Stopped at a waypoint: give RTK/NDT time to pull FAST-LIO back before leaving.
WAYPOINT_SETTLE_TIMEOUT_SECONDS = 12.0
# Outdoor clean arrival (no pending correction): brief health check only.
WAYPOINT_SETTLE_OUTDOOR_TIMEOUT_SECONDS = 2.0
# A pose older than this may be used for diagnostics, but not to unlock a
# post-correction arrival transaction.
ARRIVAL_POSE_MAX_AGE_SECONDS = 1.5
# A stationary arrival must be observed in consecutive fresh samples before
# it can advance the route.  Precision/docking points use a longer run.
ARRIVAL_CONFIRMATION_FRAMES = 3
PRECISION_CONFIRMATION_FRAMES = 5
# Wait for /localization_info updates where available. The polling value is a
# compatibility fallback for older adapters without that wait API.
ARRIVAL_CONFIRMATION_SAMPLE_WAIT_SECONDS = 1.0
ARRIVAL_CONFIRMATION_INTERVAL_SECONDS = 0.01
PRECISION_ARRIVAL_YAW_TOLERANCE_RAD = 0.25
# Match localization lio_primary.drift_xy_m default; above this, wait for correction.
WAYPOINT_CORRECTION_DRIFT_M = 0.30
# A fixed-quality status alone is insufficient for the RTK initial-pose
# transaction.  These failures mean there is no safe RTK seed for this start;
# the deterministic map-origin -> route -> global NDT path must take over
# instead of trying a saved or manually selected pose first.
RTK_STARTUP_PROGRESSIVE_FALLBACK_CODES = frozenset({
    "RTK_FIXED_NOT_STABLE",
    "RTK_INITIAL_POSE_UNAVAILABLE",
    "RTK_INITIAL_POSE_TIMEOUT",
    "RTK_POSE_UNAVAILABLE",
    "RTK_INITIAL_POSE_NOT_CONVERGED",
})
# Outdoor reverse/start checks keep a looser LIO envelope while RTK performs
# the authoritative click check. Arrival verdicts use configured tolerances.
ARRIVAL_ACCEPT_LIO_M = 1.0
# When RTK is fixed, the click must also be near RTK XY. Otherwise LIO only
# "arrived" in a drifted frame and the dog is not at the real map point.
ARRIVAL_ACCEPT_RTK_M = 1.5
# Outdoor reverse-start skip also requires LIO and fixed RTK to agree this close.
REVERSE_SKIP_LIO_RTK_DRIFT_M = 0.50
ARRIVAL_CONVERGENCE_MAX_ATTEMPTS = 2
ARRIVAL_ADJUST_PERIOD_SECONDS = 0.10
ARRIVAL_ADJUST_STABLE_SAMPLES = 3
# A fine adjustment stops inside the acceptance radius, not at the click
# centre.  Reserve a short extra distance for one velocity period and braking,
# but do not reject an obstacle that lies beyond the bounded movement needed
# to enter that radius.
ARRIVAL_ADJUST_STOPPING_MARGIN_M = 0.05
# How long ABSOLUTE_LOCALIZATION_REQUIRED may wait before giving up the watch.
ABSOLUTE_LOCALIZATION_RESUME_WATCH_SECONDS = 120.0
# A completed one-shot correction may explicitly decide that no absolute
# observation is safe to consume. These outcomes keep FAST-LIO/UKF as the
# continuous estimate; they are not successful NDT/RTK corrections.
NO_CORRECTION_CONTINUE_REASONS = frozenset(
    {
        "ukf_no_correction_sources_meet_gate",
        "ndt_no_correction_continue",
        "rtk_no_correction_continue",
    }
)
# Patrol redispatches after a rejected FollowWaypoints goal. Keep this short —
# send_waypoints already waited for Nav2 readiness.
NAV_DISPATCH_RETRY_DEFAULT_SECONDS = 2.0
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
    def wait_for_pose_update(
        self, after_sampled_at: str | None, timeout_seconds: float = 1.0
    ): ...
    def latest_trusted_pose(self): ...
    def invalidate_last_trusted_pose(self) -> None: ...
    def accept_startup_trusted_pose(self) -> None: ...
    def set_localization_policy(
        self,
        source: str,
        phase: str,
        anchor_preference: str = "balanced",
        rtk_primary_allowed: bool = False,
        online_anchor_correction_allowed: bool = False,
    ) -> dict: ...
    def localization_decision(self) -> dict: ...
    def localization_diagnostics(self) -> dict: ...
    def control_localization_correction(
        self, transaction_id: str, mode: str, command: str = "start"
    ) -> dict: ...
    def set_goal_precision(self, *, enabled: bool) -> None: ...
    def set_arrival_goal_tolerance(
        self, tolerance_m: float, *, yaw_tolerance_rad: float = 0.25
    ) -> None: ...
    def set_arrival_micro_goal_profile(self, *, enabled: bool, tolerance_m: float = 0.15) -> None: ...
    def set_waypoint_profile(
        self,
        *,
        avoid_obstacles: bool,
        require_yaw: bool,
        final_approach: bool = False,
        live: bool = False,
        outdoor: bool | None = None,
        local_controller: str = "mppi",
        navigation_speed_level: str = "micro",
        reapproach: bool = False,
    ) -> None: ...
    def set_global_controller(self, mode: str) -> None: ...
    def arrival_adjust_velocity(
        self, vx: float = 0.0, vy: float = 0.0, yaw_rate: float = 0.0
    ) -> dict: ...
    def directional_clearance(
        self,
        vx: float,
        vy: float,
        travel_distance_m: float,
        *,
        max_scan_age_seconds: float = 0.5,
    ) -> dict: ...
    def execute_obstacle_recovery(
        self,
        *,
        reverse_distance_m: float,
        lateral_distance_m: float,
        lateral_direction: int,
        speed_mps: float,
        timeout_seconds: float,
    ) -> dict: ...
    def cancel_obstacle_recovery(self) -> bool: ...


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
    loop_execution: bool = False
    loop_session_id: str = ""
    continuous_rosbag: bool = False
    docking: dict | None = None
    round_number: int = 1
    loop_total: int = 1
    loop_base_waypoints: list[dict] | None = None
    trace_id: str = ""
    post_arrival_waypoint_index: int | None = None
    post_arrival_stage: str = ""
    arrival_side_effects_started: bool = False
    # Set only after the configured Nav2 re-approach budget is exhausted for a
    # normal stopping waypoint. It keeps the accepted 0.50 m coarse radius
    # durable through final-yaw/post-processing and final task completion.
    arrival_coarse_fallback_accepted: bool = False
    arrival_reapproach_waypoint_index: int | None = None
    arrival_reapproach_attempts: int = 0
    arrival_micro_adjust_total_m: float = 0.0
    arrival_micro_adjust_steps: int = 0
    arrival_micro_adjust_started_at: float | None = None
    last_safe_hold_code: str = ""
    last_safe_hold_message: str = ""


@dataclass(frozen=True)
class ArrivalStabilityResult:
    stable: bool
    reason: str
    consecutive_frames: int
    fresh_frames: int
    within_tolerance_frames: int


class TaskExecutor:
    TERMINAL_STATES = {"completed", "failed", "cancelled", "timed_out", "rejected"}
    ROSBAG_SCOPE_METADATA_KEY = "navigation_rosbag_scope"

    def __init__(
        self,
        store: LocalStore,
        navigation: NavigationAdapter,
        *,
        event_callback: Callable[[str, dict, str], None],
        start_result_callback: Callable[[str, str, dict, str, str], None],
        coarse_goal_tolerance_m: float = 0.50,
        normal_arrival_tolerance_m: float = 0.30,
        precision_arrival_tolerance_m: float = 0.15,
        arrival_reapproach_max_attempts: int = 1,
        docking_goal_tolerance_m: float = 0.08,
        docking_goal_yaw_tolerance_rad: float = 0.0872665,
        arrival_adjust_max_distance_m: float = 0.50,
        arrival_adjust_clearance_lookahead_m: float | None = None,
        arrival_adjust_speed_mps: float = 0.08,
        arrival_adjust_yaw_rate_rps: float = 0.10,
        arrival_adjust_timeout_seconds: float = 30.0,
        arrival_adjust_scan_max_age_seconds: float = 0.50,
        arrival_adjust_safety_grace_seconds: float = 2.0,
        arrival_micro_adjust_mode: str = "cmd_vel",
        arrival_micro_adjust_max_initial_error_m: float = 0.45,
        arrival_micro_adjust_total_budget_m: float = 0.30,
        arrival_micro_adjust_step_m: float = 0.15,
        arrival_micro_adjust_max_steps: int = 2,
        arrival_nav2_reapproach_max_error_m: float = 1.50,
        arrival_precision_recovery_retry_seconds: float = 5.0,
        arrival_micro_goal_tolerance_m: float = 0.15,
        arrival_ndt_max_fitness_score: float = 0.45,
        arrival_convergence_samples: int = 3,
        standup_confirmation_timeout_seconds: float = 12.0,
        stop_confirmation_seconds: float = 1.0,
        bt_recovery_lease_timeout_seconds: float = BT_RECOVERY_LEASE_TIMEOUT_SECONDS,
        navigation_dispatch_retry_seconds: float = NAV_DISPATCH_RETRY_DEFAULT_SECONDS,
        navigation_dispatch_retry_budget_seconds: float = 300.0,
        map_set_coordinator=None,
        map_activation_adapter=None,
        obstacle_speech=None,
        waypoint_speech=None,
        rosbag_recorder=None,
        obstacle_evidence=None,
        docking_arrived_handler=None,
        localization_recovery_callback: Callable[[str], None] | None = None,
        localization_recovery_cancel_callback: Callable[[], None] | None = None,
        localization_operation_active_callback: Callable[[], bool] | None = None,
    ) -> None:
        self.store = store
        self.navigation = navigation
        self.event_callback = event_callback
        self.start_result_callback = start_result_callback
        self.coarse_goal_tolerance_m = max(0.01, float(coarse_goal_tolerance_m))
        self.final_waypoint_tolerance_m = min(
            self.coarse_goal_tolerance_m,
            max(0.01, float(normal_arrival_tolerance_m)),
        )
        self.precision_arrival_tolerance_m = min(
            self.final_waypoint_tolerance_m,
            max(0.01, float(precision_arrival_tolerance_m)),
        )
        self.arrival_reapproach_max_attempts = max(
            0, int(arrival_reapproach_max_attempts)
        )
        self.docking_goal_tolerance_m = docking_goal_tolerance_m
        self.docking_goal_yaw_tolerance_rad = docking_goal_yaw_tolerance_rad
        # Keep the legacy max-distance setting loadable, but use it only as the
        # rolling scan lookahead when the explicit micro-adjust limits are absent.
        legacy_lookahead = max(0.05, float(arrival_adjust_max_distance_m))
        self.arrival_adjust_clearance_lookahead_m = max(
            0.05,
            float(
                arrival_adjust_clearance_lookahead_m
                if arrival_adjust_clearance_lookahead_m is not None
                else legacy_lookahead
            ),
        )
        self.arrival_adjust_speed_mps = max(0.01, float(arrival_adjust_speed_mps))
        self.arrival_adjust_yaw_rate_rps = max(0.01, float(arrival_adjust_yaw_rate_rps))
        self.arrival_adjust_timeout_seconds = max(0.0, float(arrival_adjust_timeout_seconds))
        self.arrival_adjust_scan_max_age_seconds = max(
            0.05, float(arrival_adjust_scan_max_age_seconds)
        )
        self.arrival_adjust_safety_grace_seconds = max(
            0.0, float(arrival_adjust_safety_grace_seconds)
        )
        self.arrival_micro_adjust_mode = (
            str(arrival_micro_adjust_mode or "cmd_vel").strip().lower()
        )
        if self.arrival_micro_adjust_mode not in {"cmd_vel", "nav2_goal"}:
            self.arrival_micro_adjust_mode = "cmd_vel"
        self.arrival_micro_adjust_max_initial_error_m = max(
            self.final_waypoint_tolerance_m, float(arrival_micro_adjust_max_initial_error_m)
        )
        self.arrival_micro_adjust_total_budget_m = max(
            0.0, float(arrival_micro_adjust_total_budget_m)
        )
        self.arrival_micro_adjust_step_m = max(0.01, float(arrival_micro_adjust_step_m))
        self.arrival_micro_adjust_max_steps = max(1, int(arrival_micro_adjust_max_steps))
        self.arrival_nav2_reapproach_max_error_m = max(
            self.final_waypoint_tolerance_m,
            float(arrival_nav2_reapproach_max_error_m),
        )
        self.arrival_precision_recovery_retry_seconds = max(
            1.0, float(arrival_precision_recovery_retry_seconds)
        )
        self.arrival_micro_goal_tolerance_m = max(
            0.01, float(arrival_micro_goal_tolerance_m)
        )
        self.arrival_ndt_max_fitness_score = max(
            0.001, float(arrival_ndt_max_fitness_score)
        )
        self.arrival_convergence_samples = max(1, int(arrival_convergence_samples))
        self.standup_confirmation_timeout_seconds = standup_confirmation_timeout_seconds
        self.stop_confirmation_seconds = max(0.0, float(stop_confirmation_seconds))
        self.bt_recovery_lease_timeout_seconds = max(
            0.1, float(bt_recovery_lease_timeout_seconds)
        )
        self.navigation_dispatch_retry_seconds = max(0.5, float(navigation_dispatch_retry_seconds))
        self.navigation_dispatch_retry_budget_seconds = max(
            self.navigation_dispatch_retry_seconds,
            float(navigation_dispatch_retry_budget_seconds),
        )
        self.map_set_coordinator = map_set_coordinator
        self.map_activation_adapter = map_activation_adapter
        self.obstacle_speech = obstacle_speech
        self.waypoint_speech = waypoint_speech
        self.rosbag_recorder = rosbag_recorder
        self.obstacle_evidence = obstacle_evidence
        self.docking_arrived_handler = docking_arrived_handler
        self.localization_recovery_callback = localization_recovery_callback
        self.localization_recovery_cancel_callback = localization_recovery_cancel_callback
        self.localization_operation_active_callback = localization_operation_active_callback
        self._rosbag_state: dict = {}
        self._segments = []
        self._lock = threading.RLock()
        self._goal_offset = 0
        self._departure_heading_index: int | None = None
        self._departure_heading_completed_index: int | None = None
        # Set after a waypoint's requested final yaw has been satisfied.  The
        # post-turn path must still re-check XY because a quadruped can
        # translate while executing a nominally pure-yaw command.  It must not
        # suppress the following-leg turn.
        self._arrival_heading_completed_index: int | None = None
        self._arrival_correction_completed_index: int | None = None
        self._arrival_convergence_attempts: dict[int, int] = {}
        self._arrival_adjustment_index: int | None = None
        self._arrival_adjustment_stop = threading.Event()
        self._arrival_adjustment_thread: threading.Thread | None = None
        self._arrival_adjustment_nav2_active = False
        # When set, a successful in-place turn should cruise to this waypoint
        # instead of running the post-arrival absolute-localization path.
        self._departure_cruise_index: int | None = None
        self._departure_heading_timer: threading.Timer | None = None
        self._departure_heading_cancel = threading.Event()
        self._departure_heading_thread: threading.Thread | None = None
        self._departure_heading_mode: str | None = None
        self._departure_heading_is_arrival = False
        self._departure_heading_tolerance_rad = DEPARTURE_HEADING_ALIGN_RAD
        self._absolute_pause_watch_stop: threading.Event | None = None
        self._absolute_pause_watch_thread: threading.Thread | None = None
        self._arrival_precision_recovery_timer: threading.Timer | None = None
        self._last_progress_emit_at: float | None = None
        self._obstacle_monitor_stop = threading.Event()
        self._obstacle_monitor_thread = None
        self._obstacle_progress_anchor = None
        self._obstacle_progress_anchor_at = None
        self._obstacle_target_distance_anchor_m = None
        self._obstacle_waypoint_key = None
        self._recovery_attempts = 0
        self._leave_route_announced = False
        self._last_obstacle_seen_at = None
        self._obstacle_episode_id = None
        self._obstacle_stage = None
        self._obstacle_clear_started_at = None
        self._obstacle_recovery_active = False
        self._recovery_arbiter = RecoveryArbiter()
        self._bt_recovery_lease_timers: dict[int, threading.Timer] = {}
        self._action_registry = WaypointActionRegistry()
        self._emitted_event_keys: set[str] = set()
        self._correction_generation = 0
        self._correction_completed_at_mono: float | None = None
        self._active_correction_transaction_id: str | None = None
        self._active_correction_mode: str | None = None
        self._expected_recovery_cancels = 0
        self._bypass_active = False
        self._task_started_at = None
        self._blocked_retry_timer = None
        self._nav_dispatch_retry_timer = None
        self._nav_dispatch_retry_started_at: float | None = None
        self._pending_dispatch_index: int | None = None
        self._paused_for_localization = False
        self._paused_localization_reason: str | None = None
        self._navigation_prepared = False
        self._segment_avoidance_enabled = True
        self._dispatched_count = 0
        self._nav_goal_generation = 0
        self._leg_generation = 0
        self._active_leg_profile: LegProfile | None = None
        self._patrol_final_approach_applied = False
        self._last_target_index = -1
        self._last_reached_index = -1
        self._last_localization_policy: tuple[str, str, str, bool, bool] | None = None
        self._speech_waiting_index: int | None = None
        self._speech_wait_finished = False
        self._speech_wait_thread: threading.Thread | None = None
        self._dwell_waiting_index: int | None = None
        self._dwell_wait_finished = False
        self._dwell_wait_stop: threading.Event | None = None
        self._dwell_wait_thread: threading.Thread | None = None
        self._waypoint_localization_ready_index: int | None = None
        raw = store.load_active_task_context()
        self.context = TaskContext(**raw) if raw else None
        persisted_reapproach_index = (
            int(self.context.arrival_reapproach_waypoint_index)
            if self.context
            and self.context.arrival_reapproach_waypoint_index is not None
            else None
        )
        persisted_reapproach_attempts = (
            max(0, int(self.context.arrival_reapproach_attempts))
            if self.context
            else 0
        )
        self._arrival_retry_counts: dict[int, int] = (
            {persisted_reapproach_index: persisted_reapproach_attempts}
            if persisted_reapproach_index is not None
            and persisted_reapproach_attempts > 0
            else {}
        )
        self._arrival_reapproach_index: int | None = persisted_reapproach_index
        if self.context and self.context.post_arrival_waypoint_index is not None:
            reached = int(self.context.post_arrival_waypoint_index)
            self._last_target_index = max(self._last_target_index, reached)
            if self.context.arrival_side_effects_started:
                self._last_reached_index = max(self._last_reached_index, reached)
        if self.context and self.context.state != "paused":
            self.context.state = "interrupted"
            self.context.state_version += 1
            self._persist()

    def acquire_recovery(self, owner: str, reason: str = "", *, distance_m: float = 0.0):
        """Acquire recovery ownership for Edge-owned recovery paths."""
        if str(owner) == "BT_NAVIGATOR" and self._obstacle_episode_id is not None:
            LOGGER.info(
                "BT recovery lease rejected while physical obstacle episode %s is Edge-owned",
                self._obstacle_episode_id,
            )
            return None
        if self._recovery_arbiter.budget_exhausted():
            self._emit_safe_hold("RECOVERY_BUDGET_EXHAUSTED", "自愈预算耗尽，进入安全保持")
            return None
        lease = self._recovery_arbiter.acquire(owner, reason, distance_m=distance_m)
        if lease is not None and lease.owner == "BT_NAVIGATOR":
            self._arm_bt_recovery_lease_timeout(lease.generation)
        if lease is None and self._recovery_arbiter.budget_exhausted():
            self._emit_safe_hold("RECOVERY_BUDGET_EXHAUSTED", "自愈预算耗尽，进入安全保持")
        return lease

    def release_recovery(self, lease) -> bool:
        released = self._recovery_arbiter.release(lease)
        if released and getattr(lease, "owner", "") == "BT_NAVIGATOR":
            self._cancel_bt_recovery_lease_timeout(int(lease.generation))
        return released

    def _cancel_bt_recovery_lease_timeout(self, generation: int) -> None:
        timer = self._bt_recovery_lease_timers.pop(int(generation), None)
        if timer is not None:
            timer.cancel()

    def _cancel_all_bt_recovery_lease_timeouts(self) -> None:
        timers = list(self._bt_recovery_lease_timers.values())
        self._bt_recovery_lease_timers.clear()
        for timer in timers:
            timer.cancel()

    def _release_bt_recovery_lease_after_stop(self, generation: int, *, trigger: str) -> bool:
        """Reclaim a cancelled/expired BT lease only after confirmed stop."""
        snapshot = self._recovery_arbiter.snapshot()
        if (
            snapshot.get("owner") != "BT_NAVIGATOR"
            or int(snapshot.get("recovery_generation") or 0) != int(generation)
        ):
            self._cancel_bt_recovery_lease_timeout(generation)
            return False
        # Lease timeout exists for a lost BT RELEASE, not to abort spin/backup.
        # Forcing zero velocity here cancelled the only motion that can leave
        # an obstacle, then the next recovery acquired the same frozen state.
        stop_motion = getattr(self.navigation, "stop_motion", None)
        if trigger != "lease_timeout" and callable(stop_motion):
            stop_motion()
        is_stopped = getattr(self.navigation, "is_robot_stopped", None)
        if callable(is_stopped):
            timeout = max(1.0, self.stop_confirmation_seconds + 0.5)
            try:
                try:
                    stopped = bool(is_stopped(timeout_seconds=timeout))
                except TypeError:
                    stopped = bool(is_stopped())
            except Exception:
                LOGGER.warning("unable to confirm stop before BT lease reclaim", exc_info=True)
                stopped = False
            if not stopped:
                return False
        reclaimer = getattr(self.navigation, "reclaim_recovery_lease", None)
        if callable(reclaimer):
            try:
                released = bool(reclaimer(int(generation)))
            except Exception:
                LOGGER.warning("BT recovery lease reclaim transport failed", exc_info=True)
                released = False
        else:
            released = self._recovery_arbiter.release_generation(
                "BT_NAVIGATOR", int(generation)
            )
        if released:
            self._cancel_bt_recovery_lease_timeout(generation)
            LOGGER.warning(
                "reclaimed BT recovery lease generation=%s after %s and confirmed stop",
                generation,
                trigger,
            )
        return released

    def _on_bt_recovery_lease_timeout(self, generation: int) -> None:
        with self._lock:
            if self._release_bt_recovery_lease_after_stop(generation, trigger="lease_timeout"):
                return
            snapshot = self._recovery_arbiter.snapshot()
            if (
                snapshot.get("owner") == "BT_NAVIGATOR"
                and int(snapshot.get("recovery_generation") or 0) == int(generation)
            ):
                timer = threading.Timer(1.0, self._on_bt_recovery_lease_timeout, args=(generation,))
                timer.daemon = True
                self._bt_recovery_lease_timers[int(generation)] = timer
                timer.start()

    def _arm_bt_recovery_lease_timeout(self, generation: int) -> None:
        self._cancel_bt_recovery_lease_timeout(generation)
        timer = threading.Timer(
            self.bt_recovery_lease_timeout_seconds,
            self._on_bt_recovery_lease_timeout,
            args=(generation,),
        )
        timer.daemon = True
        self._bt_recovery_lease_timers[int(generation)] = timer
        timer.start()

    def recovery_snapshot(self) -> dict:
        return self._recovery_arbiter.snapshot()

    def _emit_safe_hold(self, code: str, message: str) -> None:
        if not self.context:
            return
        self._cancel_arrival_adjustment()
        self.navigation.stop_motion()
        self._restore_navigation_profile()
        if self.context.state not in self.TERMINAL_STATES | {"paused"}:
            self.context.state = "paused"
            self.context.state_version += 1
        self.context.last_safe_hold_code = code
        self.context.last_safe_hold_message = message
        self._persist()
        self._emit_idempotent(
            "task.safe_hold",
            event_type_key="safe_hold",
            code=code,
            message=message,
            extra={"recovery": self._recovery_arbiter.snapshot()},
        )

    def enter_safe_hold(self, code: str, message: str) -> None:
        """Public boundary for recovery coordinators after all levels fail."""
        self._emit_safe_hold(code, message)

    def _idempotency_key(self, event_type: str, waypoint_id: str | None = None) -> str:
        waypoint = waypoint_id or ""
        if self.context and not waypoint:
            index = self.context.current_waypoint_index
            waypoints = self.context.route_snapshot.get("waypoints") or []
            if 0 <= index < len(waypoints):
                waypoint = str(waypoints[index].get("waypoint_id") or index)
        round_index = int(getattr(self.context, "round_number", 0) or 0) if self.context else 0
        execution_id = getattr(self.context, "task_execution_id", "") if self.context else ""
        return f"{execution_id}:{round_index}:{waypoint}:{self._leg_generation}:{event_type}"

    def _emit_idempotent(
        self,
        event_type: str,
        *,
        event_type_key: str,
        code: str = "",
        message: str = "",
        extra: dict | None = None,
        waypoint_id: str | None = None,
    ) -> bool:
        key = self._idempotency_key(event_type_key, waypoint_id=waypoint_id)
        if key in self._emitted_event_keys:
            return False
        self._emitted_event_keys.add(key)
        payload_extra = {"idempotency_key": key, "leg_generation": self._leg_generation}
        if extra:
            payload_extra.update(extra)
        self._emit(event_type, code=code, message=message, extra=payload_extra)
        return True

    def stop(self) -> None:
        dwell_thread = self._dwell_wait_thread
        self._cancel_waypoint_localization_correction()
        self._cancel_arrival_adjustment(reset_state=True)
        self._cancel_waypoint_dwell()
        self._stop_obstacle_monitor()
        self._stop_task_rosbag(force=True)
        heading_thread = self._departure_heading_thread
        self._clear_departure_heading(cancel_navigation=True)
        if (
            heading_thread is not None
            and heading_thread.is_alive()
            and heading_thread is not threading.current_thread()
        ):
            heading_thread.join(timeout=1.0)
        self._cancel_absolute_localization_resume_watch()
        self._cancel_arrival_precision_recovery_retry()
        self._restore_navigation_profile()
        if self._blocked_retry_timer:
            self._blocked_retry_timer.cancel()
            self._blocked_retry_timer = None
        self._clear_nav_dispatch_retry()
        if (
            dwell_thread is not None
            and dwell_thread.is_alive()
            and dwell_thread is not threading.current_thread()
        ):
            dwell_thread.join(timeout=1.0)

    def _obstacle_monitor_enabled(self) -> bool:
        return bool(
            self.obstacle_speech
            and self.obstacle_speech.enabled
            and callable(getattr(self.navigation, "obstacle_monitor_snapshot", None))
        )

    def _obstacle_motion_recovery_enabled(self) -> bool:
        return bool(
            self._segment_avoidance_enabled
            and callable(getattr(self.navigation, "execute_obstacle_recovery", None))
        )

    def _start_obstacle_monitor(self) -> None:
        if not self._obstacle_monitor_enabled():
            return
        self._sync_obstacle_waypoint_budget()
        self._obstacle_monitor_stop.clear()
        if self._obstacle_monitor_thread and self._obstacle_monitor_thread.is_alive():
            return
        self._obstacle_monitor_thread = threading.Thread(
            target=self._obstacle_monitor_loop, daemon=True, name="obstacle-speech-monitor"
        )
        self._obstacle_monitor_thread.start()

    def _suspend_obstacle_monitor(self) -> None:
        self._obstacle_monitor_stop.set()
        cancel_recovery = getattr(self.navigation, "cancel_obstacle_recovery", None)
        if callable(cancel_recovery):
            try:
                cancel_recovery()
            except Exception:
                LOGGER.warning("failed to cancel active obstacle recovery", exc_info=True)
        self._obstacle_monitor_thread = None

    def _stop_obstacle_monitor(self) -> None:
        self._suspend_obstacle_monitor()
        self._reset_obstacle_episode(reset_waypoint_budget=True)

    def _obstacle_monitor_loop(self) -> None:
        while not self._obstacle_monitor_stop.wait(0.5):
            try:
                self._evaluate_obstacle_progress()
            except Exception:
                # This monitor owns recovery and re-dispatch decisions.  A
                # transient Nav2 service timeout must not permanently remove
                # obstacle recovery for the rest of a running task.
                LOGGER.exception("obstacle monitor iteration failed; keeping monitor alive")

    def _current_obstacle_waypoint_key(self) -> tuple[str, int, int] | None:
        if not self.context:
            return None
        return (
            str(self.context.task_execution_id),
            int(self.context.round_number),
            int(self.context.current_waypoint_index),
        )

    def _sync_obstacle_waypoint_budget(self) -> None:
        waypoint_key = self._current_obstacle_waypoint_key()
        if waypoint_key == self._obstacle_waypoint_key:
            return
        self._reset_obstacle_episode(reset_waypoint_budget=True)
        self._obstacle_waypoint_key = waypoint_key

    def _reset_obstacle_episode(self, *, reset_waypoint_budget: bool = False) -> None:
        self._obstacle_progress_anchor = None
        self._obstacle_progress_anchor_at = None
        self._obstacle_target_distance_anchor_m = None
        if reset_waypoint_budget:
            self._recovery_attempts = 0
            self._obstacle_waypoint_key = None
        self._leave_route_announced = False
        self._last_obstacle_seen_at = None
        self._obstacle_episode_id = None
        self._obstacle_stage = None
        self._obstacle_clear_started_at = None
        self._obstacle_recovery_active = False
        self._recovery_arbiter.reset_budget()

    def _distance_to_current_waypoint(self, pose) -> float | None:
        if not self.context or pose is None:
            return None
        waypoints = self.context.route_snapshot.get("waypoints") or []
        index = int(self.context.current_waypoint_index)
        if index < 0 or index >= len(waypoints):
            return None
        target = waypoints[index]
        try:
            return hypot(float(pose.x) - float(target["x"]), float(pose.y) - float(target["y"]))
        except (AttributeError, KeyError, TypeError, ValueError):
            return None

    def _localization_allows_obstacle_monitor(self, observation: dict | None = None) -> bool:
        """Ignore collision-limited 'obstacles' while localization is not Normal.

        Collision Monitor zeros /cmd_vel on status!=3. Treating that as a
        physical obstacle triggers reverse/bypass while the dog should hold.
        """
        if isinstance(observation, dict) and "localization_normal" in observation:
            return bool(observation.get("localization_normal"))
        diagnostics = getattr(self.navigation, "localization_diagnostics", None)
        if not callable(diagnostics):
            return True
        try:
            payload = diagnostics() or {}
        except Exception:
            return True
        raw = payload.get("raw_pose") if isinstance(payload, dict) else None
        status = str(
            (payload or {}).get("localization_status")
            or ((raw or {}).get("localization_status") if isinstance(raw, dict) else "")
            or ""
        ).lower()
        if not status:
            return True
        return status == "normal"

    def _physical_obstacle_blocked(self, observation: dict) -> tuple[bool, str]:
        """Classify only physical stops; localization/source gates are not obstacles."""
        collision = observation.get("collision_monitor") or {}
        collision_age = collision.get("sample_age_seconds")
        collision_fresh = collision_age is not None and float(collision_age) <= 1.0
        collision_state = str(collision.get("state") or "").upper()
        collision_reason = str(collision.get("reason") or "")
        if collision_fresh and collision_state == "STOP":
            if collision_reason == "polygon" and collision.get("zone"):
                return True, "collision_polygon"
            if collision_reason in {"localization_unhealthy", "source_stale"}:
                return False, collision_reason
        distance = observation.get("front_obstacle_distance_m")
        scan_blocked = (
            distance is not None
            and float(distance) <= float(getattr(self.obstacle_speech, "obstacle_max_distance_m", 0.9))
        )
        if scan_blocked:
            return True, "front_scan"
        if collision_fresh:
            return False, collision_reason or "collision_clear"
        raw_planar = abs(float(observation.get("requested_planar_speed_mps") or 0.0))
        actual_planar = abs(float(observation.get("actual_planar_speed_mps") or 0.0))
        raw_turn = abs(float(observation.get("requested_turn_speed_rps") or 0.0))
        actual_turn = abs(float(observation.get("actual_turn_speed_rps") or 0.0))
        requested_motion = raw_planar >= 0.04 or raw_turn >= 0.15
        actual_motion = max(actual_planar, actual_turn * 0.25)
        requested_magnitude = max(raw_planar, raw_turn * 0.25)
        collision_limited = (
            requested_motion
            and actual_motion
            <= requested_magnitude * float(getattr(self.obstacle_speech, "collision_limit_ratio", 0.6))
        )
        return collision_limited, "velocity_limited" if collision_limited else "clear"

    @staticmethod
    def _obstacle_scan_is_fresh(observation: dict) -> bool:
        return observation.get("stale") is not True

    def _obstacle_is_clear(self, observation: dict) -> bool:
        blocked, _ = self._physical_obstacle_blocked(observation)
        return self._obstacle_scan_is_fresh(observation) and not blocked

    def _evaluate_obstacle_progress(self) -> None:
        next_action: tuple[str, int | None] | None = None
        with self._lock:
            if (
                not self.context
                or self.context.state != "running"
                or not self._obstacle_monitor_enabled()
            ):
                return
            observation = self.navigation.obstacle_monitor_snapshot()
            self._sync_obstacle_waypoint_budget()
            if not self._localization_allows_obstacle_monitor(observation):
                self._obstacle_clear_started_at = None
                return
            blocked, trigger_reason = self._physical_obstacle_blocked(observation)
            now = time.monotonic()
            pose = self.navigation.latest_pose()
            if self._obstacle_episode_id and self._obstacle_stage == "SAFE_OBSERVING":
                if not self._obstacle_is_clear(observation):
                    self._obstacle_clear_started_at = None
                elif self._obstacle_clear_started_at is None:
                    self._obstacle_clear_started_at = now
                elif now - self._obstacle_clear_started_at >= float(
                    getattr(self.obstacle_speech, "obstacle_clear_seconds", 3.0)
                ):
                    self._emit_obstacle_stage("CLEAR_CONFIRMED", observation)
                    next_action = ("resume", None)
            elif not blocked or not pose:
                if self._obstacle_episode_id is None:
                    return
                if self._obstacle_clear_started_at is None:
                    self._obstacle_clear_started_at = now
                elif now - self._obstacle_clear_started_at >= float(
                    getattr(self.obstacle_speech, "obstacle_clear_seconds", 3.0)
                ):
                    self._emit_obstacle_stage("CLEAR_CONFIRMED", observation)
                    self._emit_obstacle_stage("RESUMED", observation)
                    self._reset_obstacle_episode()
            else:
                self._obstacle_clear_started_at = None
                self._last_obstacle_seen_at = now
                if self._obstacle_progress_anchor is None:
                    self._obstacle_episode_id = str(uuid.uuid4())
                    self._obstacle_progress_anchor = (float(pose.x), float(pose.y))
                    self._obstacle_progress_anchor_at = now
                    self._obstacle_target_distance_anchor_m = self._distance_to_current_waypoint(pose)
                    self._emit_obstacle_stage(
                        "DETECTED_STOP", observation, reason=trigger_reason
                    )
                    if not self._obstacle_motion_recovery_enabled():
                        next_action = ("safe_observing", None)
                    else:
                        self._emit_obstacle_stage(
                            "WAITING_PROGRESS", observation, reason=trigger_reason
                        )
                else:
                    current_target_distance = self._distance_to_current_waypoint(pose)
                    if (
                        self._obstacle_target_distance_anchor_m is not None
                        and current_target_distance is not None
                    ):
                        progressed = max(
                            0.0,
                            self._obstacle_target_distance_anchor_m - current_target_distance,
                        )
                    else:
                        anchor_x, anchor_y = self._obstacle_progress_anchor
                        progressed = hypot(float(pose.x) - anchor_x, float(pose.y) - anchor_y)
                    if progressed >= float(getattr(self.obstacle_speech, "min_progress_m", 0.5)):
                        self._obstacle_progress_anchor = (float(pose.x), float(pose.y))
                        self._obstacle_progress_anchor_at = now
                        self._obstacle_target_distance_anchor_m = current_target_distance
                    elif (
                        now - self._obstacle_progress_anchor_at
                        >= float(getattr(self.obstacle_speech, "no_progress_seconds", 5.0))
                        and not self._obstacle_recovery_active
                        and not self._leave_route_announced
                    ):
                        max_attempts = int(getattr(
                            self.obstacle_speech,
                            "max_recovery_attempts",
                            OBSTACLE_RECOVERY_MAX_ATTEMPTS,
                        ))
                        if self._recovery_attempts >= max_attempts:
                            next_action = ("safe_observing", None)
                        else:
                            self._recovery_attempts += 1
                            attempt = int(self._recovery_attempts)
                            self._obstacle_progress_anchor_at = now
                            self._emit_obstacle_stage(
                                "RECOVERY_ATTEMPT", observation, attempt=attempt
                            )
                            next_action = ("recover", attempt)

        if next_action and next_action[0] == "resume":
            self._resume_after_obstacle(observation, automatic=True)
        elif next_action and next_action[0] == "safe_observing":
            self._enter_obstacle_safe_observing(observation)
        elif next_action and next_action[0] == "recover":
            self._perform_bounded_obstacle_recovery(int(next_action[1] or 0))

    @staticmethod
    def _finite_clearance(value) -> float | None:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        return parsed if isfinite(parsed) else None

    def _select_obstacle_recovery_direction(self, observation: dict) -> dict:
        """Fail closed on stale scans and select a verified body-side corridor."""
        if not self._obstacle_scan_is_fresh(observation):
            return {"safe": False, "reason": "scan_stale"}
        clearance = getattr(self.navigation, "directional_clearance", None)
        if not callable(clearance):
            return {"safe": False, "reason": "directional_clearance_unavailable"}
        reverse_distance = float(getattr(self.obstacle_speech, "reverse_distance_m", 0.25))
        lateral_distance = float(getattr(self.obstacle_speech, "lateral_distance_m", 0.20))
        speed = float(getattr(self.obstacle_speech, "recovery_speed_mps", 0.06))
        max_age = float(getattr(self.obstacle_speech, "scan_max_age_seconds", 0.50))
        rear_min = float(getattr(self.obstacle_speech, "rear_clearance_m", 0.70))
        side_min = float(getattr(self.obstacle_speech, "side_clearance_m", 0.80))
        rear = self._finite_clearance(observation.get("rear_clearance_m"))
        if rear is None:
            return {"safe": False, "reason": "rear_clearance_unknown"}
        if rear is not None and rear < rear_min:
            return {"safe": False, "reason": "rear_clearance", "rear_clearance_m": rear}
        reverse_check = clearance(
            -abs(speed), 0.0, reverse_distance, max_scan_age_seconds=max_age
        )
        if not reverse_check.get("clear"):
            return {"safe": False, "reason": "reverse_" + str(reverse_check.get("reason")),
                    "reverse_check": reverse_check}
        candidates = []
        for direction, key in ((1, "left_clearance_m"), (-1, "right_clearance_m")):
            measured = self._finite_clearance(observation.get(key))
            if measured is None or measured < side_min:
                continue
            check = clearance(
                0.0,
                direction * abs(speed),
                lateral_distance,
                max_scan_age_seconds=max_age,
            )
            if check.get("clear"):
                # Unknown means "no nearest point", not infinity. It is ranked
                # below a measured-clear side and accepted only by the full
                # swept-corridor check above.
                candidates.append((measured is not None, measured or 0.0, direction, check))
        if not candidates:
            return {"safe": False, "reason": "side_clearance"}
        _, measured, direction, side_check = max(candidates, key=lambda item: (item[0], item[1]))
        return {
            "safe": True,
            "lateral_direction": direction,
            "lateral_side": "left" if direction > 0 else "right",
            "selected_side_clearance_m": measured or None,
            "reverse_check": reverse_check,
            "side_check": side_check,
        }

    def _perform_bounded_obstacle_recovery(self, attempt: int) -> None:
        with self._lock:
            if not self.context or self.context.state != "running" or self._obstacle_recovery_active:
                return
            execution_id = self.context.task_execution_id
            observation = self.navigation.obstacle_monitor_snapshot() or {}
            selection = self._select_obstacle_recovery_direction(observation)
            self._obstacle_recovery_active = True
        result = {"success": False, "reason": selection.get("reason", "unsafe")}
        lease = None
        navigation_goal_cancelled = False
        try:
            if not selection.get("safe"):
                return
            total_distance = float(getattr(self.obstacle_speech, "reverse_distance_m", 0.25)) + float(
                getattr(self.obstacle_speech, "lateral_distance_m", 0.20)
            )
            cancel = getattr(self.navigation, "cancel_navigation", None)
            if callable(cancel):
                # A late success from the cancelled FollowWaypoints goal must
                # not advance the route while the same obstacle episode is
                # still being recovered. Invalidate its callback generation
                # before sending the cancel request, not after it returns.
                self._invalidate_nav_results()
                with self._lock:
                    self._expected_recovery_cancels += 1
                navigation_goal_cancelled = bool(cancel(timeout_seconds=2.0))
            if not navigation_goal_cancelled:
                with self._lock:
                    self._expected_recovery_cancels = max(
                        0, self._expected_recovery_cancels - 1
                    )
                result = {"success": False, "reason": "navigation_cancel_not_confirmed"}
                return
            self.navigation.stop_motion()
            if not self.navigation.is_robot_stopped():
                result = {"success": False, "reason": "stop_not_confirmed"}
                return
            lease = self.acquire_recovery(
                "EDGE_OBSTACLE", f"physical_obstacle_attempt_{attempt}", distance_m=total_distance
            )
            if lease is None:
                result = {"success": False, "reason": "recovery_lease_unavailable"}
                return
            clearer = getattr(self.navigation, "clear_local_costmap", None)
            if callable(clearer):
                clearer()
            result = self.navigation.execute_obstacle_recovery(
                reverse_distance_m=float(getattr(self.obstacle_speech, "reverse_distance_m", 0.25)),
                lateral_distance_m=float(getattr(self.obstacle_speech, "lateral_distance_m", 0.20)),
                lateral_direction=int(selection["lateral_direction"]),
                speed_mps=float(getattr(self.obstacle_speech, "recovery_speed_mps", 0.06)),
                timeout_seconds=float(getattr(self.obstacle_speech, "recovery_timeout_seconds", 6.0)),
            ) or {"success": False, "reason": "empty_behavior_result"}
        except Exception as exc:
            LOGGER.exception("bounded obstacle recovery attempt %s failed", attempt)
            result = {"success": False, "reason": str(exc)}
        finally:
            if lease is not None:
                self.release_recovery(lease)
            with self._lock:
                self._obstacle_recovery_active = False
                still_active = bool(
                    self.context
                    and self.context.state == "running"
                    and self.context.task_execution_id == execution_id
                )
                latest = self.navigation.obstacle_monitor_snapshot() if still_active else observation
                if still_active:
                    self._emit_obstacle_stage(
                        "RECOVERY_ATTEMPT",
                        latest or observation,
                        attempt=attempt,
                        action_result={**selection, **result},
                    )
                    pose = self.navigation.latest_pose()
                    if pose is not None:
                        self._obstacle_progress_anchor = (float(pose.x), float(pose.y))
                        self._obstacle_target_distance_anchor_m = self._distance_to_current_waypoint(pose)
                    self._obstacle_progress_anchor_at = time.monotonic()
                    if navigation_goal_cancelled:
                        self._send_from(self.context.current_waypoint_index)

    def _enter_obstacle_safe_observing(self, observation: dict) -> None:
        with self._lock:
            if not self.context or not self._obstacle_episode_id or self._leave_route_announced:
                return
            self._leave_route_announced = True
            self._obstacle_clear_started_at = None
            self._emit_obstacle_stage(
                "DISSUASION", observation, attempt=self._recovery_attempts
            )
            self._emit_obstacle_stage(
                "SAFE_OBSERVING", observation, attempt=self._recovery_attempts
            )
            self._expected_recovery_cancels += 1
        cancel = getattr(self.navigation, "cancel_navigation", None)
        cancelled = bool(cancel(timeout_seconds=2.0)) if callable(cancel) else False
        if not cancelled:
            with self._lock:
                self._expected_recovery_cancels = max(0, self._expected_recovery_cancels - 1)
            LOGGER.warning("safe observation entered without Nav2 cancel acknowledgement")
        self.navigation.stop_motion()

    def _resume_after_obstacle(self, observation: dict, *, automatic: bool) -> None:
        with self._lock:
            if not self.context or self.context.state != "running" or not self._obstacle_episode_id:
                return
            index = self.context.current_waypoint_index
            self._emit_obstacle_stage(
                "RESUMED", observation, attempt=self._recovery_attempts,
                reason="automatic_clear" if automatic else "manual_continue",
            )
            self._reset_obstacle_episode()
            if self._post_arrival_active(index):
                self._resume_post_arrival(index)
            else:
                self._send_from(index)

    def _emit_obstacle_stage(
        self,
        stage: str,
        observation: dict,
        *,
        attempt: int = 0,
        reason: str = "",
        action_result: dict | None = None,
    ) -> None:
        if not self.context or not self._obstacle_episode_id:
            return
        self._obstacle_stage = stage
        collision = dict(observation.get("collision_monitor") or {})
        latest_pose = self.navigation.latest_pose()
        pose = {}
        if latest_pose is not None:
            try:
                pose = {
                    "frame_id": str(getattr(latest_pose, "frame_id", "map") or "map"),
                    "x": float(latest_pose.x),
                    "y": float(latest_pose.y),
                    "yaw": float(getattr(latest_pose, "yaw", 0.0) or 0.0),
                }
            except (AttributeError, TypeError, ValueError):
                pose = {}
        max_attempts = int(
            getattr(self.obstacle_speech, "max_recovery_attempts", OBSTACLE_RECOVERY_MAX_ATTEMPTS)
        )
        alert_description = {
            "DETECTED_STOP": "发现障碍物，已停车",
            "RECOVERY_ATTEMPT": f"正在进行第 {int(attempt)}/{max_attempts} 次后退绕行避障",
            "DISSUASION": "三次避障失败，请离开巡检线路",
        }.get(stage, "")
        event_id = str(uuid.uuid5(
            uuid.NAMESPACE_URL,
            f"roamerx:obstacle:{self.context.task_execution_id}:{self._obstacle_episode_id}",
        ))
        payload = {
            "task_execution_id": self.context.task_execution_id,
            "round_number": self.context.round_number,
            "waypoint_index": self.context.current_waypoint_index,
            "obstacle_episode_id": self._obstacle_episode_id,
            "event_id": event_id,
            "stage": stage,
            "alert_description": alert_description,
            "recovery_attempt": int(attempt),
            "max_recovery_attempts": max_attempts,
            "detour_enabled": bool(self._segment_avoidance_enabled),
            "reason": reason,
            "front_obstacle_distance_m": observation.get("front_obstacle_distance_m"),
            "rear_clearance_m": observation.get("rear_clearance_m"),
            "left_clearance_m": observation.get("left_clearance_m"),
            "right_clearance_m": observation.get("right_clearance_m"),
            "collision_state": collision.get("state"),
            "collision_zone": collision.get("zone"),
            "collision_motion_scope": collision.get("motion_scope"),
            "collision_points_inside": collision.get("points_inside"),
            "pose": pose,
            "action_result": action_result or {},
            "reported_at": now_iso(),
        }
        self.event_callback("task.obstacle_stage", payload, "")
        if stage == "DETECTED_STOP" and callable(self.obstacle_evidence):
            try:
                self.obstacle_evidence(dict(payload))
            except Exception:
                LOGGER.warning("failed to schedule obstacle evidence capture", exc_info=True)
        if stage == "RECOVERY_ATTEMPT":
            self.event_callback("navigation.obstacle_recovery", payload, "")
        speech = {
            "DETECTED_STOP": ("obstacle_detected", "发现障碍物"),
            "RECOVERY_ATTEMPT": ("recovery_attempt", "后退尝试避障"),
            "DISSUASION": ("leave_route", "劝阻离开线路"),
        }.get(stage)
        if (
            speech
            and action_result is None
            and bool(getattr(self.obstacle_speech, "announce", True))
        ):
            self.event_callback(
                "task.obstacle_speech",
                {
                    **payload,
                    "speech_stage": speech[0],
                    "template_name": speech[1],
                },
                "",
            )
        LOGGER.info(
            "obstacle episode=%s stage=%s attempt=%s zone=%s distance=%s",
            self._obstacle_episode_id,
            stage,
            attempt,
            collision.get("zone"),
            observation.get("front_obstacle_distance_m"),
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

    def current_localization_waypoint(self) -> dict | None:
        """Return the pending waypoint as a localization seed, if a task exists."""
        with self._lock:
            if not self.context or self.context.state in self.TERMINAL_STATES:
                return None
            points = self.context.route_snapshot.get("waypoints") or []
            index = int(self.context.current_waypoint_index)
            if index < 0 or index >= len(points):
                return None
            point = dict(points[index])
            point["waypoint_index"] = index
            point["round_number"] = self.context.round_number
            return point

    def restore_paused_localization_recovery(self) -> bool:
        """Reconnect a persisted pause to its live one-shot correction.

        Edge restarts do not restart the localization node, so an NDT
        transaction can still be waiting in ROS while only its in-memory Edge
        ownership flags were lost. Re-adopt only a transaction whose id belongs
        to this exact execution and waypoint; an ordinary operator pause remains
        untouched. Do not arm automatic resume here: center reconciliation must
        run first, and process restart must never start physical motion.
        """
        with self._lock:
            if not self.context or self.context.state != "paused":
                return False
            decision = self._localization_decision()
            transaction = decision.get("one_shot_correction")
            if not isinstance(transaction, dict):
                return False
            transaction_id = str(transaction.get("transaction_id") or "")
            expected_prefix = (
                f"{self.context.task_execution_id}:waypoint:"
                f"{self.context.current_waypoint_index}:correction:"
            )
            status = str(transaction.get("status") or "")
            if not transaction_id.startswith(expected_prefix) or status not in {
                "waiting_source",
                "smoothing",
                "completed",
            }:
                return False
            self._active_correction_transaction_id = transaction_id
            self._active_correction_mode = waypoint_localization_mode(
                transaction.get("mode")
            )
            self._paused_for_localization = True
            self._paused_localization_reason = "absolute_required"
            LOGGER.info(
                "restored paused waypoint localization transaction id=%s status=%s",
                transaction_id,
                status,
            )
            return True

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
            waiting_at_waypoint = self._waiting_waypoint_index() is not None
            cancelled = waiting_at_waypoint or self._cancel_active_navigation()
            self.navigation.stop_motion()
            if not cancelled:
                # A cancel acknowledgement can time out while Nav2 is already
                # stopping. The explicit zero command is the safety boundary;
                # keep the task recoverable instead of incorrectly failing it.
                LOGGER.warning(
                    "Nav2 cancel was not acknowledged after localization loss; "
                    "holding the task paused after zeroing motion"
                )

            # Drop any in-flight pre-leg spin so a late Nav2 success cannot
            # cruise to a stale index after recovery redispatches.
            self._cancel_arrival_adjustment(reset_state=False)
            self._arrival_convergence_attempts.pop(
                self.context.current_waypoint_index, None
            )
            self._clear_departure_heading(cancel_navigation=False)
            self._paused_for_localization = True
            self._paused_localization_reason = "lost"
            # Localization loss zeros /cmd_vel; the SDK then goes passive and
            # the dog lies down. Resume must stand up again before cruising.
            self._navigation_prepared = False
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
            self._cancel_absolute_localization_resume_watch()
            self._cancel_arrival_precision_recovery_retry()
            if not self.context or self.context.state in self.TERMINAL_STATES:
                return
            # Keep the pending target. Re-picking the nearest remaining point
            # on a round-trip treats the return copy (point 4 of 1-2-3-2-1) as
            # closer and skips the outbound legs.
            resume_index = max(0, self.context.current_waypoint_index)
            pause_reason = self._paused_localization_reason
            if self.context.state == "running":
                # Automatic RTK recovery often cancels Nav2 after the task has
                # already been marked running again. Re-dispatch the pending
                # goal instead of leaving the dog standing with no Nav2 action.
                if self._post_arrival_active(resume_index):
                    self._paused_for_localization = False
                    self._paused_localization_reason = None
                    self._resume_post_arrival(resume_index)
                    return
                if self._waiting_waypoint_index() is not None:
                    self._paused_for_localization = False
                    self._paused_localization_reason = None
                    return
                self._paused_for_localization = False
                self._paused_localization_reason = None
                self._clear_departure_heading(cancel_navigation=True)
                self._navigation_prepared = False
                LOGGER.info(
                    "localization recovered while the task is running; re-dispatching from waypoint %s",
                    resume_index,
                )
                self._send_from(resume_index)
                return
            if self.context.state != "paused" or not self._paused_for_localization:
                return
            self._paused_for_localization = False
            self._paused_localization_reason = None
            self.context.state = "resuming"
            self.context.state_version += 1
            self._persist()
            self._emit(
                "task.resuming",
                code="LOCALIZATION_RECOVERED",
                message="localization is stable; resuming from the pending waypoint",
            )
            if self._post_arrival_active(resume_index):
                self.context.state = "running"
                self.context.state_version += 1
                self._persist()
                self._emit("task.resumed")
                self._resume_post_arrival(resume_index)
                return
            waiting_index = self._waiting_waypoint_index()
            if waiting_index is not None and pause_reason != "absolute_required":
                reached_index = waiting_index
                self._waypoint_localization_ready_index = reached_index
                self.context.state = "running"
                self.context.state_version += 1
                self._persist()
                self._emit("task.resumed")
                self._maybe_continue_after_waypoint(reached_index)
                return
            # Absolute-correction pauses happen after Nav2 already accepted the
            # waypoint, but relocalization can move the map pose materially.
            # Re-check the click in both indoor and outdoor modes before
            # advancing so a corrected pose cannot turn a stale arrival into a
            # skipped waypoint.
            if pause_reason == "absolute_required":
                self.context.state = "running"
                self.context.state_version += 1
                self._persist()
                self._emit("task.resumed")
                if self._post_arrival_active(resume_index):
                    self._resume_post_arrival(resume_index)
                    return
                points = self.context.route_snapshot.get("waypoints") or []
                waypoint = points[resume_index] if resume_index < len(points) else None
                if waypoint and self._arrival_xy_within_policy_tolerance(
                    waypoint, resume_index
                ):
                    LOGGER.info(
                        "absolute localization pause cleared at waypoint %s; continuing arrival convergence",
                        resume_index,
                    )
                    self._arrival_correction_completed_index = resume_index
                    self._correction_completed_at_mono = time.monotonic()
                    self._goal_offset = resume_index
                    self._dispatched_count = 1
                    self.on_navigation_result(
                        "succeeded", generation=self._nav_goal_generation
                    )
                    return
                LOGGER.info(
                    "absolute localization pause cleared at waypoint %s but "
                    "the corrected pose is still off-click; re-approaching from the corrected pose",
                    resume_index,
                )
                self._navigation_prepared = False
                self._clear_departure_heading(cancel_navigation=True)
                # This callback is emitted only after the localization layer
                # has accepted a stable recovery.  Do not poll it again with
                # a zero-second budget here: that can observe no sample at all
                # and turn a successful recovery into a false safe hold.
                if not self._reapproach_rejected_arrival(
                    resume_index, localization_recovered=True
                ):
                    self._emit_safe_hold(
                        "PHYSICAL_REAPPROACH_EXHAUSTED",
                        "重定位后仍未到达当前航点，进入安全保持",
                    )
                return
            self._clear_departure_heading(cancel_navigation=True)
            self._navigation_prepared = False
            self._send_from(resume_index)
            # Pre-leg face-turn keeps state at "resuming" until cruise starts;
            # mark running once a Nav2 goal (spin or cruise) is accepted.
            if (
                self.context.state == "resuming"
                and (
                    self._departure_heading_index is not None
                    or self._dispatched_count > 0
                )
            ):
                self.context.state = "running"
                self.context.state_version += 1
                self._persist()
                self._emit("task.resumed")

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

    def reconcile_center_state_version(
        self,
        execution_id: str,
        expected_state: str | None,
        expected_version: int | None,
    ) -> bool:
        """Adopt a newer center version when both sides already agree on state.

        Center may advance its version while repairing a center-only timeout.
        Persisting that version locally prevents the next legitimate Edge event
        from being discarded as stale. This method never changes task state or
        starts motion.
        """
        with self._lock:
            if (
                not self.context
                or self.context.task_execution_id != execution_id
                or self.context.state != expected_state
            ):
                return False
            try:
                version = int(expected_version)
            except (TypeError, ValueError):
                return False
            if version <= self.context.state_version:
                return False
            self.context.state_version = version
            self._persist()
            LOGGER.info(
                "adopted reconciled center task version execution=%s state=%s version=%d",
                execution_id,
                expected_state,
                version,
            )
            return True

    def prepare_task_start(self, envelope: MessageEnvelope) -> None:
        """Persist an accepted task before its command acknowledgement is sent."""
        with self._lock:
            operation_active = self.localization_operation_active_callback
            if callable(operation_active) and operation_active():
                raise ProtocolError(
                    "LOCALIZATION_COMMAND_BUSY",
                    "an operator localization request is still running; task start is deferred",
                )
            self._cancel_waypoint_dwell()
            command = envelope.payload["command"]
            route = dict(command["route_snapshot"])
            route.setdefault("map", dict(command.get("map") or {}))
            route["waypoints"] = [dict(waypoint) for waypoint in route.get("waypoints") or []]
            base_waypoints = [dict(point) for point in route["waypoints"]]
            self._assert_map_constraints(route)
            map_info = dict(route.get("map") or {})
            map_id = str(map_info.get("map_id") or "")
            map_version = str(map_info.get("map_version") or "")
            # A trusted pose is task-local evidence.  Reusing it across task
            # starts can retain a valid but distant pose on the same map and
            # prevent the new RTK/NDT initialization from becoming canonical.
            self.store.clear_last_trusted_pose(map_id, map_version)
            invalidate_trusted = getattr(self.navigation, "invalidate_last_trusted_pose", None)
            if callable(invalidate_trusted):
                invalidate_trusted()
            LOGGER.info("invalidated last_trusted pose for new task map=%s version=%s", map_id, map_version)
            docking = dict(command.get("docking") or {})
            dock_points = [
                index
                for index, waypoint in enumerate(route["waypoints"])
                if str(waypoint.get("arrival_policy") or "").strip().lower() == "dock"
            ]
            if dock_points and not bool(docking.get("enabled")):
                raise ProtocolError(
                    "DOCKING_CONTEXT_REQUIRED",
                    f"dock arrival_policy requires docking context (waypoints={dock_points})",
                )
            for index, waypoint in enumerate(route["waypoints"]):
                actions = list(waypoint.get("actions") or [])
                try:
                    self._action_registry.validate(actions)
                except ValueError as exc:
                    raise ProtocolError("UNSUPPORTED_WAYPOINT_ACTION", str(exc)) from exc
                mode = str(waypoint.get("speech_mode") or "").strip().lower()
                if mode and mode not in {"blocking", "non_blocking", "disabled"}:
                    raise ProtocolError(
                        "INVALID_MESSAGE",
                        f"waypoint {index} speech_mode must be blocking, non_blocking, or disabled",
                    )
            # Docking is deliberately ordered: waypoint 0 establishes the
            # safe approach line and must never be skipped by nearest-point
            # task startup behavior.
            initial_waypoint_index = 0 if docking.get("enabled") else self._nearest_waypoint_index(route)
            loop_direction = str(command.get("loop_direction") or "").strip().lower()
            explicit_loop_round = bool(
                command.get("loop_execution")
                and int(command.get("round_number", 1) or 1) > 1
                and loop_direction in {"forward", "reverse"}
            )
            if explicit_loop_round and loop_direction == "reverse":
                route["waypoints"].reverse()
                for sequence, waypoint in enumerate(route["waypoints"]):
                    waypoint.setdefault("map_point_number", int(waypoint.get("sequence", 0)) + 1)
                    waypoint["sequence"] = sequence
                route["execution_order"] = "reverse_from_route_end"
                initial_waypoint_index = 0
            elif explicit_loop_round:
                route["execution_order"] = "forward"
                initial_waypoint_index = 0
            if explicit_loop_round:
                route["loop_round_continuation"] = True
            reverse_return = bool(
                command.get("loop_execution")
                and not explicit_loop_round
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
            route["initial_waypoint_index"] = initial_waypoint_index
            self._strip_non_navigation_waypoint_actions(route)
            if base_waypoints is not None:
                self._strip_non_navigation_waypoint_actions({"waypoints": base_waypoints})
            self.context = TaskContext(
                task_execution_id=envelope.payload["task_execution_id"],
                trace_id=str(envelope.trace_id or ""),
                state="accepted",
                # The center creates state_version=0 then advances it to 1 for
                # task.start dispatching.  The edge acknowledgement must be
                # strictly newer than that dispatch state.
                state_version=2,
                route_snapshot=route,
                current_waypoint_index=initial_waypoint_index,
                start_command_id=envelope.payload["command_id"],
                record_rosbag=bool(command.get("record_rosbag", False)),
                loop_execution=bool(command.get("loop_execution", False)),
                loop_session_id=str(command.get("loop_session_id") or ""),
                continuous_rosbag=bool(command.get("continuous_rosbag", False)),
                docking=docking,
                round_number=max(1, int(command.get("round_number", 1))),
                loop_total=max(1, int(command.get("loop_total", 1))),
                loop_base_waypoints=base_waypoints,
            )
            self._last_target_index = initial_waypoint_index - 1
            self._last_reached_index = initial_waypoint_index - 1
            self._last_progress_emit_at = None
            self._last_localization_policy = None
            self._speech_waiting_index = None
            self._speech_wait_finished = False
            self._dwell_waiting_index = None
            self._dwell_wait_finished = False
            self._waypoint_localization_ready_index = None
            self._arrival_retry_counts = {}
            self._arrival_reapproach_index = None
            self.context.arrival_reapproach_waypoint_index = None
            self.context.arrival_reapproach_attempts = 0
            self._arrival_correction_completed_index = None
            self._arrival_heading_completed_index = None
            self._arrival_convergence_attempts = {}
            self._cancel_arrival_adjustment()
            self._emitted_event_keys.clear()
            self._action_registry.reset()
            self._recovery_arbiter.reset_budget()
            self._recovery_arbiter.force_release()
            self._cancel_all_bt_recovery_lease_timeouts()
            self._correction_generation = 0
            self._correction_completed_at_mono = None
            self._active_correction_transaction_id = None
            self._active_correction_mode = None
            self._leg_generation = 0
            self._active_leg_profile = None
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
            requested_scene_scope=str(map_info.get("scene_scope") or route.get("scene_scope") or ""),
        )
        try:
            validate_route_against_map(
                constraints,
                scene_scope=str(map_info.get("scene_scope") or constraints.get("scene_scope") or ""),
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

    def _localization_decision(self) -> dict:
        getter = getattr(self.navigation, "localization_decision", None)
        decision = getter() if callable(getter) else {}
        if isinstance(decision, dict) and decision:
            return decision
        diagnostics = getattr(self.navigation, "localization_diagnostics", None)
        payload = diagnostics() if callable(diagnostics) else {}
        nested = (payload or {}).get("decision") if isinstance(payload, dict) else {}
        return nested if isinstance(nested, dict) else {}

    def _localization_sample_fresh(self, decision: dict | None = None) -> bool:
        """Reject stale localization samples when age metadata is available."""
        decision = decision if isinstance(decision, dict) else self._localization_decision()
        age = decision.get("sample_age_seconds")
        if age is None and isinstance(decision.get("localization"), dict):
            age = decision["localization"].get("sample_age_seconds")
        if age is None:
            # Older Edge/robot versions did not publish age; retain backward
            # compatibility while still enforcing freshness on new telemetry.
            return True
        try:
            return 0.0 <= float(age) <= ARRIVAL_POSE_MAX_AGE_SECONDS
        except (TypeError, ValueError):
            return False

    def _rtk_good_for_navigation(self) -> bool:
        decision = self._localization_decision()
        if decision.get("rtk_good_for_navigation") is True:
            return True
        return (
            decision.get("rtk_usable") is True
            and str(decision.get("rtk_quality") or "").lower() == "fixed"
            and decision.get("rtk_heading_usable") is True
        )

    def _rtk_position_good_for_navigation(self) -> bool:
        decision = self._localization_decision()
        if decision.get("rtk_position_good_for_navigation") is True:
            return True
        return (
            decision.get("rtk_usable") is True
            and str(decision.get("rtk_quality") or "").lower() == "fixed"
        )

    def _startup_rtk_xy_needs_reanchor(self, decision: dict) -> bool:
        """True when LIO map pose has drifted away from fixed RTK before Nav2 starts."""
        try:
            rtk_x = float(decision["rtk_x"])
            rtk_y = float(decision["rtk_y"])
        except (KeyError, TypeError, ValueError):
            return False
        if not isfinite(rtk_x) or not isfinite(rtk_y):
            return False
        pose_xy = self._current_pose_xy()
        if pose_xy is None:
            return False
        return hypot(pose_xy[0] - rtk_x, pose_xy[1] - rtk_y) > WAYPOINT_CORRECTION_DRIFT_M

    def _reported_localization_status(self) -> str:
        diagnostics = getattr(self.navigation, "localization_diagnostics", None)
        payload = diagnostics() if callable(diagnostics) else {}
        if not isinstance(payload, dict):
            return ""
        raw = payload.get("raw_pose") if isinstance(payload.get("raw_pose"), dict) else {}
        return str(
            payload.get("localization_status")
            or raw.get("localization_status")
            or ""
        ).lower()

    def _startup_localization_already_ready(self, decision: dict) -> bool:
        """True when the current pose is already good enough to skip reseeding.

        Indoor loop rounds were waiting 30-90s in `accepted` because startup
        always re-ran active_relocalize, even after rest-period repair left
        FAST-LIO/NDT stable. Outdoor RTK seeding stays on its own path.
        """
        if bool(decision.get("lio_motion_anomaly")):
            return False
        source = str(decision.get("active_source") or "")
        if source not in {"lio_imu", "ndt_imu", "rtk_imu"}:
            return False
        if not bool(decision.get("absolute_stable")):
            return False
        status = self._reported_localization_status()
        if status and status != "normal":
            return False
        return True

    def initialize_before_navigation(self) -> None:
        self._initialize_before_navigation()
        if getattr(self, "_startup_localization_reused_stable", False):
            LOGGER.info(
                "startup localization reused stable FAST-LIO+IMU pose; skipping fresh handoff acceptance"
            )
            return
        accept_trusted = getattr(self.navigation, "accept_startup_trusted_pose", None)
        if callable(accept_trusted):
            accept_trusted()

    def _progressive_startup_relocalize(self, points: list[dict]) -> None:
        """Run the cold-start search in its fixed, map-scoped order.

        Outdoor routes cannot safely treat a float/unavailable RTK position as
        a reason to verify a remembered or operator-clicked seed.  The ROS
        adapter owns the stationary sequence and its quality/handoff gate:
        mapping origin and surrounding candidates, each route waypoint, then
        keyframe-global matching.
        """
        relocalize = getattr(self.navigation, "progressive_relocalize", None)
        if not callable(relocalize):
            raise ProtocolError(
                "PROGRESSIVE_RELOCALIZATION_UNAVAILABLE",
                "outdoor RTK fallback requires mapping-origin progressive relocalization",
            )

        origin = None
        if self.map_activation_adapter is not None:
            try:
                origin = self.map_activation_adapter.mapping_start_pose()
            except ProtocolError as exc:
                origin = {
                    "unavailable_error_code": exc.code,
                    "unavailable_error_message": exc.message,
                }
            except Exception as exc:
                origin = {
                    "unavailable_error_code": "MAPPING_START_POSE_INVALID",
                    "unavailable_error_message": str(exc),
                }

        waypoints = []
        for raw in points:
            if not isinstance(raw, dict):
                continue
            try:
                waypoints.append({
                    "x": float(raw["x"]),
                    "y": float(raw["y"]),
                    "yaw": float(raw.get("yaw", 0.0) or 0.0),
                })
            except (KeyError, TypeError, ValueError):
                LOGGER.warning("skipping invalid route waypoint in startup progressive search")

        LOGGER.info(
            "startup localization running progressive search: mapping origin, %d route waypoint(s), global fallback",
            len(waypoints),
        )
        relocalize(origin=origin, waypoints=waypoints, wait_seconds=180.0)

    def _initialize_before_navigation(self) -> None:
        """Require a verified absolute pose before the first Nav2 goal."""
        self._startup_localization_reused_stable = False
        if not self.context or self.context.state != "accepted":
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "accepted task context is missing")
        decision = self._localization_decision()
        outdoor = self._outdoor_navigation_profile()
        points = self.context.route_snapshot.get("waypoints") or []
        if outdoor:
            seed_rtk = getattr(self.navigation, "set_initial_pose_from_rtk", None)
            if callable(seed_rtk):
                try:
                    LOGGER.info(
                        "startup localization checking live RTK before progressive fallback"
                    )
                    seed_rtk()
                    return
                except ProtocolError as exc:
                    if exc.code not in RTK_STARTUP_PROGRESSIVE_FALLBACK_CODES:
                        raise
                    LOGGER.warning(
                        "fixed RTK startup verification failed (%s); using mapping-origin progressive localization: %s",
                        exc.code,
                        exc.message,
                    )
                except Exception as exc:
                    LOGGER.warning(
                        "fixed RTK startup initialization raised %s; using mapping-origin progressive localization",
                        exc,
                    )
            else:
                LOGGER.warning(
                    "startup RTK verification API is unavailable; using progressive localization"
                )
            self._progressive_startup_relocalize(points)
            return
        if not self._outdoor_navigation_profile() and self._startup_localization_already_ready(
            decision
        ):
            self._startup_localization_reused_stable = True
            LOGGER.info(
                "startup localization already stable (source=%s status=%s); skipping reseeding",
                decision.get("active_source"),
                self._reported_localization_status() or "unspecified",
            )
            return
        index = min(max(0, self.context.current_waypoint_index), max(0, len(points) - 1))
        candidate_indexes = [index, index - 1, index + 1, 0, len(points) - 1]
        relocalize = getattr(self.navigation, "active_relocalize", None)
        seen = set()
        if callable(relocalize):
            map_info = dict(self.context.route_snapshot.get("map") or {})
            latest_getter = getattr(self.navigation, "latest_pose", None)
            latest = latest_getter() if callable(latest_getter) else None
            trusted_pose_getter = getattr(self.navigation, "latest_trusted_pose", None)
            memory_trusted = trusted_pose_getter() if callable(trusted_pose_getter) else None
            disk_trusted = self.store.load_last_trusted_pose(
                str(map_info.get("map_id") or ""),
                str(map_info.get("map_version") or ""),
            )
            waypoint = points[index] if index < len(points) else None
            max_drift = float(
                getattr(getattr(self, "safety_config", None), "localization_trusted_seed_max_drift_m", 15.0)
            )
            startup_seed = select_recovery_seed(
                latest_pose=latest,
                memory_trusted=memory_trusted,
                disk_trusted=disk_trusted,
                waypoint=waypoint,
                max_drift_m=max_drift,
            )
            if startup_seed:
                trusted_seed = dict(startup_seed)
                trusted_seed.update({"max_attempts": 12, "source": "startup_trusted"})
                try:
                    LOGGER.info(
                        "startup localization using %s seed",
                        startup_seed.get("source"),
                    )
                    relocalize(trusted_seed)
                    return
                except Exception as exc:
                    LOGGER.warning("startup trusted-pose localization failed: %s", exc)
            for candidate_index in candidate_indexes:
                if candidate_index < 0 or candidate_index >= len(points):
                    continue
                waypoint = dict(points[candidate_index])
                key = (round(float(waypoint["x"]), 3), round(float(waypoint["y"]), 3))
                if key in seen:
                    continue
                seen.add(key)
                try:
                    LOGGER.info("startup localization waypoint candidate index=%d", candidate_index)
                    relocalize({"x": float(waypoint["x"]), "y": float(waypoint["y"]), "yaw": float(waypoint.get("yaw", 0.0)), "max_attempts": 12, "source": "startup_waypoint", "waypoint_index": candidate_index})
                    return
                except Exception as exc:
                    LOGGER.warning("startup waypoint %d localization failed: %s", candidate_index, exc)
        global_relocalize = getattr(self.navigation, "global_relocalize", None)
        if callable(global_relocalize):
            try:
                global_relocalize(wait_seconds=90.0)
                return
            except Exception as exc:
                LOGGER.warning("startup global localization failed: %s", exc)
        raise ProtocolError("INITIALIZATION_FAILED", "startup waypoint and global localization both failed")

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

    def _invalidate_nav_results(self) -> None:
        """Drop in-flight Nav2 callbacks so a cancelled goal cannot advance the route."""
        self._nav_goal_generation += 1

    def _cancel_localization_recovery(self) -> None:
        """Invalidate an application-owned recovery worker before task teardown.

        Recovery runs outside the task lock and can otherwise observe a stale
        paused context while a new loop round is already being prepared.
        """
        callback = self.localization_recovery_cancel_callback
        if callable(callback):
            try:
                callback()
            except Exception:
                LOGGER.exception("failed to cancel localization recovery during task teardown")

    def _bind_nav_result(self):
        generation = self._nav_goal_generation + 1
        self._nav_goal_generation = generation

        def _on_result(status: str, error_message: str = "", details: dict | None = None) -> None:
            self.on_navigation_result(status, error_message, details, generation=generation)

        return _on_result

    def _cancel_active_navigation(self, timeout_seconds: float | None = None) -> bool:
        self._invalidate_nav_results()
        cancel = getattr(self.navigation, "cancel_navigation", None)
        if not callable(cancel):
            return True
        try:
            if timeout_seconds is None:
                cancelled = bool(cancel())
            else:
                cancelled = bool(cancel(timeout_seconds=timeout_seconds))
        except TypeError:
            cancelled = bool(cancel())
        except Exception:
            LOGGER.warning("Nav2 cancel raised", exc_info=True)
            return False
        if cancelled:
            snapshot = self._recovery_arbiter.snapshot()
            if snapshot.get("owner") == "BT_NAVIGATOR":
                self._release_bt_recovery_lease_after_stop(
                    int(snapshot.get("recovery_generation") or 0),
                    trigger="navigation_cancelled",
                )
        return cancelled

    def _clear_departure_heading(self, *, cancel_navigation: bool = False) -> None:
        """Drop an in-flight pre-leg spin so a later redispatch owns Nav2."""
        self._cancel_departure_heading_timeout()
        self._departure_heading_cancel.set()
        had_departure = (
            self._departure_heading_index is not None
            or self._departure_cruise_index is not None
            or self._departure_heading_mode is not None
        )
        mode = self._departure_heading_mode
        self._departure_heading_index = None
        self._departure_cruise_index = None
        self._departure_heading_completed_index = None
        self._arrival_heading_completed_index = None
        self._departure_heading_mode = None
        self._departure_heading_is_arrival = False
        self._departure_heading_tolerance_rad = DEPARTURE_HEADING_ALIGN_RAD
        self._departure_heading_thread = None
        if not had_departure:
            return
        LOGGER.info("cleared in-flight departure heading before re-dispatch")
        if mode == "teleop" or cancel_navigation:
            stop = getattr(self.navigation, "stop_motion", None)
            velocity = getattr(self.navigation, "teleop_velocity", None)
            try:
                if callable(stop):
                    stop()
                elif callable(velocity):
                    velocity(vx=0.0, vy=0.0, yaw_rate=0.0)
            except Exception:
                LOGGER.warning(
                    "failed to stop teleop while clearing departure heading",
                    exc_info=True,
                )
        if cancel_navigation and mode != "teleop":
            try:
                self._cancel_active_navigation()
            except Exception:
                LOGGER.warning(
                    "failed to cancel navigation while clearing departure heading",
                    exc_info=True,
                )
        elif cancel_navigation:
            self._invalidate_nav_results()
        self._restore_navigation_profile()

    def _cancel_departure_heading_timeout(self) -> None:
        timer = self._departure_heading_timer
        self._departure_heading_timer = None
        if timer is not None:
            timer.cancel()

    def _arm_departure_heading_timeout(self) -> None:
        self._cancel_departure_heading_timeout()
        timer = threading.Timer(
            DEPARTURE_HEADING_TIMEOUT_SECONDS,
            self._on_departure_heading_timeout,
        )
        timer.daemon = True
        self._departure_heading_timer = timer
        timer.start()

    def _departure_heading_context_active(self) -> bool:
        return bool(
            self.context
            and self.context.state
            not in self.TERMINAL_STATES | {"pausing", "cancelling", "paused"}
        )

    def _on_departure_heading_timeout(self) -> None:
        with self._lock:
            if not self._departure_heading_context_active():
                return
            if (
                self._departure_heading_index is None
                and self._departure_cruise_index is None
            ):
                return
            cruise_index = self._departure_cruise_index
            reached_index = self._departure_heading_index
            mode = self._departure_heading_mode or "nav2"
            is_arrival = self._departure_heading_is_arrival
            if is_arrival:
                LOGGER.error(
                    "arrival heading timed out after %.1fs (%s); keeping the yaw requirement",
                    DEPARTURE_HEADING_TIMEOUT_SECONDS,
                    mode,
                )
                self._clear_departure_heading(cancel_navigation=True)
                attempts = int(
                    self._arrival_convergence_attempts.get(int(reached_index or 0), 0)
                )
                if reached_index is not None and attempts < ARRIVAL_CONVERGENCE_MAX_ATTEMPTS:
                    self.on_navigation_result(
                        "succeeded", generation=self._nav_goal_generation
                    )
                    return
                self._emit_safe_hold(
                    "ARRIVAL_POSE_CONVERGENCE_FAILED",
                    "最终航向两次调整仍未完成，机器人保持停车",
                )
                return
            LOGGER.error(
                "departure heading timed out after %.1fs (%s); parking task "
                "(reached=%s cruise=%s)",
                DEPARTURE_HEADING_TIMEOUT_SECONDS,
                mode,
                reached_index,
                cruise_index,
            )
            self._clear_departure_heading(cancel_navigation=True)
            self.navigation.stop_motion()
            self.context.state = "paused"
            self.context.state_version += 1
            self._persist()
            self._emit(
                "task.paused",
                code="DEPARTURE_HEADING_ALIGNMENT_REQUIRED",
                message="departure heading did not align; robot remains parked",
            )

    def _complete_departure_heading_locked(
        self, reached_index: int | None, cruise_index: int | None
    ) -> None:
        """Finish a successful in-place align and continue cruise or post-arrival."""
        self._cancel_departure_heading_timeout()
        self._departure_heading_cancel.set()
        self._departure_heading_index = None
        self._departure_cruise_index = None
        self._departure_heading_mode = None
        self._departure_heading_is_arrival = False
        self._departure_heading_tolerance_rad = DEPARTURE_HEADING_ALIGN_RAD
        self._restore_navigation_profile()
        if cruise_index is not None:
            LOGGER.info(
                "pre-leg departure heading complete; cruising to waypoint %d",
                cruise_index,
            )
            self._dispatch_navigation(cruise_index)
            return
        if reached_index is not None:
            self._departure_heading_completed_index = reached_index

    def _use_teleop_departure_heading(self) -> bool:
        """Nav2 require_yaw weaves on 180deg turns; spin with teleop yaw instead."""
        return callable(getattr(self.navigation, "teleop_velocity", None))

    def _use_teleop_arrival_heading(self, waypoint: dict, waypoint_index: int) -> bool:
        """Whether this waypoint's final yaw is handled after XY arrival.

        Docking retains Nav2's precision-goal ownership.  Pass-through points
        intentionally have no stationary arrival phase, so they cannot ask for
        an independent final turn.
        """
        return bool(
            not self._is_docking_task()
            and bool(waypoint.get("require_yaw", False))
            and self._arrival_policy(waypoint, waypoint_index) != "pass_through"
            and self._use_teleop_departure_heading()
        )

    def _start_departure_heading(
        self,
        *,
        desired_yaw: float,
        reached_index: int,
        cruise_index: int | None,
        profile_waypoint: dict,
        error_rad: float | None = None,
        arrival_heading: bool = False,
        tolerance_rad: float = DEPARTURE_HEADING_ALIGN_RAD,
    ) -> bool:
        # At a waypoint Nav2 has already completed the XY goal.  Do not send
        # another Nav2 goal just to rotate: RPP uses path geometry here and can
        # orbit the click.  The direct yaw controller is stationary instead.
        if arrival_heading:
            if not self._use_teleop_departure_heading():
                return False
            return self._start_teleop_departure_heading(
                desired_yaw=desired_yaw,
                reached_index=reached_index,
                cruise_index=cruise_index,
                error_rad=error_rad,
                arrival_heading=True,
                tolerance_rad=tolerance_rad,
                cancel_navigation=False,
            )
        if self._use_teleop_departure_heading():
            return self._start_teleop_departure_heading(
                desired_yaw=desired_yaw,
                reached_index=reached_index,
                cruise_index=cruise_index,
                error_rad=error_rad,
            )
        return self._start_nav2_departure_heading(
            desired_yaw=desired_yaw,
            reached_index=reached_index,
            cruise_index=cruise_index,
            profile_waypoint=profile_waypoint,
            error_rad=error_rad,
        )

    def _start_nav2_departure_heading(
        self,
        *,
        desired_yaw: float,
        reached_index: int,
        cruise_index: int | None,
        profile_waypoint: dict,
        error_rad: float | None = None,
    ) -> bool:
        pose = self.navigation.latest_pose() if self.navigation else None
        goal = {
            "x": float(getattr(pose, "x", profile_waypoint["x"]) if pose is not None else profile_waypoint["x"]),
            "y": float(getattr(pose, "y", profile_waypoint["y"]) if pose is not None else profile_waypoint["y"]),
            "yaw": desired_yaw,
            "require_yaw": True,
            "waypoint_id": profile_waypoint.get("waypoint_id"),
            "map_point_number": profile_waypoint.get("map_point_number"),
        }
        setter = getattr(self.navigation, "set_waypoint_profile", None)
        if callable(setter):
            setter(
                avoid_obstacles=bool(profile_waypoint.get("avoidance_to_next", True)),
                require_yaw=True,
                final_approach=True,
                outdoor=self._outdoor_navigation_profile(),
                local_controller=str(profile_waypoint.get("local_controller") or "mppi"),
            )
        self._departure_heading_cancel.clear()
        self._departure_heading_index = reached_index
        self._departure_cruise_index = cruise_index
        self._departure_heading_mode = "nav2"
        self._goal_offset, self._dispatched_count = reached_index, 1
        accepted = self.navigation.send_waypoints([goal], self.on_feedback, self._bind_nav_result())
        if not accepted:
            self._departure_heading_index = None
            self._departure_cruise_index = None
            self._departure_heading_mode = None
            self._restore_navigation_profile()
            return False
        self._arm_departure_heading_timeout()
        self.on_feedback(0, milestone="departure_heading_dispatched")
        if cruise_index is not None:
            LOGGER.info(
                "pre-leg departure heading toward waypoint %d yaw=%.3f (error=%.1fdeg)",
                cruise_index,
                desired_yaw,
                abs(error_rad or 0.0) * 180.0 / pi,
            )
        else:
            LOGGER.info(
                "departure heading dispatched at waypoint %d yaw=%.3f",
                reached_index,
                desired_yaw,
            )
        return True

    def _start_teleop_departure_heading(
        self,
        *,
        desired_yaw: float,
        reached_index: int,
        cruise_index: int | None,
        error_rad: float | None = None,
        arrival_heading: bool = False,
        tolerance_rad: float = DEPARTURE_HEADING_ALIGN_RAD,
        cancel_navigation: bool = True,
    ) -> bool:
        if cancel_navigation:
            try:
                self._cancel_active_navigation(timeout_seconds=2.0)
            except Exception:
                LOGGER.warning(
                    "failed to cancel navigation before teleop departure heading",
                    exc_info=True,
                )
        self._departure_heading_cancel.clear()
        self._departure_heading_index = reached_index
        self._departure_cruise_index = cruise_index
        self._departure_heading_mode = "teleop"
        self._departure_heading_is_arrival = arrival_heading
        self._departure_heading_tolerance_rad = max(0.01, float(tolerance_rad))
        self._goal_offset, self._dispatched_count = reached_index, 1
        self._arm_departure_heading_timeout()
        self.on_feedback(
            0,
            milestone=("arrival_heading_dispatched" if arrival_heading else "departure_heading_dispatched"),
        )
        thread = threading.Thread(
            target=self._teleop_departure_heading_worker,
            args=(desired_yaw, reached_index, cruise_index, arrival_heading),
            daemon=True,
            name="departure-heading-teleop",
        )
        self._departure_heading_thread = thread
        thread.start()
        LOGGER.info(
            "teleop %s heading toward %s yaw=%.3f (error=%.1fdeg)",
            "arrival" if arrival_heading else "departure",
            f"waypoint {cruise_index}" if cruise_index is not None else f"index {reached_index}",
            desired_yaw,
            abs(error_rad or 0.0) * 180.0 / pi,
        )
        return True

    def _teleop_departure_heading_worker(
        self,
        desired_yaw: float,
        reached_index: int,
        cruise_index: int | None,
        arrival_heading: bool,
    ) -> None:
        velocity = getattr(self.navigation, "teleop_velocity", None)
        stop = getattr(self.navigation, "stop_motion", None)
        aligned = False
        stable = 0
        try:
            if not callable(velocity):
                return
            while not self._departure_heading_cancel.is_set():
                with self._lock:
                    if not self._departure_heading_context_active():
                        return
                    if (
                        self._departure_heading_mode != "teleop"
                        or self._departure_heading_index != reached_index
                        or self._departure_cruise_index != cruise_index
                        or self._departure_heading_is_arrival != arrival_heading
                    ):
                        return
                pose = self.navigation.latest_pose() if self.navigation else None
                if pose is None:
                    if self._departure_heading_cancel.wait(DEPARTURE_HEADING_TELEOP_PERIOD_SECONDS):
                        return
                    continue
                try:
                    yaw = float(getattr(pose, "yaw", 0.0) or 0.0)
                except (AttributeError, TypeError, ValueError):
                    if self._departure_heading_cancel.wait(DEPARTURE_HEADING_TELEOP_PERIOD_SECONDS):
                        return
                    continue
                error = self._heading_error_rad(desired_yaw, yaw)
                if abs(error) <= self._departure_heading_tolerance_rad:
                    stable += 1
                    if stable >= DEPARTURE_HEADING_STABLE_SAMPLES:
                        aligned = True
                        break
                    if self._departure_heading_cancel.wait(DEPARTURE_HEADING_TELEOP_PERIOD_SECONDS):
                        return
                    continue
                stable = 0
                # Larger heading errors use the full teleop yaw rate; mid-range
                # errors slow down so the controller can settle without weave.
                scale = 1.0 if abs(error) >= DEPARTURE_HEADING_INPLACE_RAD else 0.7
                rate = scale * DEPARTURE_HEADING_TELEOP_YAW_RATE
                if error < 0:
                    rate = -rate
                try:
                    velocity(vx=0.0, vy=0.0, yaw_rate=rate)
                except Exception:
                    LOGGER.warning("teleop departure heading command failed", exc_info=True)
                    break
                if self._departure_heading_cancel.wait(DEPARTURE_HEADING_TELEOP_PERIOD_SECONDS):
                    return
        finally:
            try:
                if callable(stop):
                    stop()
                elif callable(velocity):
                    velocity(vx=0.0, vy=0.0, yaw_rate=0.0)
            except Exception:
                LOGGER.warning("failed to stop teleop after departure heading", exc_info=True)
        try:
            if not aligned:
                return
            with self._lock:
                if not self._departure_heading_context_active():
                    return
                if (
                    self._departure_heading_mode != "teleop"
                    or self._departure_heading_index != reached_index
                    or self._departure_cruise_index != cruise_index
                    or self._departure_heading_is_arrival != arrival_heading
                ):
                    return
                if arrival_heading:
                    self._complete_departure_heading_locked(reached_index, None)
                    # A completed arrival yaw must not be mistaken for a
                    # departure turn: the next leg still gets its own heading
                    # check. Re-enter normal post-arrival confirmation once.
                    self._departure_heading_completed_index = None
                    self._arrival_heading_completed_index = reached_index
                    self._set_post_arrival_stage(
                        reached_index, "heading_aligned"
                    )
                    self._emit_idempotent(
                        "task.arrival_heading_aligned",
                        event_type_key="arrival_heading_aligned",
                        waypoint_id=str(reached_index),
                        message="已原地对准目标朝向",
                    )
                    self.on_navigation_result(
                        "succeeded", generation=self._nav_goal_generation
                    )
                    return
                if cruise_index is not None:
                    self._complete_departure_heading_locked(reached_index, cruise_index)
                    return
                # Arrival/speech already ran. Finish the next-leg spin and
                # dispatch the following cruise without re-entering Nav2 success
                # (that would restart speech and look like another arrival).
                self._complete_departure_heading_locked(reached_index, None)
                self._continue_after_waypoint(reached_index)
        finally:
            # Keep the worker marked alive until cruise/post-arrival continue
            # finishes; otherwise tests (and persist) can observe a completed
            # spin before the next waypoint is dispatched.
            with self._lock:
                if self._departure_heading_thread is threading.current_thread():
                    self._departure_heading_thread = None

    def _reverse_start_colocation_m(self) -> float:
        if self._outdoor_navigation_profile():
            return REVERSE_START_COLOCATION_M
        return WAYPOINT_COLOCATION_M

    def _pose_agrees_with_click_for_skip(
        self, click_xy: tuple[float, float], pose_xy: tuple[float, float], colocation_m: float
    ) -> bool:
        """Outdoor reverse skip needs LIO near click AND fixed RTK near click."""
        if hypot(pose_xy[0] - click_xy[0], pose_xy[1] - click_xy[1]) > colocation_m:
            return False
        if not self._outdoor_navigation_profile():
            return True
        decision = self._localization_decision()
        rtk_ok = (
            decision.get("rtk_position_good_for_navigation") is True
            or str(decision.get("rtk_quality") or "").lower() == "fixed"
            or self._rtk_position_good_for_navigation()
        )
        if not rtk_ok:
            LOGGER.info(
                "outdoor reverse skip blocked: RTK not fixed/usable at colocated start"
            )
            return False
        rtk_xy = self._rtk_xy_from_decision(decision)
        if rtk_xy is None:
            LOGGER.info("outdoor reverse skip blocked: RTK XY unavailable")
            return False
        rtk_dist = hypot(rtk_xy[0] - click_xy[0], rtk_xy[1] - click_xy[1])
        if rtk_dist > ARRIVAL_ACCEPT_RTK_M:
            LOGGER.info(
                "outdoor reverse skip blocked: RTK is %.2fm from click (limit %.2fm)",
                rtk_dist,
                ARRIVAL_ACCEPT_RTK_M,
            )
            return False
        lio_rtk = hypot(pose_xy[0] - rtk_xy[0], pose_xy[1] - rtk_xy[1])
        if lio_rtk > REVERSE_SKIP_LIO_RTK_DRIFT_M:
            LOGGER.info(
                "outdoor reverse skip blocked: LIO↔RTK drift %.2fm exceeds %.2fm",
                lio_rtk,
                REVERSE_SKIP_LIO_RTK_DRIFT_M,
            )
            return False
        return True

    def _advance_past_colocated_waypoints(self, index: int) -> int:
        """Skip poses the robot already occupies before dispatching a cruise.

        Reverse loop starts at the route end. Cruising back to that same click
        produces a long in-place spin and local weaving. Only skip at reverse
        start (index 0). Recovery/retry redispatches from a later index must
        still cruise to that click; skipping them after a localization jump
        or Nav2 reject abandons the current point and later fails
        FINAL_POSE_OUT_OF_TOLERANCE.

        Outdoor RTK maps must also agree in the RTK frame; otherwise a drifted
        LIO "already here" skip drops the end-point correction and the return
        leg plans from the wrong map pose.
        """
        if not self.context or self._is_docking_task():
            return index
        order = str(self.context.route_snapshot.get("execution_order") or "")
        continuation = bool(self.context.route_snapshot.get("loop_round_continuation"))
        if order not in {"reverse_from_route_end", "reverse", "forward"}:
            return index
        if order == "forward" and not continuation:
            return index
        if index != 0:
            return index
        waypoints = self.context.route_snapshot.get("waypoints") or []
        pose_xy = self._current_pose_xy()
        if pose_xy is None or index < 0 or index >= len(waypoints):
            return index
        colocation_m = self._reverse_start_colocation_m()
        advanced = index
        while advanced < len(waypoints) - 1:
            xy = _waypoint_xy(waypoints[advanced])
            if xy is None:
                break
            if not self._pose_agrees_with_click_for_skip(xy, pose_xy, colocation_m):
                break
            LOGGER.info(
                "skipping colocated waypoint %d before dispatch (within %.2fm)",
                advanced,
                colocation_m,
            )
            progress = self._build_progress_locked(
                advanced,
                "waypoint_reached",
                advanced + 1,
                None,
            )
            self.event_callback("task.progress", progress, "")
            self._last_reached_index = max(self._last_reached_index, advanced)
            self._last_target_index = max(self._last_target_index, advanced)
            advanced += 1
        if advanced != index:
            self.context.current_waypoint_index = advanced
            self.context.state_version += 1
            self._persist()
        return advanced

    def _confirm_and_skip_loop_anchor(self, previous_terminal: dict | None) -> int:
        """Skip a repeated loop anchor only after a fresh full arrival check.

        A route such as A-B-C-A finishes each round already at the next
        round's first click.  Treating that click as a normal arrival would
        replay its speech, actions and dwell.  This is deliberately separate
        from reverse-start skipping: it applies only at a newly created loop
        round and emits a dedicated audit event instead of ``waypoint_reached``.
        """
        if not self.context or self._is_docking_task() or not previous_terminal:
            return 0
        waypoints = self.context.route_snapshot.get("waypoints") or []
        if len(waypoints) < 2:
            return 0
        terminal_xy = _waypoint_xy(previous_terminal)
        anchor_xy = _waypoint_xy(waypoints[0])
        if terminal_xy is None or anchor_xy is None:
            return 0
        if hypot(terminal_xy[0] - anchor_xy[0], terminal_xy[1] - anchor_xy[1]) > WAYPOINT_COLOCATION_M:
            return 0
        if not self._arrival_pose_is_stable(waypoints[0], 0):
            LOGGER.info(
                "loop round %d anchor confirmation failed; dispatching waypoint 0 normally",
                self.context.round_number,
            )
            return 0

        self.context.current_waypoint_index = 1
        self.context.state_version += 1
        # Keep transport-side monotonic indices coherent, without producing a
        # business arrival event or invoking post-arrival side effects.
        self._last_target_index = 0
        self._last_reached_index = 0
        self._persist()
        self._emit(
            "task.loop_anchor_confirmed",
            extra={
                "round_number": self.context.round_number,
                "skipped_waypoint_index": 0,
                "skipped_waypoint_id": waypoints[0].get("waypoint_id"),
                "next_waypoint_index": 1,
            },
        )
        LOGGER.info(
            "loop round %d confirmed repeated anchor waypoint 0; dispatching waypoint 1",
            self.context.round_number,
        )
        return 1

    def _send_from(self, index: int) -> None:
        # Face the travel direction before every cruise leg. Restart, obstacle
        # redispatch, and localization recovery all enter here and previously
        # skipped the post-arrival departure turn.
        if (
            self.context
            and getattr(self.context, "arrival_reapproach_waypoint_index", None)
            == index
            and int(getattr(self.context, "arrival_reapproach_attempts", 0) or 0) > 0
        ):
            self._arrival_reapproach_index = index
            self._arrival_retry_counts[index] = int(
                getattr(self.context, "arrival_reapproach_attempts", 0) or 0
            )
            self._dispatch_navigation(index, reapproach=True)
            return
        index = self._advance_past_colocated_waypoints(index)
        if self._maybe_face_travel_direction(index):
            return
        self._dispatch_navigation(index)

    def _heading_error_rad(self, desired_yaw: float, current_yaw: float) -> float:
        return atan2(sin(desired_yaw - current_yaw), cos(desired_yaw - current_yaw))

    def _pre_leg_heading_error_requires_spin(self, error_rad: float | None) -> bool:
        """Whether a cruise leg should stop and teleop-spin before Nav2.

        Indoor and outdoor use the same 10° absorb. Larger errors still turn
        in place first so cruise starts already facing the next click.
        """
        if error_rad is None:
            return True
        return abs(float(error_rad)) > DEPARTURE_HEADING_SKIP_RAD

    def _maybe_face_travel_direction(self, target_index: int) -> bool:
        """Rotate in place toward the travel leg when the heading error is large."""
        if not self.context or self._is_docking_task():
            return False
        if self._departure_heading_index is not None or self._departure_cruise_index is not None:
            return False
        waypoints = self.context.route_snapshot.get("waypoints") or []
        if target_index < 0 or target_index >= len(waypoints):
            return False
        pose = self.navigation.latest_pose() if self.navigation else None
        if pose is None:
            return False
        try:
            x = float(pose.x)
            y = float(pose.y)
            yaw = float(getattr(pose, "yaw", 0.0) or 0.0)
            target_x = float(waypoints[target_index]["x"])
            target_y = float(waypoints[target_index]["y"])
        except (AttributeError, KeyError, TypeError, ValueError):
            return False
        dx, dy = target_x - x, target_y - y
        # Already on the click: spinning here is the in-place loop seen when
        # outdoor arrival is rejected and the same waypoint is re-dispatched.
        if hypot(dx, dy) < ARRIVAL_ACCEPT_LIO_M:
            return False
        desired = atan2(dy, dx)
        error = self._heading_error_rad(desired, yaw)
        if not self._pre_leg_heading_error_requires_spin(error):
            return False
        current = waypoints[max(0, target_index - 1)] if target_index > 0 else waypoints[target_index]
        return self._start_departure_heading(
            desired_yaw=desired,
            reached_index=max(0, target_index - 1),
            cruise_index=target_index,
            profile_waypoint=current,
            error_rad=error,
        )

    def _batch_end_index(self, start_index: int) -> int:
        """Last exclusive index of one Nav2 goal.

        Every waypoint is an independent Nav2 goal. This keeps localization
        correction, arrival confirmation, dwell and recovery ownership scoped
        to one leg; NavigateThroughPoses is intentionally not used here.
        """
        waypoints = self.context.route_snapshot["waypoints"]
        total = len(waypoints)
        end = total
        if self._segments:
            end = min(self._segments[self.context.current_segment_index].end_index, total)
        # Do not merge adjacent waypoints. The next leg is generated only after
        # the current waypoint has been confirmed with its fresh corrected pose.
        return min(start_index + 1, end)
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

    def _distance_to_waypoint(self, waypoint: dict | None) -> float | None:
        xy = _waypoint_xy(waypoint or {})
        pose_xy = self._current_pose_xy()
        if xy is None or pose_xy is None:
            return None
        return hypot(pose_xy[0] - xy[0], pose_xy[1] - xy[1])

    def _apply_batch_travel_yaw(self, batch: list[dict], start_index: int) -> None:
        """Point pass-through poses along the Nav2 goal, not the cloud click yaw."""
        route = self.context.route_snapshot["waypoints"]
        for item_index, waypoint in enumerate(batch):
            stop = item_index + 1 == len(batch)
            route_index = start_index + item_index
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
            # Last pose in this Nav2 batch but not the route terminus: face the
            # next leg (outgoing), not the incoming segment from the previous click.
            if stop and route_index + 1 < len(route):
                nxt = route[route_index + 1]
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
                route_index, waypoint, stop=stop
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

    def _dispatch_navigation(self, index: int, *, reapproach: bool = False) -> None:
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
        if reapproach and batch:
            # A fine re-approach is XY-only.  Preserve the corrected robot
            # heading in the Nav2 goal so MPPI/RPP/iLQR do not spend the
            # progress-checker window rotating toward the waypoint yaw.  The
            # requested waypoint yaw is applied later by the stationary
            # arrival-heading stage, after localization and XY acceptance.
            pose = self.navigation.latest_pose()
            try:
                current_yaw = float(getattr(pose, "yaw", float("nan")))
            except (AttributeError, TypeError, ValueError):
                current_yaw = float("nan")
            if isfinite(current_yaw):
                batch[-1]["yaw"] = current_yaw
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
        # Speech/dwell splits create a one-pose batch, but that travel leg is
        # still a cruise. Use the slow DiffDrive profile only for a yaw stop
        # or when the robot is already inside the last metre.
        require_yaw_stop = single and bool(waypoints[last_index].get("require_yaw", False))
        remaining = self._distance_to_waypoint(waypoints[last_index])
        already_close = remaining is not None and remaining <= PATROL_FINAL_APPROACH_M
        pass_through_goal = self._waypoint_is_pass_through(last_index)
        initial_final_approach = (
            (not self._is_docking_task())
            and not pass_through_goal
            and (require_yaw_stop or already_close)
        )
        self._patrol_final_approach_applied = initial_final_approach
        self._bypass_active = False
        self._leg_generation += 1
        self._apply_navigation_profile(
            index,
            force_final=(
                (initial_final_approach or reapproach)
                if not self._is_docking_task() else None
            ),
            force_require_yaw=(False if reapproach else require_yaw_stop),
            reapproach=reapproach,
        )
        self._set_navigation_arrival_tolerance(index)
        # The reached point owns both its departure correction and the
        # following leg. Before any point has been reached, the first target
        # owns the initial leg.
        policy_waypoint = batch[0]
        if self._last_reached_index == index - 1 and index > 0:
            policy_waypoint = waypoints[index - 1]
        self._set_localization_policy(policy_waypoint, "moving")
        self._goal_offset = index
        self._dispatched_count = len(batch)
        accepted = self.navigation.send_waypoints(batch, self.on_feedback, self._bind_nav_result())
        if not accepted:
            self._schedule_nav_dispatch_retry(index)
            return
        self._clear_nav_dispatch_retry()
        self.context.current_waypoint_index = index
        self.context.state = "running"
        self.context.state_version += 1
        # The first leg publishes its policy before `send_waypoints` is known
        # to be accepted. Re-publish after the state transition so the normal
        # cruise-only online anchor gate is enabled for that first leg too.
        self._set_localization_policy(policy_waypoint, "moving")
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
                "leg_generation": self._leg_generation,
                "leg_profile": self._active_leg_profile.__dict__ if self._active_leg_profile else None,
            },
        )
        self.on_feedback(0, milestone="target_dispatched")
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
        anchor_preference = str(
            waypoint.get("localization_anchor_preference")
            or (mode if mode in {"ndt", "rtk"} else "balanced")
        ).strip().lower()
        if anchor_preference not in {"ndt", "rtk", "balanced"}:
            anchor_preference = "balanced"
        rtk_primary_allowed = (
            bool(waypoint.get("rtk_primary_allowed", False))
            and mode == "rtk"
            and phase == "moving"
            and self._outdoor_navigation_profile()
        )
        # The localization node independently enforces LIO freshness, source
        # quality, low-speed motion and one active anchor update.  The Edge
        # owns task-level exclusions: final approach, recovery, safety pause
        # and obstacle handling must never start an online map->lio update.
        online_anchor_correction_allowed = (
            phase == "moving"
            and bool(self.context)
            and self.context.state == "running"
            and not self._patrol_final_approach_applied
            and not self._bypass_active
            and not self._obstacle_recovery_active
            and not self._paused_for_localization
        )
        policy = (
            mode,
            phase,
            anchor_preference,
            rtk_primary_allowed,
            online_anchor_correction_allowed,
        )
        if policy == self._last_localization_policy:
            return
        try:
            setter(
                mode,
                phase,
                anchor_preference,
                rtk_primary_allowed,
                online_anchor_correction_allowed,
            )
        except TypeError:
            # Test/simulation adapters published before policy metadata was
            # introduced retain the two-argument contract.
            setter(mode, phase)
        self._last_localization_policy = policy

    def _start_waypoint_localization_correction(
        self, waypoint: dict, reached_index: int
    ) -> bool:
        requester = getattr(self.navigation, "control_localization_correction", None)
        if not callable(requester):
            # Keep non-ROS test and simulation adapters compatible. Production
            # RosAdapter always exposes the transaction service.
            self._active_correction_transaction_id = None
            self._active_correction_mode = None
            return True
        mode = waypoint_localization_mode(waypoint.get("localization_mode"))
        decision = self._localization_decision()
        existing = self._matching_waypoint_correction(
            decision,
            waypoint,
            reached_index,
            statuses={"waiting_source", "smoothing"},
        )
        if existing is not None:
            self._adopt_waypoint_correction(existing, mode)
            LOGGER.info(
                "reusing live waypoint correction transaction id=%s mode=%s status=%s",
                self._active_correction_transaction_id,
                mode,
                existing.get("status"),
            )
            return True
        transaction_id = (
            f"{self.context.task_execution_id}:waypoint:{reached_index}:"
            f"correction:{self._correction_generation}"
        )
        self._active_correction_transaction_id = transaction_id
        self._active_correction_mode = mode
        result = requester(transaction_id, mode, "start") or {}
        accepted = bool(result.get("accepted"))
        if not accepted:
            if str(result.get("status") or "") == "busy":
                existing = self._matching_waypoint_correction(
                    self._localization_decision(),
                    waypoint,
                    reached_index,
                    statuses={"waiting_source", "smoothing"},
                )
                if existing is not None:
                    self._adopt_waypoint_correction(existing, mode)
                    LOGGER.warning(
                        "waypoint correction request raced with live transaction; "
                        "adopted id=%s status=%s",
                        self._active_correction_transaction_id,
                        existing.get("status"),
                    )
                    return True
            LOGGER.warning(
                "waypoint correction transaction not accepted yet id=%s mode=%s status=%s",
                transaction_id,
                mode,
                result.get("status"),
            )
        return accepted

    def _matching_waypoint_correction(
        self,
        decision: dict,
        waypoint: dict,
        reached_index: int,
        *,
        statuses: set[str],
    ) -> dict | None:
        """Return a live correction owned by this execution and waypoint."""
        if not self.context:
            return None
        transaction = decision.get("one_shot_correction")
        if not isinstance(transaction, dict):
            return None
        transaction_id = str(transaction.get("transaction_id") or "")
        expected_prefix = (
            f"{self.context.task_execution_id}:waypoint:{reached_index}:correction:"
        )
        if not transaction_id.startswith(expected_prefix):
            return None
        if str(transaction.get("status") or "") not in statuses:
            return None
        requested_mode = waypoint_localization_mode(waypoint.get("localization_mode"))
        reported_mode = str(transaction.get("mode") or "").strip().lower()
        if reported_mode and waypoint_localization_mode(reported_mode) != requested_mode:
            return None
        return transaction

    def _adopt_waypoint_correction(self, transaction: dict, mode: str) -> None:
        transaction_id = str(transaction.get("transaction_id") or "")
        self._active_correction_transaction_id = transaction_id
        self._active_correction_mode = mode
        try:
            generation = int(transaction_id.rsplit(":correction:", 1)[1])
        except (IndexError, TypeError, ValueError):
            return
        self._correction_generation = max(self._correction_generation, generation)

    def _retry_waypoint_localization_correction(self, decision: dict) -> None:
        transaction_id = self._active_correction_transaction_id
        mode = self._active_correction_mode
        if not transaction_id or not mode:
            return
        if self.context:
            route_snapshot = getattr(self.context, "route_snapshot", {}) or {}
            points = route_snapshot.get("waypoints") or []
            reached_index = int(getattr(self.context, "current_waypoint_index", 0))
            if 0 <= reached_index < len(points):
                existing = self._matching_waypoint_correction(
                    decision,
                    points[reached_index],
                    reached_index,
                    statuses={"waiting_source", "smoothing"},
                )
                if existing is not None and str(
                    existing.get("transaction_id") or ""
                ) != transaction_id:
                    self._adopt_waypoint_correction(existing, mode)
                    LOGGER.warning(
                        "reconciled waypoint correction ownership to id=%s status=%s",
                        self._active_correction_transaction_id,
                        existing.get("status"),
                    )
                    return
        transaction = decision.get("one_shot_correction")
        if isinstance(transaction, dict) and str(transaction.get("transaction_id") or "") == transaction_id:
            if str(transaction.get("status") or "") in {
                "waiting_source",
                "smoothing",
                "completed",
            }:
                return
        requester = getattr(self.navigation, "control_localization_correction", None)
        if callable(requester):
            result = requester(transaction_id, mode, "start") or {}
            if str(result.get("status") or "") == "busy" and self.context:
                route_snapshot = getattr(self.context, "route_snapshot", {}) or {}
                points = route_snapshot.get("waypoints") or []
                reached_index = int(getattr(self.context, "current_waypoint_index", 0))
                if 0 <= reached_index < len(points):
                    existing = self._matching_waypoint_correction(
                        self._localization_decision(),
                        points[reached_index],
                        reached_index,
                        statuses={"waiting_source", "smoothing"},
                    )
                    if existing is not None:
                        self._adopt_waypoint_correction(existing, mode)

    def _absolute_localization_wait_message(self, decision: dict) -> str:
        if self._active_correction_mode != "ndt":
            return "FAST-LIO 已到达航点，正在等待所配置的绝对定位校正源"
        transaction = decision.get("one_shot_correction")
        transaction = transaction if isinstance(transaction, dict) else {}
        reason = str(transaction.get("reason") or "")
        if reason == "waiting_for_fresh_ndt_measurement":
            return "FAST-LIO 已到达航点，正在等待静止 NDT 补采样"
        gates = decision.get("correction_gates")
        gates = gates if isinstance(gates, dict) else {}
        if str(gates.get("ndt_score_band") or "") == "unavailable":
            return "FAST-LIO 已到达航点，尚未获得本次静止 NDT 采样，正在等待匹配"
        score = decision.get("ndt_score")
        inlier = decision.get("ndt_inlier_fraction")
        details = []
        try:
            details.append(f"匹配分数 {float(score):.3f}/{self.arrival_ndt_max_fitness_score:.3f}")
        except (TypeError, ValueError):
            pass
        try:
            details.append(f"内点率 {float(inlier) * 100:.1f}%")
        except (TypeError, ValueError):
            pass
        suffix = f"（{'，'.join(details)}）" if details else ""
        return f"FAST-LIO 已到达航点，NDT 暂无合格匹配{suffix}，正在静止重定位"

    def _no_correction_stop_recheck_timeout(self) -> float:
        """Give each stop check enough time for continuous-zero confirmation.

        RosAdapter begins its one-second zero-motion timer per invocation, so
        the former 0.5-second probe could never pass for a stationary robot.
        """
        return max(
            NO_CORRECTION_STOP_RECHECK_MIN_SECONDS,
            self.stop_confirmation_seconds + 0.5,
        )

    def _waypoint_correction_completed(self, decision: dict) -> bool | None:
        transaction_id = self._active_correction_transaction_id
        if not transaction_id:
            return None
        transaction = decision.get("one_shot_correction")
        if not isinstance(transaction, dict):
            return False
        return (
            str(transaction.get("transaction_id") or "") == transaction_id
            and str(transaction.get("status") or "") == "completed"
        )

    def _waypoint_no_correction_continue_reason(self, decision: dict) -> str | None:
        """Return a matching explicit no-correction completion reason, if any."""
        transaction_id = self._active_correction_transaction_id
        transaction = decision.get("one_shot_correction")
        if not transaction_id or not isinstance(transaction, dict):
            return None
        if (
            str(transaction.get("transaction_id") or "") != transaction_id
            or str(transaction.get("status") or "") != "completed"
        ):
            return None
        reason = str(transaction.get("reason") or "")
        return reason if reason in NO_CORRECTION_CONTINUE_REASONS else None

    def _cancel_waypoint_localization_correction(self) -> None:
        transaction_id = self._active_correction_transaction_id
        mode = self._active_correction_mode
        self._active_correction_transaction_id = None
        self._active_correction_mode = None
        requester = getattr(self.navigation, "control_localization_correction", None)
        if transaction_id and mode and callable(requester):
            try:
                requester(transaction_id, mode, "cancel")
            except Exception:
                LOGGER.warning(
                    "failed to cancel localization correction transaction %s",
                    transaction_id,
                    exc_info=True,
                )

    def _needs_stationary_correction(self, decision: dict) -> bool:
        """True when a stopped robot still has correctable absolute drift."""
        if bool(decision.get("correction_smoothing_active")):
            return True
        if bool(decision.get("lio_motion_anomaly")):
            return False
        rtk_can_correct = (
            decision.get("rtk_position_good_for_navigation") is True
            or str(decision.get("rtk_quality") or "").lower() == "fixed"
        )
        rtk_drift = decision.get("rtk_drift") if isinstance(decision.get("rtk_drift"), dict) else {}
        rtk_xy = self._lio_rtk_drift_m(decision)
        try:
            rtk_thr = float(rtk_drift.get("threshold_xy_m") or WAYPOINT_CORRECTION_DRIFT_M)
        except (TypeError, ValueError):
            rtk_thr = WAYPOINT_CORRECTION_DRIFT_M
        if rtk_can_correct and rtk_xy is not None and rtk_xy > rtk_thr:
            return True
        rtk_decision = str(decision.get("rtk_drift_decision") or rtk_drift.get("decision") or "")
        # Only wait on true pending gates. Accept/corrected names mean the
        # localization node already chose a correction; either smoothing is
        # active (handled above) or residual XY is already within the gate.
        rtk_pending = rtk_decision in {
            "stable_pending",
            "rtk_stable_pending",
            "awaiting_rtk_self_stable",
        }
        if (
            rtk_can_correct
            and rtk_pending
            and (rtk_xy is None or rtk_xy > rtk_thr)
        ):
            return True
        if self._outdoor_navigation_profile():
            return False
        ndt_drift = decision.get("ndt_drift") if isinstance(decision.get("ndt_drift"), dict) else {}
        try:
            ndt_xy = float(ndt_drift.get("xy_m")) if ndt_drift.get("xy_m") is not None else None
        except (TypeError, ValueError):
            ndt_xy = None
        try:
            ndt_thr = float(ndt_drift.get("threshold_xy_m") or WAYPOINT_CORRECTION_DRIFT_M)
        except (TypeError, ValueError):
            ndt_thr = WAYPOINT_CORRECTION_DRIFT_M
        if ndt_xy is not None and ndt_xy > ndt_thr and bool(decision.get("ndt_healthy")):
            return True
        ndt_decision = str(decision.get("ndt_drift_decision") or ndt_drift.get("decision") or "")
        if ndt_decision in {"stable_pending"} and (
            ndt_xy is None or ndt_xy > ndt_thr
        ):
            return True
        return False

    def _outdoor_settle_can_continue(self, decision: dict) -> bool:
        """Outdoor arrival may proceed without pausing for absolute correction.

        Field issue: settle timeout previously paused with
        ABSOLUTE_LOCALIZATION_REQUIRED even when fixed RTK + LIO were already
        usable. That pause does not self-heal, so the dog stands forever.

        Active correction_smoothing must still block departure: leaving mid
        pull-in aborts the RTK XY fix and the next leg plans from a drifted
        LIO frame (field: reverse 6→5 "arrived", left while smoothing, then
        hit the wall toward point 4).
        """
        if bool(decision.get("lio_motion_anomaly")):
            return False
        if bool(decision.get("correction_smoothing_active")):
            return False
        source = str(decision.get("active_source") or "")
        if source == "rtk_imu" and bool(decision.get("absolute_stable")):
            return True
        if source != "lio_imu":
            return False
        rtk_ok = (
            decision.get("rtk_position_good_for_navigation") is True
            or str(decision.get("rtk_quality") or "").lower() == "fixed"
            or self._rtk_position_good_for_navigation()
        )
        if rtk_ok:
            return True
        # No usable RTK: only continue when nothing is still asking for a pull-in.
        return not self._needs_stationary_correction(decision)

    def _outdoor_absolute_pause_can_resume(self, decision: dict) -> bool:
        """Resume an outdoor ABSOLUTE_LOCALIZATION_REQUIRED pause.

        Settle may continue on FAST-LIO without RTK, but auto-resume must not:
        that combination re-approaches the same click, teleop-spins ~180deg,
        and Nav2 immediately reports arrived again.
        """
        if bool(decision.get("lio_motion_anomaly")):
            return False
        if bool(decision.get("correction_smoothing_active")):
            return False
        return (
            decision.get("rtk_position_good_for_navigation") is True
            or str(decision.get("rtk_quality") or "").lower() == "fixed"
        )

    def _outdoor_settle_timeout_seconds(self, decision: dict) -> float:
        """Use the full settle budget when a correction is in flight."""
        if bool(decision.get("correction_smoothing_active")) or self._needs_stationary_correction(
            decision
        ):
            return WAYPOINT_SETTLE_TIMEOUT_SECONDS
        return WAYPOINT_SETTLE_OUTDOOR_TIMEOUT_SECONDS

    def _rtk_xy_from_decision(self, decision: dict) -> tuple[float, float] | None:
        try:
            rtk_x = float(decision["rtk_x"])
            rtk_y = float(decision["rtk_y"])
        except (KeyError, TypeError, ValueError):
            return None
        if not isfinite(rtk_x) or not isfinite(rtk_y):
            return None
        return rtk_x, rtk_y

    def _lio_rtk_drift_m(self, decision: dict | None = None) -> float | None:
        """LIO map pose vs fixed-RTK XY. Prefer published rtk_drift, else compute."""
        decision = decision if isinstance(decision, dict) else self._localization_decision()
        rtk_drift = decision.get("rtk_drift") if isinstance(decision.get("rtk_drift"), dict) else {}
        try:
            if rtk_drift.get("xy_m") is not None:
                value = float(rtk_drift["xy_m"])
                if isfinite(value):
                    return value
        except (TypeError, ValueError):
            pass
        rtk_xy = self._rtk_xy_from_decision(decision)
        pose_xy = self._current_pose_xy()
        if rtk_xy is None or pose_xy is None:
            return None
        return hypot(pose_xy[0] - rtk_xy[0], pose_xy[1] - rtk_xy[1])

    def _arrival_within_tolerance(self, waypoint: dict, reached_index: int) -> bool:
        """Reject Nav2 'reached' when the dog is not actually at the click.

        Nav2 goal-checks against LIO. With XY drift, LIO can report arrived
        while RTK (and the real world) are still metres from the map click.
        After stationary correction finishes, re-check both frames.

        Indoor stop policies also require a post-correction click check; only
        docking contact legs keep their dedicated final-pose gate.
        """
        policy = self._arrival_policy(waypoint, reached_index)
        if policy == "pass_through":
            return True
        if self._is_docking_task() and policy != "dock":
            return True
        lio_dist = self._distance_to_waypoint(waypoint)
        if not self._outdoor_navigation_profile():
            limit, _ = self._arrival_pose_tolerances(waypoint, reached_index)
            if lio_dist is None:
                LOGGER.warning(
                    "waypoint %d arrival rejected: indoor pose unavailable for click check",
                    reached_index,
                )
                return False
            if lio_dist > limit:
                LOGGER.warning(
                    "waypoint %d arrival rejected: indoor LIO is %.2fm from click (limit %.2fm)",
                    reached_index,
                    lio_dist,
                    limit,
                )
                return False
            return True
        xy_limit, _ = self._arrival_pose_tolerances(waypoint, reached_index)
        if lio_dist is None or lio_dist > xy_limit:
            LOGGER.warning(
                "waypoint %d arrival rejected: LIO is %s from click (limit %.2fm)",
                reached_index,
                f"{lio_dist:.2f}m" if lio_dist is not None else "unavailable",
                xy_limit,
            )
            return False

        # A stopped NDT/UKF waypoint has already requested its own absolute
        # correction transaction before this final gate is reached. Its
        # selected source may be NDT, NDT+float-RTK fusion, or the explicit
        # no-correction continuation on fresh stopped LIO/UKF. Requiring an
        # additional raw fixed-RTK-to-click comparison here overrides that
        # selection and makes source disagreement look like a physical XY
        # error, repeatedly sending Nav2 back to the same click.
        #
        # RTK policy is intentionally different: fixed RTK is its configured
        # absolute source, so retain the raw RTK click proof below. The
        # surrounding arrival transaction blocks progression until a selected
        # source has completed (or explicitly chose no-correction-continue).
        mode = (
            waypoint_localization_mode(waypoint.get("localization_mode"))
            if waypoint.get("localization_mode") is not None
            else "rtk"
        )
        if mode != "rtk":
            return True

        decision = self._localization_decision()
        rtk_ok = (
            decision.get("rtk_position_good_for_navigation") is True
            or str(decision.get("rtk_quality") or "").lower() == "fixed"
            or self._rtk_position_good_for_navigation()
        )
        if not rtk_ok:
            LOGGER.warning(
                "waypoint %d arrival rejected: outdoor RTK not fixed/usable; "
                "LIO-only arrival is not trusted",
                reached_index,
            )
            return False
        rtk_xy = self._rtk_xy_from_decision(decision)
        click = _waypoint_xy(waypoint)
        if rtk_xy is None or click is None:
            LOGGER.warning(
                "waypoint %d arrival rejected: outdoor RTK XY unavailable for click check",
                reached_index,
            )
            return False
        rtk_dist = hypot(rtk_xy[0] - click[0], rtk_xy[1] - click[1])
        rtk_limit = self._outdoor_patrol_rtk_click_limit_m(waypoint, reached_index, xy_limit)
        if rtk_dist > rtk_limit:
            LOGGER.warning(
                "waypoint %d arrival rejected: RTK is %.2fm from click (limit %.2fm); "
                "LIO-only arrival is not trusted",
                reached_index,
                rtk_dist,
                rtk_limit,
            )
            return False
        return True

    def _arrival_convergence_failure_message(
        self, waypoint: dict, reached_index: int
    ) -> str:
        """Describe only the acceptance dimensions that this waypoint uses."""
        _, yaw_tolerance = self._arrival_pose_tolerances(waypoint, reached_index)
        if yaw_tolerance is None:
            return "航点位置未满足验收条件"
        return "航点位置与航向无法同时满足验收条件"

    @staticmethod
    def _arrival_sample_marker(pose, decision: dict) -> str | None:
        marker = getattr(pose, "sampled_at", None) if pose is not None else None
        if marker is None and isinstance(decision.get("localization"), dict):
            marker = decision["localization"].get("sampled_at")
        if marker is None:
            marker = decision.get("sampled_at")
        return str(marker) if marker is not None else None

    def _wait_for_arrival_pose_update(
        self, after_sampled_at: str | None, timeout_seconds: float
    ):
        waiter = getattr(self.navigation, "wait_for_pose_update", None)
        if callable(waiter):
            try:
                return waiter(
                    after_sampled_at=after_sampled_at,
                    timeout_seconds=timeout_seconds,
                )
            except TypeError:
                # Older adapters can be upgraded independently from Edge.
                return waiter(after_sampled_at, timeout_seconds)
            except Exception:
                LOGGER.warning("waiting for a fresh localization pose failed", exc_info=True)
        time.sleep(min(ARRIVAL_CONFIRMATION_INTERVAL_SECONDS, max(0.0, timeout_seconds)))
        return self.navigation.latest_pose() if self.navigation else None

    def _arrival_stability_result(
        self, waypoint: dict, reached_index: int, *, xy_only: bool
    ) -> ArrivalStabilityResult:
        """Observe consecutive *new* final poses after stationary correction.

        A missing or stale localization stream is not evidence that the robot
        is off-click. Only fresh frames that all prove XY is out of tolerance
        may result in physical re-approach; every other failure stays parked.
        """
        policy = self._arrival_policy(waypoint, reached_index)
        required = (
            PRECISION_CONFIRMATION_FRAMES
            if policy in {"precision", "dock"}
            else self.arrival_convergence_samples
        )
        wait_for_update = callable(getattr(self.navigation, "wait_for_pose_update", None))
        timeout = (
            max(2.0, required * ARRIVAL_CONFIRMATION_SAMPLE_WAIT_SECONDS)
            if wait_for_update
            else max(0.5, required * ARRIVAL_CONFIRMATION_INTERVAL_SECONDS * 4)
        )
        deadline = time.monotonic() + timeout
        initial_pose = self.navigation.latest_pose() if self.navigation else None
        initial_decision = self._localization_decision()
        last_sample_marker = self._arrival_sample_marker(initial_pose, initial_decision)
        stable = 0
        fresh_frames = 0
        within_tolerance_frames = 0
        observed_new_frame = False

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            pose = self._wait_for_arrival_pose_update(
                last_sample_marker,
                min(ARRIVAL_CONFIRMATION_SAMPLE_WAIT_SECONDS, remaining),
            )
            decision = self._localization_decision()
            sample_marker = self._arrival_sample_marker(pose, decision)
            if pose is None or sample_marker is None or sample_marker == last_sample_marker:
                continue
            last_sample_marker = sample_marker
            observed_new_frame = True
            fresh = self._localization_sample_fresh(decision)
            valid = (
                self._arrival_xy_within_policy_tolerance(waypoint, reached_index)
                if xy_only
                else self._arrival_pose_within_combined_tolerance(waypoint, reached_index)
            )
            if fresh:
                fresh_frames += 1
            if fresh and valid:
                within_tolerance_frames += 1
                stable += 1
                if stable >= required:
                    return ArrivalStabilityResult(
                        stable=True,
                        reason="stable",
                        consecutive_frames=stable,
                        fresh_frames=fresh_frames,
                        within_tolerance_frames=within_tolerance_frames,
                    )
            else:
                stable = 0

        if not observed_new_frame:
            reason = "no_fresh_samples"
        elif fresh_frames == 0:
            reason = "stale_samples"
        elif within_tolerance_frames == 0:
            reason = "outside_tolerance"
        else:
            reason = "inconsistent_samples"
        LOGGER.warning(
            "waypoint %d %s confirmation was not stable: %d/%d consecutive "
            "(reason=%s fresh=%d within_tolerance=%d)",
            reached_index,
            "XY" if xy_only else "pose",
            stable,
            required,
            reason,
            fresh_frames,
            within_tolerance_frames,
        )
        return ArrivalStabilityResult(
            stable=False,
            reason=reason,
            consecutive_frames=stable,
            fresh_frames=fresh_frames,
            within_tolerance_frames=within_tolerance_frames,
        )

    def _arrival_pose_is_stable(self, waypoint: dict, reached_index: int) -> bool:
        """Require consecutive fresh post-correction samples before release."""
        return self._arrival_stability_result(
            waypoint, reached_index, xy_only=False
        ).stable

    def _arrival_pose_tolerances(
        self, waypoint: dict, reached_index: int
    ) -> tuple[float, float | None]:
        policy = self._arrival_policy(waypoint, reached_index)
        if self._is_docking_task() and policy != "dock":
            # Staging points in a docking task are owned by the dedicated
            # docking sequence; only its final dock point gets the tight gate.
            return float("inf"), None
        if policy == "dock" and self._is_docking_task():
            yaw_tolerance = (
                self.docking_goal_yaw_tolerance_rad
                if bool(waypoint.get("require_yaw", False))
                else None
            )
            return self.docking_goal_tolerance_m, yaw_tolerance
        if policy == "precision":
            yaw_tolerance = (
                PRECISION_ARRIVAL_YAW_TOLERANCE_RAD
                if bool(waypoint.get("require_yaw", False))
                else None
            )
            return self.precision_arrival_tolerance_m, yaw_tolerance
        yaw_tolerance = (
            ARRIVAL_HEADING_ALIGN_RAD if bool(waypoint.get("require_yaw", False)) else None
        )
        if self._coarse_arrival_fallback_active(reached_index):
            return self.coarse_goal_tolerance_m, yaw_tolerance
        return self.final_waypoint_tolerance_m, yaw_tolerance

    def _arrival_pose_errors(
        self, waypoint: dict, reached_index: int
    ) -> tuple[float | None, float | None]:
        pose = self.navigation.latest_pose() if self.navigation else None
        click = _waypoint_xy(waypoint)
        if pose is None or click is None:
            return None, None
        try:
            distance = hypot(float(pose.x) - click[0], float(pose.y) - click[1])
        except (AttributeError, TypeError, ValueError):
            return None, None
        _, yaw_tolerance = self._arrival_pose_tolerances(waypoint, reached_index)
        if yaw_tolerance is None:
            return distance, None
        try:
            yaw_error = abs(
                self._heading_error_rad(
                    float(waypoint.get("yaw") or 0.0),
                    float(getattr(pose, "yaw", 0.0) or 0.0),
                )
            )
        except (AttributeError, TypeError, ValueError):
            yaw_error = None
        return distance, yaw_error

    def _arrival_pose_within_combined_tolerance(
        self, waypoint: dict, reached_index: int
    ) -> bool:
        distance, yaw_error = self._arrival_pose_errors(waypoint, reached_index)
        xy_tolerance, yaw_tolerance = self._arrival_pose_tolerances(
            waypoint, reached_index
        )
        if distance is None or distance > xy_tolerance:
            return False
        if yaw_tolerance is not None and (yaw_error is None or yaw_error > yaw_tolerance):
            return False
        return self._arrival_within_tolerance(waypoint, reached_index)

    def _arrival_xy_within_policy_tolerance(
        self, waypoint: dict, reached_index: int
    ) -> bool:
        distance, _ = self._arrival_pose_errors(waypoint, reached_index)
        xy_tolerance, _ = self._arrival_pose_tolerances(waypoint, reached_index)
        return bool(
            distance is not None
            and distance <= xy_tolerance
            and self._arrival_within_tolerance(waypoint, reached_index)
        )

    def _arrival_xy_is_stable(self, waypoint: dict, reached_index: int) -> bool:
        """Confirm corrected XY over consecutive fresh localization samples."""
        result = self._arrival_stability_result(waypoint, reached_index, xy_only=True)
        self._last_arrival_xy_stability_result = result
        return result.stable

    def _post_arrival_active(self, waypoint_index: int | None = None) -> bool:
        if not self.context or not self.context.post_arrival_stage:
            return False
        active_index = self.context.post_arrival_waypoint_index
        return waypoint_index is None or active_index == waypoint_index

    def _coarse_arrival_fallback_active(self, waypoint_index: int) -> bool:
        """Whether this stopping waypoint has exhausted fine re-approaches.

        The relaxed radius is deliberately scoped to the one waypoint that
        consumed its configured Nav2 fine re-approach. It never applies to a
        precision or docking policy, which cannot enter the fallback.
        """
        return bool(
            self.context
            and bool(getattr(self.context, "arrival_coarse_fallback_accepted", False))
            and self.context.post_arrival_waypoint_index == waypoint_index
        )

    def _set_post_arrival_stage(self, waypoint_index: int, stage: str) -> None:
        if not self.context:
            return
        self.context.post_arrival_waypoint_index = waypoint_index
        self.context.post_arrival_stage = stage
        self._persist()

    def _clear_post_arrival_state(self) -> None:
        if not self.context:
            return
        self.context.post_arrival_waypoint_index = None
        self.context.post_arrival_stage = ""
        self.context.arrival_side_effects_started = False
        self.context.arrival_coarse_fallback_accepted = False
        self._reset_arrival_micro_adjustment()

    def _reset_arrival_micro_adjustment(self) -> None:
        if not self.context:
            return
        self.context.arrival_micro_adjust_total_m = 0.0
        self.context.arrival_micro_adjust_steps = 0
        self.context.arrival_micro_adjust_started_at = None

    def _arrival_micro_adjust_mode_for(self, waypoint: dict, reached_index: int) -> str:
        policy = self._arrival_policy(waypoint, reached_index)
        requested = str(
            waypoint.get("arrival_micro_adjust_mode") or self.arrival_micro_adjust_mode
        ).strip().lower()
        if requested == "nav2_goal" and policy in {"stop_and_confirm", "precision"}:
            return "nav2_goal"
        return "cmd_vel"

    def _arrival_micro_adjust_adapter_available(
        self, waypoint: dict, reached_index: int
    ) -> bool:
        """Whether the selected post-yaw adjustment mechanism is present.

        This deliberately answers only the adapter-capability question.  A
        rejected state/budget gate must not be presented to operators as an
        unavailable ROS interface.
        """
        if self._arrival_micro_adjust_mode_for(waypoint, reached_index) == "nav2_goal":
            return bool(
                callable(getattr(self.navigation, "set_arrival_micro_goal_profile", None))
                and callable(getattr(self.navigation, "send_waypoints", None))
            )
        return bool(
            callable(getattr(self.navigation, "arrival_adjust_velocity", None))
            and callable(getattr(self.navigation, "directional_clearance", None))
        )

    def _start_arrival_side_effects(
        self,
        waypoint_index: int,
        waypoint: dict,
        *,
        arrival_details: dict | None = None,
    ) -> None:
        """Publish business arrival and start all arrival-owned side effects once."""
        if not self.context or self.context.arrival_side_effects_started:
            return
        speech_configured = (
            self._waypoint_speech_enabled()
            and bool(waypoint.get("speech_template_id"))
            and self._waypoint_speech_mode(waypoint) != "disabled"
        )
        if speech_configured and self._waypoint_speech_blocks_navigation(waypoint):
            self._clear_waypoint_speech_status(waypoint_index)
            self._start_waypoint_speech_wait(waypoint_index)
        self.on_feedback(
            waypoint_index - self._goal_offset,
            0.0,
            milestone="arrival_confirmed",
            completed_waypoints=waypoint_index + 1,
            arrival_extra=arrival_details,
        )
        self._run_waypoint_actions(waypoint_index, waypoint)
        self._start_waypoint_dwell(waypoint_index)
        self.context.arrival_side_effects_started = True
        self._persist()

    def _restore_arrival_side_effect_gates(
        self, waypoint_index: int, waypoint: dict
    ) -> None:
        """Reattach blocking waits after a paused process was restarted."""
        if not self.context or not self.context.arrival_side_effects_started:
            return
        speech_configured = (
            self._waypoint_speech_enabled()
            and bool(waypoint.get("speech_template_id"))
            and self._waypoint_speech_blocks_navigation(waypoint)
        )
        if (
            speech_configured
            and self._speech_waiting_index != waypoint_index
            and not self._speech_wait_finished
        ):
            # Do not clear the durable playback result on restore.
            self._start_waypoint_speech_wait(waypoint_index)
        if (
            float(waypoint.get("dwell_seconds") or 0.0) > 0.0
            and self._dwell_waiting_index != waypoint_index
        ):
            # Conservatively restart the dwell duration; leaving early after a
            # restart is worse than holding the already-reached waypoint longer.
            self._start_waypoint_dwell(waypoint_index)

    def _resume_post_arrival(self, waypoint_index: int) -> bool:
        """Continue post-arrival yaw/XY work without redispatching Nav2."""
        if not self.context or not self._post_arrival_active(waypoint_index):
            return False
        waypoints = self.context.route_snapshot.get("waypoints") or []
        if waypoint_index < 0 or waypoint_index >= len(waypoints):
            return False
        waypoint = waypoints[waypoint_index]
        stage = self.context.post_arrival_stage
        self._arrival_correction_completed_index = waypoint_index
        if stage in {"heading_aligned", "xy_adjusting", "post_arrival_ready"}:
            self._arrival_heading_completed_index = waypoint_index
        self._goal_offset = waypoint_index
        self._dispatched_count = 1
        self._last_target_index = max(self._last_target_index, waypoint_index)
        if self.context.arrival_side_effects_started:
            self._last_reached_index = max(self._last_reached_index, waypoint_index)
        if stage == "post_arrival_ready":
            if self.context.arrival_side_effects_started:
                self._restore_arrival_side_effect_gates(waypoint_index, waypoint)
            else:
                self._start_arrival_side_effects(waypoint_index, waypoint)
            self._waypoint_localization_ready_index = waypoint_index
            self._maybe_continue_after_waypoint(waypoint_index)
        else:
            self._restore_arrival_side_effect_gates(waypoint_index, waypoint)
            self.on_navigation_result(
                "succeeded", generation=self._nav_goal_generation
            )
        return True

    def _emit_arrival_stage(
        self, reached_index: int, stage: str, message: str
    ) -> None:
        self._emit_idempotent(
            "task.recovery_active",
            event_type_key=f"arrival_{stage}",
            waypoint_id=str(reached_index),
            code="ARRIVAL_POSE_CONVERGENCE",
            message=message,
            extra={"arrival_stage": stage},
        )

    def _cancel_arrival_adjustment(self, *, reset_state: bool = False) -> None:
        self._arrival_adjustment_stop.set()
        if self._arrival_adjustment_nav2_active:
            try:
                self.navigation.cancel_navigation(timeout_seconds=2.0)
            except Exception:
                LOGGER.warning("failed to cancel Nav2 arrival micro goal", exc_info=True)
            profile = getattr(self.navigation, "set_arrival_micro_goal_profile", None)
            if callable(profile):
                try:
                    profile(enabled=False, tolerance_m=self.arrival_micro_goal_tolerance_m)
                except Exception:
                    LOGGER.warning("failed to restore Nav2 arrival micro profile", exc_info=True)
        self._arrival_adjustment_nav2_active = False
        velocity = getattr(self.navigation, "arrival_adjust_velocity", None)
        if callable(velocity):
            try:
                velocity(vx=0.0, vy=0.0, yaw_rate=0.0)
            except Exception:
                LOGGER.warning("failed to stop arrival fine adjustment", exc_info=True)
        self._arrival_adjustment_index = None
        self._arrival_adjustment_thread = None
        if reset_state:
            self._arrival_correction_completed_index = None
            self._arrival_heading_completed_index = None
            self._reset_arrival_micro_adjustment()

    def _start_arrival_adjustment(self, waypoint: dict, reached_index: int) -> bool:
        # This is exclusively the post-final-yaw XY compensation path. A
        # failed initial XY arrival must be corrected and re-approached by
        # Nav2, never pushed by raw velocity.
        if (
            not self.context
            or not self._post_arrival_active(reached_index)
            or self.context.post_arrival_stage
            not in {"heading_aligned", "xy_adjusted", "xy_adjusting", "xy_adjustment_recheck"}
            or self._arrival_heading_completed_index != reached_index
        ):
            LOGGER.warning(
                "refusing XY micro-adjust before waypoint %d has verified XY and final yaw",
                reached_index,
            )
            return False
        policy = self._arrival_policy(waypoint, reached_index)
        if policy == "dock" and self._is_docking_task():
            return False
        distance, _ = self._arrival_pose_errors(waypoint, reached_index)
        if distance is None:
            return False
        xy_tolerance, _ = self._arrival_pose_tolerances(waypoint, reached_index)
        if distance <= xy_tolerance:
            return False
        micro_adjust_max_residual = (
            xy_tolerance + self.arrival_micro_adjust_total_budget_m
        )
        if distance > micro_adjust_max_residual:
            LOGGER.warning(
                "waypoint %d post-yaw XY residual %.3fm exceeds bounded micro-adjust limit %.3fm",
                reached_index,
                distance,
                micro_adjust_max_residual,
            )
            return False
        # Do not reject a valid post-yaw residual solely because it is larger
        # than the legacy initial-error setting.  This controller is bounded
        # by *actual observed travel*, segment count, timeout, fresh
        # localization and directional clearance.  In particular a 0.675 m
        # residual may need only 0.375 m of safe travel to re-enter the
        # ordinary 0.30 m arrival tolerance.
        if distance > self.arrival_micro_adjust_max_initial_error_m:
            LOGGER.info(
                "waypoint %d post-yaw XY residual %.3fm exceeds legacy initial "
                "micro-adjust value %.3fm; proceeding under travel budget %.3fm",
                reached_index,
                distance,
                self.arrival_micro_adjust_max_initial_error_m,
                self.arrival_micro_adjust_total_budget_m,
            )
        if self.context and self.context.arrival_micro_adjust_started_at is not None:
            elapsed = time.time() - float(self.context.arrival_micro_adjust_started_at)
            if elapsed >= self.arrival_adjust_timeout_seconds:
                return False
        mode = self._arrival_micro_adjust_mode_for(waypoint, reached_index)
        if mode == "nav2_goal":
            return self._start_arrival_nav2_goal_adjustment(waypoint, reached_index)
        velocity = getattr(self.navigation, "arrival_adjust_velocity", None)
        clearance = getattr(self.navigation, "directional_clearance", None)
        if not self._arrival_micro_adjust_adapter_available(waypoint, reached_index):
            LOGGER.warning(
                "arrival XY micro-adjust adapter is unavailable for waypoint %d",
                reached_index,
            )
            return False
        if self._arrival_adjustment_thread and self._arrival_adjustment_thread.is_alive():
            return self._arrival_adjustment_index == reached_index
        self._arrival_adjustment_stop = threading.Event()
        self._arrival_adjustment_index = reached_index
        if self.context and self.context.arrival_micro_adjust_started_at is None:
            self.context.arrival_micro_adjust_started_at = time.time()
            self.context.arrival_micro_adjust_total_m = 0.0
            self.context.arrival_micro_adjust_steps = 0
        self._set_post_arrival_stage(reached_index, "xy_adjusting")
        self._emit_arrival_stage(
            reached_index,
            "heading_preserving_adjustment",
            "最终航向后位置轻微偏移，正在保持目标航向微调",
        )
        thread = threading.Thread(
            target=self._arrival_adjustment_worker,
            args=(dict(waypoint), reached_index, self._nav_goal_generation),
            daemon=True,
            name="arrival-heading-preserving-adjustment",
        )
        self._arrival_adjustment_thread = thread
        thread.start()
        return True

    def _start_arrival_nav2_goal_adjustment(self, waypoint: dict, reached_index: int) -> bool:
        profile = getattr(self.navigation, "set_arrival_micro_goal_profile", None)
        if not callable(profile):
            LOGGER.warning("Nav2 micro-goal requested but the navigation adapter does not support it")
            return False
        if self._arrival_adjustment_nav2_active:
            return self._arrival_adjustment_index == reached_index
        if self.context and self.context.arrival_micro_adjust_started_at is not None:
            LOGGER.warning("waypoint %d Nav2 micro-goal already consumed", reached_index)
            return False
        try:
            profile(enabled=True, tolerance_m=self.arrival_micro_goal_tolerance_m)
        except Exception:
            LOGGER.warning("unable to enable Nav2 micro-goal profile", exc_info=True)
            return False
        self._arrival_adjustment_index = reached_index
        self._arrival_adjustment_nav2_active = True
        if self.context and self.context.arrival_micro_adjust_started_at is None:
            self.context.arrival_micro_adjust_started_at = time.time()
            self.context.arrival_micro_adjust_total_m = 0.0
            self.context.arrival_micro_adjust_steps = 1
        self._set_post_arrival_stage(reached_index, "xy_adjusting")
        self._emit_arrival_stage(
            reached_index,
            "heading_preserving_nav2_goal",
            "最终航向后位置轻微偏移，正在使用单个 Nav2 精确目标微调",
        )

        def _result(status: str, message: str = "", _detail: dict | None = None) -> None:
            with self._lock:
                if not self._arrival_adjustment_nav2_active or self._arrival_adjustment_index != reached_index:
                    return
                self._arrival_adjustment_nav2_active = False
                self._arrival_adjustment_index = None
                try:
                    profile(enabled=False, tolerance_m=self.arrival_micro_goal_tolerance_m)
                except Exception:
                    LOGGER.warning("failed to restore Nav2 micro-goal profile", exc_info=True)
                if status != "succeeded":
                    self._emit_safe_hold(
                        "ARRIVAL_POSE_CONVERGENCE_FAILED",
                        f"Nav2 精确微目标未完成：{message or status}",
                    )
                    return
                self._arrival_correction_completed_index = None
                self._set_post_arrival_stage(reached_index, "xy_adjustment_recheck")
                self.on_navigation_result("succeeded", generation=self._nav_goal_generation)

        if not self.navigation.send_waypoints([dict(waypoint)], lambda *_: None, _result):
            self._arrival_adjustment_nav2_active = False
            self._arrival_adjustment_index = None
            try:
                profile(enabled=False, tolerance_m=self.arrival_micro_goal_tolerance_m)
            except Exception:
                pass
            return False
        return True

    def _arrival_adjustment_worker(
        self, waypoint: dict, reached_index: int, generation: int
    ) -> None:
        velocity = getattr(self.navigation, "arrival_adjust_velocity", None)
        clearance = getattr(self.navigation, "directional_clearance", None)
        stop_event = self._arrival_adjustment_stop
        stable = 0
        failure_message = ""
        succeeded = False
        segment_complete = False
        cancelled_by_state = False
        transient_reason = ""
        transient_started_at: float | None = None
        last_xy: tuple[float, float] | None = None
        segment_distance_m = 0.0

        def _transient_expired(reason: str) -> bool:
            nonlocal transient_reason, transient_started_at
            now = time.monotonic()
            transient_reason = reason
            if transient_started_at is None:
                transient_started_at = now
            return now - transient_started_at >= self.arrival_adjust_safety_grace_seconds

        try:
            while not stop_event.is_set():
                wait_for_safety = False
                with self._lock:
                    if (
                        not self.context
                        or self.context.state != "running"
                        or self.context.current_waypoint_index != reached_index
                        or self._arrival_adjustment_index != reached_index
                    ):
                        cancelled_by_state = True
                        break
                    started_at = self.context.arrival_micro_adjust_started_at
                    if started_at is not None and (
                        time.time() - float(started_at) >= self.arrival_adjust_timeout_seconds
                    ):
                        failure_message = (
                            "到点微调超过"
                            f"{self.arrival_adjust_timeout_seconds:.1f}秒时间预算"
                        )
                        break
                    if (
                        self.context.arrival_micro_adjust_total_m
                        >= self.arrival_micro_adjust_total_budget_m
                    ):
                        failure_message = (
                            "到点微调已达到"
                            f"{self.arrival_micro_adjust_total_budget_m:.2f}米总位移预算"
                        )
                        break
                    if self.context.arrival_micro_adjust_steps >= self.arrival_micro_adjust_max_steps:
                        failure_message = (
                            "到点微调已达到"
                            f"{self.arrival_micro_adjust_max_steps}段步数预算"
                        )
                        break
                    decision = self._localization_decision()
                    localization_reason = ""
                    if not self._localization_sample_fresh(decision):
                        localization_reason = "localization_stale"
                    elif bool(decision.get("lio_motion_anomaly")):
                        localization_reason = "lio_motion_anomaly"
                    elif bool(decision.get("correction_smoothing_active")):
                        localization_reason = "correction_smoothing"
                    if localization_reason:
                        velocity(vx=0.0, vy=0.0, yaw_rate=0.0)
                        if _transient_expired(localization_reason):
                            failure_message = (
                                "到点微调等待定位恢复超过"
                                f"{self.arrival_adjust_safety_grace_seconds:.1f}秒"
                                f"（{localization_reason}）"
                            )
                            break
                        wait_for_safety = True
                    if wait_for_safety:
                        pass
                    else:
                        pose = self.navigation.latest_pose()
                        if pose is None:
                            velocity(vx=0.0, vy=0.0, yaw_rate=0.0)
                            if _transient_expired("pose_unavailable"):
                                failure_message = (
                                    "到点微调等待机器人位姿超过"
                                    f"{self.arrival_adjust_safety_grace_seconds:.1f}秒"
                                )
                                break
                            wait_for_safety = True
                    if wait_for_safety:
                        pass
                    else:
                        try:
                            dx = float(waypoint["x"]) - float(pose.x)
                            dy = float(waypoint["y"]) - float(pose.y)
                            yaw = float(getattr(pose, "yaw", 0.0) or 0.0)
                            target_yaw = float(waypoint.get("yaw") or 0.0)
                        except (AttributeError, KeyError, TypeError, ValueError):
                            velocity(vx=0.0, vy=0.0, yaw_rate=0.0)
                            if _transient_expired("pose_invalid"):
                                failure_message = (
                                    "到点微调等待有效位姿超过"
                                    f"{self.arrival_adjust_safety_grace_seconds:.1f}秒"
                                )
                                break
                            wait_for_safety = True
                    if wait_for_safety:
                        pass
                    else:
                        distance = hypot(dx, dy)
                        current_xy = (float(pose.x), float(pose.y))
                        if last_xy is not None:
                            observed_delta = hypot(
                                current_xy[0] - last_xy[0], current_xy[1] - last_xy[1]
                            )
                            # Reject a localization jump as travel accounting;
                            # correction smoothing is already blocked above.
                            if observed_delta <= self.arrival_micro_adjust_step_m * 2.0:
                                self.context.arrival_micro_adjust_total_m += observed_delta
                                segment_distance_m += observed_delta
                        last_xy = current_xy
                        if segment_distance_m >= self.arrival_micro_adjust_step_m:
                            self.context.arrival_micro_adjust_steps += 1
                            segment_distance_m = 0.0
                            self._persist()
                            if self.context.arrival_micro_adjust_steps >= self.arrival_micro_adjust_max_steps:
                                failure_message = (
                                    "到点微调已达到"
                                    f"{self.arrival_micro_adjust_max_steps}段步数预算"
                                )
                                break
                            segment_complete = True
                            break
                        yaw_error = self._heading_error_rad(target_yaw, yaw)
                        xy_tolerance, yaw_tolerance = self._arrival_pose_tolerances(
                            waypoint, reached_index
                        )
                        # Drive a little inside the business acceptance radius.
                        # Stopping exactly on a floating-point boundary often
                        # makes the subsequent three-sample confirmation fail
                        # after the velocity controller has already released.
                        xy_control_tolerance = max(0.01, xy_tolerance - 0.02)
                        yaw_ok = yaw_tolerance is None or abs(yaw_error) <= yaw_tolerance
                        if distance <= xy_control_tolerance and yaw_ok:
                            stable += 1
                            if stable >= ARRIVAL_ADJUST_STABLE_SAMPLES:
                                succeeded = True
                                break
                        else:
                            stable = 0
                        body_x = cos(yaw) * dx + sin(yaw) * dy
                        body_y = -sin(yaw) * dx + cos(yaw) * dy
                        if distance > xy_control_tolerance:
                            speed = min(
                                self.arrival_adjust_speed_mps,
                                max(0.03, distance * 0.35),
                            )
                            vx = speed * body_x / max(distance, 1e-6)
                            vy = speed * body_y / max(distance, 1e-6)
                        else:
                            vx = vy = 0.0
                        yaw_rate = max(
                            -self.arrival_adjust_yaw_rate_rps,
                            min(self.arrival_adjust_yaw_rate_rps, yaw_error * 0.8),
                        )
                        # Keep the final yaw while translating only as far as
                        # needed to enter the XY acceptance radius.  Checking
                        # up to the click centre falsely treats a rear object
                        # beyond the actual bounded micro-move as a collision.
                        # With no translation, directional_clearance() returns
                        # rotation_only, so an obstacle behind the final yaw
                        # cannot pause a pure yaw correction.
                        required_translation_m = max(
                            0.0, distance - xy_control_tolerance
                        )
                        observation = clearance(
                            vx,
                            vy,
                            min(
                                self.arrival_adjust_clearance_lookahead_m,
                                required_translation_m
                                + ARRIVAL_ADJUST_STOPPING_MARGIN_M,
                            ),
                            max_scan_age_seconds=self.arrival_adjust_scan_max_age_seconds,
                        ) or {}
                        if not bool(observation.get("clear")):
                            reason = str(observation.get("reason") or "unknown")
                            velocity(vx=0.0, vy=0.0, yaw_rate=0.0)
                            if reason == "scan_stale":
                                if _transient_expired(reason):
                                    failure_message = (
                                        "到点微调等待新鲜扫描超过"
                                        f"{self.arrival_adjust_safety_grace_seconds:.1f}秒"
                                    )
                                    break
                                wait_for_safety = True
                            else:
                                failure_message = f"到点微调方向不安全（{reason}）"
                                break
                        else:
                            transient_reason = ""
                            transient_started_at = None
                            velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)
                if stop_event.wait(ARRIVAL_ADJUST_PERIOD_SECONDS):
                    break
        except Exception:
            LOGGER.exception("arrival heading-preserving adjustment failed")
            failure_message = "到点微调执行异常"
        finally:
            try:
                if callable(velocity):
                    velocity(vx=0.0, vy=0.0, yaw_rate=0.0)
            except Exception:
                LOGGER.warning("failed to zero arrival adjustment velocity", exc_info=True)
        with self._lock:
            if (
                self._arrival_adjustment_stop is not stop_event
                or self._arrival_adjustment_index != reached_index
            ):
                return
            self._arrival_adjustment_index = None
            self._arrival_adjustment_thread = None
            if cancelled_by_state or stop_event.is_set():
                return
            if succeeded:
                self._set_post_arrival_stage(reached_index, "post_arrival_ready")
                self.on_navigation_result("succeeded", generation=generation)
                return
            if segment_complete:
                # Stop between bounded segments and return through the normal
                # arrival transaction. This repeats stationary correction and
                # fresh-sample confirmation before another physical movement.
                self._arrival_correction_completed_index = None
                self._set_post_arrival_stage(reached_index, "xy_adjustment_recheck")
                self.on_navigation_result("succeeded", generation=generation)
                return
            self._emit_safe_hold(
                "ARRIVAL_POSE_CONVERGENCE_FAILED",
                failure_message
                or self._arrival_convergence_failure_message(waypoint, reached_index),
            )

    def _is_last_route_waypoint(self, index: int) -> bool:
        if not self.context:
            return False
        waypoints = self.context.route_snapshot.get("waypoints") or []
        return bool(waypoints) and index >= len(waypoints) - 1

    def _outdoor_arrival_confirmed(self, index: int) -> bool:
        """True when this outdoor waypoint may be marked complete."""
        if not self._outdoor_navigation_profile() or self._is_docking_task():
            return True
        if not self.context:
            return False
        waypoints = self.context.route_snapshot.get("waypoints") or []
        if index < 0 or index >= len(waypoints):
            return False
        return self._arrival_within_tolerance(waypoints[index], index)

    def _hold_unconfirmed_outdoor_arrival(self, reached_index: int, message: str) -> None:
        self.navigation.stop_motion()
        self._restore_navigation_profile()
        self._paused_for_localization = True
        self._paused_localization_reason = "absolute_required"
        self._navigation_prepared = False
        self.context.state = "paused"
        self.context.current_waypoint_index = reached_index
        self.context.state_version += 1
        self._persist()
        self._emit(
            "task.paused",
            code="ABSOLUTE_LOCALIZATION_REQUIRED",
            message=message,
        )
        self._arm_absolute_localization_resume_watch()

    def _cancel_arrival_precision_recovery_retry(self) -> None:
        timer = self._arrival_precision_recovery_timer
        self._arrival_precision_recovery_timer = None
        if timer is not None:
            timer.cancel()

    def _hold_for_arrival_precision_recovery(self, message: str) -> None:
        """Keep a far off-click arrival stopped and re-request localization.

        Localization recovery itself may finish with a still-invalid map pose.
        Retry requests every five seconds while that condition remains, then
        let ``on_localization_recovered`` decide whether to re-approach.
        """
        if not self.context:
            return
        self._paused_for_localization = True
        self._paused_localization_reason = "absolute_required"
        self._emit_safe_hold("ARRIVAL_XY_UNVERIFIED", message)

        def _retry() -> None:
            with self._lock:
                if (
                    not self.context
                    or self.context.state in self.TERMINAL_STATES
                    or not self._paused_for_localization
                    or self._paused_localization_reason != "absolute_required"
                ):
                    self._arrival_precision_recovery_timer = None
                    return
                callback = self.localization_recovery_callback
                if callable(callback):
                    callback("arrival_precision_recovery")
                timer = threading.Timer(
                    self.arrival_precision_recovery_retry_seconds, _retry
                )
                timer.daemon = True
                self._arrival_precision_recovery_timer = timer
                timer.start()

        callback = self.localization_recovery_callback
        if not callable(callback):
            return
        callback("arrival_precision_recovery")
        self._cancel_arrival_precision_recovery_retry()
        timer = threading.Timer(self.arrival_precision_recovery_retry_seconds, _retry)
        timer.daemon = True
        self._arrival_precision_recovery_timer = timer
        timer.start()

    def _outdoor_patrol_rtk_click_limit_m(
        self, waypoint: dict, reached_index: int, xy_limit: float
    ) -> float:
        """Normal outdoor patrol uses the 0.50 m coarse circle for RTK proof.

        A strict RTK-to-click check can reject a dog already inside the original
        coarse circle. Precision, dock, and yaw-stop clicks keep the tighter
        configured limit.
        """
        if self._is_docking_task():
            return xy_limit
        if self._arrival_policy(waypoint, reached_index) != "stop_and_confirm":
            return xy_limit
        if bool(waypoint.get("require_yaw", False)):
            return xy_limit
        return max(xy_limit, self.coarse_goal_tolerance_m)

    def _clear_arrival_reapproach_tracking(self, reached_index: int) -> int:
        """Clear both in-memory and durable fine re-approach state."""
        retries = int(self._arrival_retry_counts.pop(reached_index, 0))
        self._arrival_reapproach_index = None
        if self.context:
            self.context.arrival_reapproach_waypoint_index = None
            self.context.arrival_reapproach_attempts = 0
        return retries

    def _accept_coarse_patrol_arrival(
        self, reached_index: int, waypoint: dict, distance: float, *, retries: int
    ) -> bool:
        """Accept a normal patrol click inside the original 0.50 m Nav2 circle."""
        self._clear_arrival_reapproach_tracking(reached_index)
        self._cancel_arrival_adjustment(reset_state=True)
        self._arrival_convergence_attempts.pop(reached_index, None)
        self._arrival_correction_completed_index = reached_index
        self.context.arrival_coarse_fallback_accepted = True
        self._set_post_arrival_stage(reached_index, "xy_verified")
        self._emit_idempotent(
            "task.arrival_degraded_accepted",
            event_type_key="arrival_coarse_fallback_accepted",
            waypoint_id=str(waypoint.get("waypoint_id") or reached_index),
            code="ARRIVAL_FINE_REAPPROACH_EXHAUSTED",
            message=(
                "普通巡检点一次精细重靠近后仍在 0.50 米粗到达半径内，按降级规则放行"
            ),
            extra={
                "distance_m": round(distance, 3),
                "fine_tolerance_m": self.final_waypoint_tolerance_m,
                "coarse_tolerance_m": self.coarse_goal_tolerance_m,
                "reapproach_attempts": retries,
            },
        )
        self.on_navigation_result(
            "succeeded", generation=self._nav_goal_generation
        )
        return True

    def _reapproach_rejected_arrival(
        self, reached_index: int, *, localization_recovered: bool = False
    ) -> bool:
        """Re-dispatch the original waypoint from the latest corrected pose.

        A residual above 1.50m first requires stationary precision
        localization.  Once that recovery reports a stable absolute pose, the
        same distance is no longer a motion prohibition: Nav2 can safely plan
        from the corrected current pose to the original waypoint.
        """
        retries = int(self._arrival_retry_counts.get(reached_index, 0))
        waypoint = self.context.route_snapshot["waypoints"][reached_index]
        distance, _ = self._arrival_pose_errors(waypoint, reached_index)
        if distance is None:
            LOGGER.warning(
                "waypoint %d corrected arrival error is unavailable; requires precision localization before re-approach",
                reached_index,
            )
            self._hold_for_arrival_precision_recovery(
                "校正后航点残差超过 1.50 米或不可用，先执行精准定位恢复再重接近",
            )
            return True
        if (
            distance > self.arrival_nav2_reapproach_max_error_m
            and not localization_recovered
        ):
            LOGGER.warning(
                "waypoint %d corrected arrival error %.2fm requires precision localization before re-approach",
                reached_index,
                distance,
            )
            self._hold_for_arrival_precision_recovery(
                "校正后航点残差超过 1.50 米，先执行精准定位恢复再重接近",
            )
            return True
        if retries >= self.arrival_reapproach_max_attempts:
            if (
                retries > 0
                and self._arrival_policy(waypoint, reached_index) == "stop_and_confirm"
                and not self._is_docking_task()
                and distance <= self.coarse_goal_tolerance_m
                and self._localization_sample_fresh()
                and self._arrival_correction_completed_index == reached_index
            ):
                LOGGER.warning(
                    "waypoint %d remains %.2fm from click after one fine re-approach; accepting corrected coarse arrival",
                    reached_index,
                    distance,
                )
                return self._accept_coarse_patrol_arrival(
                    reached_index, waypoint, distance, retries=retries
                )
            LOGGER.warning(
                "waypoint %d still off-click after %d re-approaches",
                reached_index,
                retries,
            )
            self._clear_arrival_reapproach_tracking(reached_index)
            self._emit_safe_hold(
                "PHYSICAL_REAPPROACH_EXHAUSTED",
                "航点重接近重试耗尽，进入安全保持",
            )
            return True
        self._arrival_retry_counts[reached_index] = retries + 1
        self._arrival_reapproach_index = reached_index
        self.context.arrival_reapproach_waypoint_index = reached_index
        self.context.arrival_reapproach_attempts = retries + 1
        self._cancel_arrival_adjustment(reset_state=True)
        self._arrival_convergence_attempts.pop(reached_index, None)
        LOGGER.warning(
            "re-approaching waypoint %d after rejected arrival (attempt %d/%d)",
            reached_index,
            retries + 1,
            self.arrival_reapproach_max_attempts,
        )
        self.context.current_waypoint_index = reached_index
        self.context.state = "running"
        self.context.state_version += 1
        self._persist()
        self._dispatch_navigation(reached_index, reapproach=True)
        return True

    def _cancel_absolute_localization_resume_watch(self) -> None:
        stop = self._absolute_pause_watch_stop
        self._absolute_pause_watch_stop = None
        self._absolute_pause_watch_thread = None
        if stop is not None:
            stop.set()

    def _arm_absolute_localization_resume_watch(self) -> None:
        """Auto-resume ABSOLUTE_LOCALIZATION_REQUIRED once RTK/LIO is usable again."""
        self._cancel_absolute_localization_resume_watch()
        stop = threading.Event()
        self._absolute_pause_watch_stop = stop

        def _loop() -> None:
            recovery_requested = False
            started_at = time.monotonic()
            while not stop.wait(0.5):
                request_recovery = False
                no_correction_reason = None
                with self._lock:
                    if (
                        not self.context
                        or self.context.state != "paused"
                        or not self._paused_for_localization
                    ):
                        return
                    decision = self._localization_decision()
                    transaction_completed = self._waypoint_correction_completed(decision)
                    if transaction_completed is not None:
                        self._retry_waypoint_localization_correction(decision)
                        no_correction_reason = (
                            self._waypoint_no_correction_continue_reason(decision)
                        )
                        ready = bool(
                            transaction_completed
                            and decision.get("lio_healthy", True)
                            and self._localization_sample_fresh(decision)
                        )
                        transaction = decision.get("one_shot_correction")
                        request_recovery = bool(
                            not ready
                            and not recovery_requested
                            and self._active_correction_mode == "ndt"
                            and isinstance(transaction, dict)
                            and str(transaction.get("transaction_id") or "")
                            == self._active_correction_transaction_id
                            and str(transaction.get("status") or "")
                            == "waiting_source"
                        )
                        if request_recovery:
                            recovery_requested = True
                    else:
                        outdoor = self._outdoor_navigation_profile()
                        if outdoor:
                            ready = self._outdoor_absolute_pause_can_resume(decision)
                        else:
                            ready = (
                                not bool(decision.get("lio_motion_anomaly"))
                                and not bool(decision.get("correction_smoothing_active"))
                                and bool(decision.get("absolute_stable"))
                                and str(decision.get("active_source") or "")
                                in {"lio_imu", "rtk_imu", "ndt_imu"}
                            )
                if request_recovery:
                    callback = self.localization_recovery_callback
                    if callable(callback):
                        LOGGER.warning(
                            "waypoint NDT correction has no eligible match; "
                            "requesting bounded stationary relocalization"
                        )
                        try:
                            callback("ndt_waypoint_correction_unavailable")
                        except Exception:
                            LOGGER.exception(
                                "unable to start waypoint NDT relocalization recovery"
                            )
                            recovery_requested = False
                    else:
                        LOGGER.warning(
                            "waypoint NDT correction is waiting for a source but "
                            "no relocalization recovery callback is configured"
                        )
                if ready and no_correction_reason is not None:
                    # A transaction may finish while an older task instance is
                    # already paused. Reconfirm zero motion before this watch
                    # resumes it, matching the normal arrival path.
                    ready = self._hold_final_pose(
                        timeout_seconds=self._no_correction_stop_recheck_timeout()
                    )
                if not ready:
                    if (
                        time.monotonic() - started_at
                        >= ABSOLUTE_LOCALIZATION_RESUME_WATCH_SECONDS
                    ):
                        LOGGER.error(
                            "absolute localization did not recover within %.0fs; "
                            "task remains safely paused",
                            ABSOLUTE_LOCALIZATION_RESUME_WATCH_SECONDS,
                        )
                        return
                    continue
                LOGGER.info(
                    "absolute localization pause cleared (source=%s rtk=%s); resuming task",
                    decision.get("active_source"),
                    decision.get("rtk_quality"),
                )
                self.on_localization_recovered()
                return

        thread = threading.Thread(
            target=_loop,
            daemon=True,
            name="absolute-loc-resume-watch",
        )
        self._absolute_pause_watch_thread = thread
        thread.start()

    def _absolute_localization_ready(self, timeout_seconds: float | None = None) -> bool:
        getter = getattr(self.navigation, "localization_decision", None)
        if not callable(getter):
            return True
        outdoor = self._outdoor_navigation_profile()
        first_decision = getter() or {}
        if timeout_seconds is None:
            timeout_seconds = (
                self._outdoor_settle_timeout_seconds(first_decision)
                if outdoor
                else WAYPOINT_SETTLE_TIMEOUT_SECONDS
            )
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        while time.monotonic() <= deadline:
            decision = getter() or {}
            if bool(decision.get("lio_motion_anomaly")):
                LOGGER.warning(
                    "waypoint settle blocked by LIO motion anomaly reason=%s",
                    decision.get("lio_motion_anomaly_reason") or "unknown",
                )
                return False
            transaction_completed = self._waypoint_correction_completed(decision)
            if transaction_completed is not None:
                no_correction_reason = self._waypoint_no_correction_continue_reason(
                    decision
                )
                ndt_score_ok = True
                if (
                    no_correction_reason is None
                    and self._active_correction_mode == "ndt"
                    and decision.get("ndt_score") is not None
                ):
                    try:
                        ndt_score_ok = float(decision["ndt_score"]) <= self.arrival_ndt_max_fitness_score
                    except (TypeError, ValueError):
                        ndt_score_ok = False
                stop_confirmed = True
                if transaction_completed and no_correction_reason is not None:
                    stop_confirmed = self._hold_final_pose(
                        timeout_seconds=self._no_correction_stop_recheck_timeout()
                    )
                if (
                    transaction_completed
                    and bool(decision.get("lio_healthy", True))
                    and self._localization_sample_fresh(decision)
                    and ndt_score_ok
                    and stop_confirmed
                ):
                    if no_correction_reason is not None:
                        LOGGER.warning(
                            "waypoint correction completed without absolute update reason=%s; "
                            "continuing on fresh stopped LIO/UKF",
                            no_correction_reason,
                        )
                    return True
                stop_motion = getattr(self.navigation, "stop_motion", None)
                if callable(stop_motion):
                    stop_motion()
                time.sleep(0.1)
                continue
            correction_active = bool(decision.get("correction_smoothing_active", False))
            needs_correction = self._needs_stationary_correction(decision)
            if outdoor:
                # Never leave while smoothing/pull-in is active. Clean outdoor
                # arrivals can depart quickly; mid-correction must wait out the
                # full settle budget (or pause + auto-resume on timeout).
                if correction_active or needs_correction:
                    stop_motion = getattr(self.navigation, "stop_motion", None)
                    if callable(stop_motion):
                        stop_motion()
                    time.sleep(0.1)
                    continue
                if not self._localization_sample_fresh(decision):
                    LOGGER.warning(
                        "outdoor waypoint settle blocked by stale localization sample age=%s",
                        decision.get("sample_age_seconds"),
                    )
                    time.sleep(0.1)
                    continue
                if self._outdoor_settle_can_continue(decision):
                    return True
                time.sleep(0.1)
                continue
            if correction_active or needs_correction:
                # Keep the controller at the safety boundary while the
                # localization node is moving map->lio_odom.  A single stop
                # command can be overwritten by an active Nav2 controller.
                stop_motion = getattr(self.navigation, "stop_motion", None)
                if callable(stop_motion):
                    stop_motion()
                time.sleep(0.1)
                continue
            source = str(decision.get("active_source") or "")
            policy_source_ready = decision.get("policy_source_ready")
            if not self._localization_sample_fresh(decision):
                LOGGER.warning(
                    "waypoint settle blocked by stale localization sample age=%s",
                    decision.get("sample_age_seconds"),
                )
                time.sleep(0.1)
                continue
            if (
                self._rtk_good_for_navigation()
                and source == "rtk_imu"
                and bool(decision.get("absolute_stable"))
            ):
                return True
            if (
                source in {"ndt_imu", "rtk_imu", "lio_imu"}
                and bool(decision.get("absolute_stable"))
                and (policy_source_ready is None or policy_source_ready is True)
            ):
                return True
            time.sleep(0.1)
        decision = getter() or {}
        if bool(decision.get("lio_motion_anomaly")):
            return False
        if outdoor:
            # Mid-correction timeout: pause and let the resume watch finish the
            # pull-in. Do not cruise away with an unfinished RTK XY fix.
            if bool(decision.get("correction_smoothing_active")) or (
                self._needs_stationary_correction(decision)
                and not self._outdoor_settle_can_continue(decision)
            ):
                LOGGER.warning(
                    "outdoor waypoint settle timed out while absolute correction "
                    "was still pending; pausing for recovery "
                    "(source=%s rtk=%s smoothing=%s drift=%s)",
                    decision.get("active_source"),
                    decision.get("rtk_quality"),
                    decision.get("correction_smoothing_active"),
                    decision.get("rtk_drift_decision"),
                )
                return False
            if self._outdoor_settle_can_continue(decision):
                if self._needs_stationary_correction(decision):
                    LOGGER.warning(
                        "outdoor waypoint settle timed out with residual absolute drift; "
                        "continuing on FAST-LIO because fixed RTK/LIO remains usable "
                        "(decision=%s xy=%s)",
                        decision.get("rtk_drift_decision"),
                        (decision.get("rtk_drift") or {}).get("xy_m")
                        if isinstance(decision.get("rtk_drift"), dict)
                        else None,
                    )
                else:
                    LOGGER.warning(
                        "outdoor waypoint arrival timed out waiting for a settled pose; "
                        "continuing on FAST-LIO without further absolute correction"
                    )
                return True
            LOGGER.warning(
                "outdoor waypoint settle timed out without a usable absolute pose; "
                "pausing so localization can recover before the next leg "
                "(source=%s rtk=%s smoothing=%s)",
                decision.get("active_source"),
                decision.get("rtk_quality"),
                decision.get("correction_smoothing_active"),
            )
            return False
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
            self._paused_localization_reason = None
            self._cancel_absolute_localization_resume_watch()
            self._assert_execution(execution_id)
            self._clear_nav_dispatch_retry()
            self._suspend_obstacle_monitor()
            self._cancel_arrival_adjustment(reset_state=False)
            if self.context.state == "paused":
                return {"final_task_state": "paused", "state_version": self.context.state_version, "robot_stopped": True}
            if self.context.state not in {"accepted", "running", "pausing", "resuming", "interrupted"}:
                raise ProtocolError("INVALID_TASK_STATE", f"cannot pause from {self.context.state}")
            previous_state = self.context.state
            self.context.state = "pausing"
            self.context.state_version += 1
            self._persist()
            self._emit("task.pausing")
            waiting_at_waypoint = self._waiting_waypoint_index() is not None
            if (
                previous_state != "accepted"
                and not waiting_at_waypoint
                and not self._cancel_active_navigation()
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

    def resume_forward(self, execution_id: str) -> dict:
        """Operator acknowledgement to retry a task held by an obstacle."""
        with self._lock:
            self._assert_execution(execution_id)
            if self.context.state != "running":
                raise ProtocolError("INVALID_TASK_STATE", "task is not running")
            if self._obstacle_stage != "SAFE_OBSERVING" or not self._obstacle_episode_id:
                raise ProtocolError(
                    "OBSTACLE_NOT_OBSERVING", "task is not waiting in obstacle safe observation"
                )
            self.navigation.stop_motion()
            if not self.navigation.is_robot_stopped():
                raise ProtocolError("STOP_NOT_CONFIRMED", "robot stop is not confirmed")
            observation = self.navigation.obstacle_monitor_snapshot() or {}
            if not self._localization_allows_obstacle_monitor(observation):
                raise ProtocolError("LOCALIZATION_NOT_NORMAL", "localization is not normal")
            if not self._obstacle_is_clear(observation):
                self._obstacle_clear_started_at = None
                raise ProtocolError("OBSTACLE_NOT_CLEAR", "obstacle or stale scan still blocks recovery")
            clear_for = (
                time.monotonic() - self._obstacle_clear_started_at
                if self._obstacle_clear_started_at is not None
                else 0.0
            )
            required = float(getattr(self.obstacle_speech, "obstacle_clear_seconds", 3.0))
            if clear_for < required:
                if self._obstacle_clear_started_at is None:
                    self._obstacle_clear_started_at = time.monotonic()
                raise ProtocolError(
                    "OBSTACLE_CLEAR_NOT_STABLE",
                    f"obstacle clearance must remain stable for {required:.1f}s",
                )
            waypoint_index = self.context.current_waypoint_index
            self._emit_obstacle_stage(
                "CLEAR_CONFIRMED", observation, attempt=self._recovery_attempts,
                reason="manual_continue_check",
            )
        self._resume_after_obstacle(observation, automatic=False)
        return {
            "final_task_state": "running",
            "resumed_forward": True,
            "waypoint_index": waypoint_index,
        }

    def resume_task(
        self,
        execution_id: str,
        resume_index: int,
        *,
        _navigation_cancelled: bool = False,
    ) -> dict:
        with self._lock:
            self._assert_execution(execution_id)
            if self.context.state == "running":
                return {"final_task_state": "running", "state_version": self.context.state_version, "resume_from_waypoint_index": self.context.current_waypoint_index}
            if self.context.state not in {"accepted", "paused", "pausing", "resuming", "interrupted"}:
                raise ProtocolError("INVALID_TASK_STATE", f"cannot resume from {self.context.state}")
            if resume_index != self.context.current_waypoint_index:
                raise ProtocolError("TASK_CONTEXT_MISMATCH", "resume index does not match persisted context")
            if (
                self.context.state == "paused"
                and self._paused_for_localization
                and self._paused_localization_reason == "absolute_required"
            ):
                points = self.context.route_snapshot.get("waypoints") or []
                waypoint = points[resume_index] if 0 <= resume_index < len(points) else None
                decision = self._localization_decision()
                if waypoint is not None:
                    existing = self._matching_waypoint_correction(
                        decision,
                        waypoint,
                        resume_index,
                        statuses={"waiting_source", "smoothing", "completed"},
                    )
                    if existing is not None:
                        self._adopt_waypoint_correction(
                            existing,
                            waypoint_localization_mode(waypoint.get("localization_mode")),
                        )
                # Give the readiness probe one scheduler tick.  A literal
                # zero-second deadline can skip its first sample altogether.
                if self._absolute_localization_ready(timeout_seconds=0.01):
                    self.on_localization_recovered()
                    return {
                        "final_task_state": self.context.state if self.context else "completed",
                        "state_version": self.context.state_version if self.context else 0,
                        "resume_from_waypoint_index": resume_index,
                    }
                # Operator acknowledgement must not bypass a required anchor
                # or replace its in-flight transaction. The recovery watcher
                # owns stationary relocalization and resumes automatically.
                self.context.state_version += 1
                self._persist()
                message = self._absolute_localization_wait_message(decision)
                self._emit(
                    "task.paused",
                    code="ABSOLUTE_LOCALIZATION_REQUIRED",
                    message=message,
                )
                self._arm_absolute_localization_resume_watch()
                return {
                    "final_task_state": "paused",
                    "state_version": self.context.state_version,
                    "resume_from_waypoint_index": resume_index,
                    "resume_blocked": True,
                    "reason_code": "ABSOLUTE_LOCALIZATION_REQUIRED",
                    "reason_message": message,
                }
            self._paused_for_localization = False
            self._paused_localization_reason = None
            self._cancel_absolute_localization_resume_watch()
            self.context.state = "resuming"
            self.context.state_version += 1
            self._persist()
            self._emit("task.resuming")
            if self._post_arrival_active(resume_index):
                self.context.state = "running"
                self.context.state_version += 1
                self._persist()
                self._emit("task.resumed")
                self._resume_post_arrival(resume_index)
                return {
                    "final_task_state": self.context.state,
                    "state_version": self.context.state_version,
                    "resume_from_waypoint_index": resume_index,
                    "post_arrival_stage": self.context.post_arrival_stage,
                }
            waiting_index = self._waiting_waypoint_index()
            if waiting_index is not None:
                reached_index = waiting_index
                self._waypoint_localization_ready_index = reached_index
                self.context.state = "running"
                self.context.state_version += 1
                self._persist()
                self._emit("task.resumed")
                self._maybe_continue_after_waypoint(reached_index)
                return {
                    "final_task_state": "running",
                    "state_version": self.context.state_version,
                    "resume_from_waypoint_index": reached_index,
                    "waiting_for_waypoint_speech": (
                        self._speech_waiting_index == reached_index
                        and not self._speech_wait_finished
                    ),
                    "waiting_for_waypoint_dwell": (
                        self._dwell_waiting_index == reached_index
                        and not self._dwell_wait_finished
                    ),
                }
            # A paused task may still have a live FollowWaypoints goal (or a
            # goal whose cancel acknowledgement is late). Never start the
            # replacement heading/cruise leg until the old goal is confirmed
            # cancelled, otherwise both controllers can move the robot.
            if not _navigation_cancelled and not self._cancel_active_navigation(timeout_seconds=2.0):
                self.navigation.stop_motion()
                self.context.state = "paused"
                self.context.state_version += 1
                self._persist()
                raise ProtocolError(
                    "NAVIGATION_CANCEL_FAILED",
                    "cannot resume until the previous Nav2 goal cancellation is confirmed",
                )
            self._send_from(resume_index)
            # A pre-leg heading action returns before _dispatch_navigation(),
            # so that path used to leave the persisted Edge state at
            # ``resuming`` even though a motion goal had already been accepted.
            # Publish the same running state as an accepted cruise goal before
            # releasing this lock.  A delayed center recovery command will
            # then be recognized as obsolete instead of cancelling the turn.
            if (
                self.context.state == "resuming"
                and self._departure_heading_index is not None
            ):
                self.context.state = "running"
                self.context.state_version += 1
                self._persist()
                self._emit("task.resumed")
            return {
                "final_task_state": "running",
                "state_version": self.context.state_version,
                "resume_from_waypoint_index": resume_index,
            }

    def recover_task(
        self,
        execution_id: str,
        *,
        trigger_reason_code: str,
        recovery_episode_id: str,
        attempt: int,
    ) -> dict:
        """Resume a system-held task after the center's five-second gate."""
        with self._lock:
            self._assert_execution(execution_id)
            # A delayed center command can arrive after Edge has already
            # resumed the task.  Never cancel or zero a live replacement goal
            # just to acknowledge that obsolete recovery command.
            if self.context.state == "running":
                return {
                    "final_task_state": "running",
                    "state_version": self.context.state_version,
                    "recovery_action": "already_recovered",
                    "recovery_status": "already_running",
                    "recovery_episode_id": recovery_episode_id,
                    "attempt": int(attempt),
                }
            if self.context.state == "accepted":
                return {
                    "final_task_state": "accepted",
                    "state_version": self.context.state_version,
                    "recovery_action": "task_start_initializing",
                    "recovery_status": "in_progress",
                    "reason_code": "TASK_START_INITIALIZING",
                    "reason_message": "启动指令仍在初始化，等待导航开始后再恢复",
                    "recovery_episode_id": recovery_episode_id,
                    "attempt": int(attempt),
                }

            # A localization-loss pause can race a Nav2/BT velocity update.
            # Zeroing /cmd_vel alone is not sufficient: an uncancelled goal
            # may immediately cause Collision Monitor to publish a non-zero
            # command again.  Invalidate local callbacks and cancel that goal
            # before opening the confirmed-stop window.
            self._cancel_arrival_adjustment()
            self._clear_departure_heading(cancel_navigation=False)
            navigation_cancelled = self._cancel_active_navigation(timeout_seconds=2.0)

            # Recovery may be requested long after the original pause. Refresh
            # the zero command after cancellation so the confirmation is based
            # on a current collision-monitor output rather than a stale sample.
            stop_motion = getattr(self.navigation, "stop_motion", None)
            if callable(stop_motion):
                stop_motion()
            if not self.navigation.is_robot_stopped():
                snapshot_getter = getattr(self.navigation, "obstacle_monitor_snapshot", None)
                try:
                    observation = snapshot_getter() if callable(snapshot_getter) else {}
                except Exception:
                    LOGGER.warning("failed to collect stop-confirmation evidence", exc_info=True)
                    observation = {}
                observation = observation if isinstance(observation, dict) else {}
                evidence = {
                    "navigation_cancelled": navigation_cancelled,
                    "stop_confirmation_seconds": self.stop_confirmation_seconds,
                    "actual_planar_speed_mps": observation.get("actual_planar_speed_mps"),
                    "actual_turn_speed_rps": observation.get("actual_turn_speed_rps"),
                    "actual_velocity_sample_age_seconds": observation.get(
                        "actual_velocity_sample_age_seconds"
                    ),
                    "requested_planar_speed_mps": observation.get("requested_planar_speed_mps"),
                    "requested_turn_speed_rps": observation.get("requested_turn_speed_rps"),
                    "requested_velocity_sample_age_seconds": observation.get(
                        "requested_velocity_sample_age_seconds"
                    ),
                }
                raise ProtocolError(
                    "ROBOT_NOT_STOPPED",
                    "recovery requires a confirmed stop"
                    f" (nav_cancelled={navigation_cancelled}; "
                    f"actual_planar_speed_mps={evidence['actual_planar_speed_mps']}; "
                    f"actual_turn_speed_rps={evidence['actual_turn_speed_rps']})",
                    details={"stop_confirmation": evidence},
                )
            if self._recovery_arbiter.budget_exhausted():
                return {
                    "final_task_state": self.context.state,
                    "state_version": self.context.state_version,
                    "resume_blocked": True,
                    "reason_code": "RECOVERY_BUDGET_EXHAUSTED",
                    "reason_message": "Edge 自愈动作预算耗尽，保持停车",
                    "recovery_action": "safe_hold",
                    "recovery_status": "non_retryable",
                    "recovery_episode_id": recovery_episode_id,
                    "attempt": int(attempt),
                    "recovery": self._recovery_arbiter.snapshot(),
                }
            code = str(
                self.context.last_safe_hold_code or trigger_reason_code or "TASK_INTERRUPTED"
            )
            if code in {
                "ARRIVAL_POSE_CONVERGENCE_FAILED",
                "ARRIVAL_POST_ADJUSTMENT_UNSTABLE",
                "ARRIVAL_MICRO_ADJUST_UNAVAILABLE",
                "ARRIVAL_MICRO_ADJUST_POSE_UNAVAILABLE",
                "ARRIVAL_MICRO_ADJUST_RESIDUAL_EXCEEDED",
            }:
                index = (
                    self.context.post_arrival_waypoint_index
                    if self.context.post_arrival_waypoint_index is not None
                    else self.context.current_waypoint_index
                )
                waypoints = self.context.route_snapshot.get("waypoints") or []
                if index < 0 or index >= len(waypoints):
                    raise ProtocolError("TASK_CONTEXT_MISMATCH", "arrival waypoint is missing")
                waypoint = waypoints[index]
                decision = self._localization_decision()
                if not self._localization_sample_fresh(decision):
                    self._hold_for_arrival_precision_recovery(
                        "到点定位数据不新鲜，正在执行精准定位恢复",
                    )
                    return {
                        "final_task_state": "paused",
                        "state_version": self.context.state_version,
                        "resume_blocked": True,
                        "reason_code": "LOCALIZATION_RECOVERY_IN_PROGRESS",
                        "reason_message": "到点定位数据不新鲜，正在执行精准定位恢复",
                        "recovery_action": "precision_localization_recovery",
                        "recovery_status": "in_progress",
                        "retry_after_seconds": 1,
                        "recovery_episode_id": recovery_episode_id,
                        "attempt": int(attempt),
                    }
                distance, _yaw_error = self._arrival_pose_errors(waypoint, index)
                if (
                    distance is None
                    or distance > self.arrival_nav2_reapproach_max_error_m
                ):
                    # Probe the now-stopped pose once before requesting a new
                    # precision-recovery episode.
                    if not self._absolute_localization_ready(timeout_seconds=0.01):
                        self._hold_for_arrival_precision_recovery(
                            "到点残差超过 1.50 米，正在执行精准定位恢复",
                        )
                        return {
                            "final_task_state": "paused",
                            "state_version": self.context.state_version,
                            "resume_blocked": True,
                            "reason_code": "LOCALIZATION_RECOVERY_IN_PROGRESS",
                            "reason_message": "到点残差超过 1.50 米，正在执行精准定位恢复",
                            "recovery_action": "precision_localization_recovery",
                            "recovery_status": "in_progress",
                            "retry_after_seconds": 1,
                            "recovery_episode_id": recovery_episode_id,
                            "attempt": int(attempt),
                        }
                self.context.last_safe_hold_code = ""
                self.context.last_safe_hold_message = ""
                if not self._reapproach_rejected_arrival(
                    index,
                    localization_recovered=distance is not None
                    and distance > self.arrival_nav2_reapproach_max_error_m,
                ):
                    self._emit_safe_hold(
                        "PHYSICAL_REAPPROACH_EXHAUSTED",
                        "到点自愈无法重新接近航点，保持停车",
                    )
                return {
                    "final_task_state": self.context.state,
                    "state_version": self.context.state_version,
                    "recovery_action": "nav2_reapproach",
                    "recovery_status": "in_progress",
                    "waypoint_index": index,
                    "distance_m": distance,
                    "recovery_episode_id": recovery_episode_id,
                    "attempt": int(attempt),
                }
            if "LOCALIZATION" in code.upper() or code in {"ARRIVAL_XY_UNVERIFIED", "ARRIVAL_CORRECTION_FAILED"}:
                if callable(self.localization_recovery_callback):
                    self.localization_recovery_callback("center_loop_recovery")
                return {
                    "final_task_state": "paused",
                    "state_version": self.context.state_version,
                    "resume_blocked": True,
                    "reason_code": "LOCALIZATION_RECOVERY_IN_PROGRESS",
                    "reason_message": "定位恢复正在执行，保持停车",
                    "recovery_action": "localization_recovery_in_progress",
                    "recovery_status": "in_progress",
                    "retry_after_seconds": 1,
                    "recovery_episode_id": recovery_episode_id,
                    "attempt": int(attempt),
                }
            self.context.last_safe_hold_code = ""
            self.context.last_safe_hold_message = ""
            self._persist()
            result = self.resume_task(
                execution_id,
                self.context.current_waypoint_index,
                _navigation_cancelled=navigation_cancelled,
            )
            result.update(
                {
                    "recovery_action": "resume_pending_waypoint",
                    "recovery_status": "recovered",
                    "recovery_episode_id": recovery_episode_id,
                    "attempt": int(attempt),
                }
            )
            return result

    def force_exit(
        self,
        execution_id: str,
        *,
        reason_code: str = "FORCE_EXIT",
        reason_message: str = "task force-exited",
        report_start_result: bool = True,
    ) -> dict:
        """Idempotently clear any local motion task, regardless of its state."""
        with self._lock:
            self._cancel_localization_recovery()
            self._clear_nav_dispatch_retry()
            self._cancel_waypoint_localization_correction()
            self._cancel_arrival_adjustment(reset_state=True)
            self._cancel_absolute_localization_resume_watch()
            self._clear_departure_heading(cancel_navigation=False)
            self._invalidate_nav_results()
            self._cancel_waypoint_dwell()
            self._stop_obstacle_monitor()
            self._stop_task_rosbag()
            context = self.context
            if context:
                cancelled = self._cancel_active_navigation(timeout_seconds=8.0)
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
                result = {
                    "final_task_state": "cancelled",
                    "state_version": context.state_version,
                    "robot_stopped": self.navigation.is_robot_stopped(),
                    "cleared": True,
                    "reason_code": reason_code,
                    "reason_message": reason_message,
                    "rosbag": self._rosbag_state if context.record_rosbag else None,
                }
                if report_start_result:
                    try:
                        self.start_result_callback(
                            context.start_command_id,
                            "cancelled",
                            result,
                            reason_code,
                            reason_message,
                        )
                    except Exception:
                        # The callback stores to the durable outbox before it
                        # publishes. Network failure must never retain a motion
                        # context after force-exit.
                        LOGGER.exception("failed to publish force-exit start result")
                self.store.clear_task_context(context.task_execution_id, "cancelled")
                self.context = None
                return result
            return {
                "final_task_state": "cancelled",
                "state_version": 0,
                "robot_stopped": self.navigation.is_robot_stopped(),
                "cleared": True,
                "reason_code": reason_code,
                "reason_message": reason_message,
                "rosbag": None,
            }

    def cancel_task(self, execution_id: str) -> dict:
        with self._lock:
            self._cancel_localization_recovery()
            self._assert_execution(execution_id)
            self._clear_nav_dispatch_retry()
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
                    "rosbag": self._rosbag_state if self.context.record_rosbag else None,
                }
            if self.context.state not in {"running", "paused", "pausing", "resuming", "interrupted"}:
                raise ProtocolError("INVALID_TASK_STATE", f"cannot cancel from {self.context.state}")
            previous_state = self.context.state
            self._cancel_waypoint_localization_correction()
            self.context.state = "cancelling"
            self.context.state_version += 1
            self._persist()
            self._emit("task.cancelling")
            waiting_at_waypoint = self._waiting_waypoint_index() is not None
            if (
                previous_state != "paused"
                and not waiting_at_waypoint
                and not self._cancel_active_navigation()
            ):
                raise ProtocolError("NAVIGATION_CANCEL_FAILED", "Nav2 action cancel failed")
            if not self.navigation.is_robot_stopped():
                raise ProtocolError("ROBOT_NOT_STOPPED", "robot speed did not reach stop threshold")
            self._stop_task_rosbag()
            self._cancel_waypoint_dwell()
            self._restore_navigation_profile()
            self.context.state = "cancelled"
            self.context.state_version += 1
            self._persist()
            self._emit("task.cancelled")
            result = {
                "final_task_state": "cancelled",
                "state_version": self.context.state_version,
                "robot_stopped": True,
                "already_terminal": False,
                "cancel_performed": True,
                "rosbag": self._rosbag_state if self.context.record_rosbag else None,
            }
            try:
                self.start_result_callback(
                    self.context.start_command_id,
                    "cancelled",
                    result,
                    "TASK_CANCELLED",
                    "task cancelled",
                )
            except Exception:
                LOGGER.exception("failed to publish cancelled start result")
            self.store.clear_task_context(execution_id, "cancelled")
            return result

    def on_feedback(
        self,
        current_waypoint_index: int,
        distance_remaining_m: float | None = None,
        *,
        milestone: str = "",
        completed_waypoints: int | None = None,
        arrival_extra: dict | None = None,
    ) -> None:
        apply_final = False
        policy_waypoint = None
        speed_distance_remaining = None
        progress_updates: list[dict] = []
        with self._lock:
            if not self.context or self.context.state != "running":
                return
            current_waypoint_index += self._goal_offset
            total = len(self.context.route_snapshot["waypoints"])
            dispatched_final_index = self._goal_offset + max(self._dispatched_count, 1) - 1
            if current_waypoint_index < 0 or current_waypoint_index >= total:
                return
            if milestone != "waypoint_reached" and milestone != "arrival_confirmed":
                policy_waypoint = self.context.route_snapshot["waypoints"][current_waypoint_index]
                speed_distance_remaining = distance_remaining_m
                if speed_distance_remaining is None:
                    speed_distance_remaining = self._distance_to_waypoint(policy_waypoint)
            if (
                not self._is_docking_task()
                and current_waypoint_index == dispatched_final_index
                and not self._patrol_final_approach_applied
                and milestone not in {"waypoint_reached", "arrival_confirmed"}
            ):
                remaining = distance_remaining_m
                if remaining is None:
                    remaining = self._distance_to_waypoint(
                        self.context.route_snapshot["waypoints"][dispatched_final_index]
                    )
                if (
                    remaining is not None
                    and remaining <= PATROL_FINAL_APPROACH_M
                    and not self._waypoint_is_pass_through(dispatched_final_index)
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
            elif milestone in {"waypoint_reached", "arrival_confirmed"}:
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
                    arrival_progress = self._build_progress_locked(
                        reached_index,
                        "arrival_confirmed" if milestone == "arrival_confirmed" else "waypoint_reached",
                        reached_index + 1,
                        0.0,
                    )
                    if (
                        milestone == "arrival_confirmed"
                        and reached_index == current_waypoint_index
                        and arrival_extra
                    ):
                        arrival_progress.update(arrival_extra)
                    progress_updates.append(arrival_progress)
                    if milestone == "arrival_confirmed":
                        # Compatibility alias for older UI/consumers that still
                        # key off waypoint_reached for completion coloring.
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
        if speed_distance_remaining is not None:
            speed_updater = getattr(self.navigation, "update_navigation_speed_envelope", None)
            if callable(speed_updater):
                try:
                    speed_updater(speed_distance_remaining)
                except Exception:
                    LOGGER.warning("navigation speed-envelope update failed", exc_info=True)
        if apply_final:
            try:
                self._apply_patrol_final_approach()
            except ProtocolError:
                LOGGER.exception(
                    "final-approach navigation profile was not confirmed; entering safe hold"
                )
                self._emit_safe_hold(
                    "FINAL_APPROACH_PROFILE_NOT_CONFIRMED",
                    "终点控制参数未确认，禁止继续前进，已进入安全保持",
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
                self.event_callback("task.progress", progress, self._event_trace_id())

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
            "trace_id": self.context.trace_id,
            "round_number": self.context.round_number,
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
            "leg_generation": self._leg_generation,
            "idempotency_key": self._idempotency_key(
                milestone or "progress",
                waypoint_id=str(waypoint.get("waypoint_id") or waypoint_index),
            ),
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

    def on_navigation_result(self, status: str, error_message: str = "", details: dict | None = None, generation: int | None = None) -> None:
        with self._lock:
            # Result callbacks can arrive after cancellation or after a
            # terminal failure (the Nav2 waypoint server may finish its
            # worker thread later).  Never let such a stale success advance
            # the route or dispatch the next waypoint.
            if generation is not None and generation != self._nav_goal_generation:
                # Obstacle recovery invalidates the old goal generation before
                # asking Nav2 to cancel it. Account for that expected cancel
                # even when its acknowledgement arrives through the now-stale
                # callback, while still discarding every stale success.
                if status == "cancelled" and self._expected_recovery_cancels > 0:
                    self._expected_recovery_cancels -= 1
                LOGGER.info(
                    "ignoring stale Nav2 result generation=%s current=%s status=%s",
                    generation,
                    self._nav_goal_generation,
                    status,
                )
                return
            if not self.context or self.context.state in self.TERMINAL_STATES | {"pausing", "cancelling", "paused"}:
                return
            if status == "succeeded":
                if self._bypass_active:
                    self._bypass_active = False
                    resume_index = self.context.current_waypoint_index
                    LOGGER.info(
                        "obstacle bypass via reached; resuming waypoint %s",
                        resume_index,
                    )
                    self._send_from(resume_index)
                    return
                # Nav2's coarse success is followed by precise arrival
                # validation and may re-dispatch this same waypoint. Preserve
                # the obstacle episode and the waypoint-level attempt budget.
                self._suspend_obstacle_monitor()
                missed = list((details or {}).get("missed_waypoints") or [])
                if missed:
                    absolute_missed = [index + self._goal_offset for index in missed]
                    self._fail(
                        "NAVIGATION_MISSED_WAYPOINTS",
                        f"Nav2 reported missed waypoints: {absolute_missed}",
                    )
                    return
                departure_heading_completed = self._departure_heading_index is not None
                if departure_heading_completed:
                    if self._departure_heading_mode == "teleop":
                        # Teleop spins complete on their worker thread, not via Nav2.
                        return
                    reached_index = self._departure_heading_index
                    cruise_index = self._departure_cruise_index
                    self._complete_departure_heading_locked(reached_index, cruise_index)
                    if cruise_index is not None:
                        return
                    # Fall through into the post-arrival path for the reached index.
                else:
                    reached_index = self._goal_offset + max(self._dispatched_count, 1) - 1
                reached_waypoint = self.context.route_snapshot["waypoints"][reached_index]
                total_waypoints = len(self.context.route_snapshot["waypoints"])
                if self._arrival_policy(reached_waypoint, reached_index) == "pass_through":
                    # A pass-through point is progress only: do not trigger
                    # stationary correction, dwell, actions, or arrival speech.
                    self._waypoint_localization_ready_index = reached_index
                    self.on_feedback(
                        reached_index - self._goal_offset,
                        0.0,
                        milestone="waypoint_passed",
                        completed_waypoints=reached_index + 1,
                    )
                    self._maybe_continue_after_waypoint(reached_index)
                    return
                # A FollowWaypoints success only means Nav2's goal checker
                # accepted the pose; it does not guarantee that the
                # quadruped has finished coasting.  Confirm zero motion before
                # switching to the stationary NDT/RTK policy, otherwise the
                # first correction sample can be taken while the body is
                # still moving and create an avoidable TF correction.
                waypoint_id = str(reached_waypoint.get("waypoint_id") or reached_index)
                if not self._hold_final_pose(
                    stage_callback=lambda stage, details: self._emit_arrival_stop_stage(
                        reached_index, waypoint_id, stage, details
                    )
                ):
                    self._emit_safe_hold(
                        "ARRIVAL_STOP_NOT_CONFIRMED",
                        "Nav2 到点后未确认零速，禁止进入定位校正与到点验收",
                    )
                    return
                if self._handle_lightweight_arrival(reached_index, reached_waypoint):
                    return
                correction_completed = (
                    self._arrival_correction_completed_index == reached_index
                )
                if not correction_completed:
                    # Absolute correction owns the first stationary stage. A
                    # later arrival-heading callback must not start it again.
                    self._emit_arrival_stage(
                        reached_index, "correction", "正在进行航点绝对定位校正"
                    )
                    self._correction_generation += 1
                    self._correction_completed_at_mono = None
                    self._emit_idempotent(
                        "task.arrival_correcting",
                        event_type_key="arrival_correcting",
                        waypoint_id=str(reached_waypoint.get("waypoint_id") or reached_index),
                        message="切换静止定位并校正",
                        extra={"correction_generation": self._correction_generation},
                    )
                    self._set_localization_policy(reached_waypoint, "stationary")
                    self._start_waypoint_localization_correction(
                        reached_waypoint, reached_index
                    )
                    if not self._absolute_localization_ready():
                        self.navigation.stop_motion()
                        self._restore_navigation_profile()
                        self._paused_for_localization = True
                        self._paused_localization_reason = "absolute_required"
                        self.context.state = "paused"
                        self.context.current_waypoint_index = reached_index
                        self.context.state_version += 1
                        self._persist()
                        self._emit(
                            "task.paused",
                            code="ABSOLUTE_LOCALIZATION_REQUIRED",
                            message=self._absolute_localization_wait_message(
                                self._localization_decision()
                            ),
                        )
                        self._arm_absolute_localization_resume_watch()
                        return
                    self._correction_completed_at_mono = time.monotonic()
                    self._arrival_correction_completed_index = reached_index

                post_arrival_active = self._post_arrival_active(reached_index)
                requires_micro_recheck = bool(
                    post_arrival_active
                    and self.context.post_arrival_stage == "xy_adjustment_recheck"
                )
                if not post_arrival_active or requires_micro_recheck:
                    # Position approach is deliberately before final yaw. A
                    # Nav2 re-approach invalidates both stage latches.
                    self._emit_arrival_stage(
                        reached_index, "position_approach", "正在按校正后位置确认航点"
                    )
                    self._last_arrival_xy_stability_result = None
                    if not self._arrival_xy_is_stable(reached_waypoint, reached_index):
                        stability = getattr(self, "_last_arrival_xy_stability_result", None)
                        # Only fresh frames which all prove that XY is outside
                        # tolerance may physically re-approach the click. A
                        # missing/stale/inconsistent stream has no trustworthy
                        # direction to drive, so it must remain parked.
                        if (
                            isinstance(stability, ArrivalStabilityResult)
                            and stability.reason != "outside_tolerance"
                        ):
                            self._emit_safe_hold(
                                "ARRIVAL_LOCALIZATION_EVIDENCE_UNAVAILABLE",
                                "到点后未取得连续新鲜定位帧，禁止重新靠近；"
                                "机器人保持停车等待定位恢复",
                            )
                            return
                        if requires_micro_recheck:
                            # A bounded cmd_vel segment intentionally stops
                            # every 0.15 m for a fresh localization check.
                            # If it has not reached the acceptance radius
                            # yet, preserve the completed heading and let the
                            # combined-pose gate start the next bounded
                            # segment.  Business arrival side effects stay
                            # latched and are never replayed here.
                            # A bounded segment does not invalidate the final
                            # yaw that preceded it.  The in-memory latch is
                            # required by the next segment's state guard and
                            # was previously left vulnerable to a duplicate
                            # result/recheck callback.
                            self._arrival_heading_completed_index = reached_index
                            current_distance, _ = self._arrival_pose_errors(
                                reached_waypoint, reached_index
                            )
                            xy_tolerance, _ = self._arrival_pose_tolerances(
                                reached_waypoint, reached_index
                            )
                            if (
                                current_distance is not None
                                and current_distance <= xy_tolerance
                            ):
                                # Motion has already reached the XY acceptance
                                # circle. Do not issue another raw-velocity
                                # segment merely because the short fresh-frame
                                # verification window was incomplete.
                                self.navigation.stop_motion()
                                self._emit_safe_hold(
                                    "ARRIVAL_CONFIRMATION_UNSTABLE",
                                    "微调后已进入 XY 验收范围，但连续新鲜定位帧未通过；"
                                    "保持停车等待重新确认",
                                )
                                return
                            self._set_post_arrival_stage(reached_index, "heading_aligned")
                        elif post_arrival_active:
                            self._emit_safe_hold(
                                "ARRIVAL_POST_ADJUSTMENT_UNSTABLE",
                                "微调分段后的定位校正或稳定位置验收未通过，保持停车等待自愈",
                            )
                            return
                        else:
                            outdoor = self._outdoor_navigation_profile()
                            decision = self._localization_decision()
                            if outdoor and (
                                not self._rtk_position_good_for_navigation()
                                or self._rtk_xy_from_decision(decision) is None
                            ):
                                self._hold_unconfirmed_outdoor_arrival(
                                    reached_index,
                                    "waypoint reached by FAST-LIO but outdoor RTK is not fixed/usable; waiting before trusting arrival",
                                )
                                return
                            # Initial XY verification never uses cmd_vel.
                            # Only the post-final-yaw path may make the
                            # bounded heading-preserving correction.
                            if self._reapproach_rejected_arrival(reached_index):
                                return
                            self._emit_safe_hold(
                                "ARRIVAL_POSE_CONVERGENCE_FAILED",
                                "校正后位置仍未到达当前航点，已停止继续收敛",
                            )
                            return
                    if (
                        requires_micro_recheck
                        and self.context.post_arrival_stage == "heading_aligned"
                    ):
                        # Preserve the completed heading while the next
                        # bounded XY segment is selected below.
                        pass
                    elif self.context.arrival_side_effects_started:
                        self._set_post_arrival_stage(reached_index, "xy_adjusted")
                    else:
                        self._set_post_arrival_stage(reached_index, "xy_verified")
                else:
                    self._restore_arrival_side_effect_gates(
                        reached_index, reached_waypoint
                    )

                arrival_heading_completed = (
                    self._arrival_heading_completed_index == reached_index
                )
                use_arrival_heading = self._use_teleop_arrival_heading(
                    reached_waypoint, reached_index
                )
                if not arrival_heading_completed and use_arrival_heading:
                    pose = self.navigation.latest_pose() if self.navigation else None
                    error = None
                    if pose is not None:
                        try:
                            error = self._heading_error_rad(
                                float(reached_waypoint["yaw"]),
                                float(getattr(pose, "yaw", 0.0) or 0.0),
                            )
                        except (AttributeError, KeyError, TypeError, ValueError):
                            error = None
                    if error is None or abs(error) > ARRIVAL_HEADING_ALIGN_RAD:
                        attempts = int(self._arrival_convergence_attempts.get(reached_index, 0))
                        if attempts >= ARRIVAL_CONVERGENCE_MAX_ATTEMPTS:
                            self._emit_safe_hold(
                                "ARRIVAL_POSE_CONVERGENCE_FAILED",
                                "最终航向两次调整后仍未收敛，机器人保持停车",
                            )
                            return
                        self._arrival_convergence_attempts[reached_index] = attempts + 1
                        self._set_post_arrival_stage(
                            reached_index, "heading_pending"
                        )
                        self._emit_arrival_stage(
                            reached_index, "heading_alignment", "位置确认完成，正在调整最终航向"
                        )
                        self._emit_idempotent(
                            "task.arrival_heading_aligning",
                            event_type_key=f"arrival_heading_aligning_{attempts + 1}",
                            waypoint_id=str(reached_waypoint.get("waypoint_id") or reached_index),
                            message="目标点位置已确认，原地对准最终朝向",
                            extra={"target_yaw": reached_waypoint.get("yaw")},
                        )
                        if self._start_departure_heading(
                            desired_yaw=float(reached_waypoint["yaw"]),
                            reached_index=reached_index,
                            cruise_index=None,
                            profile_waypoint=reached_waypoint,
                            error_rad=error,
                            arrival_heading=True,
                            tolerance_rad=ARRIVAL_HEADING_ALIGN_RAD,
                        ):
                            return
                    else:
                        self._arrival_heading_completed_index = reached_index
                        arrival_heading_completed = True
                        self._set_post_arrival_stage(
                            reached_index, "heading_aligned"
                        )

                self._emit_arrival_stage(
                    reached_index, "pose_verification", "正在同时验收航点位置和最终航向"
                )
                if not self._arrival_pose_within_combined_tolerance(
                    reached_waypoint, reached_index
                ):
                    distance, yaw_error = self._arrival_pose_errors(
                        reached_waypoint, reached_index
                    )
                    if distance is None:
                        self._emit_safe_hold(
                            "ARRIVAL_MICRO_ADJUST_POSE_UNAVAILABLE",
                            "到达后无法取得有效当前位姿，禁止执行 XY 微调",
                        )
                        return
                    xy_tolerance, yaw_tolerance = self._arrival_pose_tolerances(
                        reached_waypoint, reached_index
                    )
                    if (
                        use_arrival_heading
                        and arrival_heading_completed
                        and distance <= xy_tolerance
                        and yaw_tolerance is not None
                        and (yaw_error is None or yaw_error > yaw_tolerance)
                    ):
                        # The pose used to finish the in-place heading can be
                        # superseded by a later fresh localization sample.
                        # There is no XY residual to adjust in this case;
                        # retry the bounded final-heading stage instead of
                        # misreporting an unavailable XY-adjust interface.
                        self._arrival_heading_completed_index = None
                        self._set_post_arrival_stage(reached_index, "xy_adjusted")
                        self.on_navigation_result(
                            "succeeded", generation=self._nav_goal_generation
                        )
                        return
                    micro_adjust_limit = (
                        xy_tolerance + self.arrival_micro_adjust_total_budget_m
                    )
                    if distance > micro_adjust_limit:
                        self._emit_safe_hold(
                            "ARRIVAL_MICRO_ADJUST_RESIDUAL_EXCEEDED",
                            "最终航向后 XY 偏差"
                            f" {distance:.2f} 米超过可微调上限"
                            f" {micro_adjust_limit:.2f} 米，保持停车",
                        )
                        return
                    if self._start_arrival_adjustment(reached_waypoint, reached_index):
                        return
                    if use_arrival_heading and arrival_heading_completed:
                        LOGGER.warning(
                            "waypoint %d final pose outside combined tolerance: xy=%s yaw=%s",
                            reached_index,
                            distance,
                            yaw_error,
                        )
                        if self._arrival_micro_adjust_adapter_available(
                            reached_waypoint, reached_index
                        ):
                            self._emit_safe_hold(
                                "ARRIVAL_MICRO_ADJUST_STATE_INVALID",
                                "到达后 XY 微调状态门禁未满足，机器人保持停车",
                            )
                        else:
                            self._emit_safe_hold(
                                "ARRIVAL_MICRO_ADJUST_UNAVAILABLE",
                                "到达后 XY 微调接口不可用，机器人保持停车",
                            )
                        return
                    self._emit_safe_hold(
                        "ARRIVAL_POSE_CONVERGENCE_FAILED",
                        self._arrival_convergence_failure_message(
                            reached_waypoint, reached_index
                        ),
                    )
                    return
                if not self._arrival_pose_is_stable(reached_waypoint, reached_index):
                    self.navigation.stop_motion()
                    self._restore_navigation_profile()
                    self.context.state = "paused"
                    self.context.current_waypoint_index = reached_index
                    self.context.state_version += 1
                    self._persist()
                    self._emit(
                        "task.paused",
                        code="ARRIVAL_CONFIRMATION_UNSTABLE",
                        message="到点位姿未连续满足新鲜度和策略精度要求，等待重新确认",
                    )
                    return
                self._set_post_arrival_stage(
                    reached_index, "post_arrival_ready"
                )
                distance, _ = self._arrival_pose_errors(
                    reached_waypoint, reached_index
                )
                xy_tolerance, _ = self._arrival_pose_tolerances(
                    reached_waypoint, reached_index
                )
                retries = self._clear_arrival_reapproach_tracking(reached_index)
                self._start_arrival_side_effects(
                    reached_index,
                    reached_waypoint,
                    arrival_details={
                        "arrival_mode": "full_correction",
                        "localization_correction": "completed",
                        "distance_m": round(distance, 3) if distance is not None else None,
                        "acceptance_tolerance_m": xy_tolerance,
                        "reapproach_attempts": retries,
                        "coarse_completed": self._coarse_arrival_fallback_active(
                            reached_index
                        ),
                    },
                )
                self._waypoint_localization_ready_index = reached_index
                self._maybe_continue_after_waypoint(reached_index)
            elif status == "cancelled":
                if self._expected_recovery_cancels > 0:
                    self._expected_recovery_cancels -= 1
                    LOGGER.info("ignoring Nav2 cancel from obstacle recovery")
                    return
                if self._obstacle_recovery_active or self._bypass_active:
                    LOGGER.info("ignoring Nav2 cancel while obstacle recovery is in progress")
                    return
                if self.context.state == "running":
                    waypoint_index = self.context.current_waypoint_index
                    if self._post_arrival_active(waypoint_index):
                        adjustment_active = bool(
                            self._arrival_adjustment_thread
                            and self._arrival_adjustment_thread.is_alive()
                        )
                        heading_active = bool(
                            self._departure_heading_thread
                            and self._departure_heading_thread.is_alive()
                        )
                        if adjustment_active or heading_active:
                            LOGGER.info(
                                "ignoring Nav2 cancel during waypoint %s post-arrival processing",
                                waypoint_index,
                            )
                        else:
                            LOGGER.warning(
                                "Nav2 cancelled after waypoint %s was reached; resuming post-arrival processing",
                                waypoint_index,
                            )
                            self._resume_post_arrival(waypoint_index)
                        return
                    # FollowPath/BT recovery can cancel a child goal. Keep the
                    # task alive and put a goal back on the current waypoint.
                    LOGGER.warning(
                        "Nav2 cancelled while the task is running; re-dispatching waypoint %s",
                        self.context.current_waypoint_index,
                    )
                    self._send_from(self.context.current_waypoint_index)
                    return
                self._stop_obstacle_monitor()
                self._stop_task_rosbag()
                self._restore_navigation_profile()
                return
            else:
                self._restore_navigation_profile()
                if self._hold_blocked_task():
                    return
                self._stop_obstacle_monitor()
                self._emit_safe_hold(
                    "NAVIGATION_RECOVERY_EXHAUSTED",
                    error_message or "导航自愈等级已耗尽，进入安全保持",
                )

    def _dispatch_departure_heading(self, reached_index: int) -> bool:
        """Rotate in place toward the next waypoint before departing.

        Teleop yaw is used for indoor and outdoor patrol so MPPI/Nav2 cannot
        weave a stationary 180deg spin. A timeout still aborts and continues cruise.
        """
        if not self.context:
            return False
        if self._is_docking_task():
            return False
        waypoints = self.context.route_snapshot.get("waypoints") or []
        next_index = reached_index + 1
        if next_index >= len(waypoints):
            return False
        current, target = waypoints[reached_index], waypoints[next_index]
        pose = self.navigation.latest_pose() if self.navigation else None
        if pose is not None:
            try:
                x = float(pose.x)
                y = float(pose.y)
                target_x = float(target["x"])
                target_y = float(target["y"])
            except (AttributeError, KeyError, TypeError, ValueError):
                x = y = target_x = target_y = None
        else:
            x = y = target_x = target_y = None
        if x is not None and y is not None and target_x is not None and target_y is not None:
            dx, dy = target_x - x, target_y - y
            if hypot(dx, dy) < 1e-3:
                dx = float(target["x"]) - float(current["x"])
                dy = float(target["y"]) - float(current["y"])
        else:
            dx = float(target["x"]) - float(current["x"])
            dy = float(target["y"]) - float(current["y"])
        if hypot(dx, dy) < 1e-3:
            return False
        desired = atan2(dy, dx)
        error = None
        if pose is not None:
            try:
                yaw = float(getattr(pose, "yaw", 0.0) or 0.0)
            except (AttributeError, TypeError, ValueError):
                yaw = None
            else:
                error = self._heading_error_rad(desired, yaw)
                if not self._pre_leg_heading_error_requires_spin(error):
                    return False
        return self._start_departure_heading(
            desired_yaw=desired,
            reached_index=reached_index,
            cruise_index=None,
            profile_waypoint=current,
            error_rad=error,
        )

    def _continue_after_waypoint(self, reached_index: int) -> None:
        with self._lock:
            if not self.context or self.context.state != "running":
                return
            self._active_correction_transaction_id = None
            self._active_correction_mode = None
            self._arrival_correction_completed_index = None
            self._clear_arrival_reapproach_tracking(reached_index)
            self._arrival_convergence_attempts.pop(reached_index, None)
            self._cancel_arrival_adjustment()
            if self._departure_heading_completed_index == reached_index:
                self._departure_heading_completed_index = None
            elif self._departure_heading_index is None and self._dispatch_departure_heading(reached_index):
                return
            next_waypoint_index = reached_index + 1
            self.context.current_waypoint_index = next_waypoint_index
            self.context.state_version += 1
            total_waypoints = len(self.context.route_snapshot["waypoints"])
            if next_waypoint_index < total_waypoints:
                self._clear_post_arrival_state()
                self._persist()
                self._arrival_heading_completed_index = None
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
            # Keep a terminal waypoint's coarse-fallback marker until the
            # final pose gate has evaluated the same accepted tolerance.
            self._clear_post_arrival_state()
            self._persist()
            self._arrival_heading_completed_index = None
            self._restore_navigation_profile()
            if pose_error:
                self._fail(*pose_error)
                return
            if self.context.round_number < self.context.loop_total:
                previous_terminal = dict(self.context.route_snapshot["waypoints"][-1])
                self.context.round_number += 1
                base = [dict(point) for point in (self.context.loop_base_waypoints or self.context.route_snapshot["waypoints"])]
                pose = self.navigation.latest_pose()
                if pose is not None and len(base) > 1:
                    start = base[0]
                    end = base[-1]
                    start_distance = hypot(float(pose.x) - float(start["x"]), float(pose.y) - float(start["y"]))
                    end_distance = hypot(float(pose.x) - float(end["x"]), float(pose.y) - float(end["y"]))
                    # Near the original start: continue forward. Otherwise
                    # return through the route in reverse order.
                    if start_distance <= 1.0 or start_distance <= end_distance:
                        ordered = base
                        direction = "forward"
                    else:
                        ordered = list(reversed(base))
                        direction = "reverse"
                    self.context.route_snapshot["waypoints"] = ordered
                    self.context.route_snapshot["execution_order"] = direction
                    LOGGER.info("loop round %d waypoint order=%s start_distance=%.2f end_distance=%.2f", self.context.round_number, direction, start_distance, end_distance)
                else:
                    self.context.route_snapshot["waypoints"] = base
                self.context.route_snapshot["initial_waypoint_index"] = 0
                self.context.current_waypoint_index = 0
                self.context.state_version += 1
                self._last_target_index = -1
                self._last_reached_index = -1
                self._goal_offset = 0
                self._dispatched_count = 0
                self._patrol_final_approach_applied = False
                self._clear_arrival_reapproach_tracking(reached_index)
                self._persist()
                self._emit("task.round_started", extra={"round_number": self.context.round_number, "loop_total": self.context.loop_total})
                # A new loop round starts from the just-confirmed terminal
                # pose.  Do not gate the first leg on the optional departure
                # heading worker: if localization yaw is stale that worker can
                # hold the round for its timeout without ever dispatching a
                # Nav2 goal.  The selected local controller is able to turn
                # while following this first leg; subsequent legs retain the
                # normal pre-departure heading policy.
                next_round_index = self._confirm_and_skip_loop_anchor(previous_terminal)
                if next_round_index == 0:
                    next_round_index = self._advance_past_colocated_waypoints(0)
                self._dispatch_navigation(next_round_index)
                return
            if self._is_docking_task() and callable(self.docking_arrived_handler):
                try:
                    self.docking_arrived_handler(dict(self.context.docking or {}))
                except Exception as exc:
                    self._fail("DOCK_CHARGE_START_FAILED", str(exc))
                    return
            # Stop/clear control ownership before exposing a terminal state.
            self._cancel_localization_recovery()
            self._finalize_navigation_control()
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

    def _waypoint_speech_enabled(self) -> bool:
        speech = self.waypoint_speech
        if speech is None:
            return False
        return bool(getattr(speech, "enabled", False))

    def _waypoint_speech_mode(self, waypoint: dict | None = None) -> str:
        waypoint = waypoint or {}
        mode = str(waypoint.get("speech_mode") or "").strip().lower()
        if not self._waypoint_speech_enabled():
            return "disabled"
        if mode == "disabled":
            return "disabled"
        # Robot policy is authoritative.  A route created by an older center
        # may still contain speech_mode=blocking, but speaker/TTS/command
        # delivery failures must never hold up patrol navigation when the
        # edge is configured for non-blocking speech.
        if not bool(getattr(self.waypoint_speech, "block_navigation", False)):
            return "non_blocking"
        if mode in {"blocking", "non_blocking"}:
            return mode
        if bool(getattr(self.waypoint_speech, "block_navigation", False)):
            return "blocking"
        return "non_blocking"

    def _waypoint_speech_blocks_navigation(self, waypoint: dict | None = None) -> bool:
        return self._waypoint_speech_mode(waypoint) == "blocking"

    def _run_waypoint_actions(self, waypoint_index: int, waypoint: dict) -> None:
        actions = list(waypoint.get("actions") or [])
        if not actions:
            return
        key = self._idempotency_key(
            "actions",
            waypoint_id=str(waypoint.get("waypoint_id") or waypoint_index),
        )
        results = self._action_registry.execute(
            actions=actions,
            idempotency_key=key,
            context={
                "waypoint_id": waypoint.get("waypoint_id"),
                "waypoint_index": waypoint_index,
                "task_execution_id": self.context.task_execution_id if self.context else "",
            },
        )
        self._emit_idempotent(
            "task.waypoint_actions",
            event_type_key="waypoint_actions",
            waypoint_id=str(waypoint.get("waypoint_id") or waypoint_index),
            extra={"results": [result.__dict__ for result in results]},
        )

    def _strip_non_navigation_waypoint_actions(self, route: dict) -> None:
        """Drop speech/alert waypoint extras so navigation ignores them."""
        if self._waypoint_speech_enabled():
            return
        stripped = 0
        for waypoint in route.get("waypoints") or []:
            if waypoint.pop("speech_template_id", None) is not None:
                stripped += 1
            waypoint.pop("speech_template_name", None)
            waypoint["speech_mode"] = "disabled"
        if stripped:
            LOGGER.info(
                "waypoint speech disabled; stripped speech templates from %d waypoints",
                stripped,
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

    def _waiting_waypoint_index(self) -> int | None:
        if self._speech_waiting_index is not None:
            return self._speech_waiting_index
        return self._dwell_waiting_index

    def _cancel_waypoint_dwell(self) -> None:
        stop = self._dwell_wait_stop
        self._dwell_wait_stop = None
        self._dwell_waiting_index = None
        self._dwell_wait_finished = False
        if stop is not None:
            stop.set()

    def _start_waypoint_dwell(self, waypoint_index: int) -> bool:
        if not self.context:
            return False
        waypoint = self.context.route_snapshot["waypoints"][waypoint_index]
        try:
            duration = max(0.0, float(waypoint.get("dwell_seconds") or 0.0))
        except (TypeError, ValueError):
            duration = 0.0
        self._cancel_waypoint_dwell()
        if duration <= 0.0:
            return False
        execution_id = self.context.task_execution_id
        stop = threading.Event()
        self._dwell_wait_stop = stop
        self._dwell_waiting_index = waypoint_index
        self._dwell_wait_finished = False
        LOGGER.info(
            "waypoint %d dwell started duration=%.1fs",
            waypoint_index,
            duration,
        )

        def _wait() -> None:
            try:
                if stop.wait(duration):
                    return
                with self._lock:
                    if (
                        stop is not self._dwell_wait_stop
                        or not self.context
                        or self.context.task_execution_id != execution_id
                        or self._dwell_waiting_index != waypoint_index
                    ):
                        return
                    self._dwell_wait_finished = True
                    LOGGER.info("waypoint %d dwell finished", waypoint_index)
                    self._maybe_continue_after_waypoint(waypoint_index)
            finally:
                with self._lock:
                    if self._dwell_wait_thread is threading.current_thread():
                        self._dwell_wait_thread = None

        thread = threading.Thread(
            target=_wait,
            daemon=True,
            name=f"waypoint-dwell-{waypoint_index}",
        )
        self._dwell_wait_thread = thread
        thread.start()
        return True

    def _maybe_continue_after_waypoint(self, waypoint_index: int) -> bool:
        if not self.context or self.context.state != "running":
            return False
        if self._waypoint_localization_ready_index != waypoint_index:
            return False
        if (
            self._speech_waiting_index == waypoint_index
            and not self._speech_wait_finished
        ):
            return False
        if (
            self._dwell_waiting_index == waypoint_index
            and not self._dwell_wait_finished
        ):
            return False
        if self._speech_waiting_index == waypoint_index:
            self._speech_waiting_index = None
            self._speech_wait_finished = False
        if self._dwell_waiting_index == waypoint_index:
            self._dwell_waiting_index = None
            self._dwell_wait_finished = False
            self._dwell_wait_stop = None
        self._waypoint_localization_ready_index = None
        self._set_post_arrival_stage(waypoint_index, "postprocess_completed")
        self._emit_idempotent(
            "task.waypoint_postprocess_completed",
            event_type_key="waypoint_postprocess_completed",
            waypoint_id=str(
                self.context.route_snapshot["waypoints"][waypoint_index].get("waypoint_id")
                or waypoint_index
            ),
            message="航点最终位置、航向与后处理已完成",
        )
        self._active_correction_transaction_id = None
        self._active_correction_mode = None
        self._continue_after_waypoint(waypoint_index)
        return True

    def _complete_waypoint_speech_wait(
        self,
        execution_id: str,
        waypoint_index: int,
        *,
        reason: str,
    ) -> None:
        """Finish a speech wait without failing the task.

        Playback/alert outcomes are treated as successful for navigation so a
        stuck speaker or missing status file cannot abort the route.
        """
        if not self.context or self.context.task_execution_id != execution_id:
            return
        if self.context.state == "paused":
            self._speech_wait_finished = True
            LOGGER.warning(
                "waypoint %d speech %s while paused; continuing after resume",
                waypoint_index,
                reason,
            )
            return
        if self.context.state != "running":
            return
        self._speech_wait_finished = True
        if not self._maybe_continue_after_waypoint(waypoint_index):
            LOGGER.warning(
                "waypoint %d speech %s while another arrival gate is pending",
                waypoint_index,
                reason,
            )
        if reason != "finished":
            LOGGER.warning(
                "waypoint %d speech %s; treating as success and continuing navigation",
                waypoint_index,
                reason,
            )

    def _wait_for_waypoint_speech(self, execution_id: str, waypoint_index: int) -> None:
        path = self._waypoint_speech_status_path(waypoint_index)
        if not path:
            with self._lock:
                self._complete_waypoint_speech_wait(
                    execution_id,
                    waypoint_index,
                    reason="unavailable",
                )
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
                    self._complete_waypoint_speech_wait(
                        execution_id,
                        waypoint_index,
                        reason="finished",
                    )
                return
            if state in {"failed", "superseded"}:
                with self._lock:
                    self._complete_waypoint_speech_wait(
                        execution_id,
                        waypoint_index,
                        reason=state,
                    )
                return
            time.sleep(float(self.waypoint_speech.poll_interval_seconds))
        with self._lock:
            self._complete_waypoint_speech_wait(
                execution_id,
                waypoint_index,
                reason="timeout",
            )

    def _is_docking_task(self) -> bool:
        if not self.context:
            return False
        docking = getattr(self.context, "docking", None) or {}
        return bool(docking.get("enabled"))

    def _arrival_policy(self, waypoint: dict | None, index: int | None = None) -> str:
        """Return explicit arrival semantics, with a safe legacy default."""
        waypoint = waypoint or {}
        value = str(waypoint.get("arrival_policy") or "").strip().lower()
        if value in {"pass_through", "stop_and_confirm", "precision", "dock"}:
            return value
        if index is not None and self._is_last_route_waypoint(index):
            return "stop_and_confirm"
        if waypoint.get("require_yaw") or float(waypoint.get("dwell_seconds") or 0) > 0:
            return "stop_and_confirm"
        if waypoint.get("actions") or waypoint.get("speech_template_id"):
            return "stop_and_confirm"
        return "stop_and_confirm"

    def _waypoint_is_pass_through(self, waypoint_index: int) -> bool:
        if not self.context:
            return False
        waypoints = self.context.route_snapshot.get("waypoints") or []
        if waypoint_index < 0 or waypoint_index >= len(waypoints):
            return False
        return self._arrival_policy(waypoints[waypoint_index], waypoint_index) == "pass_through"

    def _waypoint_requires_localization_correction(
        self, waypoint: dict, waypoint_index: int
    ) -> bool:
        """Every intentional stop must correct before it can be confirmed.

        A coarse 0.50 m Nav2 success is only a safe stationary point for
        localization; it is never a business arrival.  In particular, an
        ordinary middle click must not be accepted from a drifting LIO pose.
        ``pass_through`` remains the sole opt-out because it deliberately
        represents route geometry rather than a physical stopping point.
        """
        return self._arrival_policy(waypoint, waypoint_index) != "pass_through"

    def _complete_lightweight_arrival(
        self,
        reached_index: int,
        waypoint: dict,
        *,
        distance_m: float,
    ) -> None:
        retries = self._clear_arrival_reapproach_tracking(reached_index)
        self._set_post_arrival_stage(reached_index, "post_arrival_ready")
        self._emit_idempotent(
            "task.arrival_confirmed",
            event_type_key="lightweight_arrival_confirmed",
            waypoint_id=str(waypoint.get("waypoint_id") or reached_index),
            message="普通中间点已在精细半径内完成",
            extra={
                "arrival_mode": "lightweight",
                "distance_m": round(distance_m, 3),
                "acceptance_tolerance_m": self.final_waypoint_tolerance_m,
                "reapproach_attempts": retries,
                "coarse_completed": False,
            },
        )
        self._start_arrival_side_effects(
            reached_index,
            waypoint,
            arrival_details={
                "arrival_mode": "lightweight",
                "localization_correction": "skipped",
                "distance_m": round(distance_m, 3),
                "acceptance_tolerance_m": self.final_waypoint_tolerance_m,
                "reapproach_attempts": retries,
                "coarse_completed": False,
            },
        )
        self._waypoint_localization_ready_index = reached_index
        self._maybe_continue_after_waypoint(reached_index)

    def _handle_lightweight_arrival(
        self, reached_index: int, waypoint: dict
    ) -> bool:
        """Verify a plain intermediate point without a stationary NDT/RTK pass."""
        if self._waypoint_requires_localization_correction(waypoint, reached_index):
            return False
        decision = self._localization_decision()
        distance, _ = self._arrival_pose_errors(waypoint, reached_index)
        retries = int(self._arrival_retry_counts.get(reached_index, 0))
        self._emit_idempotent(
            "task.arrival_check",
            event_type_key=f"lightweight_arrival_check_{retries}",
            waypoint_id=str(waypoint.get("waypoint_id") or reached_index),
            message="普通中间点正在检查新鲜位姿与靠近次数",
            extra={
                "arrival_mode": "lightweight",
                "distance_m": round(distance, 3) if distance is not None else None,
                "fine_tolerance_m": self.final_waypoint_tolerance_m,
                "coarse_tolerance_m": self.coarse_goal_tolerance_m,
                "reapproach_attempts": retries,
                "reapproach_max_attempts": self.arrival_reapproach_max_attempts,
            },
        )
        if distance is None or not self._localization_sample_fresh(decision):
            self._emit_safe_hold(
                "LIGHTWEIGHT_ARRIVAL_POSE_UNAVAILABLE",
                "普通中间点到达位姿缺失或陈旧，保持停车等待定位恢复",
            )
            return True
        if distance <= self.final_waypoint_tolerance_m:
            self._complete_lightweight_arrival(
                reached_index,
                waypoint,
                distance_m=distance,
            )
            return True
        if distance > self.coarse_goal_tolerance_m:
            self._emit_safe_hold(
                "LIGHTWEIGHT_ARRIVAL_OUTSIDE_COARSE_RADIUS",
                f"普通中间点距目标 {distance:.2f} 米，超过粗到达半径"
                f" {self.coarse_goal_tolerance_m:.2f} 米",
            )
            return True
        return self._reapproach_rejected_arrival(reached_index)

    def _navigation_arrival_tolerance(self, waypoint_index: int) -> float:
        """Return the Nav2 radius for this dispatch, not the final verdict."""
        if not self.context or self._arrival_reapproach_index != waypoint_index:
            return self.coarse_goal_tolerance_m
        waypoint = self.context.route_snapshot["waypoints"][waypoint_index]
        policy = self._arrival_policy(waypoint, waypoint_index)
        if policy == "dock" and self._is_docking_task():
            return self.docking_goal_tolerance_m
        if policy == "precision":
            return self.precision_arrival_tolerance_m
        return self.final_waypoint_tolerance_m

    def _set_navigation_arrival_tolerance(self, waypoint_index: int) -> None:
        setter = getattr(self.navigation, "set_arrival_goal_tolerance", None)
        if callable(setter):
            waypoint = self.context.route_snapshot["waypoints"][waypoint_index]
            policy = self._arrival_policy(waypoint, waypoint_index)
            yaw_tolerance = 3.14
            if bool(waypoint.get("require_yaw", False)):
                yaw_tolerance = (
                    self.docking_goal_yaw_tolerance_rad
                    if policy == "dock" and self._is_docking_task()
                    else PRECISION_ARRIVAL_YAW_TOLERANCE_RAD
                )
            if self.context and self._arrival_reapproach_index == waypoint_index:
                # Re-approach must not rotate toward the final waypoint yaw;
                # that turn is performed only after corrected XY acceptance.
                yaw_tolerance = 3.14
            try:
                setter(
                    self._navigation_arrival_tolerance(waypoint_index),
                    yaw_tolerance_rad=yaw_tolerance,
                )
            except TypeError:
                setter(self._navigation_arrival_tolerance(waypoint_index))

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
        reapproach: bool = False,
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
        detour_enabled = bool(
            profile_waypoint.get(
                "detour_enabled",
                profile_waypoint.get("avoidance_to_next", True),
            )
        )
        slowdown_enabled = bool(
            profile_waypoint.get(
                "collision_slowdown_enabled",
                profile_waypoint.get("avoidance_to_next", True),
            )
        )
        # Hard stop stays forced on for ordinary legs.
        stop_enabled = True
        if "collision_stop_enabled" in profile_waypoint and not bool(
            profile_waypoint.get("collision_stop_enabled")
        ):
            LOGGER.warning(
                "waypoint requested collision_stop_enabled=false; keeping hard stop enabled"
            )
        avoid_obstacles = detour_enabled
        precision_goal = False
        if self._is_docking_task():
            final_index = int((self.context.docking or {}).get("final_waypoint_index", 1))
            if waypoint_index >= final_index:
                avoid_obstacles = False
                detour_enabled = False
                slowdown_enabled = False
            precision_goal = waypoint_index == final_index
        require_yaw = bool(target.get("require_yaw", False))
        arrival_policy = self._arrival_policy(target, waypoint_index)
        patrol_final = (
            (not self._is_docking_task())
            and waypoint_index == len(waypoints) - 1
            and arrival_policy != "pass_through"
        )
        final_approach = patrol_final or precision_goal
        if force_require_yaw is not None:
            require_yaw = bool(force_require_yaw)
        # Do not ask RPP to solve the final orientation while it is still
        # tracking a path.  Patrol require_yaw is enforced immediately after
        # XY arrival by the direct, stationary teleop turn in
        # on_navigation_result.  If teleop is unavailable we retain Nav2's
        # original yaw-goal behavior as the compatibility fallback.
        if self._use_teleop_arrival_heading(target, waypoint_index):
            require_yaw = False
        if force_final is not None:
            final_approach = bool(force_final) or precision_goal
        self._segment_avoidance_enabled = avoid_obstacles
        outdoor_profile = self._outdoor_navigation_profile()
        local_controller = str(target.get("local_controller") or "mppi")
        navigation_speed_level = normalize_navigation_speed_level(
            profile_waypoint.get("navigation_speed_level")
        )
        speed_profile = "final" if final_approach else "cruise"
        goal_checker_id = (
            "precision_goal_checker"
            if arrival_policy == "precision" or precision_goal
            else "general_goal_checker"
        )
        leg_profile = LegProfile(
            localization_mode=str(target.get("localization_mode") or "ndt"),
            global_planner_id=str(target.get("global_controller") or self.context.route_snapshot.get("global_controller") or "theta_star"),
            local_controller_id=local_controller,
            detour_enabled=detour_enabled,
            collision_slowdown_enabled=slowdown_enabled,
            collision_stop_enabled=stop_enabled,
            speed_profile=speed_profile,
            goal_checker_id=goal_checker_id,
            arrival_policy=arrival_policy,
            # The filter preserves both endpoints, so it is also safe for
            # outdoor and precision legs. The BT owns simple/raw fallbacks.
            smoother_id="savitzky_golay",
        )
        if leg_profile != self._active_leg_profile:
            LOGGER.info("applying leg profile generation=%s profile=%s", self._leg_generation + 1, leg_profile)
            self._active_leg_profile = leg_profile
        apply_profile = getattr(self.navigation, "apply_navigation_profile", None)
        if callable(apply_profile):
            apply_profile(
                generation=self._leg_generation + 1,
                global_controller=str(
                    target.get("global_controller")
                    or self.context.route_snapshot.get("global_controller")
                    or "theta_star"
                ),
                local_controller=local_controller,
                detour_enabled=leg_profile.detour_enabled,
                collision_slowdown_enabled=leg_profile.collision_slowdown_enabled,
                collision_stop_enabled=leg_profile.collision_stop_enabled,
                require_yaw=require_yaw,
                final_approach=final_approach,
                outdoor=outdoor_profile,
                smoother_id=leg_profile.smoother_id,
                navigation_speed_level=navigation_speed_level,
                reapproach=reapproach,
            )
        else:
            safety_setter = getattr(self.navigation, "set_safety_profile", None)
            if callable(safety_setter):
                safety_setter(
                    detour_enabled=leg_profile.detour_enabled,
                    collision_slowdown_enabled=leg_profile.collision_slowdown_enabled,
                    collision_stop_enabled=leg_profile.collision_stop_enabled,
                )
            global_setter = getattr(self.navigation, "set_global_controller", None)
            if callable(global_setter):
                global_setter(
                    str(
                        target.get("global_controller")
                        or self.context.route_snapshot.get("global_controller")
                        or "theta_star"
                    )
                )
            setter = getattr(self.navigation, "set_waypoint_profile", None)
            if callable(setter):
                profile_kwargs = {
                    "avoid_obstacles": avoid_obstacles,
                    "require_yaw": require_yaw,
                    "final_approach": final_approach,
                    "outdoor": outdoor_profile,
                    "local_controller": local_controller,
                    "navigation_speed_level": navigation_speed_level,
                    "reapproach": reapproach,
                }
                try:
                    setter(**profile_kwargs)
                except TypeError:
                    # Older simulation/test adapters retain the pre-tier
                    # signature. Production RosAdapter always receives it.
                    profile_kwargs.pop("navigation_speed_level", None)
                    profile_kwargs.pop("reapproach", None)
                    setter(**profile_kwargs)
            smoother_setter = getattr(self.navigation, "set_smoother", None)
            if callable(smoother_setter):
                smoother_setter(leg_profile.smoother_id)
            outdoor_setter = getattr(self.navigation, "apply_outdoor_gps_profile", None)
            if callable(outdoor_setter):
                outdoor_setter(outdoor=outdoor_profile)
        precision_setter = getattr(self.navigation, "set_goal_precision", None)
        if callable(precision_setter):
            precision_setter(enabled=precision_goal or arrival_policy == "precision")
        if self._is_docking_task():
            self._apply_docking_profile(waypoint_index)

    def _outdoor_navigation_profile(self) -> bool:
        if not self.context:
            return False
        map_info = self.context.route_snapshot.get("map") or {}
        scene_scope = str(
            map_info.get("scene_scope")
            or self.context.route_snapshot.get("scene_scope")
            or ""
        ).lower()
        coordinate_mode = str(map_info.get("coordinate_mode") or "").lower()
        return coordinate_mode == "rtk_fixed" and scene_scope in {
            "outdoor",
            "transition",
        }

    def _waypoint_local_controller(self, waypoint_index: int | None = None) -> str:
        if not self.context:
            return "mppi"
        waypoints = self.context.route_snapshot.get("waypoints") or []
        if not waypoints:
            return "mppi"
        index = self.context.current_waypoint_index if waypoint_index is None else waypoint_index
        index = min(max(index, 0), len(waypoints) - 1)
        return str(waypoints[index].get("local_controller") or "mppi")

    def _apply_patrol_final_approach(self) -> None:
        """Slow the live FollowPath goal; do not touch costmaps or docking precision."""
        dispatched_final_index = self._goal_offset + max(self._dispatched_count, 1) - 1
        if self._waypoint_is_pass_through(dispatched_final_index):
            return
        setter = getattr(self.navigation, "set_waypoint_profile", None)
        if not callable(setter):
            return
        kwargs = {
            "avoid_obstacles": self._segment_avoidance_enabled,
            "require_yaw": False,
            "final_approach": True,
            "outdoor": self._outdoor_navigation_profile(),
            "local_controller": self._waypoint_local_controller(),
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
                setter(
                    avoid_obstacles=True,
                    require_yaw=False,
                    final_approach=False,
                    outdoor=self._outdoor_navigation_profile(),
                    local_controller=self._waypoint_local_controller(),
                )
            precision_setter = getattr(self.navigation, "set_goal_precision", None)
            if callable(precision_setter):
                precision_setter(enabled=False)
            arrival_setter = getattr(self.navigation, "set_arrival_goal_tolerance", None)
            if callable(arrival_setter):
                try:
                    arrival_setter(
                        self.coarse_goal_tolerance_m,
                        yaw_tolerance_rad=0.25,
                    )
                except TypeError:
                    arrival_setter(self.coarse_goal_tolerance_m)
            if self._is_docking_task():
                self._restore_docking_profile()
        except Exception:
            LOGGER.exception("failed to restore default navigation profile")

    def _finalize_navigation_control(self) -> None:
        """Make terminal task state a hard control boundary.

        A successful Nav2 result can race with an Edge heading/recovery worker.
        Invalidate callbacks and publish an explicit zero command before the
        business terminal event is persisted.  This is intentionally idempotent
        so completion, cancellation and failure paths can all use it.
        """
        self._invalidate_nav_results()
        self._clear_departure_heading(cancel_navigation=False)
        stop = getattr(self.navigation, "stop_motion", None)
        if callable(stop):
            try:
                stop()
            except Exception:
                LOGGER.exception("failed to zero motion at navigation terminal state")


    def _clear_nav_dispatch_retry(self) -> None:
        if self._nav_dispatch_retry_timer:
            self._nav_dispatch_retry_timer.cancel()
            self._nav_dispatch_retry_timer = None
        self._nav_dispatch_retry_started_at = None
        self._pending_dispatch_index = None

    def _schedule_nav_dispatch_retry(self, index: int) -> None:
        """Keep the task alive when Nav2 rejects a goal and retry dispatch.

        Localization recovery often resumes before FollowWaypoints is ready.
        Failing the whole patrol there is worse than waiting and re-sending.
        """
        if not self.context or self.context.state in self.TERMINAL_STATES | {"cancelling"}:
            return
        now = time.monotonic()
        if self._nav_dispatch_retry_started_at is None:
            self._nav_dispatch_retry_started_at = now
        elapsed = now - self._nav_dispatch_retry_started_at
        budget = self.navigation_dispatch_retry_budget_seconds
        if elapsed >= budget:
            self._clear_nav_dispatch_retry()
            self._fail(
                "NAV_STACK_NOT_READY",
                "FollowWaypoints goal was rejected after "
                f"{budget:.0f}s of retries",
            )
            return
        self._pending_dispatch_index = index
        self.context.current_waypoint_index = index
        if self.context.state not in {"paused", "pausing"}:
            self.context.state = "running"
        self.context.state_version += 1
        self._persist()
        remaining = max(0.0, budget - elapsed)
        self.event_callback(
            "task.progress",
            {
                "task_execution_id": self.context.task_execution_id,
                "trace_id": self.context.trace_id,
                "state": self.context.state,
                "state_version": self.context.state_version,
                "current_waypoint_index": index,
                "completed_waypoints": index,
                "total_waypoints": len(self.context.route_snapshot["waypoints"]),
                "distance_remaining_m": None,
                "estimated_time_remaining_s": round(remaining),
                "nav_stack_retry": True,
                "reported_at": now_iso(),
            },
            self._event_trace_id(),
        )
        delay = self.navigation_dispatch_retry_seconds
        LOGGER.warning(
            "Nav2 goal rejected at waypoint %s; retrying in %.1fs (%.0fs budget left)",
            index,
            delay,
            remaining,
        )
        if self._nav_dispatch_retry_timer:
            self._nav_dispatch_retry_timer.cancel()
        self._nav_dispatch_retry_timer = threading.Timer(delay, self._retry_nav_dispatch)
        self._nav_dispatch_retry_timer.daemon = True
        self._nav_dispatch_retry_timer.start()

    def _retry_nav_dispatch(self) -> None:
        with self._lock:
            if not self.context or self.context.state in self.TERMINAL_STATES | {"cancelling"}:
                return
            if self.context.state in {"paused", "pausing"} and self._paused_for_localization:
                # Wait for localization recovery to re-dispatch; do not fight it.
                self._nav_dispatch_retry_timer = threading.Timer(
                    self.navigation_dispatch_retry_seconds,
                    self._retry_nav_dispatch,
                )
                self._nav_dispatch_retry_timer.daemon = True
                self._nav_dispatch_retry_timer.start()
                return
            index = (
                self._pending_dispatch_index
                if self._pending_dispatch_index is not None
                else self.context.current_waypoint_index
            )
            LOGGER.info("retrying Nav2 dispatch from waypoint %s", index)
            self._send_from(index)

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
                "trace_id": self.context.trace_id,
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
        self._evaluate_obstacle_progress()
        with self._lock:
            if not self.context or self.context.state != "running":
                return
            if self._obstacle_episode_id is not None:
                # The episode monitor owns retries and the continuous 3 s clear
                # window. A Nav2 failure must not create a second recovery path.
                interval = 0.5 if self._obstacle_stage == "SAFE_OBSERVING" else float(
                    self.obstacle_speech.navigation_retry_seconds
                )
                self._blocked_retry_timer = threading.Timer(
                    interval, self._retry_blocked_navigation
                )
                self._blocked_retry_timer.daemon = True
                self._blocked_retry_timer.start()
                return
            self._send_from(self.context.current_waypoint_index)

    def _hold_final_pose(
        self,
        timeout_seconds: float | None = None,
        stage_callback: Callable[[str, dict], None] | None = None,
    ) -> bool:
        """Stop the dog before measuring the last waypoint.

        Nav2's checker does not require zero velocity, and a quadruped still
        coasts after /cmd_vel goes to zero. Measuring immediately lets that
        coast fail a 0.35 m check the controller had already accepted.
        """
        if timeout_seconds is None:
            timeout_seconds = (
                HOLD_FINAL_POSE_OUTDOOR_TIMEOUT_SECONDS
                if self._outdoor_navigation_profile()
                else HOLD_FINAL_POSE_TIMEOUT_SECONDS
            )
        started_at = time.monotonic()

        def report(stage: str, **details) -> None:
            if not callable(stage_callback):
                return
            try:
                stage_callback(
                    stage,
                    {
                        "elapsed_seconds": round(time.monotonic() - started_at, 3),
                        "stop_confirmation_seconds": self.stop_confirmation_seconds,
                        "timeout_seconds": timeout_seconds,
                        **details,
                    },
                )
            except Exception:
                # Stage reporting is diagnostic only; a delivery failure must
                # never weaken the parking gate.
                LOGGER.warning("failed to report arrival stop stage", exc_info=True)

        report("nav2_stopping")
        stop_motion = getattr(self.navigation, "stop_motion", None)
        if callable(stop_motion):
            stop_motion()
        report("zero_confirming")
        is_stopped = getattr(self.navigation, "is_robot_stopped", None)
        if not callable(is_stopped):
            report("zero_confirmed", confirmation_source="adapter_unavailable")
            return True
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        while time.monotonic() <= deadline:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                try:
                    stopped = bool(is_stopped(timeout_seconds=remaining))
                except TypeError:
                    stopped = bool(is_stopped())
                if stopped:
                    report("zero_confirmed", confirmation_source="collision_monitor")
                    return True
            except Exception:
                LOGGER.exception("robot stop confirmation failed")
                report("zero_timeout", failure_reason="confirmation_exception")
                return False
            if callable(stop_motion):
                stop_motion()
            time.sleep(0.05)
        report("zero_timeout", failure_reason="confirmation_timeout")
        return False

    def _emit_arrival_stop_stage(
        self, reached_index: int, waypoint_id: str, stage: str, details: dict
    ) -> None:
        messages = {
            "nav2_stopping": "Nav2 已到目标附近，等待 Nav2 控制输出停止",
            "zero_confirming": "Nav2 控制输出已停止，正在连续确认零速 1 秒",
            "zero_confirmed": "零速已连续确认 1 秒，进入静止定位校正",
            "zero_timeout": "零速确认超时，禁止进入定位校正与到点验收",
        }
        self._emit_idempotent(
            f"task.arrival_{stage}",
            event_type_key=f"arrival_{stage}",
            waypoint_id=waypoint_id,
            code="ARRIVAL_STOP_CONFIRMATION",
            message=messages[stage],
            extra={
                "arrival_stage": stage,
                "execution_waypoint_index": reached_index,
                **details,
            },
        )

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
            else self.coarse_goal_tolerance_m
            if self._coarse_arrival_fallback_active(len(waypoints) - 1)
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
        if (
            self._outdoor_navigation_profile()
            and not self._is_docking_task()
            and not self._arrival_within_tolerance(final_waypoint, len(waypoints) - 1)
        ):
            return (
                "FINAL_POSE_OUT_OF_TOLERANCE",
                "final outdoor pose is not confirmed by fixed RTK against the last click",
            )
        policy = self._arrival_policy(final_waypoint, len(waypoints) - 1)
        require_final_yaw = bool(final_waypoint.get("require_yaw", False))
        if require_final_yaw:
            try:
                yaw_error = abs(
                    atan2(
                        sin(float(pose.yaw) - float(final_waypoint["yaw"])),
                        cos(float(pose.yaw) - float(final_waypoint["yaw"])),
                    )
                )
            except (AttributeError, KeyError, TypeError, ValueError):
                return ("FINAL_YAW_UNAVAILABLE", "final waypoint heading is unavailable")
            yaw_tolerance = (
                self.docking_goal_yaw_tolerance_rad
                if self._is_docking_task()
                else PRECISION_ARRIVAL_YAW_TOLERANCE_RAD
                if policy == "precision"
                else ARRIVAL_HEADING_ALIGN_RAD
            )
            if yaw_error > yaw_tolerance:
                return (
                    "FINAL_YAW_OUT_OF_TOLERANCE",
                    (
                        f"final heading is {yaw_error:.3f}rad from requested heading "
                        f"(tolerance {yaw_tolerance:.3f}rad)"
                    ),
                )
        return None

    def _fail(self, code: str, message: str) -> None:
        if not self.context:
            return
        self._cancel_localization_recovery()
        self._cancel_waypoint_localization_correction()
        self._cancel_arrival_adjustment(reset_state=True)
        self._clear_nav_dispatch_retry()
        self._cancel_waypoint_dwell()
        self._invalidate_nav_results()
        self._finalize_navigation_control()
        self._stop_task_rosbag()
        self._navigation_prepared = False
        self._departure_heading_index = None
        self._departure_cruise_index = None
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
        if not self.context:
            return
        try:
            self.store.save_task_context(self.context.__dict__)
        except sqlite3.ProgrammingError:
            LOGGER.debug("skip task persist; local store is already closed")

    def _event_trace_id(self) -> str:
        return str(self.context.trace_id if self.context else "")

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
        if not self.rosbag_recorder:
            self._rosbag_state = (
                {"running": False, "available": False, "error": "recorder unavailable"}
                if self.context and self.context.record_rosbag
                else {}
            )
            return
        context = self.context
        continuous = bool(
            context
            and context.record_rosbag
            and context.loop_execution
            and context.continuous_rosbag
            and context.loop_session_id
        )
        scope_id = context.loop_session_id if continuous else context.task_execution_id if context else ""
        scope_kind = "loop" if continuous else "task"
        label = (
            f"loop_{scope_id.replace('-', '')}"
            if continuous
            else f"task_{scope_id[:8]}"
        )
        try:
            try:
                status = self.rosbag_recorder.status()
            except Exception:
                LOGGER.warning("could not inspect navigation rosbag before task start", exc_info=True)
                status = {}
            remembered = self.store.get_metadata(self.ROSBAG_SCOPE_METADATA_KEY) or {}
            bag_name = Path(str(status.get("bag_dir") or "")).name
            same_scope = bool(
                status.get("running")
                and context
                and context.record_rosbag
                and remembered.get("kind") == scope_kind
                and remembered.get("id") == scope_id
                and label in bag_name
            )
            if same_scope:
                self._rosbag_state = status
                return
            if status.get("running"):
                LOGGER.warning(
                    "stopping stale navigation rosbag before starting task %s",
                    context.task_execution_id if context else "without-recording",
                )
                self.rosbag_recorder.stop()
            self.store.set_metadata(self.ROSBAG_SCOPE_METADATA_KEY, None)
            if not context or not context.record_rosbag:
                self._rosbag_state = {}
                return
            self._rosbag_state = self.rosbag_recorder.start(label)
            self.store.set_metadata(
                self.ROSBAG_SCOPE_METADATA_KEY,
                {
                    "kind": scope_kind,
                    "id": scope_id,
                    "label": label,
                    "bag_dir": self._rosbag_state.get("bag_dir"),
                },
            )
        except Exception as exc:
            LOGGER.exception("failed to start navigation rosbag")
            self._rosbag_state = {"running": False, "available": True, "error": str(exc)}

    def _stop_task_rosbag(self, *, force: bool = False) -> None:
        if not self.rosbag_recorder:
            return
        context = self.context
        if (
            not force
            and context
            and context.record_rosbag
            and context.loop_execution
            and context.continuous_rosbag
            and context.loop_session_id
        ):
            try:
                self._rosbag_state = self.rosbag_recorder.status()
            except Exception:
                LOGGER.warning("could not inspect continuous loop rosbag", exc_info=True)
            return
        if not force and (not context or not context.record_rosbag):
            return
        try:
            status = self.rosbag_recorder.status()
            self._rosbag_state = self.rosbag_recorder.stop() if status.get("running") else status
            self.store.set_metadata(self.ROSBAG_SCOPE_METADATA_KEY, None)
        except Exception as exc:
            LOGGER.exception("failed to stop navigation rosbag")
            self._rosbag_state = {
                **self._rosbag_state,
                "running": False,
                "error": str(exc),
            }

    def stop_loop_rosbag(self, loop_session_id: str) -> dict:
        """Stop one continuous loop recording without touching a newer session."""
        with self._lock:
            requested = str(uuid.UUID(str(loop_session_id)))
            if not self.rosbag_recorder:
                return {
                    "running": False,
                    "available": False,
                    "loop_session_id": requested,
                    "error": "recorder unavailable",
                }
            remembered = self.store.get_metadata(self.ROSBAG_SCOPE_METADATA_KEY) or {}
            status = self.rosbag_recorder.status()
            if remembered.get("kind") != "loop" or remembered.get("id") != requested:
                return {
                    **status,
                    "loop_session_id": requested,
                    "ignored": True,
                    "reason": "recording scope does not match",
                }
            self._rosbag_state = self.rosbag_recorder.stop() if status.get("running") else status
            self.store.set_metadata(self.ROSBAG_SCOPE_METADATA_KEY, None)
            return {
                **self._rosbag_state,
                "loop_session_id": requested,
                "ignored": False,
            }
