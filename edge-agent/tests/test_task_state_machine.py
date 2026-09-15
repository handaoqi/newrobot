from types import SimpleNamespace
from math import atan2, cos, hypot, sin
import hashlib
import json
import threading
import time

import pytest

from roamerx_edge.local_store import LocalStore
from roamerx_edge.protocol import ProtocolError, decode_message
from roamerx_edge.task_executor import (
    ArrivalStabilityResult,
    TaskExecutor,
    straighten_pass_through_waypoints,
)


class FakeNavigation:
    def __init__(self):
        self.feedback = None
        self.result = None
        self.sent = []
        self.cancelled = 0
        self.stop_commands = 0
        self.stopped = True
        self.pose = SimpleNamespace(x=1.0, y=2.0, yaw=0.0)
        self.teleop = []
        self.arrival_adjustments = []
        self.arrival_micro_goal_profiles = []
        self.arrival_clearance = {"clear": True, "reason": "clear"}
        self.costmap_clears = 0
        self.trusted_pose = {
            "x": 3.0,
            "y": 4.0,
            "z": 0.0,
            "yaw": 0.5,
            "sampled_at": "2026-08-10T12:00:00+08:00",
            "frame_id": "map",
        }
        self.stand_confirmed = True
        self.stand_requests = 0
        self.localization_policies = []
        self.waypoint_profiles = []
        self.goal_precisions = []
        self.arrival_goal_tolerances = []
        self.docking_profiles = []
        self.live_profiles = []
        self.outdoor_profiles = []
        self.global_controllers = []
        self.localization_state = {"active_source": "ndt_imu", "absolute_stable": True}
        self.progressive_relocalize_requests = []
        self.obstacle_recoveries = []
        self.obstacle_recovery_cancels = 0

    def prepare_for_navigation(self, timeout_seconds=12):
        self.stand_requests += 1
        return self.stand_confirmed

    def send_waypoints(self, waypoints, feedback_cb, result_cb):
        if getattr(self, "reject_remaining", 0):
            self.reject_remaining -= 1
            self.sent.append(waypoints)
            self.feedback = feedback_cb
            self.result = result_cb
            return False
        self.sent.append(waypoints)
        self.feedback = feedback_cb
        self.result = result_cb
        # Mirror a completed in-place turn so subsequent pre-leg face checks
        # see the updated heading (production TF/odom does this naturally).
        if len(waypoints) == 1 and bool(waypoints[0].get("require_yaw")):
            try:
                goal_x = float(waypoints[0]["x"])
                goal_y = float(waypoints[0]["y"])
                # Only snap heading for stationary spins, not yaw-stop cruise goals.
                if hypot(goal_x - self.pose.x, goal_y - self.pose.y) < 0.5:
                    self.pose.yaw = float(waypoints[0]["yaw"])
            except (KeyError, TypeError, ValueError):
                pass
        return True

    def cancel_navigation(self, timeout_seconds=5):
        self.cancelled += 1
        return True

    def teleop_velocity(self, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.teleop.append((vx, vy, yaw_rate))
        # Advance yaw each tick so outdoor teleop departure-heading unit tests
        # can finish without waiting on wall-clock timeouts.
        if abs(float(yaw_rate)) > 1e-6:
            self.pose = SimpleNamespace(
                x=float(self.pose.x),
                y=float(self.pose.y),
                yaw=float(getattr(self.pose, "yaw", 0.0) or 0.0) + float(yaw_rate),
            )

    def arrival_adjust_velocity(self, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.arrival_adjustments.append((vx, vy, yaw_rate))
        yaw = float(getattr(self.pose, "yaw", 0.0) or 0.0)
        # Accelerate the deterministic fake relative to the 10 Hz production
        # loop while preserving the body->map transform used by the controller.
        dt = 0.5
        self.pose = SimpleNamespace(
            x=float(self.pose.x) + (float(vx) * cos(yaw) - float(vy) * sin(yaw)) * dt,
            y=float(self.pose.y) + (float(vx) * sin(yaw) + float(vy) * cos(yaw)) * dt,
            yaw=yaw + float(yaw_rate) * dt,
        )
        return {"topic": "/cmd_vel_raw"}

    def set_arrival_micro_goal_profile(self, *, enabled, tolerance_m=0.15):
        self.arrival_micro_goal_profiles.append((enabled, tolerance_m))

    def directional_clearance(
        self, vx, vy, travel_distance_m, *, max_scan_age_seconds=0.5
    ):
        return dict(self.arrival_clearance)

    def clear_local_costmap(self, timeout_seconds=1.0):
        self.costmap_clears += 1
        return True

    def execute_obstacle_recovery(self, **kwargs):
        self.obstacle_recoveries.append(dict(kwargs))
        return {
            "success": True,
            "actions": [
                {"action": "backup", "status": "succeeded", "success": True},
                {"action": "drive_on_heading", "status": "succeeded", "success": True},
            ],
        }

    def cancel_obstacle_recovery(self):
        self.obstacle_recovery_cancels += 1
        return True

    def stop_motion(self):
        self.stop_commands += 1

    def is_robot_stopped(self):
        return self.stopped

    def obstacle_monitor_snapshot(self):
        return {
            "requested_planar_speed_mps": 0.20,
            "requested_turn_speed_rps": 0.10,
            "requested_velocity_sample_age_seconds": 0.02,
            "actual_planar_speed_mps": 0.12,
            "actual_turn_speed_rps": 0.08,
            "actual_velocity_sample_age_seconds": 0.01,
        }

    def latest_pose(self):
        # Production pose snapshots carry a changing sampled_at marker.  Give
        # the fake the same contract so consecutive-arrival confirmation tests
        # exercise the real freshness gate rather than a permanently stale
        # synthetic sample.
        values = vars(self.pose).copy()
        values.setdefault("sampled_at", time.monotonic())
        return SimpleNamespace(**values)

    def latest_trusted_pose(self):
        return dict(self.trusted_pose)

    def localization_diagnostics(self):
        return {
            "raw_pose": {"x": 5.0, "y": 6.0, "yaw": 0.8, "frame_id": "map"},
            "quality": {"matching_error": 0.75, "has_converged": False},
            "decision": {"active_source": "unavailable"},
        }

    def set_localization_policy(self, source, phase):
        self.localization_policies.append((source, phase))

    def set_waypoint_profile(
        self,
        *,
        avoid_obstacles,
        require_yaw,
        final_approach=False,
        live=False,
        outdoor=None,
        local_controller="rpp",
    ):
        self.waypoint_profiles.append((avoid_obstacles, require_yaw, final_approach))
        self.live_profiles.append(live)
        self.outdoor_profiles.append(outdoor)

    def set_docking_profile(self, *, final_approach):
        self.docking_profiles.append(final_approach)

    def set_global_controller(self, mode):
        self.global_controllers.append(mode)

    def set_goal_precision(self, *, enabled):
        self.goal_precisions.append(enabled)

    def set_arrival_goal_tolerance(self, tolerance_m, *, yaw_tolerance_rad=0.25):
        self.arrival_goal_tolerances.append(
            (float(tolerance_m), float(yaw_tolerance_rad))
        )

    def localization_decision(self):
        return dict(self.localization_state)

    def progressive_relocalize(self, *, origin, waypoints, wait_seconds=180.0):
        request = {
            "origin": dict(origin or {}),
            "waypoints": [dict(point) for point in waypoints],
            "wait_seconds": float(wait_seconds),
        }
        self.progressive_relocalize_requests.append(request)
        return {"accepted": True, "selected_stage": "mapping_origin_bounded"}


class FakeBlockedNavigation(FakeNavigation):
    def obstacle_monitor_snapshot(self):
        return {
            "requested_forward_speed_mps": 0.2,
            "actual_forward_speed_mps": 0.0,
            "localized_speed_mps": 0.0,
            "front_obstacle_distance_m": 0.45,
            "rear_clearance_m": 1.5,
            "left_clearance_m": 2.4,
            "right_clearance_m": 0.3,
            "stale": False,
            "localization_normal": True,
            "collision_monitor": {
                "state": "STOP",
                "reason": "polygon",
                "zone": "front_stop",
                "motion_scope": "forward",
                "points_inside": 7,
                "sample_age_seconds": 0.02,
            },
        }


class FakeProfileTimeoutNavigation(FakeBlockedNavigation):
    def __init__(self):
        super().__init__()
        self.fail_profile_apply = False
        self.profile_attempts = []

    def apply_navigation_profile(self, **kwargs):
        self.profile_attempts.append(kwargs)
        if self.fail_profile_apply:
            raise ProtocolError(
                "NAV_PROFILE_APPLY_FAILED",
                "/planner_server get_parameters timed out",
            )
        return kwargs


class FakeCollisionLimitedNavigation(FakeNavigation):
    def obstacle_monitor_snapshot(self):
        return {
            "requested_planar_speed_mps": 0.2,
            "actual_planar_speed_mps": 0.04,
            "requested_turn_speed_rps": 0.0,
            "actual_turn_speed_rps": 0.0,
            "front_obstacle_distance_m": None,
        }


class FakeRosbagRecorder:
    def __init__(self):
        self.started = []
        self.stopped = 0
        self.running = False
        self.bag_dir = None

    def start(self, label):
        self.started.append(label)
        self.running = True
        self.bag_dir = f"/home/dogrobot/runtime/nx-edge/data/rosbags/navigation/{label}"
        return {
            "running": True,
            "bag_dir": self.bag_dir,
            "started_at_unix": 100,
            "size_bytes": 0,
        }

    def stop(self):
        self.stopped += 1
        self.running = False
        return {
            "running": False,
            "bag_dir": self.bag_dir or "/home/dogrobot/runtime/nx-edge/data/rosbags/navigation/test",
            "duration_seconds": 12,
            "size_bytes": 1024,
        }

    def status(self):
        return {
            "running": self.running,
            "bag_dir": self.bag_dir or (
                "/home/dogrobot/runtime/nx-edge/data/rosbags/navigation/stale"
                if self.running else None
            ),
        }


class FakeMapSetCoordinator:
    def __init__(self):
        self.activated = []

    def build_segments(self, route):
        return [
            SimpleNamespace(start_index=0, end_index=1, waypoints=route["waypoints"][:1]),
            SimpleNamespace(start_index=1, end_index=3, waypoints=route["waypoints"][1:]),
        ]

    def activate(self, segment):
        self.activated.append(segment)

def ids(batch):
    return [waypoint["waypoint_id"] for waypoint in batch]


def drive_patrol(nav, *, until_ids=None, until_state=None, executor=None, steps=40):
    """Advance patrol goals/spins until a sent batch or task state matches."""
    for _ in range(steps):
        if (
            until_ids is not None
            and nav.sent
            and (ids(nav.sent[-1]) == until_ids or ids(nav.sent[-1]) == until_ids[-1:])
        ):
            return
        if (
            executor is not None
            and until_state is not None
            and executor.context
            and executor.context.state == until_state
        ):
            return
        if nav.result is None:
            raise AssertionError("navigation has no pending result callback")
        last = nav.sent[-1][-1]
        nav.pose = SimpleNamespace(
            x=float(last["x"]),
            y=float(last["y"]),
            yaw=float(last.get("yaw") or getattr(nav.pose, "yaw", 0.0) or 0.0),
        )
        nav.result("succeeded", "", {"missed_waypoints": []})
    raise AssertionError(
        f"did not reach until_ids={until_ids} until_state={until_state}; "
        f"last={ids(nav.sent[-1]) if nav.sent else None} "
        f"state={getattr(getattr(executor, 'context', None), 'state', None)}"
    )


def command(message_type, state_version=0, resume_index=None):
    import json
    from pathlib import Path

    payload = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    payload["message_type"] = message_type
    payload["payload"]["expected_robot_state_version"] = state_version
    if message_type != "task.start":
        payload["payload"]["command"] = (
            {"resume_from_waypoint_index": resume_index}
            if message_type == "task.resume"
            else {"reason": "test"}
        )
    return decode_message(payload)


def test_pause_resume_cancel(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    assert executor.context.state == "running"
    assert nav.stand_requests == 1
    nav.feedback(1, None)
    paused = executor.pause_task(executor.context.task_execution_id)
    assert paused["final_task_state"] == "paused"
    assert nav.cancelled == 1
    # Face the travel direction before the resumed cruise leg.
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=atan2(1.0, 1.0))
    resumed = executor.resume_task(executor.context.task_execution_id, 1)
    assert resumed["final_task_state"] == "running"
    assert ids(nav.sent[-1]) == ["wp-2"]
    cancelled = executor.cancel_task(executor.context.task_execution_id)
    assert cancelled["final_task_state"] == "cancelled"
    store.close()


def test_center_recovery_resumes_the_persisted_pending_waypoint(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    execution_id = executor.context.task_execution_id
    executor.pause_task(execution_id)

    result = executor.recover_task(
        execution_id,
        trigger_reason_code="NAV_STACK_NOT_READY",
        recovery_episode_id="episode-1",
        attempt=1,
    )

    assert result["final_task_state"] == "running"
    assert result["recovery_action"] == "resume_pending_waypoint"
    assert result["recovery_status"] == "recovered"
    assert result["recovery_episode_id"] == "episode-1"
    assert executor.context.current_waypoint_index == 0
    assert nav.cancelled == 2  # pause + recovery must cancel a stale Nav2 goal.
    assert nav.stop_commands >= 2  # pause + recovery confirmation refresh
    store.close()


def test_center_recovery_keeps_stop_gate_and_reports_motion_evidence(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    execution_id = executor.context.task_execution_id
    executor.pause_task(execution_id)
    nav.stopped = False

    try:
        executor.recover_task(
            execution_id,
            trigger_reason_code="NAV_STACK_NOT_READY",
            recovery_episode_id="episode-stop-gate",
            attempt=1,
        )
        assert False, "recovery must not bypass a failed stop confirmation"
    except ProtocolError as exc:
        assert exc.code == "ROBOT_NOT_STOPPED"
        assert exc.details["stop_confirmation"]["navigation_cancelled"] is True
        assert exc.details["stop_confirmation"]["actual_planar_speed_mps"] == 0.12
        assert "actual_planar_speed_mps=0.12" in exc.message
    finally:
        store.close()


def test_delayed_center_recovery_does_not_stop_an_already_resumed_task(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    execution_id = executor.context.task_execution_id
    stops_before = nav.stop_commands
    cancels_before = nav.cancelled

    result = executor.recover_task(
        execution_id,
        trigger_reason_code="LOCALIZATION_LOST",
        recovery_episode_id="episode-delayed",
        attempt=1,
    )

    assert result["recovery_status"] == "already_running"
    assert nav.stop_commands == stops_before
    assert nav.cancelled == cancels_before
    store.close()


def test_center_recovery_does_not_interrupt_accepted_task_start(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.prepare_task_start(command("task.start"))
    execution_id = executor.context.task_execution_id
    cancels_before = nav.cancelled
    stops_before = nav.stop_commands

    result = executor.recover_task(
        execution_id,
        trigger_reason_code="COMMAND_TIMED_OUT",
        recovery_episode_id="episode-start-init",
        attempt=1,
    )

    assert executor.context.state == "accepted"
    assert result["recovery_status"] == "in_progress"
    assert result["recovery_action"] == "task_start_initializing"
    assert result["reason_code"] == "TASK_START_INITIALIZING"
    assert nav.cancelled == cancels_before
    assert nav.stop_commands == stops_before
    store.close()


def test_preleg_heading_is_running_before_delayed_center_recovery(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # Keep the heading action pending under Nav2 so the test can inspect the
    # state between accepted spin and the following cruise goal.
    nav.teleop_velocity = None
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    nav.feedback(1, None)
    execution_id = executor.context.task_execution_id
    executor.pause_task(execution_id)
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=0.0)

    resumed = executor.resume_task(execution_id, 1)

    assert resumed["final_task_state"] == "running"
    assert executor.context.state == "running"
    assert executor._departure_heading_index is not None
    assert any(event[0] == "task.resumed" for event in events)
    stops_before = nav.stop_commands
    cancels_before = nav.cancelled

    recovery = executor.recover_task(
        execution_id,
        trigger_reason_code="LOCALIZATION_LOST",
        recovery_episode_id="episode-delayed-heading",
        attempt=1,
    )

    assert recovery["recovery_status"] == "already_running"
    assert nav.stop_commands == stops_before
    assert nav.cancelled == cancels_before
    executor._clear_departure_heading(cancel_navigation=False)
    store.close()


def test_center_recovery_reapproaches_instead_of_skipping_final_yaw(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    envelope = command("task.start")
    waypoint = envelope.payload["command"]["route_snapshot"]["waypoints"][0]
    waypoint["arrival_policy"] = "stop_and_confirm"
    waypoint["require_yaw"] = True
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        arrival_nav2_reapproach_max_error_m=1.50,
    )
    executor.start_task(envelope)
    executor.context.state = "paused"
    executor.context.current_waypoint_index = 0
    executor.context.post_arrival_waypoint_index = 0
    executor.context.post_arrival_stage = "xy_adjusting"
    executor.context.last_safe_hold_code = "ARRIVAL_POSE_CONVERGENCE_FAILED"
    nav.pose = SimpleNamespace(x=1.2, y=2.0, yaw=1.5)

    result = executor.recover_task(
        executor.context.task_execution_id,
        trigger_reason_code="ARRIVAL_POSE_CONVERGENCE_FAILED",
        recovery_episode_id="episode-arrival",
        attempt=1,
    )

    assert result["recovery_action"] == "nav2_reapproach"
    assert result["recovery_status"] == "in_progress"
    assert result["distance_m"] < 1.50
    assert not any(event[0] == "task.waypoint_degraded" for event in events)
    assert executor.context.state == "running"
    assert len(nav.sent) == 2
    store.close()


def test_waypoint_global_controller_overrides_route_default(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    route = envelope.payload["command"]["route_snapshot"]
    route["global_controller"] = "navfn"
    route["waypoints"][0]["global_controller"] = "theta_star"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)
    executor._apply_navigation_profile(0)
    executor._apply_navigation_profile(1)

    assert nav.global_controllers[-2:] == ["theta_star", "navfn"]
    store.close()


def test_terminal_task_cancel_is_idempotent_without_nav2_wait(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    execution_id = executor.context.task_execution_id
    executor.context.state = "failed"
    executor.context.state_version += 1

    first = executor.cancel_task(execution_id)
    second = executor.cancel_task(execution_id)

    assert first == second
    assert first["final_task_state"] == "failed"
    assert first["already_terminal"] is True
    assert first["cancel_performed"] is False
    assert nav.cancelled == 0
    assert nav.stop_commands == 2
    store.close()


def test_task_starts_from_nearest_waypoint_and_reports_earlier_points_complete(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=2.1, y=3.1)
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )

    executor.start_task(command("task.start"))

    assert executor.context.current_waypoint_index == 1
    assert ids(nav.sent[0]) == ["wp-2"]
    assert events[0][0] == "task.started"
    assert events[0][1]["initial_waypoint_index"] == 1
    assert events[1][0] == "task.progress"
    assert events[1][1]["completed_waypoints"] == 1
    navigation = events[1][1]["navigation_progress"]
    assert navigation["phase"] == "target_dispatched"
    assert navigation["waypoint"]["map_point_number"] == 2
    assert navigation["progress"] == {
        "completed_waypoints": 1,
        "total_waypoints": 3,
        "distance_remaining_m": None,
    }
    assert navigation["modules"]["local_controller"] == "mppi"
    assert navigation["strategy"]["speed_level"] == "micro"
    assert navigation["strategy"]["speed_profile"] == "final"
    assert navigation["strategy"]["configured_linear_limit_mps"] == 0.15
    assert navigation["strategy"]["collision_stop_enabled"] is True
    store.close()


def test_navigation_feedback_switches_each_waypoint_correction_policy(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    waypoints = envelope.payload["command"]["route_snapshot"]["waypoints"]
    waypoints[0]["localization_mode"] = "ndt"
    waypoints[1]["localization_mode"] = "ndt"
    waypoints[2]["localization_mode"] = "ukf"
    envelope.payload["command"]["map"].update({
        "coordinate_mode": "local_only",
        "scene_scope": "indoor",
        "localization_mode": "rtk_ndt",
        "origin_status": "local_only",
    })
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "indoor"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)
    assert ids(nav.sent[-1]) == ["wp-1"]
    drive_patrol(nav, until_ids=["wp-3"], executor=executor)
    assert ids(nav.sent[-1]) == ["wp-3"]

    assert ("ndt", "moving") in nav.localization_policies
    assert ("ndt", "stationary") in nav.localization_policies
    assert ("ukf", "moving") in nav.localization_policies
    store.close()


def test_waypoint_arrival_requires_requested_correction_source_ready(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "policy_source_ready": False,
    }
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is False
    nav.localization_state["policy_source_ready"] = True
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is True
    nav.localization_state["correction_smoothing_active"] = True
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is False
    assert nav.stop_commands > 0
    store.close()


def test_waypoint_correction_transaction_is_idempotent_and_required(tmp_path):
    class TransactionNavigation(FakeNavigation):
        def __init__(self):
            super().__init__()
            self.correction_requests = []

        def control_localization_correction(self, transaction_id, mode, command="start"):
            self.correction_requests.append((transaction_id, mode, command))
            return {"accepted": True, "transaction_id": transaction_id, "status": "waiting_source"}

    store = LocalStore(str(tmp_path / "edge.db"))
    nav = TransactionNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = SimpleNamespace(
        task_execution_id="task-7",
        route_snapshot={"map": {}, "waypoints": []},
    )
    executor._correction_generation = 2

    assert executor._start_waypoint_localization_correction(
        {"localization_mode": "ukf"}, 3
    )
    transaction_id = executor._active_correction_transaction_id
    assert nav.correction_requests == [(transaction_id, "ukf", "start")]
    nav.localization_state = {
        "active_source": "lio_imu",
        "lio_healthy": True,
        "one_shot_correction": {
            "transaction_id": transaction_id,
            "status": "waiting_source",
        },
    }
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is False
    nav.localization_state["one_shot_correction"]["status"] = "completed"
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is True

    executor._cancel_waypoint_localization_correction()
    assert nav.correction_requests[-1] == (transaction_id, "ukf", "cancel")
    store.close()


def test_explicit_no_correction_completion_bypasses_ndt_score_after_stop_recheck(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    cases = (
        ("ndt", "ndt_no_correction_continue"),
        ("rtk", "rtk_no_correction_continue"),
        ("ukf", "ukf_no_correction_sources_meet_gate"),
    )
    for index, (mode, reason) in enumerate(cases):
        transaction_id = f"task-7:waypoint:{index}:correction:1"
        executor._active_correction_transaction_id = transaction_id
        executor._active_correction_mode = mode
        nav.localization_state = {
            "active_source": "lio_imu",
            "lio_healthy": True,
            "ndt_score": 0.99,
            "one_shot_correction": {
                "transaction_id": transaction_id,
                "mode": mode,
                "status": "completed",
                "selected_source": "none",
                "reason": reason,
            },
        }
        assert executor._absolute_localization_ready(timeout_seconds=0.05) is True

    assert nav.stop_commands >= len(cases)
    store.close()


def test_no_correction_completion_still_requires_confirmed_stop(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.stopped = False
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    transaction_id = "task-7:waypoint:0:correction:1"
    executor._active_correction_transaction_id = transaction_id
    executor._active_correction_mode = "ndt"
    nav.localization_state = {
        "active_source": "lio_imu",
        "lio_healthy": True,
        "one_shot_correction": {
            "transaction_id": transaction_id,
            "mode": "ndt",
            "status": "completed",
            "selected_source": "none",
            "reason": "ndt_no_correction_continue",
        },
    }
    assert executor._absolute_localization_ready(timeout_seconds=0.02) is False
    assert nav.stop_commands > 0
    store.close()


def test_no_correction_completion_uses_full_zero_motion_confirmation_window(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    observed_timeouts = []

    def is_stopped(timeout_seconds=None):
        observed_timeouts.append(timeout_seconds)
        # Model the production adapter: a 1 s continuous-zero window cannot
        # be confirmed if the caller gives it a shorter timeout.
        return float(timeout_seconds or 0.0) >= 1.0

    nav.is_robot_stopped = is_stopped
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        stop_confirmation_seconds=1.0,
    )
    transaction_id = "task-7:waypoint:0:correction:1"
    executor._active_correction_transaction_id = transaction_id
    executor._active_correction_mode = "ndt"
    nav.localization_state = {
        "active_source": "lio_imu",
        "lio_healthy": True,
        "one_shot_correction": {
            "transaction_id": transaction_id,
            "mode": "ndt",
            "status": "completed",
            "selected_source": "none",
            "reason": "ndt_no_correction_continue",
        },
    }

    # A short readiness probe must still run a complete safety confirmation.
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is True
    assert observed_timeouts
    assert observed_timeouts[0] >= 1.9
    store.close()


def test_waypoint_correction_reuses_matching_live_transaction(tmp_path):
    class TransactionNavigation(FakeNavigation):
        def __init__(self):
            super().__init__()
            self.correction_requests = []

        def control_localization_correction(self, transaction_id, mode, command="start"):
            self.correction_requests.append((transaction_id, mode, command))
            return {"accepted": False, "status": "busy"}

    store = LocalStore(str(tmp_path / "edge.db"))
    nav = TransactionNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = SimpleNamespace(
        task_execution_id="task-7",
        current_waypoint_index=3,
        route_snapshot={"map": {}, "waypoints": []},
    )
    nav.localization_state = {
        "one_shot_correction": {
            "transaction_id": "task-7:waypoint:3:correction:1",
            "mode": "ndt",
            "status": "waiting_source",
        }
    }
    executor._correction_generation = 2

    assert executor._start_waypoint_localization_correction(
        {"localization_mode": "ndt"}, 3
    )
    assert executor._active_correction_transaction_id.endswith(":correction:1")
    assert executor._correction_generation == 2
    assert nav.correction_requests == []
    store.close()


def test_waypoint_correction_adopts_matching_transaction_after_busy_race(tmp_path):
    class RacingNavigation(FakeNavigation):
        def __init__(self):
            super().__init__()
            self.correction_requests = []
            self.decision_calls = 0

        def localization_decision(self):
            self.decision_calls += 1
            if self.decision_calls == 1:
                return {}
            return {
                "one_shot_correction": {
                    "transaction_id": "task-7:waypoint:3:correction:1",
                    "mode": "ndt",
                    "status": "waiting_source",
                }
            }

        def control_localization_correction(self, transaction_id, mode, command="start"):
            self.correction_requests.append((transaction_id, mode, command))
            return {"accepted": False, "status": "busy"}

    store = LocalStore(str(tmp_path / "edge.db"))
    nav = RacingNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = SimpleNamespace(
        task_execution_id="task-7",
        current_waypoint_index=3,
        route_snapshot={"map": {}, "waypoints": []},
    )
    executor._correction_generation = 2

    assert executor._start_waypoint_localization_correction(
        {"localization_mode": "ndt"}, 3
    )
    assert nav.correction_requests == [
        ("task-7:waypoint:3:correction:2", "ndt", "start")
    ]
    assert executor._active_correction_transaction_id.endswith(":correction:1")
    store.close()


def test_manual_resume_keeps_required_correction_paused_without_redispatch(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    execution_id = executor.context.task_execution_id
    executor.context.state = "paused"
    executor.context.current_waypoint_index = 0
    executor._paused_for_localization = True
    executor._paused_localization_reason = "absolute_required"
    executor._active_correction_transaction_id = (
        f"{execution_id}:waypoint:0:correction:1"
    )
    executor._active_correction_mode = "ndt"
    nav.localization_state = {
        "active_source": "lio_imu",
        "lio_healthy": True,
        "absolute_stable": True,
        "policy_source_ready": False,
        "ndt_score": 1.298,
        "ndt_inlier_fraction": 0.0,
        "one_shot_correction": {
            "transaction_id": executor._active_correction_transaction_id,
            "mode": "ndt",
            "status": "waiting_source",
        },
    }
    sent_before = len(nav.sent)

    result = executor.resume_task(execution_id, 0)

    assert result["final_task_state"] == "paused"
    assert result["resume_blocked"] is True
    assert "NDT" in result["reason_message"]
    assert len(nav.sent) == sent_before
    assert executor._active_correction_transaction_id.endswith(":correction:1")
    assert events[-1][1]["reason_code"] == "ABSOLUTE_LOCALIZATION_REQUIRED"
    thread = executor._absolute_pause_watch_thread
    executor._cancel_absolute_localization_resume_watch()
    if thread is not None:
        thread.join(timeout=1.0)
    store.close()


def test_waiting_ndt_waypoint_correction_requests_stationary_relocalization(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    requests = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        localization_recovery_callback=requests.append,
    )
    executor.context = SimpleNamespace(state="paused")
    executor._paused_for_localization = True
    executor._active_correction_transaction_id = "task-7:waypoint:0:correction:1"
    executor._active_correction_mode = "ndt"
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "lio_healthy": True,
        "policy_source_ready": False,
        "one_shot_correction": {
            "transaction_id": executor._active_correction_transaction_id,
            "mode": "ndt",
            "status": "waiting_source",
            "reason": "waiting_for_eligible_anchor_observation",
        },
    }

    executor._arm_absolute_localization_resume_watch()
    deadline = time.time() + 2.0
    while not requests and time.time() < deadline:
        time.sleep(0.02)
    thread = executor._absolute_pause_watch_thread
    executor._cancel_absolute_localization_resume_watch()
    if thread is not None:
        thread.join(timeout=1.0)

    assert requests == ["ndt_waypoint_correction_unavailable"]
    store.close()


def test_outdoor_waypoint_arrival_accepts_fast_lio_without_rtk_absolute_gate(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type(
        "Ctx",
        (),
        {
            "route_snapshot": {
                "scene_scope": "outdoor",
                "map": {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"},
                "waypoints": [],
            }
        },
    )()
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": False,
        "policy_source_ready": False,
        "rtk_quality": "fixed",
        "rtk_heading_usable": False,
        "lio_motion_anomaly": False,
    }
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is True
    # Active smoothing must block departure so the RTK XY pull-in can finish.
    nav.localization_state["correction_smoothing_active"] = True
    assert executor._outdoor_settle_can_continue(nav.localization_state) is False
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is False
    assert nav.stop_commands > 0
    store.close()


def test_outdoor_waypoint_arrival_pauses_when_smoothing_times_out(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type(
        "Ctx",
        (),
        {
            "route_snapshot": {
                "scene_scope": "outdoor",
                "map": {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"},
                "waypoints": [],
            }
        },
    )()
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": False,
        "rtk_quality": "fixed",
        "rtk_position_good_for_navigation": True,
        "lio_motion_anomaly": False,
        "correction_smoothing_active": True,
    }
    started = time.monotonic()
    assert executor._absolute_localization_ready(timeout_seconds=0.2) is False
    assert time.monotonic() - started < 1.5
    store.close()


def test_arrival_within_tolerance_rejects_outdoor_without_fixed_rtk(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=10.0, y=10.0, yaw=0.0)
    nav.localization_state = {
        "active_source": "lio_imu",
        "rtk_quality": "float",
        "rtk_position_good_for_navigation": False,
        "rtk_x": 10.1,
        "rtk_y": 10.0,
    }
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type(
        "Ctx",
        (),
        {
            "route_snapshot": {
                "scene_scope": "outdoor",
                "map": {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"},
                "waypoints": [{"x": 10.0, "y": 10.0}],
            },
            "task_type": "patrol",
        },
    )()
    assert executor._arrival_within_tolerance({"x": 10.0, "y": 10.0}, 0) is False


def test_localization_sample_age_is_used_as_arrival_freshness_gate(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    assert executor._localization_sample_fresh({"sample_age_seconds": 0.4}) is True
    assert executor._localization_sample_fresh({"sample_age_seconds": 1.6}) is False
    assert executor._localization_sample_fresh({}) is True  # legacy telemetry
    store.close()
    store.close()


def test_arrival_confirmation_requires_consecutive_fresh_samples(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=0.0)
    nav.localization_state = {"sample_age_seconds": 0.2}
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    waypoint = {"x": 1.0, "y": 2.0, "yaw": 0.0, "arrival_policy": "stop_and_confirm"}
    assert executor._arrival_pose_is_stable(waypoint, 0) is True
    nav.pose.sampled_at = "fixed-sample"
    assert executor._arrival_xy_is_stable(waypoint, 0) is False
    assert executor._last_arrival_xy_stability_result.reason == "no_fresh_samples"

    pose_calls = 0

    def advancing_pose():
        nonlocal pose_calls
        sampled_at = f"sample-{pose_calls // 2}"
        pose_calls += 1
        return SimpleNamespace(x=1.0, y=2.0, yaw=0.0, sampled_at=sampled_at)

    nav.latest_pose = advancing_pose
    assert executor._arrival_xy_is_stable(waypoint, 0) is True
    nav.localization_state["sample_age_seconds"] = 2.0
    assert executor._arrival_pose_is_stable(waypoint, 0) is False
    store.close()


def test_missing_arrival_frames_hold_without_reapproach_or_departure_spin(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    executor._arrival_correction_completed_index = 0
    executor._hold_final_pose = lambda **_kwargs: True
    executor._handle_lightweight_arrival = lambda *_args: False

    def missing_frames(*_args):
        executor._last_arrival_xy_stability_result = ArrivalStabilityResult(
            stable=False,
            reason="no_fresh_samples",
            consecutive_frames=0,
            fresh_frames=0,
            within_tolerance_frames=0,
        )
        return False

    executor._arrival_xy_is_stable = missing_frames
    executor._reapproach_rejected_arrival = lambda *_args: (_ for _ in ()).throw(
        AssertionError("missing localization evidence must not re-approach")
    )
    before_sent = len(nav.sent)
    nav.result("succeeded", "", {"missed_waypoints": []})

    assert executor.context.state == "paused"
    assert len(nav.sent) == before_sent
    assert nav.teleop == []
    safe_hold = [event for event in events if event[0] == "task.safe_hold"][-1]
    assert safe_hold[1]["reason_code"] == "ARRIVAL_LOCALIZATION_EVIDENCE_UNAVAILABLE"
    executor.stop()
    store.close()


def test_precision_arrival_requires_tight_xy_and_yaw(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.localization_state = {"sample_age_seconds": 0.1}
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    waypoint = {"x": 1.0, "y": 2.0, "yaw": 0.0, "arrival_policy": "precision"}
    nav.pose = SimpleNamespace(x=1.2, y=2.0, yaw=0.0)
    assert executor._arrival_pose_is_stable(waypoint, 0) is False
    nav.pose = SimpleNamespace(x=1.05, y=2.02, yaw=0.1)
    assert executor._arrival_pose_is_stable(waypoint, 0) is True
    store.close()


def test_outdoor_reverse_skip_requires_rtk_agreement(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # LIO claims to be on route end, but RTK is metres away / not fixed.
    nav.pose = SimpleNamespace(x=3.0, y=4.0, yaw=0.0)
    nav.localization_state = {
        "active_source": "lio_imu",
        "rtk_quality": "float",
        "rtk_position_good_for_navigation": False,
        "rtk_x": 10.0,
        "rtk_y": 10.0,
        "absolute_stable": True,
    }
    envelope = command("task.start")
    envelope.payload["command"]["loop_execution"] = True
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    assert executor.context.route_snapshot["execution_order"] == "reverse_from_route_end"
    # Must NOT skip the colocated end; first cruise target stays at index 0.
    assert executor.context.current_waypoint_index == 0
    assert ids(nav.sent[0]) == ["wp-3"]
    store.close()


def test_outdoor_stationary_policy_preserves_configured_ukf_mode(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type(
        "Ctx",
        (),
        {
            "route_snapshot": {
                "scene_scope": "outdoor",
                "map": {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"},
                "waypoints": [{"x": 1.0, "y": 2.0, "localization_mode": "ukf"}],
            },
            "task_type": "patrol",
        },
    )()
    executor._set_localization_policy(
        {"x": 1.0, "y": 2.0, "localization_mode": "ukf"},
        "stationary",
    )
    assert nav.localization_policies[-1] == ("ukf", "stationary")
    store.close()


def test_moving_policy_enables_online_anchor_only_for_normal_running_leg(tmp_path):
    class PolicyNavigation(FakeNavigation):
        def __init__(self):
            super().__init__()
            self.full_localization_policies = []

        def set_localization_policy(
            self,
            source,
            phase,
            anchor_preference="balanced",
            rtk_primary_allowed=False,
            online_anchor_correction_allowed=False,
        ):
            self.full_localization_policies.append(
                (
                    source,
                    phase,
                    anchor_preference,
                    rtk_primary_allowed,
                    online_anchor_correction_allowed,
                )
            )

    store = LocalStore(str(tmp_path / "edge.db"))
    nav = PolicyNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type("Ctx", (), {"state": "running", "task_type": "patrol"})()

    executor._set_localization_policy({"localization_mode": "ndt"}, "moving")

    assert nav.full_localization_policies[-1] == ("ndt", "moving", "ndt", False, True)

    executor._patrol_final_approach_applied = True
    executor._set_localization_policy({"localization_mode": "ndt"}, "moving")

    assert nav.full_localization_policies[-1] == ("ndt", "moving", "ndt", False, False)
    store.close()


def test_arrival_within_tolerance_rejects_rtk_far_from_click(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=10.0, y=10.0, yaw=0.0)
    nav.localization_state = {
        "active_source": "lio_imu",
        "rtk_quality": "fixed",
        "rtk_position_good_for_navigation": True,
        "rtk_x": 12.5,
        "rtk_y": 10.0,
    }
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type(
        "Ctx",
        (),
        {
            "route_snapshot": {
                "scene_scope": "outdoor",
                "map": {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"},
                "waypoints": [{"x": 10.0, "y": 10.0}],
            },
            "task_type": "patrol",
        },
    )()
    waypoint = {"x": 10.0, "y": 10.0}
    # LIO is on the click, but RTK says the dog is 2.5 m away.
    assert executor._arrival_within_tolerance(waypoint, 2) is False
    nav.localization_state["rtk_x"] = 10.15
    nav.localization_state["rtk_y"] = 10.1
    assert executor._arrival_within_tolerance(waypoint, 2) is True
    # Field start at point 1: LIO on the click, RTK 0.25 m away. Fine 0.20 m
    # used to reject then skip-fine anyway; coarse 0.50 m must accept now.
    nav.localization_state["rtk_x"] = 10.25
    nav.localization_state["rtk_y"] = 10.0
    assert executor._arrival_within_tolerance(waypoint, 0) is True
    nav.localization_state["rtk_x"] = 10.60
    assert executor._arrival_within_tolerance(waypoint, 0) is False
    precision = {"x": 10.0, "y": 10.0, "arrival_policy": "precision"}
    nav.localization_state["rtk_x"] = 10.25
    assert executor._arrival_within_tolerance(precision, 0) is False
    store.close()


def test_outdoor_ukf_and_ndt_arrival_do_not_override_selected_pose_with_raw_rtk(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=10.0, y=10.0, yaw=0.0)
    nav.localization_state = {
        "active_source": "lio_imu",
        "rtk_quality": "fixed",
        "rtk_position_good_for_navigation": True,
        # The raw fixed-RTK observation disagrees with the already selected
        # stationary NDT/UKF correction by 0.34 m. It is diagnostic data, not
        # an extra arrival source for these two policies.
        "rtk_x": 10.34,
        "rtk_y": 10.0,
    }
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type(
        "Ctx",
        (),
        {
            "route_snapshot": {
                "scene_scope": "outdoor",
                "map": {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"},
                "waypoints": [{"x": 10.0, "y": 10.0}],
            },
            "task_type": "patrol",
        },
    )()
    waypoint = {"x": 10.0, "y": 10.0, "arrival_policy": "stop_and_confirm"}

    assert executor._arrival_within_tolerance(
        {**waypoint, "localization_mode": "ukf"}, 0
    ) is True
    assert executor._arrival_within_tolerance(
        {**waypoint, "localization_mode": "ndt"}, 0
    ) is True
    # RTK mode still requires a fixed-RTK click check. Normal patrol uses
    # the 0.50 m coarse circle, so 0.34 m is on-click and 0.60 m is not.
    assert executor._arrival_within_tolerance(
        {**waypoint, "localization_mode": "rtk"}, 0
    ) is True
    nav.localization_state["rtk_x"] = 10.60
    assert executor._arrival_within_tolerance(
        {**waypoint, "localization_mode": "rtk"}, 0
    ) is False
    store.close()


def test_arrival_failure_message_omits_heading_when_waypoint_does_not_require_yaw(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    executor = TaskExecutor(
        store,
        FakeNavigation(),
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    assert executor._arrival_convergence_failure_message(
        {"x": 1.0, "y": 2.0, "arrival_policy": "stop_and_confirm"}, 0
    ) == "航点位置未满足验收条件"
    assert executor._arrival_convergence_failure_message(
        {
            "x": 1.0,
            "y": 2.0,
            "yaw": 0.0,
            "arrival_policy": "stop_and_confirm",
            "require_yaw": True,
        },
        0,
    ) == "航点位置与航向无法同时满足验收条件"
    store.close()


def test_reapproach_rejected_arrival_redispatches_same_waypoint(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # Already face wp-2 so outdoor re-approach cruises immediately.
    nav.pose = SimpleNamespace(x=1.6, y=2.6, yaw=atan2(1.0, 1.0))
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    executor.start_task(envelope)
    before = len(nav.sent)
    assert executor._reapproach_rejected_arrival(1) is True
    assert len(nav.sent) == before + 1
    assert executor.context.current_waypoint_index == 1
    assert executor._arrival_retry_counts[1] == 1
    store.close()


def test_normal_patrol_runs_one_fine_reapproach_before_coarse_fallback(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    results = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"arrival_policy": "stop_and_confirm", "require_yaw": False})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + 0.40,
        y=float(waypoint["y"]),
        yaw=0.0,
    )
    executor._arrival_correction_completed_index = 0
    before = len(nav.sent)

    assert executor._reapproach_rejected_arrival(0) is True
    assert executor._arrival_reapproach_index == 0
    assert executor._arrival_retry_counts[0] == 1
    assert executor.context.arrival_reapproach_waypoint_index == 0
    assert executor.context.arrival_reapproach_attempts == 1
    assert len(nav.sent) == before + 1
    assert nav.arrival_goal_tolerances[-1] == (0.30, 3.14)
    assert executor.context.state == "running"

    # One corrected Nav2 re-approach has now been consumed. Remaining inside
    # the fresh, corrected 0.50 m circle is the only ordinary-stop fallback.
    executor._arrival_correction_completed_index = 0
    assert executor._reapproach_rejected_arrival(0) is True
    assert executor._arrival_reapproach_index is None
    assert executor.context.state == "completed"
    assert results[-1][1] == "succeeded"
    accepted = [event for event in events if event[0] == "task.arrival_degraded_accepted"]
    assert accepted[-1][1]["distance_m"] == 0.4
    executor.stop()
    store.close()


def test_expired_bt_recovery_lease_is_reclaimed_only_after_stop_confirmation(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        bt_recovery_lease_timeout_seconds=0.1,
    )
    lease = executor.acquire_recovery("BT_NAVIGATOR", "navigation_recovery_spin")
    assert lease is not None

    deadline = time.monotonic() + 1.0
    while executor.recovery_snapshot()["owner"] != "NONE" and time.monotonic() < deadline:
        time.sleep(0.02)

    assert executor.recovery_snapshot()["owner"] == "NONE"
    assert nav.stop_commands == 0
    executor.stop()
    store.close()


def test_bt_recovery_lease_timeout_does_not_stop_a_moving_recovery(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.stopped = False
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        bt_recovery_lease_timeout_seconds=0.05,
    )
    lease = executor.acquire_recovery("BT_NAVIGATOR", "navigation_recovery_backup")
    assert lease is not None

    time.sleep(0.2)
    assert executor.recovery_snapshot()["owner"] == "BT_NAVIGATOR"
    assert nav.stop_commands == 0

    nav.stopped = True
    deadline = time.monotonic() + 1.0
    while executor.recovery_snapshot()["owner"] != "NONE" and time.monotonic() < deadline:
        time.sleep(0.02)

    assert executor.recovery_snapshot()["owner"] == "NONE"
    assert nav.stop_commands == 0
    executor.stop()
    store.close()


def test_navigation_cancel_reclaims_confirmed_stopped_bt_recovery_lease(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    lease = executor.acquire_recovery("BT_NAVIGATOR", "navigation_recovery_spin")
    assert lease is not None

    assert executor._cancel_active_navigation(timeout_seconds=0.1) is True
    assert executor.recovery_snapshot()["owner"] == "NONE"
    assert nav.cancelled == 1
    assert nav.stop_commands >= 1
    executor.stop()
    store.close()


def test_initial_xy_failure_never_starts_cmd_vel_adjustment(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(x=float(waypoint["x"]) + 0.40, y=float(waypoint["y"]), yaw=0.0)

    # The adjustment entry point itself enforces the state-machine boundary:
    # before verified XY and final yaw, the caller must re-approach with Nav2.
    assert executor._start_arrival_adjustment(waypoint, 0) is False
    assert not any(
        abs(vx) > 0.0 or abs(vy) > 0.0 or abs(yaw_rate) > 0.0
        for vx, vy, yaw_rate in nav.arrival_adjustments
    )
    store.close()


def test_reapproach_above_one_point_five_metres_requires_localization_recovery(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    recovery_requests = []
    envelope = command("task.start")
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        arrival_nav2_reapproach_max_error_m=1.50,
        localization_recovery_callback=lambda reason: recovery_requests.append(reason),
    )
    executor.start_task(envelope)
    waypoint = envelope.payload["command"]["route_snapshot"]["waypoints"][0]
    nav.pose = SimpleNamespace(x=float(waypoint["x"]) + 1.51, y=float(waypoint["y"]), yaw=0.0)
    before = len(nav.sent)

    assert executor._reapproach_rejected_arrival(0) is True
    assert len(nav.sent) == before
    assert executor.context.state == "paused"
    assert recovery_requests == ["arrival_precision_recovery"]
    safe_hold = [event for event in events if event[0] == "task.safe_hold"][-1]
    assert safe_hold[1]["reason_code"] == "ARRIVAL_XY_UNVERIFIED"
    executor.stop()
    store.close()


def test_fine_reapproach_outside_coarse_circle_recovers_then_dispatches_once(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    recovery_requests = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"arrival_policy": "stop_and_confirm", "require_yaw": False})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        localization_recovery_callback=lambda reason: recovery_requests.append(reason),
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + 0.80,
        y=float(waypoint["y"]),
        yaw=0.0,
    )
    executor._arrival_correction_completed_index = 0
    executor._arrival_retry_counts[0] = executor.arrival_reapproach_max_attempts

    assert executor._reapproach_rejected_arrival(0) is True
    assert executor.context.state == "paused"
    assert executor.context.last_safe_hold_code == "ARRIVAL_REAPPROACH_COARSE_EXCEEDED"
    assert executor.context.post_arrival_stage == "coarse_recovery_pending"
    assert recovery_requests == []

    recovery = executor.recover_task(
        envelope.payload["task_execution_id"],
        trigger_reason_code="ARRIVAL_REAPPROACH_COARSE_EXCEEDED",
        recovery_episode_id="fine-outside-coarse",
        attempt=1,
    )
    assert recovery["recovery_action"] == "precision_localization_recovery"
    assert recovery["recovery_status"] == "in_progress"
    assert recovery_requests == ["arrival_precision_recovery"]
    assert executor.context.last_safe_hold_code == "ARRIVAL_REAPPROACH_COARSE_EXCEEDED"

    before = len(nav.sent)
    executor.on_localization_recovered()
    assert len(nav.sent) == before + 1
    assert nav.arrival_goal_tolerances[-1] == (0.50, 3.14)
    assert executor.context.state == "running"
    assert executor.context.post_arrival_stage == "coarse_reapproach"
    assert executor.context.arrival_coarse_fallback_accepted is True
    assert executor._arrival_reapproach_index is None

    def outside_coarse(*_args):
        executor._last_arrival_xy_stability_result = ArrivalStabilityResult(
            stable=False,
            reason="outside_tolerance",
            consecutive_frames=0,
            fresh_frames=3,
            within_tolerance_frames=0,
        )
        return False

    executor._arrival_xy_is_stable = outside_coarse
    nav.result("succeeded", "", {"missed_waypoints": []})
    assert executor.context.state == "paused"
    assert executor.context.last_safe_hold_code == "ARRIVAL_COARSE_REAPPROACH_EXHAUSTED"

    exhausted = executor.recover_task(
        envelope.payload["task_execution_id"],
        trigger_reason_code="ARRIVAL_COARSE_REAPPROACH_EXHAUSTED",
        recovery_episode_id="fine-outside-coarse",
        attempt=2,
    )
    assert exhausted["recovery_status"] == "non_retryable"
    assert len(nav.sent) == before + 1
    executor.stop()
    store.close()


def test_normal_waypoint_accepts_fresh_coarse_pose_after_one_reapproach(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    results = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"arrival_policy": "stop_and_confirm", "require_yaw": False})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + 0.40,
        y=float(waypoint["y"]),
        yaw=0.0,
    )
    executor._arrival_correction_completed_index = 0
    executor._arrival_retry_counts[0] = executor.arrival_reapproach_max_attempts

    assert executor._reapproach_rejected_arrival(0) is True
    assert executor.context.state == "completed"
    assert results[-1][1] == "succeeded"
    accepted = [event for event in events if event[0] == "task.arrival_degraded_accepted"]
    assert accepted[-1][1]["distance_m"] == 0.4
    confirmed = [
        event
        for event in events
        if event[0] == "task.progress" and event[1].get("milestone") == "arrival_confirmed"
    ][-1]
    assert confirmed[1]["coarse_completed"] is True
    executor.stop()
    store.close()


def test_precision_waypoint_does_not_relax_after_one_reapproach(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"arrival_policy": "precision", "require_yaw": False})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + 0.40,
        y=float(waypoint["y"]),
        yaw=0.0,
    )
    executor._arrival_retry_counts[0] = executor.arrival_reapproach_max_attempts

    assert executor._reapproach_rejected_arrival(0) is True
    assert executor.context.state == "paused"
    assert not any(event[0] == "task.arrival_degraded_accepted" for event in events)
    safe_hold = [event for event in events if event[0] == "task.safe_hold"][-1]
    assert safe_hold[1]["reason_code"] == "PHYSICAL_REAPPROACH_EXHAUSTED"
    executor.stop()
    store.close()


def test_reapproach_after_precision_recovery_allows_far_corrected_pose(tmp_path):
    """The 1.50 m guard gates localization recovery, not the recovered Nav2 leg."""
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        arrival_nav2_reapproach_max_error_m=1.50,
    )
    executor.start_task(envelope)
    waypoint = envelope.payload["command"]["route_snapshot"]["waypoints"][0]
    # Face the original click so the assertion observes the re-approach goal
    # itself rather than the optional pre-leg yaw-alignment phase.
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + 1.51,
        y=float(waypoint["y"]),
        yaw=3.141592653589793,
    )
    before = len(nav.sent)

    # Precision localization has declared the corrected pose stable.  Nav2
    # must now plan from that pose back to the original waypoint, even though
    # it is still farther than the recovery trigger threshold.
    assert executor._reapproach_rejected_arrival(0, localization_recovered=True) is True
    assert len(nav.sent) == before + 1
    assert executor.context.state == "running"
    assert executor.context.current_waypoint_index == 0
    assert executor._arrival_retry_counts[0] == 1
    executor.stop()
    store.close()


def _set_middle_waypoint_as_active(executor, nav, *, distance_m, sample_age=0.1):
    waypoint = executor.context.route_snapshot["waypoints"][1]
    executor.context.current_waypoint_index = 1
    executor._goal_offset = 1
    executor._dispatched_count = 1
    executor._last_target_index = 1
    executor._last_reached_index = 0
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + float(distance_m),
        y=float(waypoint["y"]),
        yaw=3.141592653589793,
    )
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "sample_age_seconds": sample_age,
    }
    return waypoint


def test_stop_and_confirm_middle_waypoint_never_uses_lightweight_arrival(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    waypoint = _set_middle_waypoint_as_active(executor, nav, distance_m=0.25)
    assert executor._waypoint_requires_localization_correction(waypoint, 1) is True
    assert executor._handle_lightweight_arrival(1, waypoint) is False
    assert not [event for event in events if event[0] == "task.arrival_confirmed"]
    executor.stop()
    store.close()


def test_pass_through_middle_waypoint_is_the_only_correction_opt_out(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    waypoint = _set_middle_waypoint_as_active(executor, nav, distance_m=0.40)
    waypoint["arrival_policy"] = "pass_through"

    assert executor._waypoint_requires_localization_correction(waypoint, 1) is False
    executor.stop()
    store.close()


def test_every_non_pass_through_waypoint_requires_full_correction(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    points = executor.context.route_snapshot["waypoints"]

    assert executor._waypoint_requires_localization_correction(points[0], 0) is True
    assert executor._waypoint_requires_localization_correction(points[-1], 2) is True
    assert executor._waypoint_requires_localization_correction(points[1], 1) is True
    variants = [
        {"force_localization_correction": True},
        {"require_yaw": True},
        {"dwell_seconds": 1.0},
        {"actions": [{"type": "capture"}]},
        {"speech_template_id": 7, "speech_mode": "non_blocking"},
        {"arrival_policy": "precision"},
        {"arrival_policy": "dock"},
    ]
    for changes in variants:
        waypoint = {**points[1], **changes}
        assert executor._waypoint_requires_localization_correction(waypoint, 1) is True
    pass_through = {**points[1], "arrival_policy": "pass_through"}
    assert executor._waypoint_requires_localization_correction(pass_through, 1) is False
    executor.stop()
    store.close()


def test_initial_and_reapproach_nav2_tolerances_include_docking_precision(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    assert nav.arrival_goal_tolerances[0] == (0.50, 3.14)

    executor._arrival_reapproach_index = 1
    assert executor._navigation_arrival_tolerance(1) == 0.30
    executor._set_navigation_arrival_tolerance(1)
    assert nav.arrival_goal_tolerances[-1] == (0.30, 3.14)

    precision = executor.context.route_snapshot["waypoints"][1]
    precision["arrival_policy"] = "precision"
    precision["require_yaw"] = False
    executor._arrival_reapproach_index = 1
    assert executor._navigation_arrival_tolerance(1) == 0.15
    executor._set_navigation_arrival_tolerance(1)
    assert nav.arrival_goal_tolerances[-1] == (0.15, 3.14)

    final = executor.context.route_snapshot["waypoints"][-1]
    final["arrival_policy"] = "dock"
    final["require_yaw"] = False
    executor.context.docking = {"enabled": True, "final_waypoint_index": 2}
    executor._arrival_reapproach_index = 2
    assert executor._navigation_arrival_tolerance(2) == 0.08
    executor._set_navigation_arrival_tolerance(2)
    assert nav.arrival_goal_tolerances[-1] == (0.08, 3.14)

    final["require_yaw"] = True
    executor._arrival_reapproach_index = None
    executor._set_navigation_arrival_tolerance(2)
    assert nav.arrival_goal_tolerances[-1] == (0.50, 0.0872665)
    executor.stop()
    store.close()


def test_outdoor_hold_final_pose_allows_stop_confirmation_window(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.stopped = False

    def is_stopped(timeout_seconds=None):
        time.sleep(min(0.05, float(timeout_seconds or 0.05)))
        return False

    nav.is_robot_stopped = is_stopped
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type(
        "Ctx",
        (),
        {
            "route_snapshot": {
                "scene_scope": "outdoor",
                "map": {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"},
                "waypoints": [{"x": 0.0, "y": 0.0}],
            }
        },
    )()
    started = time.monotonic()
    assert executor._hold_final_pose() is False
    # The timeout must exceed the 1s continuous-zero confirmation window.
    assert 1.8 <= time.monotonic() - started < 2.5
    store.close()


def test_hold_final_pose_reports_distinct_nav2_zero_confirmation_stages(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    stages = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    assert executor._hold_final_pose(
        timeout_seconds=1.0,
        stage_callback=lambda stage, details: stages.append((stage, details)),
    ) is True

    assert [stage for stage, _details in stages] == [
        "nav2_stopping",
        "zero_confirming",
        "zero_confirmed",
    ]
    assert stages[1][1]["stop_confirmation_seconds"] == 1.0
    assert stages[2][1]["confirmation_source"] == "collision_monitor"
    executor.stop()
    store.close()


def test_outdoor_waypoint_arrival_waits_for_rtk_drift_correction(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type(
        "Ctx",
        (),
        {
            "route_snapshot": {
                "scene_scope": "outdoor",
                "map": {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"},
                "waypoints": [],
            }
        },
    )()
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": False,
        "rtk_quality": "fixed",
        "rtk_position_good_for_navigation": True,
        "lio_motion_anomaly": False,
        "rtk_drift": {"xy_m": 1.2, "threshold_xy_m": 0.30, "decision": "rtk_stable_pending"},
        "rtk_drift_decision": "rtk_stable_pending",
    }
    assert executor._needs_stationary_correction(nav.localization_state) is True
    # Settle still waits while the gate is pending, but outdoor timeout must not
    # pause forever when fixed RTK + LIO remain usable (field hang at point 4).
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is True
    nav.localization_state["rtk_drift"] = {
        "xy_m": 0.05,
        "threshold_xy_m": 0.30,
        "decision": "corrected_once",
    }
    nav.localization_state["rtk_drift_decision"] = "corrected_once"
    assert executor._needs_stationary_correction(nav.localization_state) is False
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is True
    # Accept/pending names with residual already inside the gate must not block.
    nav.localization_state["rtk_drift_decision"] = "rtk_stable_accept"
    nav.localization_state["rtk_drift"]["decision"] = "rtk_stable_accept"
    assert executor._needs_stationary_correction(nav.localization_state) is False
    # Without usable RTK/LIO, outdoor settle timeout still pauses.
    nav.localization_state = {
        "active_source": "unavailable",
        "absolute_stable": False,
        "rtk_quality": "float",
        "rtk_position_good_for_navigation": False,
        "lio_motion_anomaly": False,
        "correction_smoothing_active": False,
        "rtk_drift": {"xy_m": 1.2, "threshold_xy_m": 0.30, "decision": "rtk_stable_pending"},
        "rtk_drift_decision": "rtk_stable_pending",
    }
    assert executor._outdoor_settle_can_continue(nav.localization_state) is False
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is False
    store.close()


def test_startup_uses_ndt_before_fixed_rtk_secondary_correction(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.localization_state = {
        "active_source": "unavailable",
        "absolute_stable": False,
        "rtk_good_for_navigation": True,
        "rtk_usable": True,
        "rtk_quality": "fixed",
        "rtk_heading_usable": True,
    }
    nav.rtk_calls = 0
    nav.relocalize_calls = []

    def set_initial_pose_from_rtk(wait_seconds=30.0):
        nav.rtk_calls += 1
        nav.localization_state["active_source"] = "lio_imu"
        nav.localization_state["lio_healthy"] = True
        nav.localization_state["lio_anchored"] = True
        nav.localization_state["absolute_stable"] = True
        return {"source": "rtk_fixed"}

    nav.startup_handoff_calls = 0

    def accept_startup_trusted_pose():
        nav.startup_handoff_calls += 1
        assert nav.localization_state["active_source"] == "lio_imu"
        assert nav.localization_state["lio_healthy"] is True
        assert nav.localization_state["lio_anchored"] is True
        assert nav.localization_state["absolute_stable"] is True

    original_progressive = nav.progressive_relocalize

    def progressive_relocalize(**kwargs):
        result = original_progressive(**kwargs)
        nav.localization_state.update({
            "active_source": "lio_imu",
            "lio_healthy": True,
            "lio_anchored": True,
            "absolute_stable": True,
        })
        return result

    nav.set_initial_pose_from_rtk = set_initial_pose_from_rtk
    nav.accept_startup_trusted_pose = accept_startup_trusted_pose
    nav.progressive_relocalize = progressive_relocalize
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["map"] = {
        "map_id": "outdoor-a", "map_version": "v1", "coordinate_mode": "rtk_fixed",
        "scene_scope": "outdoor",
    }
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    executor.prepare_task_start(envelope)
    executor.initialize_before_navigation()
    assert nav.rtk_calls == 0
    assert nav.startup_handoff_calls == 1
    assert len(nav.progressive_relocalize_requests) == 1
    store.close()


def test_indoor_startup_keeps_an_already_stable_fixed_rtk_pose(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.localization_state = {
        "active_source": "rtk_imu",
        "absolute_stable": True,
        "rtk_good_for_navigation": True,
        "rtk_usable": True,
        "rtk_quality": "fixed",
        "rtk_heading_usable": True,
        "policy_source_ready": False,
    }
    nav.rtk_calls = 0
    nav.set_initial_pose_from_rtk = lambda wait_seconds=30.0: setattr(nav, "rtk_calls", nav.rtk_calls + 1)
    nav.accept_startup_trusted_pose = lambda: (_ for _ in ()).throw(
        AssertionError("stable indoor startup must not require a fresh FAST-LIO handoff")
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.prepare_task_start(command("task.start"))
    executor.initialize_before_navigation()
    assert nav.rtk_calls == 0
    assert executor._absolute_localization_ready(timeout_seconds=0.01) is True
    store.close()


def test_new_task_invalidates_disk_and_memory_trusted_pose_before_initialization(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.invalidated = 0

    def invalidate_last_trusted_pose():
        nav.invalidated += 1
        nav.trusted_pose = None

    nav.invalidate_last_trusted_pose = invalidate_last_trusted_pose
    envelope = command("task.start")
    route = envelope.payload["command"]["route_snapshot"]
    route["map"] = {"map_id": "site-a", "map_version": "v1"}
    store.save_last_trusted_pose("site-a", "v1", {"x": 80.0, "y": -40.0})
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.prepare_task_start(envelope)

    assert nav.invalidated == 1
    assert nav.trusted_pose is None
    assert store.load_last_trusted_pose("site-a", "v1") is None
    store.close()


def test_dock_arrival_policy_requires_docking_context(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["waypoints"][0]["arrival_policy"] = "dock"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    try:
        executor.prepare_task_start(envelope)
    except Exception as exc:
        assert getattr(exc, "code", "") == "DOCKING_CONTEXT_REQUIRED"
    else:
        raise AssertionError("dock policy must not start without docking context")
    store.close()


def test_startup_falls_back_to_ndt_when_rtk_is_poor(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.localization_state = {"active_source": "unavailable", "absolute_stable": False}
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["map"] = {
        "map_id": "outdoor-a", "map_version": "v1", "coordinate_mode": "rtk_fixed",
        "scene_scope": "outdoor",
    }
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    executor.prepare_task_start(envelope)
    executor.initialize_before_navigation()
    assert len(nav.progressive_relocalize_requests) == 1
    request = nav.progressive_relocalize_requests[0]
    assert request["origin"] == {}
    assert request["waypoints"][0]["x"] == 1.0
    store.close()


def test_startup_skips_reseed_when_indoor_pose_already_stable(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": True,
    }
    nav.relocalize_calls = []

    def active_relocalize(seed):
        nav.relocalize_calls.append(dict(seed))
        return {"accepted": True}

    nav.active_relocalize = active_relocalize
    nav.localization_diagnostics = lambda: {
        "raw_pose": {"localization_status": "normal", "x": 1.0, "y": 2.0},
        "quality": {"has_converged": True, "matching_error": 0.12},
        "decision": dict(nav.localization_state),
    }
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.prepare_task_start(command("task.start"))
    executor.initialize_before_navigation()
    assert nav.relocalize_calls == []
    store.close()


def test_startup_reseeds_when_indoor_status_is_not_normal(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": True,
    }
    nav.relocalize_calls = []

    def active_relocalize(seed):
        nav.relocalize_calls.append(dict(seed))
        return {"accepted": True}

    nav.active_relocalize = active_relocalize
    nav.localization_diagnostics = lambda: {
        "raw_pose": {"localization_status": "lost", "x": 1.0, "y": 2.0},
        "quality": {},
        "decision": dict(nav.localization_state),
    }
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.prepare_task_start(command("task.start"))
    executor.initialize_before_navigation()
    assert len(nav.progressive_relocalize_requests) == 1
    store.close()


def test_outdoor_poor_rtk_still_reseeds_even_when_lio_looks_stable(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "rtk_quality": "float",
        "rtk_usable": False,
    }
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.prepare_task_start(envelope)
    executor.initialize_before_navigation()
    assert len(nav.progressive_relocalize_requests) == 1
    store.close()


def test_outdoor_fixed_rtk_stability_failure_uses_mapping_origin_progressive_search(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.localization_state = {
        "active_source": "unavailable",
        "absolute_stable": False,
        "rtk_usable": True,
        "rtk_quality": "fixed",
        "rtk_heading_usable": True,
    }
    nav.rtk_calls = 0

    def set_initial_pose_from_rtk(wait_seconds=30.0):
        nav.rtk_calls += 1
        raise ProtocolError(
            "RTK_FIXED_NOT_STABLE",
            "only 2/3 consecutive fixed RTK samples arrived before timeout",
        )

    nav.set_initial_pose_from_rtk = set_initial_pose_from_rtk
    mapping = SimpleNamespace(
        mapping_start_pose=lambda: {"x": 8.0, "y": 9.0, "z": 0.0, "yaw": 0.4}
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        map_activation_adapter=mapping,
    )
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["map"] = {
        "map_id": "outdoor-a", "map_version": "v1", "coordinate_mode": "rtk_fixed",
        "scene_scope": "outdoor",
    }
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    executor.prepare_task_start(envelope)
    executor.initialize_before_navigation()

    assert nav.rtk_calls == 0
    assert len(nav.progressive_relocalize_requests) == 1
    request = nav.progressive_relocalize_requests[0]
    assert request["origin"]["x"] == 8.0
    assert request["waypoints"][0]["x"] == 1.0
    assert request["wait_seconds"] == 180.0
    store.close()


def test_outdoor_rtk_heading_conflict_uses_mapping_origin_progressive_search(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "rtk_usable": True,
        "rtk_quality": "fixed",
        "rtk_heading_usable": True,
    }
    nav.rtk_calls = 0

    def set_initial_pose_from_rtk(wait_seconds=30.0):
        nav.rtk_calls += 1
        raise ProtocolError(
            "RTK_HEADING_CONFLICTS_WITH_LIDAR",
            "live lidar heading disagrees with fixed RTK by 170.0deg at the same place",
        )

    nav.set_initial_pose_from_rtk = set_initial_pose_from_rtk
    mapping = SimpleNamespace(
        mapping_start_pose=lambda: {"x": 8.0, "y": 9.0, "z": 0.0, "yaw": 0.4}
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        map_activation_adapter=mapping,
    )
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["map"] = {
        "map_id": "outdoor-a", "map_version": "v1", "coordinate_mode": "rtk_fixed",
        "scene_scope": "outdoor",
    }
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    executor.prepare_task_start(envelope)
    executor.initialize_before_navigation()

    assert nav.rtk_calls == 0
    assert len(nav.progressive_relocalize_requests) == 1
    store.close()


def test_outdoor_ndt_handoff_failure_keeps_task_stopped(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.localization_state = {
        "active_source": "unavailable",
        "absolute_stable": False,
        "rtk_usable": True,
        "rtk_quality": "fixed",
        "rtk_heading_usable": True,
    }

    def progressive_relocalize(*, origin, waypoints, wait_seconds=180.0):
        nav.progressive_relocalize_requests.append({
            "origin": dict(origin or {}),
            "waypoints": [dict(point) for point in waypoints],
            "wait_seconds": float(wait_seconds),
        })
        raise ProtocolError(
            "LIO_HANDOFF_TIMEOUT",
            "NDT pose was accepted but FAST-LIO handoff did not become ready",
        )

    nav.progressive_relocalize = progressive_relocalize
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["map"] = {
        "map_id": "outdoor-a",
        "map_version": "v1",
        "coordinate_mode": "rtk_fixed",
        "scene_scope": "outdoor",
    }
    executor.prepare_task_start(envelope)

    try:
        executor.initialize_before_navigation()
    except ProtocolError as exc:
        assert exc.code == "LIO_HANDOFF_TIMEOUT"
    else:
        raise AssertionError("FAST-LIO handoff failure must keep the task stopped")

    assert len(nav.progressive_relocalize_requests) == 1
    store.close()


def test_round_trip_starts_from_first_copy_when_start_and_end_overlap(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.12, y=2.04)
    envelope = command("task.start")
    waypoints = envelope.payload["command"]["route_snapshot"]["waypoints"]
    waypoints.extend(
        [
            {**waypoints[1], "waypoint_id": "wp-4", "sequence": 3, "name": "B-return"},
            {
                "waypoint_id": "wp-5",
                "sequence": 4,
                "name": "A-return",
                "x": 1.15,
                "y": 2.05,
                "yaw": 3.14,
                "dwell_seconds": 0,
                "actions": [],
            },
        ]
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)

    assert executor.context.current_waypoint_index == 0
    assert ids(nav.sent[0])[0] == "wp-1"
    assert "wp-1" in ids(nav.sent[0])
    store.close()


def test_round_trip_starts_from_first_when_end_click_is_more_than_one_meter_off(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.32, y=1.32, yaw=atan2(0.0698 - 1.32, 1.5705 - 1.32))
    envelope = command("task.start")
    waypoints = envelope.payload["command"]["route_snapshot"]["waypoints"]
    waypoints[:] = [
        {"waypoint_id": "wp-1", "sequence": 0, "name": "点1", "x": 1.5705, "y": 0.0698, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-2", "sequence": 1, "name": "点2", "x": 6.1205, "y": 0.0198, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-3", "sequence": 2, "name": "点3", "x": 10.7705, "y": -0.1302, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-4", "sequence": 3, "name": "点4", "x": 10.7705, "y": 1.3198, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-5", "sequence": 4, "name": "点5", "x": 5.9705, "y": 1.1698, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-6", "sequence": 5, "name": "点6", "x": 1.3205, "y": 1.3198, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
    ]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)

    assert executor.context.current_waypoint_index == 0
    assert ids(nav.sent[0])[0] == "wp-1"
    store.close()


def test_round_trip_resume_does_not_skip_outbound_legs_to_return_copy(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.12, y=2.04)
    envelope = command("task.start")
    waypoints = envelope.payload["command"]["route_snapshot"]["waypoints"]
    waypoints.extend(
        [
            {**waypoints[1], "waypoint_id": "wp-4", "sequence": 3, "name": "B-return"},
            {
                "waypoint_id": "wp-5",
                "sequence": 4,
                "name": "A-return",
                "x": 1.15,
                "y": 2.05,
                "yaw": 3.14,
                "dwell_seconds": 0,
                "actions": [],
            },
        ]
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)
    assert ids(nav.sent[0])[0] == "wp-1"
    nav.feedback(0, None)
    assert executor.context.current_waypoint_index == 0

    # Robot is nearer the return copy of B (wp-4) than the outbound B (wp-2).
    nav.pose = SimpleNamespace(x=2.05, y=3.02)
    executor.on_localization_lost()
    executor.on_localization_recovered()

    assert executor.context.current_waypoint_index == 0
    assert ids(nav.sent[-1])[0] == "wp-1"
    store.close()


def test_round_trip_keeps_running_after_outbound_through_poses_succeed(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.12, y=2.04)
    envelope = command("task.start")
    waypoints = envelope.payload["command"]["route_snapshot"]["waypoints"]
    waypoints.extend(
        [
            {**waypoints[1], "waypoint_id": "wp-4", "sequence": 3, "name": "B-return"},
            {
                "waypoint_id": "wp-5",
                "sequence": 4,
                "name": "A-return",
                "x": 1.15,
                "y": 2.05,
                "yaw": 3.14,
                "dwell_seconds": 0,
                "actions": [],
            },
        ]
    )
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)
    assert ids(nav.sent[0])[0] == "wp-1"

    drive_patrol(nav, until_ids=["wp-4", "wp-5"], executor=executor)

    assert executor.context.state == "running"
    assert "task.completed" not in [event[0] for event in events]
    assert ids(nav.sent[-1])[0] == "wp-5"
    store.close()


def test_loop_execution_reverses_when_uniquely_at_route_end(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # Already standing on the route end: reverse, then skip that colocated start.
    nav.pose = SimpleNamespace(x=3.0, y=4.0, yaw=atan2(-1.0, -1.0))
    envelope = command("task.start")
    envelope.payload["command"]["loop_execution"] = True
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)

    assert executor.context.route_snapshot["execution_order"] == "reverse_from_route_end"
    assert [waypoint["waypoint_id"] for waypoint in executor.context.route_snapshot["waypoints"]] == [
        "wp-3",
        "wp-2",
        "wp-1",
    ]
    assert executor.context.current_waypoint_index == 1
    assert ids(nav.sent[0]) == ["wp-2"]
    store.close()


def test_loop_round_dispatches_first_leg_without_heading_worker(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["loop_execution"] = True
    envelope.payload["command"]["loop_total"] = 2

    executor.start_task(envelope)
    drive_patrol(nav, until_ids=["wp-3"], executor=executor)
    nav.pose = SimpleNamespace(x=3.0, y=4.0, yaw=0.0)
    assert nav.result is not None
    nav.result("succeeded", "", {"missed_waypoints": []})
    # Completing the first round must immediately create the next Nav2 goal;
    # it must not wait for an optional departure-heading thread.
    assert executor.context.round_number == 2
    assert ids(nav.sent[-1]) == ["wp-2"]
    assert executor._departure_heading_thread is None
    executor.stop()
    store.close()


def test_center_loop_round_uses_explicit_direction_at_boundary(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # The previous round ended at the last click.  A slightly stale pose is
    # closer to the preceding click; explicit round direction must still start
    # the return leg from the terminal anchor, not redispatch that click.
    nav.pose = SimpleNamespace(x=2.7, y=4.0, yaw=0.0)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    body = envelope.payload["command"]
    body["loop_execution"] = True
    body["loop_total"] = 1
    body["round_number"] = 2
    body["loop_direction"] = "reverse"
    executor.start_task(envelope)
    _await_departure_heading(executor)
    assert executor.context.route_snapshot["execution_order"] == "reverse_from_route_end"
    assert executor.context.current_waypoint_index == 1
    assert ids(nav.sent[0]) == ["wp-2"]
    store.close()


def test_loop_round_skips_confirmed_repeated_anchor_without_arrival_side_effects(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    command_body = envelope.payload["command"]
    command_body["loop_execution"] = True
    command_body["loop_total"] = 2
    waypoints = command_body["route_snapshot"]["waypoints"]
    waypoints[-1] = {
        **waypoints[-1],
        "waypoint_id": "wp-return-anchor",
        "x": waypoints[0]["x"],
        "y": waypoints[0]["y"],
        "yaw": waypoints[0]["yaw"],
    }

    executor.start_task(envelope)
    drive_patrol(nav, until_ids=["wp-return-anchor"], executor=executor)
    nav.pose = SimpleNamespace(
        x=float(waypoints[0]["x"]), y=float(waypoints[0]["y"]), yaw=float(waypoints[0]["yaw"])
    )
    nav.result("succeeded", "", {"missed_waypoints": []})

    assert executor.context.round_number == 2
    assert executor.context.current_waypoint_index == 1
    assert ids(nav.sent[-1]) == ["wp-2"]
    milestones = [
        event[1]["waypoint"]["waypoint_id"]
        for event in events
        if event[0] == "task.progress" and event[1].get("milestone") == "waypoint_reached"
    ]
    assert milestones.count("wp-1") == 1
    assert any(event[0] == "task.loop_anchor_confirmed" for event in events)
    executor.stop()
    store.close()


def test_reverse_execution_reports_each_actual_waypoint_identity(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=3.0, y=4.0, yaw=atan2(-1.0, -1.0))
    envelope = command("task.start")
    envelope.payload["command"]["loop_execution"] = True
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)
    # Colocated start (wp-3) is marked reached immediately; cruise begins at wp-2.
    drive_patrol(nav, until_state="completed", executor=executor)

    milestones = [
        (event[1]["milestone"], event[1]["waypoint"]["waypoint_id"])
        for event in events
        if event[0] == "task.progress" and event[1].get("milestone") == "waypoint_reached"
    ]
    assert milestones == [
        ("waypoint_reached", "wp-3"),
        ("waypoint_reached", "wp-2"),
        ("waypoint_reached", "wp-1"),
    ]
    assert executor.context.state == "completed"
    store.close()


def test_reverse_start_skips_colocated_route_end_and_faces_return_leg(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # Near point C / route end but facing the wrong way.
    nav.pose = SimpleNamespace(x=3.05, y=4.02, yaw=0.0)
    envelope = command("task.start")
    envelope.payload["command"]["loop_execution"] = True
    cruised = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    original_dispatch = executor._dispatch_navigation

    def capture_dispatch(index):
        cruised.append(index)
        return original_dispatch(index)

    executor._dispatch_navigation = capture_dispatch
    executor.start_task(envelope)

    assert executor.context.current_waypoint_index == 1
    assert executor._departure_heading_mode == "teleop"
    assert executor._departure_cruise_index == 1
    _await_departure_heading(executor)
    assert cruised == [1]
    assert any(abs(cmd[2]) > 0 for cmd in nav.teleop)
    assert ids(nav.sent[-1]) == ["wp-2"]
    executor.stop()
    store.close()


def test_reverse_redispatch_does_not_skip_later_waypoints(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # Reverse start at wp-3, then a recovery/retry from index 1 while the pose
    # is already near wp-2. Mid-route skip used to abandon wp-2 and fail the
    # final pose check ~4.7m from wp-1.
    nav.pose = SimpleNamespace(x=3.05, y=4.02, yaw=atan2(-1.0, -1.0))
    envelope = command("task.start")
    envelope.payload["command"]["loop_execution"] = True
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    assert executor.context.route_snapshot["execution_order"] == "reverse_from_route_end"
    assert executor.context.current_waypoint_index == 1
    nav.pose = SimpleNamespace(x=2.05, y=3.02, yaw=atan2(-1.0, -1.0))
    before = len(nav.sent)
    executor._send_from(1)
    assert executor.context.current_waypoint_index == 1
    assert len(nav.sent) > before
    assert ids(nav.sent[-1]) == ["wp-2"]
    executor.stop()
    store.close()


def test_stale_nav2_success_after_redispatch_is_ignored(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    stale_cb = nav.result
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=atan2(1.0, 1.0))
    executor.context.current_waypoint_index = 1
    executor.context.state = "running"
    executor.on_localization_recovered()
    live_cb = nav.result
    assert live_cb is not stale_cb
    stale_cb("succeeded", "", {"missed_waypoints": []})
    assert executor.context.state == "running"
    assert results == []
    executor.stop()
    store.close()


def test_outdoor_reverse_start_skips_endpoint_within_15m_and_cruises_when_facing(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # 1.2 m off the reverse start click: inside outdoor 1.5 m gate, outside indoor 1.0 m.
    # Fixed RTK must agree with LIO (and stay near the click) before outdoor skip.
    # Face the return leg so this case only asserts colocation skip, not a spin.
    pose_x, pose_y = 3.0 + 1.2, 4.0
    target_x, target_y = 2.0, 3.0  # wp-2 after reverse
    nav.pose = SimpleNamespace(
        x=pose_x,
        y=pose_y,
        yaw=atan2(target_y - pose_y, target_x - pose_x),
    )
    nav.localization_state = {
        "active_source": "lio_imu",
        "rtk_quality": "fixed",
        "rtk_position_good_for_navigation": True,
        "rtk_x": 4.0,
        "rtk_y": 4.0,
        "absolute_stable": True,
    }
    envelope = command("task.start")
    envelope.payload["command"]["loop_execution"] = True
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)

    assert executor.context.route_snapshot["execution_order"] == "reverse_from_route_end"
    assert executor.context.current_waypoint_index == 1
    assert executor._departure_heading_index is None
    assert executor._departure_cruise_index is None
    assert ids(nav.sent[-1]) == ["wp-2"]
    store.close()


def _await_departure_heading(executor, timeout_seconds=3.0):
    import time

    deadline = time.time() + timeout_seconds
    while time.time() < deadline:
        thread = getattr(executor, "_departure_heading_thread", None)
        if (
            executor._departure_heading_index is None
            and executor._departure_cruise_index is None
            and (thread is None or not thread.is_alive())
        ):
            return
        time.sleep(0.05)
    thread = getattr(executor, "_departure_heading_thread", None)
    if thread is not None and thread.is_alive():
        thread.join(timeout=1.0)


def test_outdoor_arrival_faces_departure_heading(tmp_path):
    import time

    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    executor.start_task(envelope)
    # Face away from the next leg: outdoor must teleop-spin, not Nav2 require_yaw.
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=3.0)
    before_sent = len(nav.sent)
    assert executor._dispatch_departure_heading(0) is True
    assert executor._departure_heading_mode == "teleop"
    assert executor._departure_heading_index == 0
    assert len(nav.sent) == before_sent
    deadline = time.time() + 2.0
    while not any(abs(cmd[2]) > 0 for cmd in nav.teleop) and time.time() < deadline:
        time.sleep(0.05)
    assert any(abs(cmd[2]) > 0 for cmd in nav.teleop)
    executor._clear_departure_heading(cancel_navigation=True)
    _await_departure_heading(executor)
    # Pre-leg path also uses teleop, then cruises once aligned.
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=3.0)
    nav.teleop.clear()
    cruised = []
    executor._dispatch_navigation = lambda index: cruised.append(index)
    assert executor._maybe_face_travel_direction(1) is True
    assert executor._departure_heading_mode == "teleop"
    assert executor._departure_cruise_index == 1
    _await_departure_heading(executor)
    assert cruised == [1]
    assert any(abs(cmd[2]) > 0 for cmd in nav.teleop)
    store.close()


def test_outdoor_moderate_departure_spins_in_place_before_cruise(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    executor.start_task(envelope)
    # Field log: 76° error at point 1 while facing waypoint 2.
    # Travel to wp-2 from (1,2) is atan2(1,1) ≈ 0.785 rad.
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=0.785 + 1.326)
    before_sent = len(nav.sent)
    assert executor._dispatch_departure_heading(0) is True
    assert executor._departure_heading_mode == "teleop"
    assert executor._departure_heading_index == 0
    assert len(nav.sent) == before_sent
    executor._clear_departure_heading(cancel_navigation=True)
    _await_departure_heading(executor)
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=0.785 + 1.326)
    cruised = []
    executor._dispatch_navigation = lambda index: cruised.append(index)
    assert executor._maybe_face_travel_direction(1) is True
    assert executor._departure_heading_mode == "teleop"
    assert executor._departure_cruise_index == 1
    _await_departure_heading(executor)
    assert cruised == [1]
    store.close()


def test_outdoor_reverse_start_faces_return_leg_when_heading_is_wrong(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # Near reverse start but facing outbound (east); must teleop-spin before cruising.
    nav.pose = SimpleNamespace(x=3.0 + 1.2, y=4.0, yaw=0.0)
    nav.localization_state = {
        "active_source": "lio_imu",
        "rtk_quality": "fixed",
        "rtk_position_good_for_navigation": True,
        "rtk_x": 4.0,
        "rtk_y": 4.0,
        "absolute_stable": True,
    }
    envelope = command("task.start")
    envelope.payload["command"]["loop_execution"] = True
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    cruised = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    # Capture cruise after the teleop spin finishes.
    original_dispatch = executor._dispatch_navigation

    def capture_dispatch(index):
        cruised.append(index)
        return original_dispatch(index)

    executor._dispatch_navigation = capture_dispatch
    executor.start_task(envelope)

    assert executor.context.route_snapshot["execution_order"] == "reverse_from_route_end"
    assert executor.context.current_waypoint_index == 1
    assert executor._departure_heading_mode == "teleop"
    assert executor._departure_cruise_index == 1
    _await_departure_heading(executor)
    assert cruised == [1]
    assert any(abs(cmd[2]) > 0 for cmd in nav.teleop)
    assert ids(nav.sent[-1]) == ["wp-2"]
    store.close()


def test_absolute_localization_pause_auto_resumes_to_next_leg(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    executor.start_task(envelope)
    executor.context.current_waypoint_index = 0
    executor.context.state = "paused"
    executor._paused_for_localization = True
    executor._paused_localization_reason = "absolute_required"
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "rtk_quality": "fixed",
        "rtk_position_good_for_navigation": True,
        "rtk_x": 1.0,
        "rtk_y": 2.0,
        "correction_smoothing_active": False,
        "lio_motion_anomaly": False,
    }
    continued = []
    executor._continue_after_waypoint = lambda index: continued.append(index)
    executor.on_localization_recovered()
    assert continued == [0]
    assert executor.context.state == "running"
    assert any(event[0] == "task.resumed" for event in events)
    store.close()


def test_absolute_localization_pause_reapproaches_when_outdoor_rtk_off_click(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    executor.start_task(envelope)
    executor.context.current_waypoint_index = 0
    executor.context.state = "paused"
    executor._paused_for_localization = True
    executor._paused_localization_reason = "absolute_required"
    executor._navigation_prepared = True
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=0.0)
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "rtk_quality": "fixed",
        "rtk_position_good_for_navigation": True,
        "rtk_x": 5.0,
        "rtk_y": 8.0,
        "correction_smoothing_active": False,
        "lio_motion_anomaly": False,
    }
    continued = []
    sent_before = len(nav.sent)
    executor._continue_after_waypoint = lambda index: continued.append(index)
    executor.on_localization_recovered()
    assert continued == []
    assert executor.context.state == "running"
    assert executor.context.current_waypoint_index == 0
    assert len(nav.sent) > sent_before
    assert ids(nav.sent[-1]) == ["wp-1"]
    store.close()


def test_absolute_localization_pause_rechecks_indoor_xy_after_relocalization(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    executor.context.current_waypoint_index = 0
    executor.context.state = "paused"
    executor._paused_for_localization = True
    executor._paused_localization_reason = "absolute_required"
    first = executor.context.route_snapshot["waypoints"][0]
    nav.pose = SimpleNamespace(
        x=float(first["x"]) + 0.8,
        y=float(first["y"]),
        yaw=float(first.get("yaw") or 0.0),
    )
    sent_before = len(nav.sent)

    executor.on_localization_recovered()

    assert executor.context.state == "running"
    assert executor.context.current_waypoint_index == 0
    assert len(nav.sent) == sent_before + 1
    assert ids(nav.sent[-1]) == ["wp-1"]
    assert executor._arrival_retry_counts[0] == 1
    store.close()


def test_outdoor_absolute_pause_does_not_resume_without_rtk(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type(
        "Ctx",
        (),
        {
            "route_snapshot": {
                "scene_scope": "outdoor",
                "map": {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"},
                "waypoints": [],
            }
        },
    )()
    invalid = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "rtk_quality": "invalid",
        "rtk_position_good_for_navigation": False,
        "rtk_blocked_reason": "gnss_origin_not_loaded",
        "correction_smoothing_active": False,
        "lio_motion_anomaly": False,
    }
    assert executor._outdoor_settle_can_continue(invalid) is True
    assert executor._outdoor_absolute_pause_can_resume(invalid) is False
    invalid["rtk_quality"] = "fixed"
    invalid["rtk_position_good_for_navigation"] = True
    assert executor._outdoor_absolute_pause_can_resume(invalid) is True
    store.close()


def test_reapproach_skips_spin_when_already_at_waypoint(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # 0.4 m from the click and facing away: used to teleop-spin in place.
    nav.pose = SimpleNamespace(x=0.4, y=0.0, yaw=3.0)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = SimpleNamespace(
        state="running",
        state_version=1,
        task_execution_id="exec-reapproach-spin",
        current_waypoint_index=0,
        current_segment_index=0,
        record_rosbag=False,
        route_snapshot={
            "waypoints": [
                {"waypoint_id": "wp-0", "x": 0.0, "y": 0.0, "yaw": 0.0, "map_point_number": 1},
                {"waypoint_id": "wp-1", "x": 10.0, "y": 0.0, "yaw": 0.0, "map_point_number": 2},
            ]
        },
        docking=None,
        environment_type="outdoor",
    )
    executor._segments = []
    assert executor._maybe_face_travel_direction(0) is False
    assert executor._departure_heading_index is None
    store.close()


def test_outdoor_final_waypoint_off_click_does_not_complete(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    executor.start_task(envelope)
    last_index = len(executor.context.route_snapshot["waypoints"]) - 1
    last_wp = executor.context.route_snapshot["waypoints"][last_index]
    nav.pose = SimpleNamespace(x=float(last_wp["x"]), y=float(last_wp["y"]), yaw=0.0)
    nav.localization_state = {
        "active_source": "lio_imu",
        "rtk_quality": "fixed",
        "rtk_position_good_for_navigation": True,
        "rtk_x": float(last_wp["x"]) + 3.0,
        "rtk_y": float(last_wp["y"]),
    }
    executor._arrival_retry_counts[last_index] = executor.arrival_reapproach_max_attempts
    executor.context.current_waypoint_index = last_index
    executor._goal_offset = last_index
    executor._dispatched_count = 1
    nav.result("succeeded", "", {"missed_waypoints": []})
    assert executor.context.state == "paused"
    assert results == []
    assert executor.context.current_waypoint_index == last_index
    store.close()


def test_outdoor_final_pose_error_rejects_rtk_off_click(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    executor.start_task(envelope)
    last_wp = executor.context.route_snapshot["waypoints"][-1]
    nav.pose = SimpleNamespace(x=float(last_wp["x"]), y=float(last_wp["y"]), yaw=0.0)
    nav.localization_state = {
        "active_source": "lio_imu",
        "rtk_quality": "fixed",
        "rtk_position_good_for_navigation": True,
        "rtk_x": float(last_wp["x"]) + 2.5,
        "rtk_y": float(last_wp["y"]),
    }
    code, message = executor._final_pose_error()
    assert code == "FINAL_POSE_OUT_OF_TOLERANCE"
    assert "RTK" in message
    nav.localization_state["rtk_x"] = float(last_wp["x"]) + 0.19
    nav.localization_state["rtk_y"] = float(last_wp["y"])
    assert executor._final_pose_error() is None
    store.close()


def test_outdoor_startup_runs_ndt_before_secondary_correction_when_lio_is_stable(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=10.0, y=10.0, yaw=0.0)
    nav.localization_state = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "rtk_quality": "fixed",
        "rtk_usable": True,
        "rtk_heading_usable": True,
        "rtk_good_for_navigation": True,
        "rtk_position_good_for_navigation": True,
        "rtk_x": 10.0,
        "rtk_y": 10.8,
    }
    nav.rtk_calls = 0

    def set_initial_pose_from_rtk(wait_seconds=30.0):
        nav.rtk_calls += 1
        return {"source": "rtk_fixed"}

    nav.set_initial_pose_from_rtk = set_initial_pose_from_rtk
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["map"] = {
        "map_id": "outdoor-a", "map_version": "v1", "coordinate_mode": "rtk_fixed",
        "scene_scope": "outdoor",
    }
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    executor.prepare_task_start(envelope)
    executor.initialize_before_navigation()
    assert nav.rtk_calls == 0
    assert len(nav.progressive_relocalize_requests) == 1
    store.close()


def test_localization_recovery_clears_stale_departure_heading(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    # Simulate an in-flight pre-leg spin that must not survive recovery.
    executor._departure_heading_index = 0
    executor._departure_cruise_index = 2
    executor.context.current_waypoint_index = 1
    executor.context.state = "running"
    # Face the pending leg so recovery redispatches the cruise immediately.
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=atan2(1.0, 1.0))

    before_cancelled = nav.cancelled
    before_sent = len(nav.sent)
    executor.on_localization_recovered()

    assert executor._departure_heading_index is None
    assert executor._departure_cruise_index is None
    assert nav.cancelled > before_cancelled
    assert len(nav.sent) > before_sent
    assert ids(nav.sent[-1]) == ["wp-2"]
    store.close()


def test_obstacle_monitor_ignores_zeroed_cmd_vel_when_localization_lost(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    events = []

    class LostLocalizationCollisionNav(FakeCollisionLimitedNavigation):
        def obstacle_monitor_snapshot(self):
            snap = super().obstacle_monitor_snapshot()
            snap["localization_normal"] = False
            return snap

    executor = TaskExecutor(
        store,
        LostLocalizationCollisionNav(),
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        obstacle_speech=SimpleNamespace(
            enabled=True,
            no_progress_seconds=4.0,
            min_progress_m=0.08,
            obstacle_max_distance_m=0.9,
            collision_limit_ratio=0.6,
            obstacle_clear_seconds=1.0,
        ),
    )
    executor.start_task(command("task.start"))
    executor._evaluate_obstacle_progress()
    speech_events = [event for event in events if event[0] == "task.obstacle_speech"]
    assert speech_events == []
    executor.stop()
    store.close()


def test_waypoint_speech_blocks_next_navigation_until_playback_finishes(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    first = envelope.payload["command"]["route_snapshot"]["waypoints"][0]
    first["speech_template_id"] = 7
    status_dir = tmp_path / "audio-status"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        waypoint_speech=SimpleNamespace(
            status_dir=str(status_dir),
            timeout_seconds=2.0,
            poll_interval_seconds=0.01,
            enabled=True,
            block_navigation=True,
        ),
    )

    executor.start_task(envelope)
    assert ids(nav.sent[0]) == ["wp-1"]
    nav.pose = SimpleNamespace(x=float(first["x"]), y=float(first["y"]), yaw=0.0)
    nav.result("succeeded", "", {"missed_waypoints": []})

    waypoint_key = hashlib.sha256(b"wp-1").hexdigest()
    status_path = status_dir / executor.context.task_execution_id / f"{waypoint_key}.json"
    status_path.write_text(
        json.dumps({"status": "finished", "waypoint_id": "wp-1"}),
        encoding="utf-8",
    )
    if executor._speech_wait_thread is not None:
        executor._speech_wait_thread.join(timeout=1)
    _await_departure_heading(executor)
    for _ in range(100):
        if nav.sent and ids(nav.sent[-1])[:1] == ["wp-2"]:
            break
        time.sleep(0.01)
    assert ids(nav.sent[-1]) == ["wp-2"]
    executor.stop()
    store.close()



def test_waypoint_speech_does_not_block_when_block_navigation_disabled(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    first = envelope.payload["command"]["route_snapshot"]["waypoints"][0]
    first["speech_template_id"] = 7
    status_dir = tmp_path / "audio-status"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        waypoint_speech=SimpleNamespace(
            status_dir=str(status_dir),
            timeout_seconds=2.0,
            poll_interval_seconds=0.01,
            enabled=False,
            block_navigation=False,
        ),
    )

    executor.start_task(envelope)
    assert all(
        not waypoint.get("speech_template_id")
        for waypoint in executor.context.route_snapshot["waypoints"]
    )
    assert ids(nav.sent[0]) == ["wp-1"]
    assert executor._speech_waiting_index is None
    assert executor.context.state == "running"
    store.close()


def test_edge_non_blocking_policy_overrides_legacy_blocking_waypoint(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    first = envelope.payload["command"]["route_snapshot"]["waypoints"][0]
    first["speech_template_id"] = 7
    first["speech_mode"] = "blocking"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        waypoint_speech=SimpleNamespace(
            status_dir=str(tmp_path / "audio-status"),
            timeout_seconds=120.0,
            poll_interval_seconds=0.2,
            enabled=True,
            block_navigation=False,
        ),
    )

    executor.start_task(envelope)
    nav.pose = SimpleNamespace(x=float(first["x"]), y=float(first["y"]), yaw=0.0)
    nav.result("succeeded", "", {"missed_waypoints": []})
    _await_departure_heading(executor)

    assert executor._waypoint_speech_mode(first) == "non_blocking"
    assert executor._speech_waiting_index is None
    assert executor.context.state == "running"
    assert ids(nav.sent[-1]) == ["wp-2"]
    executor.stop()
    store.close()


def test_waypoint_speech_timeout_continues_navigation_when_blocking(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    first = envelope.payload["command"]["route_snapshot"]["waypoints"][0]
    first["speech_template_id"] = 7
    status_dir = tmp_path / "audio-status"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        waypoint_speech=SimpleNamespace(
            status_dir=str(status_dir),
            timeout_seconds=0.05,
            poll_interval_seconds=0.01,
            enabled=True,
            block_navigation=True,
        ),
    )

    executor.start_task(envelope)
    nav.pose = SimpleNamespace(x=float(first["x"]), y=float(first["y"]), yaw=0.0)
    nav.result("succeeded", "", {"missed_waypoints": []})
    if executor._speech_wait_thread is not None:
        executor._speech_wait_thread.join(timeout=2)
    _await_departure_heading(executor)
    assert executor.context.state == "running"
    assert ids(nav.sent[-1]) == ["wp-2"]
    executor.stop()
    store.close()


def test_localization_recovery_during_final_speech_does_not_redispatch_waypoint(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    final = envelope.payload["command"]["route_snapshot"]["waypoints"][-1]
    final["speech_template_id"] = 7
    status_dir = tmp_path / "audio-status"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        waypoint_speech=SimpleNamespace(
            status_dir=str(status_dir),
            timeout_seconds=2.0,
            poll_interval_seconds=0.01,
            enabled=True,
            block_navigation=True,
        ),
    )

    executor.start_task(envelope)
    # Each speech waypoint is its own single-pose goal.
    assert ids(nav.sent[0]) == ["wp-1"]
    drive_patrol(nav, until_ids=["wp-3"], executor=executor)
    before_sent = len(nav.sent)
    nav.pose = SimpleNamespace(x=float(final["x"]), y=float(final["y"]), yaw=float(final.get("yaw") or 0.0))
    nav.result("succeeded", "", {"missed_waypoints": []})
    # Departure spin at the penultimate point may still be pending; finish it.
    if executor._speech_waiting_index is None and nav.result is not None:
        nav.pose = SimpleNamespace(x=float(final["x"]), y=float(final["y"]), yaw=float(final.get("yaw") or 0.0))
        nav.result("succeeded", "", {"missed_waypoints": []})
    assert executor._speech_waiting_index == 2

    executor.on_localization_lost()
    executor.on_localization_recovered()

    assert executor.context.state == "running"
    assert len(nav.sent) == before_sent + 1 or executor._speech_waiting_index == 2
    # Recovery must not invent a new cruise while speech is still blocking.
    assert ids(nav.sent[-1]) != ["wp-1"]

    waypoint_key = hashlib.sha256(b"wp-3").hexdigest()
    status_path = status_dir / executor.context.task_execution_id / f"{waypoint_key}.json"
    status_path.parent.mkdir(parents=True, exist_ok=True)
    status_path.write_text(
        json.dumps({"status": "finished", "waypoint_id": "wp-3"}),
        encoding="utf-8",
    )
    if executor._speech_wait_thread is not None:
        executor._speech_wait_thread.join(timeout=1)

    assert executor.context.state == "completed"
    store.close()


def test_localization_recovery_after_final_speech_finishes_completes_without_redispatch(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    final = envelope.payload["command"]["route_snapshot"]["waypoints"][-1]
    final["speech_template_id"] = 7
    status_dir = tmp_path / "audio-status"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        waypoint_speech=SimpleNamespace(
            status_dir=str(status_dir),
            timeout_seconds=2.0,
            poll_interval_seconds=0.01,
            enabled=True,
            block_navigation=True,
        ),
    )

    executor.start_task(envelope)
    assert ids(nav.sent[0]) == ["wp-1"]
    drive_patrol(nav, until_ids=["wp-3"], executor=executor)
    nav.pose = SimpleNamespace(x=float(final["x"]), y=float(final["y"]), yaw=float(final.get("yaw") or 0.0))
    nav.result("succeeded", "", {"missed_waypoints": []})
    if executor._speech_waiting_index is None and nav.result is not None:
        nav.result("succeeded", "", {"missed_waypoints": []})
    assert executor._speech_waiting_index == 2
    before_sent = len(nav.sent)
    executor.on_localization_lost()

    waypoint_key = hashlib.sha256(b"wp-3").hexdigest()
    status_path = status_dir / executor.context.task_execution_id / f"{waypoint_key}.json"
    status_path.parent.mkdir(parents=True, exist_ok=True)
    status_path.write_text(
        json.dumps({"status": "finished", "waypoint_id": "wp-3"}),
        encoding="utf-8",
    )
    if executor._speech_wait_thread is not None:
        executor._speech_wait_thread.join(timeout=1)
    assert executor.context.state == "paused"
    assert executor._speech_wait_finished is True

    executor.on_localization_recovered()

    assert executor.context.state == "completed"
    assert len(nav.sent) == before_sent
    store.close()


def test_waypoint_profile_uses_target_for_initial_approach_and_source_afterwards(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    points = envelope.payload["command"]["route_snapshot"]["waypoints"]
    points[0]["avoidance_to_next"] = False
    points[1]["require_yaw"] = True
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)
    assert ids(nav.sent[0]) == ["wp-1"]
    assert nav.waypoint_profiles[0] == (False, False, True)
    nav.pose = SimpleNamespace(x=float(points[0]["x"]), y=float(points[0]["y"]), yaw=0.0)
    nav.result("succeeded", "", {"missed_waypoints": []})
    _await_departure_heading(executor)
    assert ids(nav.sent[-1]) == ["wp-2"]
    # require_yaw is enforced with a stationary arrival turn, not by RPP
    # while it tracks the approach path.
    assert nav.waypoint_profiles[-1] == (False, False, True)
    nav.pose = SimpleNamespace(x=float(points[1]["x"]), y=float(points[1]["y"]), yaw=0.0)
    nav.result("succeeded", "", {"missed_waypoints": []})
    _await_departure_heading(executor)
    assert ids(nav.sent[-1]) == ["wp-3"]
    # Last route point is still a cruise until the final metre.
    assert nav.waypoint_profiles[-1] == (True, False, False)
    nav.feedback(0, 0.6)
    assert nav.waypoint_profiles[-1] == (True, False, True)
    executor.stop()
    store.close()


def test_patrol_require_yaw_directly_adjusts_large_turn_drift_after_reaching_xy(tmp_path):
    from math import pi

    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    envelope = command("task.start")
    first = envelope.payload["command"]["route_snapshot"]["waypoints"][0]
    first["require_yaw"] = True
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        # This test exercises the bounded post-yaw cmd_vel path with its
        # 0.30 m total travel budget after the 0.30 m normal acceptance radius.
        normal_arrival_tolerance_m=0.30,
    )

    executor.start_task(envelope)
    # Nav2 may finish the XY click while the body still faces away from the
    # requested yaw. Its profile must not ask RPP to weave toward that yaw.
    assert nav.waypoint_profiles[0] == (True, False, True)
    nav.pose = SimpleNamespace(x=float(first["x"]), y=float(first["y"]), yaw=pi)
    original_teleop_velocity = nav.teleop_velocity

    def drifting_teleop_velocity(vx=0.0, vy=0.0, yaw_rate=0.0):
        original_teleop_velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)
        if abs(float(yaw_rate)) > 1e-6:
            nav.pose.x = float(first["x"]) + 0.55

    nav.teleop_velocity = drifting_teleop_velocity
    nav.result("succeeded", "", {"missed_waypoints": []})

    reached_events = [
        event
        for event in events
        if event[0] == "task.progress"
        and event[1].get("milestone") == "waypoint_reached"
    ]
    # Business arrival must not be published until heading and the combined
    # final-pose gate have both passed.
    assert reached_events == []
    assert executor.context.post_arrival_waypoint_index == 0
    assert executor.context.post_arrival_stage == "heading_pending"

    deadline = time.time() + 2.0
    while (
        executor._departure_heading_mode != "teleop"
        and time.time() < deadline
    ):
        time.sleep(0.02)
    assert executor._departure_heading_mode == "teleop"
    assert executor._departure_heading_is_arrival is True
    assert nav.cancelled == 0
    _await_departure_heading(executor)
    # The following-leg turn may run after arrival confirmation, so the final
    # fake pose can face the next leg. Negative yaw commands prove the arrival
    # controller first turned back toward this waypoint's yaw.
    assert any(command[2] < 0.0 for command in nav.teleop)
    assert any(event[0] == "task.arrival_heading_aligning" for event in events)
    assert any(event[0] == "task.arrival_heading_aligned" for event in events)
    deadline = time.time() + 5.0
    while ids(nav.sent[-1]) != ["wp-2"] and time.time() < deadline:
        time.sleep(0.02)
    # Post-arrival XY convergence has no initial-distance cap and must not
    # redispatch the already-reached waypoint through Nav2.
    assert sum(ids(batch) == ["wp-1"] for batch in nav.sent) == 1
    assert ids(nav.sent[-1]) == ["wp-2"]
    assert executor.context.state == "running"
    reached_events = [
        event
        for event in events
        if event[0] == "task.progress"
        and event[1].get("milestone") == "waypoint_reached"
    ]
    assert len(reached_events) == 1
    assert any(abs(vx) > 0.0 or abs(vy) > 0.0 for vx, vy, _ in nav.arrival_adjustments)
    assert not any(event[0] == "task.safe_hold" for event in events)
    stages = [
        event[1].get("arrival_stage")
        for event in events
        if event[0] == "task.recovery_active"
    ]
    assert stages.index("correction") < stages.index("position_approach")
    assert stages.index("position_approach") < stages.index("heading_alignment")
    assert stages.index("heading_alignment") < stages.index("pose_verification")
    executor.stop()
    store.close()


def test_final_configured_heading_large_translation_recovers_after_bounded_adjustment(tmp_path):
    from math import pi

    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    results = []
    events = []
    envelope = command("task.start")
    final = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    final["require_yaw"] = True
    final["yaw"] = 0.0
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [final]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
        # Leave enough wall-clock budget for the worker to issue at least one
        # bounded translation even when the full suite is under load.
        arrival_adjust_timeout_seconds=0.25,
    )

    executor.start_task(envelope)
    nav.pose = SimpleNamespace(x=float(final["x"]), y=float(final["y"]), yaw=pi)
    original_teleop_velocity = nav.teleop_velocity

    def drifting_teleop_velocity(vx=0.0, vy=0.0, yaw_rate=0.0):
        original_teleop_velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)
        if abs(float(yaw_rate)) > 1e-6:
            nav.pose.x = float(final["x"]) + 0.8

    nav.teleop_velocity = drifting_teleop_velocity
    nav.result("succeeded", "", {"missed_waypoints": []})
    _await_departure_heading(executor)

    deadline = time.time() + 5.0
    while (
        executor.context.state == "running"
        or executor._arrival_adjustment_thread is not None
        and executor._arrival_adjustment_thread.is_alive()
    ) and time.time() < deadline:
        time.sleep(0.02)
    while not any(
        event[0] == "task.safe_hold"
        and event[1].get("reason_code") == "ARRIVAL_POST_YAW_COARSE_EXCEEDED"
        for event in events
    ) and time.time() < deadline:
        time.sleep(0.01)

    assert executor.context.state == "paused"
    assert len(nav.sent) == 1
    assert executor.context.arrival_side_effects_started is False
    assert results == []
    assert any(
        event[0] == "task.safe_hold"
        and event[1].get("reason_code") == "ARRIVAL_POST_YAW_COARSE_EXCEEDED"
        for event in events
    )
    assert any(abs(vx) > 0.0 or abs(vy) > 0.0 for vx, vy, _ in nav.arrival_adjustments)
    executor.stop()
    store.close()


def test_final_heading_small_xy_drift_uses_cmd_vel_raw_adjustment(tmp_path):
    from math import pi

    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    results = []
    envelope = command("task.start")
    final = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    final.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [final]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(x=float(final["x"]), y=float(final["y"]), yaw=pi)
    original_teleop_velocity = nav.teleop_velocity

    def drifting_teleop_velocity(vx=0.0, vy=0.0, yaw_rate=0.0):
        original_teleop_velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)
        if abs(float(yaw_rate)) > 1e-6:
            nav.pose.x = float(final["x"]) + 0.40

    nav.teleop_velocity = drifting_teleop_velocity
    nav.result("succeeded", "", {"missed_waypoints": []})
    deadline = time.time() + 5.0
    while (executor.context.state == "running" or not results) and time.time() < deadline:
        time.sleep(0.02)

    assert executor.context.state == "completed"
    assert len(nav.sent) == 1
    assert any(abs(vx) > 0.0 or abs(vy) > 0.0 for vx, vy, _ in nav.arrival_adjustments)
    assert results[-1][1] == "succeeded"
    stages = [
        event[1].get("arrival_stage")
        for event in events
        if event[0] == "task.recovery_active"
    ]
    assert "heading_preserving_adjustment" in stages
    store.close()


def test_post_heading_yaw_drift_realigns_instead_of_reporting_xy_adjust_unavailable(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    results = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(envelope)
    executor.context.arrival_side_effects_started = True
    executor._arrival_correction_completed_index = 0
    executor._arrival_heading_completed_index = 0
    executor._set_post_arrival_stage(0, "heading_aligned")
    # A fresh localization sample after a completed turn can show yaw drift
    # while XY remains valid. This must restart heading alignment, not try to
    # start an XY correction and emit the misleading unavailable error.
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]),
        y=float(waypoint["y"]),
        yaw=0.35,
    )

    executor.on_navigation_result("succeeded", generation=executor._nav_goal_generation)
    deadline = time.time() + 5.0
    while (executor.context.state == "running" or not results) and time.time() < deadline:
        time.sleep(0.02)

    assert executor.context.state == "completed"
    assert results[-1][1] == "succeeded"
    assert not any(
        event[0] == "task.safe_hold"
        and event[1].get("reason_code") == "ARRIVAL_MICRO_ADJUST_UNAVAILABLE"
        for event in events
    )
    executor.stop()
    store.close()


def test_post_yaw_arrival_adjustment_allows_residual_above_legacy_diagnostic_value(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    executor.context.arrival_side_effects_started = True
    executor._arrival_heading_completed_index = 0
    executor._set_post_arrival_stage(0, "heading_aligned")
    # 0.55 m is above the legacy 0.45 m diagnostic gate but remains within
    # the 0.30 m acceptance radius plus the bounded 0.30 m adjustment budget.
    # collision-monitored *post-yaw* XY controller was even allowed to run.
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + 0.55,
        y=float(waypoint["y"]),
        yaw=0.0,
    )

    assert executor._start_arrival_adjustment(waypoint, 0) is True
    deadline = time.time() + 1.0
    while not any(
        abs(vx) > 0.0 or abs(vy) > 0.0 for vx, vy, _ in nav.arrival_adjustments
    ) and time.time() < deadline:
        time.sleep(0.01)

    assert any(abs(vx) > 0.0 or abs(vy) > 0.0 for vx, vy, _ in nav.arrival_adjustments)
    executor._cancel_arrival_adjustment(reset_state=True)
    executor.stop()
    store.close()


def test_micro_adjust_recheck_preserves_final_yaw_latch_for_next_segment(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    executor._arrival_correction_completed_index = 0
    executor._arrival_heading_completed_index = 0
    executor._set_post_arrival_stage(0, "xy_adjustment_recheck")
    executor._hold_final_pose = lambda **_kwargs: True
    executor._handle_lightweight_arrival = lambda *_args: False
    executor._arrival_xy_is_stable = lambda *_args: False
    executor._arrival_pose_errors = lambda *_args: (0.40, 0.0)
    executor._arrival_pose_tolerances = lambda *_args: (0.30, 0.25)
    executor._arrival_pose_within_combined_tolerance = lambda *_args: False
    starts = []

    def start_adjustment(point, index):
        starts.append((index, executor._arrival_heading_completed_index))
        return True

    executor._start_arrival_adjustment = start_adjustment
    nav.result("succeeded", "", {"missed_waypoints": []})

    assert starts == [(0, 0)]
    assert executor.context.post_arrival_stage == "heading_aligned"
    assert not any(event[0] == "task.safe_hold" for event in events)
    executor.stop()
    store.close()


def test_micro_adjust_recheck_inside_xy_radius_waits_for_stability_not_another_move(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    executor._arrival_correction_completed_index = 0
    executor._arrival_heading_completed_index = 0
    executor._set_post_arrival_stage(0, "xy_adjustment_recheck")
    executor._hold_final_pose = lambda **_kwargs: True
    executor._handle_lightweight_arrival = lambda *_args: False
    executor._arrival_xy_is_stable = lambda *_args: False
    executor._arrival_pose_errors = lambda *_args: (0.20, 0.0)
    executor._arrival_pose_tolerances = lambda *_args: (0.30, 0.25)
    executor._arrival_pose_within_combined_tolerance = lambda *_args: False
    executor._start_arrival_adjustment = lambda *_args: (_ for _ in ()).throw(
        AssertionError("must not start another XY micro-adjustment inside tolerance")
    )

    nav.result("succeeded", "", {"missed_waypoints": []})

    safe_hold = [event for event in events if event[0] == "task.safe_hold"][-1]
    assert safe_hold[1]["reason_code"] == "ARRIVAL_CONFIRMATION_UNSTABLE"
    assert nav.stop_commands >= 1
    executor.stop()
    store.close()


def test_post_yaw_state_guard_is_not_reported_as_missing_micro_adjust_adapter(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    executor._arrival_correction_completed_index = 0
    executor._arrival_heading_completed_index = 0
    # xy_verified is deliberately not a post-yaw micro-adjust stage.
    executor._set_post_arrival_stage(0, "xy_verified")
    executor._hold_final_pose = lambda **_kwargs: True
    executor._handle_lightweight_arrival = lambda *_args: False
    executor._arrival_pose_errors = lambda *_args: (0.40, 0.0)
    executor._arrival_pose_tolerances = lambda *_args: (0.30, 0.25)
    executor._arrival_pose_within_combined_tolerance = lambda *_args: False

    nav.result("succeeded", "", {"missed_waypoints": []})

    safe_hold = [event for event in events if event[0] == "task.safe_hold"][-1]
    assert safe_hold[1]["reason_code"] == "ARRIVAL_MICRO_ADJUST_STATE_INVALID"
    assert "接口不可用" not in safe_hold[1]["reason_message"]
    executor.stop()
    store.close()


def test_arrival_adjustment_obstacle_stops_and_safe_pauses(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.arrival_clearance = {"clear": False, "reason": "obstacle"}
    events = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    executor.context.arrival_side_effects_started = True
    executor._arrival_heading_completed_index = 0
    executor._set_post_arrival_stage(0, "heading_aligned")
    nav.pose = SimpleNamespace(x=float(waypoint["x"]) + 0.40, y=float(waypoint["y"]), yaw=0.0)

    assert executor._start_arrival_adjustment(waypoint, 0) is True
    deadline = time.time() + 1.0
    while (
        executor.context.state == "running"
        or not any(event[0] == "task.safe_hold" for event in events)
    ) and time.time() < deadline:
        time.sleep(0.01)

    assert executor.context.state == "paused"
    assert nav.arrival_adjustments[-1] == (0.0, 0.0, 0.0)
    safe_hold = [event for event in events if event[0] == "task.safe_hold"][-1]
    assert safe_hold[1]["reason_code"] == "ARRIVAL_POSE_CONVERGENCE_FAILED"
    assert "obstacle" in safe_hold[1]["reason_message"]
    executor.stop()
    store.close()


def test_arrival_adjustment_ignores_rear_obstacle_beyond_required_travel(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    results = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(envelope)
    # The target lies behind the final yaw. From 0.40 m away, the 0.30 m
    # acceptance radius requires only 0.12 m of translation plus the 0.05 m
    # stopping margin, not the legacy 0.45 m lookahead to the click centre.
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + 0.40,
        y=float(waypoint["y"]),
        yaw=0.0,
    )
    clearance_distances = []

    def clearance(vx, vy, travel_distance_m, *, max_scan_age_seconds=0.5):
        clearance_distances.append((vx, vy, travel_distance_m))
        if abs(vx) > 0.0 or abs(vy) > 0.0:
            # Simulate a rear point at 0.25 m: it is outside the repaired
            # finite sweep, but would have blocked the old 0.45 m lookahead.
            nav.pose = SimpleNamespace(
                x=float(waypoint["x"]) + 0.15,
                y=float(waypoint["y"]),
                yaw=0.0,
            )
            return {"clear": travel_distance_m <= 0.25, "reason": "obstacle"}
        return {"clear": True, "reason": "rotation_only"}

    nav.directional_clearance = clearance
    executor.context.arrival_side_effects_started = True
    executor._arrival_heading_completed_index = 0
    executor._set_post_arrival_stage(0, "heading_aligned")

    assert executor._start_arrival_adjustment(waypoint, 0) is True
    deadline = time.time() + 2.0
    while (executor.context.state == "running" or not results) and time.time() < deadline:
        time.sleep(0.02)

    moving_clearance = next(
        travel_distance_m
        for vx, vy, travel_distance_m in clearance_distances
        if abs(vx) > 0.0 or abs(vy) > 0.0
    )
    assert abs(moving_clearance - 0.17) <= 0.01
    assert executor.context.state == "completed"
    assert results[-1][1] == "succeeded"
    executor.stop()
    store.close()


def test_nav2_micro_goal_requires_post_action_pose_revalidation(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    results = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update(
        {
            "require_yaw": True,
            "yaw": 0.0,
            "arrival_micro_adjust_mode": "nav2_goal",
        }
    )
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(x=float(waypoint["x"]) + 0.40, y=float(waypoint["y"]), yaw=0.0)
    executor.context.arrival_side_effects_started = True
    executor._arrival_heading_completed_index = 0
    executor._set_post_arrival_stage(0, "heading_aligned")

    assert executor._start_arrival_adjustment(waypoint, 0) is True
    assert nav.arrival_micro_goal_profiles == [(True, 0.15)]
    assert len(nav.sent) == 2

    # Nav2 success alone is deliberately insufficient. Only the following
    # corrected pose check is allowed to complete the waypoint.
    nav.pose = SimpleNamespace(x=float(waypoint["x"]), y=float(waypoint["y"]), yaw=0.0)
    nav.result("succeeded", "", {"missed_waypoints": []})

    assert nav.arrival_micro_goal_profiles[-1] == (False, 0.15)
    assert executor.context.state == "completed"
    assert results[-1][1] == "succeeded"
    store.close()


def test_arrival_adjustment_retries_transient_stale_scan_then_completes(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    results = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
        arrival_adjust_safety_grace_seconds=0.5,
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + 0.40,
        y=float(waypoint["y"]),
        yaw=0.0,
    )
    clearance_calls = 0

    def transient_clearance(vx, vy, travel_distance_m, *, max_scan_age_seconds=0.5):
        nonlocal clearance_calls
        clearance_calls += 1
        if clearance_calls <= 2:
            return {"clear": False, "reason": "scan_stale"}
        return {"clear": True, "reason": "clear"}

    nav.directional_clearance = transient_clearance
    executor.context.arrival_side_effects_started = True
    executor._arrival_heading_completed_index = 0
    executor._set_post_arrival_stage(0, "heading_aligned")

    assert executor._start_arrival_adjustment(waypoint, 0) is True
    deadline = time.time() + 5.0
    while (executor.context.state == "running" or not results) and time.time() < deadline:
        time.sleep(0.02)

    assert clearance_calls > 2
    assert executor.context.state == "completed"
    assert results[-1][1] == "succeeded"
    first_nonzero = next(
        index
        for index, command_value in enumerate(nav.arrival_adjustments)
        if abs(command_value[0]) > 0.0 or abs(command_value[1]) > 0.0
    )
    assert any(
        command_value == (0.0, 0.0, 0.0)
        for command_value in nav.arrival_adjustments[:first_nonzero]
    )
    executor.stop()
    store.close()


def test_arrival_adjustment_pauses_after_stale_scan_grace_expires(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.arrival_clearance = {"clear": False, "reason": "scan_stale"}
    events = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        arrival_adjust_safety_grace_seconds=0.05,
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + 0.40,
        y=float(waypoint["y"]),
        yaw=0.0,
    )
    executor.context.arrival_side_effects_started = True
    executor._arrival_heading_completed_index = 0
    executor._set_post_arrival_stage(0, "heading_aligned")

    assert executor._start_arrival_adjustment(waypoint, 0) is True
    deadline = time.time() + 1.0
    while (
        executor.context.state == "running"
        or not any(event[0] == "task.safe_hold" for event in events)
    ) and time.time() < deadline:
        time.sleep(0.01)

    assert executor.context.state == "paused"
    safe_hold = [event for event in events if event[0] == "task.safe_hold"][-1]
    assert "扫描" in safe_hold[1]["reason_message"]
    assert nav.arrival_adjustments[-1] == (0.0, 0.0, 0.0)
    executor.stop()
    store.close()


def test_live_pause_resume_continues_post_arrival_adjustment_without_nav2_redispatch(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    results = []
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(
        x=float(waypoint["x"]) + 0.40,
        y=float(waypoint["y"]),
        yaw=0.0,
    )
    executor._arrival_heading_completed_index = 0
    executor._set_post_arrival_stage(0, "heading_aligned")
    executor._start_arrival_side_effects(0, waypoint)
    assert executor._start_arrival_adjustment(waypoint, 0) is True

    deadline = time.time() + 1.0
    while not any(
        abs(vx) > 0.0 or abs(vy) > 0.0
        for vx, vy, _ in nav.arrival_adjustments
    ) and time.time() < deadline:
        time.sleep(0.01)
    executor.pause_task(envelope.payload["task_execution_id"])
    assert executor.context.state == "paused"
    assert executor.context.post_arrival_stage == "xy_adjusting"

    executor.resume_task(envelope.payload["task_execution_id"], 0)
    deadline = time.time() + 5.0
    while (executor.context.state == "running" or not results) and time.time() < deadline:
        time.sleep(0.02)

    assert executor.context.state == "completed"
    assert len(nav.sent) == 1
    assert results[-1][1] == "succeeded"
    executor.stop()
    store.close()


def test_restart_resumes_post_arrival_without_redispatch_or_duplicate_reached(tmp_path):
    from math import pi

    path = tmp_path / "edge.db"
    first_store = LocalStore(str(path))
    first_nav = FakeNavigation()
    envelope = command("task.start")
    waypoint = dict(envelope.payload["command"]["route_snapshot"]["waypoints"][0])
    waypoint.update({"require_yaw": True, "yaw": 0.0})
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [waypoint]
    first = TaskExecutor(
        first_store,
        first_nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    first.start_task(envelope)
    first.context.post_arrival_waypoint_index = 0
    first.context.post_arrival_stage = "heading_pending"
    first.context.arrival_side_effects_started = True
    first.context.state = "paused"
    first._persist()
    first_store.close()

    events = []
    results = []
    second_store = LocalStore(str(path))
    second_nav = FakeNavigation()
    second_nav.pose = SimpleNamespace(
        x=float(waypoint["x"]),
        y=float(waypoint["y"]),
        yaw=pi,
    )
    second = TaskExecutor(
        second_store,
        second_nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )

    response = second.resume_task(envelope.payload["task_execution_id"], 0)
    deadline = time.time() + 5.0
    while (second.context.state == "running" or not results) and time.time() < deadline:
        time.sleep(0.02)

    assert response["post_arrival_stage"] in {"heading_pending", "heading_aligned"}
    assert second.context.state == "completed"
    assert second_nav.sent == []
    assert not any(
        event[0] == "task.progress"
        and event[1].get("milestone") == "waypoint_reached"
        for event in events
    )
    assert results[-1][1] == "succeeded"
    second.stop()
    second_store.close()


def test_restart_preserves_fine_reapproach_identity_and_attempt_budget(tmp_path):
    path = tmp_path / "edge-reapproach.db"
    first_store = LocalStore(str(path))
    first_nav = FakeNavigation()
    envelope = command("task.start")
    first = TaskExecutor(
        first_store,
        first_nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    first.start_task(envelope)
    first.context.arrival_reapproach_waypoint_index = 0
    first.context.arrival_reapproach_attempts = 1
    first._persist()
    first_store.close()

    second_store = LocalStore(str(path))
    second = TaskExecutor(
        second_store,
        FakeNavigation(),
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    dispatched = []
    second._dispatch_navigation = lambda index, *, reapproach=False: dispatched.append(
        (index, reapproach)
    )

    second._send_from(0)

    assert second._arrival_reapproach_index == 0
    assert second._arrival_retry_counts == {0: 1}
    assert dispatched == [(0, True)]
    second_store.close()


def test_outdoor_rtk_route_selects_outdoor_detour_profile(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {
            "coordinate_mode": "rtk_fixed",
            "scene_scope": "outdoor",
        }
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)

    assert nav.outdoor_profiles[0] is True
    assert nav.waypoint_profiles[0][0] is True
    store.close()


def test_outdoor_patrol_stops_at_each_waypoint(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {
            "coordinate_mode": "rtk_fixed",
            "scene_scope": "outdoor",
        }
    )
    points = envelope.payload["command"]["route_snapshot"]["waypoints"]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)

    assert ids(nav.sent[0]) == ["wp-1"]
    # Single-pose goals face the outgoing leg toward the next waypoint.
    assert abs(
        nav.sent[0][0]["yaw"]
        - atan2(points[1]["y"] - points[0]["y"], points[1]["x"] - points[0]["x"])
    ) < 1e-6
    store.close()


def test_batch_last_pass_through_point_faces_outgoing_leg(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = SimpleNamespace(
        route_snapshot={
            "waypoints": [
                {"x": 0.0, "y": 0.0, "yaw": 0.0},
                {"x": 10.0, "y": 0.0, "yaw": 0.0},
                {"x": 20.0, "y": -5.0, "yaw": 0.0, "require_yaw": True},
            ]
        }
    )
    batch = [
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": 10.0, "y": 0.0, "yaw": 0.0},
    ]
    executor._apply_batch_travel_yaw(batch, 0)
    assert batch[1]["yaw"] == atan2(-5.0, 10.0)
    store.close()


def test_docking_final_waypoint_requires_precise_position_and_heading(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=0.0)
    envelope = command("task.start")
    envelope.payload["command"]["docking"] = {"enabled": True, "final_waypoint_index": 2}
    envelope.payload["command"]["route_snapshot"]["waypoints"][-1]["require_yaw"] = True
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
        docking_goal_tolerance_m=0.08,
        docking_goal_yaw_tolerance_rad=0.0872665,
    )

    executor.start_task(envelope)
    for _ in range(2):
        target = nav.sent[-1][-1]
        nav.pose = SimpleNamespace(
            x=float(target["x"]),
            y=float(target["y"]),
            yaw=float(target.get("yaw") or 0.0),
        )
        nav.result("succeeded", "", {"missed_waypoints": []})
    assert nav.goal_precisions[-1] is True
    assert nav.waypoint_profiles[-1] == (False, True, True)
    assert nav.arrival_goal_tolerances[-1] == (0.50, 0.25)

    final = envelope.payload["command"]["route_snapshot"]["waypoints"][-1]
    nav.pose = SimpleNamespace(x=final["x"] + 0.03, y=final["y"] - 0.02, yaw=final["yaw"] + 0.04)
    nav.result("succeeded", "", {"missed_waypoints": []})
    assert results[-1][1] == "succeeded"
    store.close()


def test_map_set_task_skips_segments_before_nearest_waypoint(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=2.1, y=3.1)
    coordinator = FakeMapSetCoordinator()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        map_set_coordinator=coordinator,
    )

    executor.start_task(command("task.start"))

    assert coordinator.activated == [coordinator.build_segments(executor.context.route_snapshot)[1]]
    assert executor.context.current_segment_index == 1
    assert ids(nav.sent[0]) == ["wp-2"]
    store.close()


def test_localization_loss_pauses_active_navigation(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))

    executor.on_localization_lost()

    assert executor.context.state == "paused"
    assert nav.cancelled == 1
    assert nav.stop_commands == 1
    assert [event[0] for event in events[-2:]] == ["task.pausing", "task.paused"]
    assert events[-2][1]["last_trusted_pose"] == nav.trusted_pose
    assert events[-2][1]["raw_pose"]["x"] == 5.0
    assert events[-2][1]["localization_quality"]["matching_error"] == 0.75
    assert events[-2][1]["current_waypoint"]["map_point_number"] == 1
    assert events[-2][1]["map_id"] == "site-a-main"
    assert "last_trusted_pose" not in events[-1][1]
    assert events[-1][1]["reason_code"] == "LOCALIZATION_LOST"
    store.close()


def test_localization_loss_cancel_timeout_stays_paused_and_auto_resumes(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.cancel_navigation = lambda timeout_seconds=5: False
    events = []
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    nav.feedback(0, None)

    executor.on_localization_lost()
    assert executor.context.state == "paused"
    assert results == []
    assert nav.stop_commands == 1

    # Align with the pending leg so recovery can cruise immediately.
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=atan2(1.0, 1.0))
    executor.on_localization_recovered()
    assert executor.context.state == "running"
    assert ids(nav.sent[-1]) == ["wp-1"]
    recovery_event = next(event for event in reversed(events) if event[0] == "task.resuming")
    assert recovery_event[1]["reason_code"] == "LOCALIZATION_RECOVERED"
    store.close()


def test_navigation_does_not_start_or_announce_obstacle_when_standup_fails(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeBlockedNavigation()
    nav.stand_confirmed = False
    events = []
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
        obstacle_speech=SimpleNamespace(
            enabled=True,
            obstacle_clear_seconds=3.0,
            no_progress_seconds=0.0,
            min_progress_m=0.5,
            obstacle_max_distance_m=0.9,
            collision_limit_ratio=0.6,
        ),
    )

    executor.start_task(command("task.start"))

    assert executor.context.state == "failed"
    assert nav.stand_requests == 1
    assert nav.sent == []
    assert not [event for event in events if event[0] == "task.obstacle_speech"]
    assert results[0][3] == "ROBOT_STANDUP_FAILED"
    executor.stop()
    store.close()


@pytest.mark.parametrize(("arrival_policy", "docking"), [("precision", False), ("dock", True)])
def test_precision_and_dock_without_required_yaw_ignore_final_heading(
    tmp_path, arrival_policy, docking
):
    from math import pi

    store = LocalStore(str(tmp_path / f"edge-{arrival_policy}.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    final_index = len(envelope.payload["command"]["route_snapshot"]["waypoints"]) - 1
    final = envelope.payload["command"]["route_snapshot"]["waypoints"][final_index]
    final.update({"arrival_policy": arrival_policy, "require_yaw": False, "yaw": 0.0})
    if docking:
        envelope.payload["command"]["docking"] = {
            "enabled": True,
            "final_waypoint_index": final_index,
        }
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(x=float(final["x"]), y=float(final["y"]), yaw=pi)

    assert executor._final_pose_error() is None

    executor.context.route_snapshot["waypoints"][final_index]["require_yaw"] = True
    assert executor._final_pose_error()[0] == "FINAL_YAW_OUT_OF_TOLERANCE"
    executor.stop()
    store.close()


def test_navigation_success_finishes_start_command(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    assert ids(nav.sent[0]) == ["wp-1"]
    drive_patrol(nav, until_state="completed", executor=executor)
    assert executor.context.state == "completed"
    assert results[0][1] == "succeeded"
    store.close()


def test_waypoint_dwell_delays_next_goal_without_blocking_result_callback(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["waypoints"][0]["dwell_seconds"] = 0.15
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=atan2(1.0, 1.0))

    started = time.monotonic()
    nav.result("succeeded", "", {"missed_waypoints": []})
    callback_elapsed = time.monotonic() - started

    assert callback_elapsed < 0.1
    assert ids(nav.sent[-1]) == ["wp-1"]
    assert any(
        event[0] == "task.progress" and event[1].get("milestone") == "waypoint_reached"
        for event in events
    )
    deadline = started + 1.0
    while len(nav.sent) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert ids(nav.sent[-1]) == ["wp-2"]
    assert time.monotonic() - started >= 0.14
    executor.stop()
    store.close()


def test_navigation_rosbag_follows_task_lifecycle(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    recorder = FakeRosbagRecorder()
    results = []
    envelope = command("task.start")
    envelope.payload["command"]["record_rosbag"] = True
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
        rosbag_recorder=recorder,
    )

    executor.start_task(envelope)
    assert recorder.started == [f"task_{executor.context.task_execution_id[:8]}"]
    assert executor.context.record_rosbag is True
    drive_patrol(nav, until_state="completed", executor=executor)

    assert recorder.stopped == 1
    assert results[0][2]["rosbag"]["size_bytes"] == 1024
    store.close()


def test_navigation_rosbag_force_exit_returns_stopped_status(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    recorder = FakeRosbagRecorder()
    envelope = command("task.start")
    envelope.payload["command"]["record_rosbag"] = True
    start_results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: start_results.append(args),
        rosbag_recorder=recorder,
    )

    executor.start_task(envelope)
    result = executor.force_exit(executor.context.task_execution_id)

    assert recorder.stopped == 1
    assert result["final_task_state"] == "cancelled"
    assert result["rosbag"]["running"] is False
    assert result["rosbag"]["size_bytes"] == 1024
    assert start_results[0][0] == envelope.payload["command_id"]
    assert start_results[0][1] == "cancelled"
    assert start_results[0][2]["final_task_state"] == "cancelled"
    store.close()


def test_force_exit_clears_context_when_result_publish_raises(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()

    def fail_publish(*_args):
        raise RuntimeError("mqtt offline")

    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=fail_publish,
    )
    envelope = command("task.start")
    executor.start_task(envelope)

    result = executor.force_exit(
        envelope.payload["task_execution_id"],
        reason_code="LOW_BATTERY",
    )

    assert result["final_task_state"] == "cancelled"
    assert executor.context is None
    assert store.load_active_task_context() is None
    store.close()


def test_navigation_rosbag_starts_a_fresh_recording_for_next_execution(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    recorder = FakeRosbagRecorder()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        rosbag_recorder=recorder,
    )

    first = command("task.start")
    first.payload["command"]["record_rosbag"] = True
    executor.start_task(first)
    nav.pose = SimpleNamespace(x=3.0, y=4.0)
    nav.result("succeeded", "", {"missed_waypoints": []})

    second = command("task.start")
    second.payload["task_execution_id"] = "55555555-5555-4555-8555-555555555555"
    second.payload["command_id"] = "66666666-6666-4666-8666-666666666666"
    second.payload["command"]["record_rosbag"] = True
    executor.start_task(second)

    assert recorder.stopped == 1
    assert recorder.started == ["task_44444444", "task_55555555"]
    store.close()


def test_navigation_rosbag_reuses_one_package_across_loop_rounds(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    recorder = FakeRosbagRecorder()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        rosbag_recorder=recorder,
    )
    loop_session_id = "63b66a16-1947-4be7-889b-d851a5f4ba20"

    first = command("task.start")
    first.payload["command"].update({
        "record_rosbag": True,
        "loop_execution": True,
        "loop_session_id": loop_session_id,
        "continuous_rosbag": True,
        "round_number": 1,
    })
    executor.start_task(first)
    drive_patrol(nav, until_state="completed", executor=executor)

    second = command("task.start")
    second.payload["task_execution_id"] = "55555555-5555-4555-8555-555555555555"
    second.payload["command_id"] = "66666666-6666-4666-8666-666666666666"
    second.payload["command"].update({
        "record_rosbag": True,
        "loop_execution": True,
        "loop_session_id": loop_session_id,
        "continuous_rosbag": True,
        "round_number": 2,
        "loop_direction": "reverse",
    })
    executor.start_task(second)

    assert recorder.started == ["loop_63b66a1619474be7889bd851a5f4ba20"]
    assert recorder.stopped == 0
    assert recorder.running is True

    stopped = executor.stop_loop_rosbag(loop_session_id)

    assert recorder.stopped == 1
    assert stopped["running"] is False
    assert stopped["ignored"] is False
    store.close()


def test_delayed_loop_rosbag_stop_does_not_stop_newer_recording(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    recorder = FakeRosbagRecorder()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        rosbag_recorder=recorder,
    )
    current_session = "63b66a16-1947-4be7-889b-d851a5f4ba20"
    stale_session = "73b66a16-1947-4be7-889b-d851a5f4ba20"
    envelope = command("task.start")
    envelope.payload["command"].update({
        "record_rosbag": True,
        "loop_execution": True,
        "loop_session_id": current_session,
        "continuous_rosbag": True,
    })
    executor.start_task(envelope)

    result = executor.stop_loop_rosbag(stale_session)

    assert result["ignored"] is True
    assert recorder.running is True
    assert recorder.stopped == 0
    executor.stop_loop_rosbag(current_session)
    store.close()


def test_navigation_rosbag_replaces_a_stale_recorder_before_task_start(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    recorder = FakeRosbagRecorder()
    recorder.running = True
    envelope = command("task.start")
    envelope.payload["command"]["record_rosbag"] = True
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        rosbag_recorder=recorder,
    )

    executor.start_task(envelope)

    assert recorder.stopped == 1
    assert recorder.started == ["task_44444444"]
    assert recorder.running is True
    store.close()


def test_navigation_missed_waypoints_fails_task(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    nav.result("succeeded", "", {"missed_waypoints": [2]})
    assert executor.context.state == "failed"
    assert results[0][1] == "failed"
    assert results[0][3] == "NAVIGATION_MISSED_WAYPOINTS"
    store.close()


def test_pass_through_waypoints_use_travel_heading(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.0, y=2.0)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    sent = nav.sent[0]
    assert ids(sent) == ["wp-1"]
    assert abs(sent[0]["yaw"] - atan2(1.0, 1.0)) < 1e-6
    store.close()


def _pass_through_outdoor_start(nav):
    envelope = command("task.start")
    envelope.payload["command"]["map"].update(
        {"coordinate_mode": "rtk_fixed", "scene_scope": "outdoor"}
    )
    envelope.payload["command"]["route_snapshot"]["scene_scope"] = "outdoor"
    envelope.payload["command"]["route_snapshot"]["map"] = dict(
        envelope.payload["command"]["map"]
    )
    for waypoint in envelope.payload["command"]["route_snapshot"]["waypoints"]:
        waypoint["arrival_policy"] = "pass_through"
    return envelope


def test_pass_through_skips_last_metre_hunt_when_already_close(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # 0.6 m from wp-1: inside the 1 m last-metre window, outside the on-click skip.
    nav.pose = SimpleNamespace(x=0.4, y=2.0, yaw=1.6)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(_pass_through_outdoor_start(nav))

    assert ids(nav.sent[0]) == ["wp-1"]
    assert nav.teleop == []
    assert executor._departure_heading_index is None
    assert executor._patrol_final_approach_applied is False
    assert nav.waypoint_profiles[-1][2] is False

    nav.feedback(0, 0.4)
    assert executor._patrol_final_approach_applied is False
    assert nav.waypoint_profiles[-1][2] is False
    store.close()


def test_last_pass_through_waypoint_stays_cruise_inside_last_metre(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = _pass_through_outdoor_start(nav)
    last = envelope.payload["command"]["route_snapshot"]["waypoints"][-1]
    nav.pose = SimpleNamespace(x=float(last["x"]) - 0.6, y=float(last["y"]), yaw=0.0)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    _await_departure_heading(executor)
    drive_patrol(nav, until_ids=[last["waypoint_id"]], executor=executor)

    assert executor._patrol_final_approach_applied is False
    assert nav.waypoint_profiles[-1][2] is False

    nav.feedback(0, 0.4)
    assert executor._patrol_final_approach_applied is False
    assert nav.waypoint_profiles[-1][2] is False
    store.close()


def test_pass_through_faces_travel_direction_when_heading_is_off(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # 1.5 m from wp-1 with ~92° heading error: cruise must face the leg first.
    nav.pose = SimpleNamespace(x=-0.5, y=2.0, yaw=1.6)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(_pass_through_outdoor_start(nav))

    _await_departure_heading(executor)
    assert ids(nav.sent[0]) == ["wp-1"]
    assert nav.teleop
    assert any(abs(cmd[2]) > 0 for cmd in nav.teleop)

    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=3.0)
    nav.teleop.clear()
    nav.result("succeeded", "", {"missed_waypoints": []})
    _await_departure_heading(executor)
    assert executor.context.current_waypoint_index == 1
    assert ids(nav.sent[-1]) == ["wp-2"]
    store.close()


def test_pass_through_does_not_spin_on_the_click(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # Already inside the 1 m click window; spinning here restarts the orbit.
    nav.pose = SimpleNamespace(x=0.4, y=2.0, yaw=1.6)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(_pass_through_outdoor_start(nav))

    assert ids(nav.sent[0]) == ["wp-1"]
    assert nav.teleop == []
    assert executor._maybe_face_travel_direction(0) is False
    store.close()


def test_straighten_pass_through_flattens_click_noise_but_keeps_real_turns():
    noisy_line = [
        {"waypoint_id": "a", "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"waypoint_id": "b", "x": 10.0, "y": 0.25, "yaw": 0.0},
        {"waypoint_id": "c", "x": 20.0, "y": -0.18, "yaw": 0.0},
        {"waypoint_id": "d", "x": 30.0, "y": 0.05, "yaw": 0.0},
    ]
    straight = straighten_pass_through_waypoints(noisy_line)
    assert straight[0]["x"] == 0.0 and straight[0]["y"] == 0.0
    assert straight[-1]["x"] == 30.0
    assert hypot(straight[1]["x"] - 10.0, straight[1]["y"]) < 0.05
    assert hypot(straight[2]["x"] - 20.0, straight[2]["y"]) < 0.05
    assert hypot(straight[1]["x"] - noisy_line[1]["x"], straight[1]["y"] - noisy_line[1]["y"]) > 0.15

    corner = [
        {"waypoint_id": "a", "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"waypoint_id": "b", "x": 10.0, "y": 0.0, "yaw": 0.0},
        {"waypoint_id": "c", "x": 10.0, "y": 10.0, "yaw": 0.0},
    ]
    kept = straighten_pass_through_waypoints(corner)
    assert kept[1]["x"] == 10.0
    assert kept[1]["y"] == 0.0


def test_patrol_nav2_goal_uses_straightened_corridor(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=0.0, y=0.0)
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [
        {"waypoint_id": "wp-1", "sequence": 0, "name": "A", "x": 0.0, "y": 0.0, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-2", "sequence": 1, "name": "B", "x": 10.0, "y": 0.25, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-3", "sequence": 2, "name": "C", "x": 20.0, "y": -0.18, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-4", "sequence": 3, "name": "D", "x": 30.0, "y": 0.0, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
    ]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    sent = nav.sent[0]
    assert ids(sent) == ["wp-1"]
    assert sent[0]["y"] == 0.0
    store.close()


def test_navigation_success_requires_final_pose_near_last_waypoint(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.0, y=2.0)
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    assert ids(nav.sent[0]) == ["wp-1"]
    drive_patrol(nav, until_ids=["wp-3"], executor=executor)
    nav.feedback(0, 0.8)
    assert nav.waypoint_profiles[-1][2] is True
    assert nav.live_profiles[-1] is True
    final = executor.context.route_snapshot["waypoints"][-1]
    face = atan2(float(final["y"]) - 2.0, float(final["x"]) - 1.0)
    # Stay far from the final waypoint so indoor click checks re-approach,
    # then safely pause once bounded convergence retries are exhausted.
    for _ in range(6):
        if executor.context.state == "paused":
            break
        nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=face)
        if executor._departure_heading_thread is not None:
            _await_departure_heading(executor)
        if nav.result is None:
            break
        nav.result("succeeded", "", {"missed_waypoints": []})
    assert nav.stop_commands >= 1
    assert executor.context.state == "paused"
    assert results == []
    store.close()


def test_patrol_coarse_arrival_continues_without_micro_adjustment(tmp_path):
    store = LocalStore(str(tmp_path / "edge-coarse.db"))
    nav = FakeNavigation()
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    assert ids(nav.sent[0]) == ["wp-1"]
    drive_patrol(nav, until_ids=["wp-3"], executor=executor)
    final = executor.context.route_snapshot["waypoints"][-1]
    nav.pose = SimpleNamespace(
        x=float(final["x"]) + 0.42,
        y=float(final["y"]),
        yaw=float(final.get("yaw") or 0.0),
    )
    if executor._departure_heading_thread is not None:
        _await_departure_heading(executor)
    nav.result("succeeded", "", {"missed_waypoints": []})
    deadline = time.time() + 2.0
    while len(nav.sent) < 4 and time.time() < deadline:
        time.sleep(0.05)
    assert executor.context.state == "running"
    assert ids(nav.sent[-1]) == ["wp-3"]
    assert nav.arrival_goal_tolerances[-1] == (0.30, 3.14)

    # The single Nav2 fine re-approach still leaves the corrected pose inside
    # 0.50 m, so ordinary stop-and-confirm may now accept the coarse fallback.
    nav.pose = SimpleNamespace(
        x=float(final["x"]) + 0.42,
        y=float(final["y"]),
        yaw=float(final.get("yaw") or 0.0),
    )
    nav.result("succeeded", "", {"missed_waypoints": []})
    deadline = time.time() + 2.0
    while executor.context.state == "running" and time.time() < deadline:
        time.sleep(0.05)
    assert executor.context.state == "completed"
    assert results[-1][1] == "succeeded"
    assert not any(
        abs(vx) > 0.0 or abs(vy) > 0.0 or abs(yaw_rate) > 0.0
        for vx, vy, yaw_rate in nav.arrival_adjustments
    )
    executor.stop()
    store.close()


def test_patrol_initial_xy_failure_uses_nav2_reapproach_not_micro_adjustment(tmp_path):
    store = LocalStore(str(tmp_path / "edge-1.19.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    assert ids(nav.sent[0]) == ["wp-1"]
    drive_patrol(nav, until_ids=["wp-3"], executor=executor)
    final = executor.context.route_snapshot["waypoints"][-1]
    nav.pose = SimpleNamespace(
        x=float(final["x"]) + 1.19,
        y=float(final["y"]),
        yaw=float(final.get("yaw") or 0.0),
    )
    if executor._departure_heading_thread is not None:
        _await_departure_heading(executor)
    nav.result("succeeded", "", {"missed_waypoints": []})
    deadline = time.time() + 2.0
    while len(nav.sent) < 4 and time.time() < deadline:
        time.sleep(0.05)
    assert executor.context.state == "running"
    assert ids(nav.sent[-1]) == ["wp-3"]
    assert not any(
        abs(vx) > 0.0 or abs(vy) > 0.0 or abs(yaw_rate) > 0.0
        for vx, vy, yaw_rate in nav.arrival_adjustments
    )
    executor.stop()
    store.close()


def test_indoor_patrol_dispatches_single_waypoint_goals(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    assert ids(nav.sent[0]) == ["wp-1"]
    store.close()


def test_single_speech_waypoint_uses_cruise_until_final_metre(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=-8.0, y=-8.0, yaw=atan2(2.0 - (-8.0), 1.0 - (-8.0)))
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["waypoints"][0].update(
        {"speech_template_id": 6, "speech_template_name": "森林火灾"}
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        waypoint_speech=SimpleNamespace(
            status_dir=str(tmp_path / "audio-status"),
            timeout_seconds=2.0,
            poll_interval_seconds=0.01,
            enabled=True,
            block_navigation=True,
        ),
    )

    executor.start_task(envelope)

    assert ids(nav.sent[0]) == ["wp-1"]
    assert nav.waypoint_profiles[-1] == (True, False, False)
    assert nav.live_profiles[-1] is False

    nav.feedback(0, 0.7)
    assert nav.waypoint_profiles[-1] == (True, False, True)
    assert nav.live_profiles[-1] is True
    store.close()


def test_repeated_nav2_feedback_is_throttled_to_one_progress_update_per_second(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    initial_version = executor.context.state_version
    initial_progress_count = len([event for event in events if event[0] == "task.progress"])

    for _ in range(50):
        nav.feedback(0, 4.0)

    assert executor.context.state_version == initial_version
    assert len([event for event in events if event[0] == "task.progress"]) == initial_progress_count

    executor._last_progress_emit_at -= 1.1
    nav.feedback(0, 3.9)

    assert executor.context.state_version == initial_version + 1
    assert len([event for event in events if event[0] == "task.progress"]) == initial_progress_count + 1
    store.close()


def test_restart_reports_interrupted_task_and_closes_start_command(tmp_path):
    db_path = str(tmp_path / "edge.db")
    store = LocalStore(db_path)
    executor = TaskExecutor(
        store,
        FakeNavigation(),
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    store.close()

    recovered_store = LocalStore(db_path)
    events = []
    results = []
    recovered = TaskExecutor(
        recovered_store,
        FakeNavigation(),
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )
    recovered.report_startup_interruption()

    assert recovered.context.state == "failed"
    assert events[0][0] == "task.failed"
    assert results[0][1] == "failed"
    assert results[0][2]["final_task_state"] == "failed"
    assert results[0][3] == "EDGE_RESTARTED"
    recovered_store.close()


def test_obstacle_speech_escalates_after_three_no_progress_recovery_attempts(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeBlockedNavigation()
    events = []
    speech = SimpleNamespace(
        enabled=True,
        no_progress_seconds=0.0,
        min_progress_m=0.08,
        obstacle_max_distance_m=0.9,
        obstacle_clear_seconds=3.0,
        collision_limit_ratio=0.6,
        announce=True,
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        obstacle_speech=speech,
    )
    executor.start_task(command("task.start"))
    executor._evaluate_obstacle_progress()  # first front obstacle detection
    executor._evaluate_obstacle_progress()  # recovery attempt 1
    executor._evaluate_obstacle_progress()  # recovery attempt 2
    executor._evaluate_obstacle_progress()  # recovery attempt 3
    executor._evaluate_obstacle_progress()  # three attempts exhausted + leave-route
    speech_events = [event for event in events if event[0] == "task.obstacle_speech"]
    assert [event[1]["template_name"] for event in speech_events] == [
        "发现障碍物",
        "后退尝试避障",
        "后退尝试避障",
        "后退尝试避障",
        "劝阻离开线路",
    ]
    assert [event[1]["recovery_attempt"] for event in speech_events] == [0, 1, 2, 3, 3]
    assert nav.costmap_clears >= 1
    assert nav.cancelled == 4
    assert len(nav.obstacle_recoveries) == 3
    assert all(call["reverse_distance_m"] == 0.25 for call in nav.obstacle_recoveries)
    assert all(call["lateral_distance_m"] == 0.20 for call in nav.obstacle_recoveries)
    assert executor._obstacle_stage == "SAFE_OBSERVING"
    executor.stop()
    store.close()


def test_successful_recovery_motion_still_counts_when_same_waypoint_remains_blocked(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeBlockedNavigation()
    events = []
    evidence = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        obstacle_speech=SimpleNamespace(
            enabled=True,
            announce=False,
            no_progress_seconds=0.0,
            min_progress_m=0.5,
            obstacle_max_distance_m=0.9,
            obstacle_clear_seconds=3.0,
            collision_limit_ratio=0.6,
            max_recovery_attempts=3,
        ),
        obstacle_evidence=lambda payload: evidence.append(payload),
    )
    executor._start_obstacle_monitor = lambda: None
    executor.start_task(command("task.start"))

    executor._evaluate_obstacle_progress()  # detection
    executor._evaluate_obstacle_progress()  # successful backup + left shift

    assert nav.obstacle_recoveries[-1]["lateral_direction"] == 1
    assert executor._recovery_attempts == 1
    recovery_results = [
        item[1]
        for item in events
        if item[0] == "task.obstacle_stage"
        and item[1]["stage"] == "RECOVERY_ATTEMPT"
        and item[1]["action_result"]
    ]
    assert recovery_results[-1]["action_result"]["success"] is True
    assert recovery_results[-1]["alert_description"] == "正在进行第 1/3 次后退绕行避障"
    assert len(evidence) == 1
    assert evidence[0]["alert_description"] == "发现障碍物，已停车"

    executor._evaluate_obstacle_progress()  # obstacle is still ahead: attempt 2
    assert executor._recovery_attempts == 2
    executor._suspend_obstacle_monitor()
    executor._sync_obstacle_waypoint_budget()
    assert executor._recovery_attempts == 2

    executor.context.current_waypoint_index += 1
    executor._sync_obstacle_waypoint_budget()
    assert executor._recovery_attempts == 0
    assert executor._obstacle_episode_id is None
    executor.stop()
    store.close()


def test_new_obstacle_episode_same_waypoint_keeps_recovery_budget(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeBlockedNavigation()
    blocked_snapshot = nav.obstacle_monitor_snapshot
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        obstacle_speech=SimpleNamespace(
            enabled=True,
            announce=False,
            no_progress_seconds=0.0,
            min_progress_m=0.5,
            obstacle_max_distance_m=0.9,
            obstacle_clear_seconds=3.0,
            collision_limit_ratio=0.6,
            max_recovery_attempts=3,
        ),
    )
    executor._start_obstacle_monitor = lambda: None
    executor.start_task(command("task.start"))
    executor._evaluate_obstacle_progress()
    executor._evaluate_obstacle_progress()
    first_episode = executor._obstacle_episode_id
    assert executor._recovery_attempts == 1

    nav.obstacle_monitor_snapshot = lambda: {
        "front_obstacle_distance_m": None,
        "stale": False,
        "localization_normal": True,
        "collision_monitor": {"state": "CLEAR", "reason": "clear", "sample_age_seconds": 0.02},
    }
    executor._obstacle_clear_started_at = time.monotonic() - 3.1
    executor._evaluate_obstacle_progress()
    assert executor._obstacle_episode_id is None
    assert executor._recovery_attempts == 1

    nav.obstacle_monitor_snapshot = blocked_snapshot
    executor._evaluate_obstacle_progress()
    executor._evaluate_obstacle_progress()
    assert executor._obstacle_episode_id != first_episode
    assert executor._recovery_attempts == 2
    executor.stop()
    store.close()


def test_obstacle_recovery_rejects_unknown_rear_clearance(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeBlockedNavigation()
    observation = nav.obstacle_monitor_snapshot()
    observation["rear_clearance_m"] = None
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.obstacle_speech = SimpleNamespace()
    selection = executor._select_obstacle_recovery_direction(observation)
    assert selection == {"safe": False, "reason": "rear_clearance_unknown"}
    store.close()


def test_detour_disabled_skips_motion_and_enters_safe_observing(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeBlockedNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        obstacle_speech=SimpleNamespace(
            enabled=True,
            announce=True,
            no_progress_seconds=0.0,
            min_progress_m=0.5,
            obstacle_max_distance_m=0.9,
            collision_limit_ratio=0.6,
            obstacle_clear_seconds=3.0,
        ),
    )
    executor.start_task(command("task.start"))
    executor._segment_avoidance_enabled = False
    executor._evaluate_obstacle_progress()

    assert nav.obstacle_recoveries == []
    assert executor._obstacle_stage == "SAFE_OBSERVING"
    stages = [item[1]["stage"] for item in events if item[0] == "task.obstacle_stage"]
    assert stages == ["DETECTED_STOP", "DISSUASION", "SAFE_OBSERVING"]
    executor.stop()
    store.close()


def test_manual_continue_rechecks_stable_clear_window_before_redispatch(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeBlockedNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        obstacle_speech=SimpleNamespace(
            enabled=True,
            announce=False,
            no_progress_seconds=0.0,
            min_progress_m=0.5,
            obstacle_max_distance_m=0.9,
            collision_limit_ratio=0.6,
            obstacle_clear_seconds=3.0,
        ),
    )
    executor.start_task(command("task.start"))
    executor._segment_avoidance_enabled = False
    executor._evaluate_obstacle_progress()
    sent_before = len(nav.sent)

    with pytest.raises(ProtocolError) as blocked:
        executor.resume_forward(executor.context.task_execution_id)
    assert blocked.value.code == "OBSTACLE_NOT_CLEAR"

    nav.obstacle_monitor_snapshot = lambda: {
        "requested_planar_speed_mps": 0.0,
        "actual_planar_speed_mps": 0.0,
        "requested_turn_speed_rps": 0.0,
        "actual_turn_speed_rps": 0.0,
        "front_obstacle_distance_m": None,
        "rear_clearance_m": 4.0,
        "left_clearance_m": 4.0,
        "right_clearance_m": 4.0,
        "stale": False,
        "localization_normal": True,
        "collision_monitor": {"state": "CLEAR", "reason": "clear", "sample_age_seconds": 0.02},
    }
    executor._obstacle_clear_started_at = time.monotonic() - 3.1
    result = executor.resume_forward(executor.context.task_execution_id)

    assert result["resumed_forward"] is True
    assert len(nav.sent) == sent_before + 1
    assert executor._obstacle_episode_id is None
    executor.stop()
    store.close()


def test_bt_recovery_is_suppressed_while_edge_owns_obstacle_episode(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    executor = TaskExecutor(
        store,
        FakeNavigation(),
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor._obstacle_episode_id = "episode-owned-by-edge"

    assert executor.acquire_recovery("BT_NAVIGATOR", "backup", distance_m=0.25) is None
    edge_lease = executor.acquire_recovery("EDGE_OBSTACLE", "backup", distance_m=0.25)
    assert edge_lease is not None
    assert executor.release_recovery(edge_lease) is True
    store.close()


def test_obstacle_monitor_loop_survives_iteration_failure(tmp_path):
    class StopAfterTwoIterations:
        def __init__(self):
            self.waits = 0
            self.stopped = False

        def wait(self, _timeout):
            self.waits += 1
            return self.stopped or self.waits > 2

        def set(self):
            self.stopped = True

    store = LocalStore(str(tmp_path / "edge.db"))
    executor = TaskExecutor(
        store,
        FakeNavigation(),
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    calls = []

    def evaluate():
        calls.append(True)
        if len(calls) == 1:
            raise RuntimeError("transient monitor failure")

    executor._obstacle_monitor_stop = StopAfterTwoIterations()
    executor._evaluate_obstacle_progress = evaluate
    executor._obstacle_monitor_loop()

    assert len(calls) == 2
    executor.stop()
    store.close()


def test_send_from_faces_next_waypoint_before_cruise_when_heading_is_wrong(tmp_path):
    from math import pi

    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    # Face west; next waypoint is due east → must turn in place first.
    nav.pose = SimpleNamespace(x=0.0, y=0.0, yaw=pi)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = SimpleNamespace(
        state="running",
        state_version=1,
        task_execution_id="exec-preleg",
        current_waypoint_index=1,
        current_segment_index=0,
        record_rosbag=False,
        route_snapshot={
            "waypoints": [
                {"waypoint_id": "wp-0", "x": 0.0, "y": 0.0, "yaw": 0.0, "map_point_number": 1},
                {"waypoint_id": "wp-1", "x": 10.0, "y": 0.0, "yaw": 0.0, "map_point_number": 2},
            ]
        },
        docking=None,
        environment_type="indoor",
    )
    executor._segments = []
    cruised = []
    feedback = []
    executor.on_feedback = lambda *args, **kwargs: feedback.append((args, kwargs))
    executor._dispatch_navigation = lambda index: cruised.append(index)
    executor._send_from(1)
    assert executor._departure_heading_index == 0
    assert executor._departure_cruise_index == 1
    assert executor._departure_heading_mode == "teleop"
    assert nav.sent == []
    assert any(kw.get("milestone") == "departure_heading_dispatched" for _, kw in feedback)
    _await_departure_heading(executor)
    assert executor._departure_heading_index is None
    assert executor._departure_cruise_index is None
    assert cruised == [1]
    assert any(abs(cmd[2]) > 0 for cmd in nav.teleop)
    executor.stop()
    store.close()


def test_send_from_skips_pre_leg_turn_when_already_facing_target(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=0.0, y=0.0, yaw=0.05)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = SimpleNamespace(
        state="running",
        state_version=1,
        task_execution_id="exec-preleg-aligned",
        current_waypoint_index=1,
        current_segment_index=0,
        record_rosbag=False,
        route_snapshot={
            "waypoints": [
                {"waypoint_id": "wp-0", "x": 0.0, "y": 0.0, "yaw": 0.0, "map_point_number": 1},
                {"waypoint_id": "wp-1", "x": 10.0, "y": 0.0, "yaw": 0.0, "map_point_number": 2},
            ]
        },
        docking=None,
        environment_type="outdoor",
    )
    executor._segments = []
    cruised = []
    executor._dispatch_navigation = lambda index: cruised.append(index)
    executor._send_from(1)
    assert executor._departure_heading_index is None
    assert executor._departure_cruise_index is None
    assert cruised == [1]
    assert nav.sent == []
    store.close()


def test_collision_monitor_speed_reduction_triggers_first_obstacle_speech(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    events = []
    executor = TaskExecutor(
        store,
        FakeCollisionLimitedNavigation(),
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        obstacle_speech=SimpleNamespace(
            enabled=True,
            no_progress_seconds=4.0,
            min_progress_m=0.08,
            obstacle_max_distance_m=0.9,
            collision_limit_ratio=0.6,
        ),
    )
    executor.start_task(command("task.start"))
    executor._evaluate_obstacle_progress()
    speech_events = [event for event in events if event[0] == "task.obstacle_speech"]
    assert len(speech_events) == 1
    assert speech_events[0][1]["template_name"] == "发现障碍物"
    executor.stop()
    store.close()


def test_report_docking_charge_emits_without_name_error(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    events = []
    executor = TaskExecutor(
        store,
        FakeNavigation(),
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    executor.context.docking = {"enabled": True, "final_waypoint_index": 1}

    executor.report_docking_charge(
        "task.docking_contact_checking", message="已到充电桩，正在检查蓝牙与极片"
    )

    docking_events = [event for event in events if event[0] == "task.docking_contact_checking"]
    assert len(docking_events) == 1
    executor.stop()
    store.close()


def test_obstacle_reverse_cancel_does_not_stop_running_task(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeBlockedNavigation()
    speech = SimpleNamespace(
        enabled=True,
        no_progress_seconds=0.0,
        min_progress_m=0.08,
        obstacle_max_distance_m=0.9,
        reverse_speed_mps=0.12,
        reverse_duration_seconds=0.2,
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        obstacle_speech=speech,
    )
    executor.start_task(command("task.start"))
    executor._evaluate_obstacle_progress()
    executor._evaluate_obstacle_progress()
    executor._evaluate_obstacle_progress()
    executor._evaluate_obstacle_progress()  # reverse + redispatch
    nav.result("cancelled")
    assert executor.context.state == "running"
    executor.stop()
    store.close()


def test_obstacle_monitor_does_not_hold_task_lock_during_cancel_callback(tmp_path):
    class ConcurrentCancelNavigation(FakeBlockedNavigation):
        def __init__(self):
            super().__init__()
            self.cancel_callback_completed_while_waiting = False

        def cancel_navigation(self, timeout_seconds=5):
            self.cancelled += 1
            completed = threading.Event()

            def deliver_result():
                self.result("cancelled")
                completed.set()

            callback_thread = threading.Thread(target=deliver_result, daemon=True)
            callback_thread.start()
            self.cancel_callback_completed_while_waiting = completed.wait(0.5)
            callback_thread.join(timeout=1.0)
            return self.cancel_callback_completed_while_waiting

    class RunOneMonitorIteration:
        def __init__(self):
            self.waits = 0

        def wait(self, _timeout):
            self.waits += 1
            return self.waits > 1

        def set(self):
            self.waits = 2

    store = LocalStore(str(tmp_path / "edge.db"))
    nav = ConcurrentCancelNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        obstacle_speech=SimpleNamespace(
            enabled=True,
            no_progress_seconds=0.0,
            min_progress_m=0.08,
            obstacle_max_distance_m=0.9,
            reverse_speed_mps=0.12,
            reverse_duration_seconds=0.2,
        ),
    )
    executor._start_obstacle_monitor = lambda: None
    executor.start_task(command("task.start"))
    executor._evaluate_obstacle_progress()
    executor._evaluate_obstacle_progress()
    executor._evaluate_obstacle_progress()
    executor._obstacle_monitor_stop = RunOneMonitorIteration()

    executor._obstacle_monitor_loop()

    assert nav.cancel_callback_completed_while_waiting
    assert executor._expected_recovery_cancels == 0
    assert executor.context.state == "running"
    executor.stop()
    store.close()

def test_nav_stack_not_ready_retries_instead_of_failing(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.reject_remaining = 2
    events = []
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda event_type, payload, _code: events.append((event_type, payload)),
        start_result_callback=lambda *args: results.append(args),
        navigation_dispatch_retry_seconds=0.05,
        navigation_dispatch_retry_budget_seconds=2.0,
    )
    executor.start_task(command("task.start"))
    assert executor.context.state == "running"
    assert results == []
    for _ in range(80):
        if len(nav.sent) >= 3 and executor._nav_dispatch_retry_timer is None:
            break
        time.sleep(0.05)
    assert len(nav.sent) >= 3
    assert executor.context.state == "running"
    assert results == []
    assert any(event[0] == "task.progress" and event[1].get("nav_stack_retry") for event in events)
    store.close()


def test_nav_stack_not_ready_fails_only_after_retry_budget(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.reject_remaining = 100
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
        navigation_dispatch_retry_seconds=0.05,
        navigation_dispatch_retry_budget_seconds=0.2,
    )
    executor.start_task(command("task.start"))
    for _ in range(80):
        if results:
            break
        time.sleep(0.05)
    assert results
    assert results[0][1] == "failed"
    assert results[0][3] == "NAV_STACK_NOT_READY"
    assert "after" in results[0][4]
    store.close()
