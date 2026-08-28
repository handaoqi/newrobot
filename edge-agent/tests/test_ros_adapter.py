import math
import threading
from types import SimpleNamespace

from roamerx_edge.config import SafetyConfig
from roamerx_edge.ros_adapter import RosAdapter, follow_path_patrol_params


class FakeTelemetry:
    def __init__(self, decision=None):
        self.decision = decision or {}
        self.scan_samples = 0

    def on_scan_matching_status(self, _msg):
        self.scan_samples += 1

    def localization_decision(self):
        return dict(self.decision)


def test_fresh_normal_streak_ignores_samples_before_candidate():
    samples = [(10, 3), (11, 3), (12, 3)]

    assert RosAdapter._fresh_normal_streak(samples, after_sequence=12) == 0


def test_fresh_normal_streak_requires_consecutive_successes():
    samples = [(20, 4), (21, 3), (22, 3), (23, 3)]

    assert RosAdapter._fresh_normal_streak(samples, after_sequence=20) == 3

    samples.append((24, 4))
    assert RosAdapter._fresh_normal_streak(samples, after_sequence=20) == 0


def test_relocalization_candidates_cover_full_yaw_and_nearby_positions():
    candidates = RosAdapter._relocalization_candidates(4.0, 5.0, 0.0, 0.0)

    center_yaws = {
        round(candidate["yaw"], 6)
        for candidate in candidates
        if candidate["x"] == 4.0 and candidate["y"] == 5.0
    }
    assert len(center_yaws) == 8
    assert round(math.pi, 6) in {round(abs(yaw), 6) for yaw in center_yaws}
    assert {(candidate["x"], candidate["y"]) for candidate in candidates[8:]} == {
        (5.0, 5.0),
        (3.0, 5.0),
        (4.0, 6.0),
        (4.0, 4.0),
    }


def test_ndt_score_over_threshold_triggers_recovery_after_hysteresis():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry()
    adapter.safety_config = SafetyConfig(ndt_failure_score=0.5, ndt_failure_samples=3)
    adapter._ndt_failure_count = 0
    adapter._ndt_failure_notified = False
    adapter._localization_recovery_armed = False
    triggered = threading.Event()
    reasons: list[str] = []

    def _on_failure(reason):
        reasons.append(reason)
        triggered.set()

    adapter._localization_failure_cb = _on_failure
    bad = SimpleNamespace(has_converged=True, matching_error=0.51, inlier_fraction=0.5)

    adapter._on_scan_matching_status(bad)
    adapter._on_scan_matching_status(bad)
    assert not triggered.is_set()
    adapter._on_scan_matching_status(bad)

    assert triggered.wait(1.0)
    assert adapter._localization_recovery_armed is True
    # NDT degradation must be reported as its own cause, not as a hard pose loss.
    assert reasons == ["ndt_degraded"]


def test_ndt_score_at_threshold_is_not_healthy():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry()
    adapter.safety_config = SafetyConfig(ndt_failure_score=0.5, ndt_failure_samples=1)
    adapter._ndt_failure_count = 0
    adapter._ndt_failure_notified = False
    adapter._localization_recovery_armed = False
    triggered = threading.Event()
    reasons: list[str] = []

    def _on_failure(reason):
        reasons.append(reason)
        triggered.set()

    adapter._localization_failure_cb = _on_failure

    adapter._on_scan_matching_status(
        SimpleNamespace(has_converged=True, matching_error=0.5, inlier_fraction=0.5)
    )

    assert triggered.wait(1.0)
    assert reasons == ["ndt_degraded"]


def test_only_absolute_ndt_or_rtk_decision_is_trusted():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry({"active_source": "imu_odom_bridge", "absolute_stable": False})
    assert adapter._absolute_localization_stable() is False
    adapter.telemetry.decision = {"active_source": "ndt_imu", "absolute_stable": True}
    assert adapter._absolute_localization_stable() is True
    adapter.telemetry.decision = {"active_source": "lio_imu", "absolute_stable": True}
    assert adapter._absolute_localization_stable() is True


def test_rtk_primary_ignores_ndt_degradation():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry({
        "active_source": "rtk_imu",
        "rtk_usable": True,
        "absolute_stable": True,
    })
    adapter.safety_config = SafetyConfig(ndt_failure_score=0.5, ndt_failure_samples=1)
    adapter._ndt_failure_count = 0
    adapter._ndt_failure_notified = False
    adapter._localization_recovery_armed = False
    triggered = threading.Event()

    adapter._localization_failure_cb = lambda reason: triggered.set()
    bad = SimpleNamespace(has_converged=False, matching_error=1.2, inlier_fraction=0.0)
    adapter._on_scan_matching_status(bad)
    adapter._on_scan_matching_status(bad)
    adapter._on_scan_matching_status(bad)
    assert not triggered.is_set()
    assert adapter._ndt_failure_count == 0


def test_lio_primary_ndt_degradation_triggers_recovery():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry({
        "active_source": "lio_imu",
        "absolute_stable": True,
    })
    adapter.safety_config = SafetyConfig(ndt_failure_score=0.5, ndt_failure_samples=1)
    adapter._ndt_failure_count = 0
    adapter._ndt_failure_notified = False
    adapter._localization_recovery_armed = False
    triggered = threading.Event()
    reasons: list[str] = []

    def _on_failure(reason):
        reasons.append(reason)
        triggered.set()

    adapter._localization_failure_cb = _on_failure
    bad = SimpleNamespace(has_converged=False, matching_error=1.2, inlier_fraction=0.0)
    adapter._on_scan_matching_status(bad)

    assert triggered.wait(1.0)
    assert reasons == ["ndt_degraded"]
    assert adapter._localization_recovery_armed is True


def test_patrol_cruise_profile_does_not_hug_path_orientations():
    cruise = follow_path_patrol_params(final_approach=False, local_obstacles=False)
    assert cruise["FollowPath.vx_max"] == 0.30
    assert cruise["FollowPath.wz_max"] == 0.35
    assert cruise["FollowPath.PathAlignCritic.enabled"] is False
    assert cruise["FollowPath.PathAlignCritic.use_path_orientations"] is False
    assert cruise["FollowPath.CostCritic.enabled"] is False
    assert cruise["FollowPath.PathFollowCritic.enabled"] is True
    assert cruise["FollowPath.GoalCritic.enabled"] is False

    final = follow_path_patrol_params(final_approach=True, local_obstacles=False)
    assert final["FollowPath.vx_max"] == 0.15
    assert final["FollowPath.PathAlignCritic.enabled"] is True
    assert final["FollowPath.PathAlignCritic.use_path_orientations"] is False
    assert final["FollowPath.PreferForwardCritic.enabled"] is False
    assert final["FollowPath.GoalCritic.enabled"] is True

    with_obstacles = follow_path_patrol_params(final_approach=False, local_obstacles=True)
    assert with_obstacles["FollowPath.CostCritic.enabled"] is True


def test_final_approach_follow_path_is_applied_before_costmap_timeout(monkeypatch):
    from roamerx_edge.protocol import ProtocolError

    adapter = object.__new__(RosAdapter)
    adapter._goal_yaw_required_pub = SimpleNamespace(publish=lambda *_: None)
    monkeypatch.setattr(
        "roamerx_edge.ros_adapter.Bool",
        lambda: SimpleNamespace(data=False),
    )
    adapter._rtk_is_navigation_pose_source = lambda: False
    order = []

    def _set(node_name, values, *, code, attempts=8):
        order.append((node_name, attempts, dict(values)))
        if node_name != "/controller_server":
            raise ProtocolError(code, f"{node_name} parameter request timed out")

    adapter._set_remote_parameters = _set
    adapter.set_waypoint_profile(avoid_obstacles=True, require_yaw=False, final_approach=True)

    assert order[0][0] == "/controller_server"
    assert order[0][2]["FollowPath.vx_max"] == 0.15
    assert order[0][2]["FollowPath.PreferForwardCritic.enabled"] is False
    assert order[0][2]["FollowPath.GoalCritic.enabled"] is True
    costmap_calls = [item for item in order if item[0] != "/controller_server"]
    assert costmap_calls
    assert all(item[1] == 1 for item in costmap_calls)


def test_live_final_approach_skips_costmaps_and_retries_controller_once(monkeypatch):
    from roamerx_edge.protocol import ProtocolError

    adapter = object.__new__(RosAdapter)
    adapter._goal_yaw_required_pub = SimpleNamespace(publish=lambda *_: None)
    monkeypatch.setattr(
        "roamerx_edge.ros_adapter.Bool",
        lambda: SimpleNamespace(data=False),
    )
    adapter._rtk_is_navigation_pose_source = lambda: False
    order = []

    def _set(node_name, values, *, code, attempts=8):
        order.append((node_name, attempts, dict(values)))
        raise ProtocolError(code, f"{node_name} parameter request timed out")

    adapter._set_remote_parameters = _set
    adapter.set_waypoint_profile(
        avoid_obstacles=True, require_yaw=False, final_approach=True, live=True
    )

    assert order == [
        ("/controller_server", 2, order[0][2]),
    ]
    assert order[0][2]["FollowPath.vx_max"] == 0.15


def test_disabling_goal_precision_does_not_raise_on_timeout(monkeypatch):
    from roamerx_edge.protocol import ProtocolError

    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SafetyConfig(docking_goal_tolerance_m=0.08, docking_goal_yaw_tolerance_rad=0.087)

    def _set(node_name, values, *, code, attempts=8):
        raise ProtocolError(code, f"{node_name} parameter request timed out")

    adapter._set_remote_parameters = _set
    adapter.set_goal_precision(enabled=False)


def test_zero_timeout_ready_probe_still_waits_for_action_discovery():
    assert RosAdapter._action_server_wait_timeout(0.0, 0.0) == 0.5
    assert RosAdapter._action_server_wait_timeout(10.0, 0.2) == 0.2
    assert RosAdapter._action_server_wait_timeout(10.0, 2.0) == 0.5
