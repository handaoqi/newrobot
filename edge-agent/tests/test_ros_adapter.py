import math
import threading
from types import SimpleNamespace

from roamerx_edge.config import SafetyConfig
from roamerx_edge.ros_adapter import RosAdapter


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
    adapter._localization_failure_cb = triggered.set
    bad = SimpleNamespace(has_converged=True, matching_error=0.51, inlier_fraction=0.5)

    adapter._on_scan_matching_status(bad)
    adapter._on_scan_matching_status(bad)
    assert not triggered.is_set()
    adapter._on_scan_matching_status(bad)

    assert triggered.wait(1.0)
    assert adapter._localization_recovery_armed is True


def test_ndt_score_at_threshold_is_not_healthy():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry()
    adapter.safety_config = SafetyConfig(ndt_failure_score=0.5, ndt_failure_samples=1)
    adapter._ndt_failure_count = 0
    adapter._ndt_failure_notified = False
    adapter._localization_recovery_armed = False
    triggered = threading.Event()
    adapter._localization_failure_cb = triggered.set

    adapter._on_scan_matching_status(
        SimpleNamespace(has_converged=True, matching_error=0.5, inlier_fraction=0.5)
    )

    assert triggered.wait(1.0)


def test_only_absolute_ndt_or_rtk_decision_is_trusted():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry({"active_source": "imu_odom_bridge", "absolute_stable": False})
    assert adapter._absolute_localization_stable() is False
    adapter.telemetry.decision = {"active_source": "ndt_imu", "absolute_stable": True}
    assert adapter._absolute_localization_stable() is True
