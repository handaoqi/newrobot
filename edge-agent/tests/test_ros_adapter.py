import math
import threading
import time
from types import SimpleNamespace

import pytest

from roamerx_edge.config import SafetyConfig
from roamerx_edge.protocol import ProtocolError
from roamerx_edge.ros_adapter import RosAdapter, follow_path_patrol_params


class FakeTelemetry:
    def __init__(self, decision=None):
        self.decision = decision or {}
        self.scan_samples = 0

    def on_scan_matching_status(self, _msg):
        self.scan_samples += 1

    def on_localization(self, _msg):
        return None

    def localization_decision(self):
        return dict(self.decision)

    def on_localization_decision(self, decision):
        self.decision = dict(decision)


def test_fresh_normal_streak_ignores_samples_before_candidate():
    samples = [(10, 3), (11, 3), (12, 3)]

    assert RosAdapter._fresh_normal_streak(samples, after_sequence=12) == 0


def test_lio_motion_anomaly_bypasses_localization_loss_debounce():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry()
    adapter._lio_motion_anomaly_notified = False
    adapter._localization_failure_notified = False
    adapter._localization_recovery_armed = False
    triggered = threading.Event()
    reasons = []

    def on_failure(reason):
        reasons.append(reason)
        triggered.set()

    adapter._localization_failure_cb = on_failure
    message = SimpleNamespace(data='{"lio_motion_anomaly":true,"lio_motion_anomaly_reason":"yaw_rate_exceeded"}')

    adapter._on_localization_decision(message)
    adapter._on_localization_decision(message)

    assert triggered.wait(1.0)
    assert reasons == ["lio_motion_anomaly"]
    assert adapter._localization_recovery_armed is True


def test_lio_motion_anomaly_ignored_when_rtk_xy_is_fixed():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry()
    adapter._lio_motion_anomaly_notified = False
    adapter._localization_failure_notified = False
    adapter._localization_recovery_armed = False
    triggered = threading.Event()
    adapter._localization_failure_cb = lambda reason: triggered.set()
    message = SimpleNamespace(
        data=(
            '{"lio_motion_anomaly":true,"lio_motion_anomaly_reason":"yaw_step_exceeded",'
            '"rtk_usable":true,"rtk_quality":"fixed","rtk_heading_usable":false,'
            '"rtk_good_for_navigation":false,"rtk_position_good_for_navigation":true}'
        )
    )

    adapter._on_localization_decision(message)
    adapter._on_localization_decision(message)

    assert not triggered.wait(0.2)
    assert adapter._localization_recovery_armed is False
    assert adapter._lio_motion_anomaly_notified is False


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
    positions = {(candidate["x"], candidate["y"]) for candidate in candidates[8:]}
    for radius in (0.3, 0.6, 1.0):
        assert {(4.0 + radius, 5.0), (4.0 - radius, 5.0), (4.0, 5.0 + radius), (4.0, 5.0 - radius)} <= positions


def test_new_localization_operation_supersedes_previous_search():
    adapter = object.__new__(RosAdapter)
    adapter._localization_operation_lock = threading.Lock()
    adapter._localization_operation_generation = 0

    old_generation = adapter._start_localization_operation("automatic")
    new_generation = adapter._start_localization_operation("operator")

    with pytest.raises(ProtocolError) as exc:
        adapter._assert_localization_operation(old_generation)
    assert exc.value.code == "RELOCALIZATION_SUPERSEDED"
    adapter._assert_localization_operation(new_generation)


def test_automatic_recovery_cannot_supersede_operator_localization():
    adapter = object.__new__(RosAdapter)
    adapter._localization_operation_lock = threading.Lock()
    adapter._localization_operation_generation = 0
    adapter._operator_localization_depth = 0

    adapter.begin_operator_localization()
    operator_generation = adapter._start_localization_operation("global_relocalize")
    with pytest.raises(ProtocolError) as exc:
        adapter._start_localization_operation("last_trusted", automatic=True)

    assert exc.value.code == "RELOCALIZATION_SUPERSEDED"
    adapter._assert_localization_operation(operator_generation)
    adapter.end_operator_localization()
    assert adapter.operator_localization_active() is False
    assert adapter._start_localization_operation("last_trusted", automatic=True) > operator_generation


def test_operator_localization_transition_does_not_start_auto_recovery():
    adapter = object.__new__(RosAdapter)
    adapter._localization_operation_lock = threading.Lock()
    adapter._localization_operation_generation = 0
    adapter._operator_localization_depth = 0
    adapter._localization_sample_condition = threading.Condition()
    adapter._localization_sample_sequence = 0
    adapter._localization_status_samples = []
    adapter._localization_lost_count = 0
    adapter._localization_failure_notified = False
    adapter._localization_recovery_armed = False
    adapter._latest_speed = 0.0
    adapter.safety_config = SimpleNamespace(localization_loss_samples=1)
    adapter.telemetry = SimpleNamespace(on_localization=lambda _msg: None)
    failures = []
    adapter._localization_failure_cb = failures.append

    adapter.begin_operator_localization()
    adapter._on_localization(SimpleNamespace(status=1, speed=0.0))
    adapter.end_operator_localization()

    assert failures == []
    assert adapter._localization_failure_notified is False


def test_best_ndt_candidate_uses_verified_streak_and_seed_gate():
    adapter = object.__new__(RosAdapter)
    adapter._scan_match_condition = threading.Condition()
    adapter._scan_match_records = {
        (1, 0): {
            "sequence": 1,
            "has_converged": True,
            "matching_error": 0.05,
            "inlier_fraction": 0.90,
            "matched_pose": {"x": 4.0, "y": 0.0, "z": 0.0, "yaw": 1.2},
        },
        (2, 0): {
            "sequence": 2,
            "has_converged": True,
            "matching_error": 0.20,
            "inlier_fraction": 0.70,
            "matched_pose": {"x": 0.20, "y": 0.0, "z": 0.0, "yaw": 0.10},
        },
        (3, 0): {
            "sequence": 3,
            "has_converged": True,
            "matching_error": 0.18,
            "inlier_fraction": 0.72,
            "matched_pose": {"x": 0.21, "y": 0.0, "z": 0.0, "yaw": 0.11},
        },
        (4, 0): {
            "sequence": 4,
            "has_converged": True,
            "matching_error": 0.19,
            "inlier_fraction": 0.71,
            "matched_pose": {"x": 0.19, "y": 0.0, "z": 0.0, "yaw": 0.09},
        },
    }
    adapter.safety_config = SafetyConfig(ndt_failure_score=0.5)
    adapter.telemetry = FakeTelemetry({
        "initialization": {
            "verified": True,
            "stable_frames": 3,
            "required_stable_frames": 3,
        }
    })

    result = adapter._best_ndt_candidate(0, {"x": 0.0, "y": 0.0, "yaw": 0.0})

    assert result["matched_pose"]["x"] == 0.21
    assert result["matching_error"] == 0.18
    assert result["eligible"] is True


def test_candidate_rank_prefers_stable_frames_then_inliers():
    weaker = {
        "eligible": True,
        "stable_frames": 2,
        "inlier_fraction": 0.95,
        "matching_error": 0.05,
        "geometric_rmse": 0.04,
        "position_correction_m": 0.1,
        "yaw_correction_deg": 1.0,
    }
    stronger = {
        "eligible": True,
        "stable_frames": 5,
        "inlier_fraction": 0.70,
        "matching_error": 0.20,
        "geometric_rmse": 0.15,
        "position_correction_m": 0.4,
        "yaw_correction_deg": 8.0,
    }
    assert RosAdapter._candidate_rank(stronger) < RosAdapter._candidate_rank(weaker)


def test_active_relocalize_ranks_all_eligible_candidates_before_commit():
    adapter = object.__new__(RosAdapter)
    adapter._start_localization_operation = lambda _source: 7
    adapter._persist_relocalization_state = lambda _payload: None
    adapter._trusted_pose_cb = None
    adapter._last_trusted_pose_report_monotonic = 0.0
    adapter._last_trusted_pose = None
    localized = SimpleNamespace(x=0.4, y=0.1, z=0.0, yaw=0.2, localization_status="normal")
    adapter.telemetry = SimpleNamespace(latest_pose=lambda: localized)
    calls = []

    def set_once(pose, generation):
        calls.append((dict(pose), generation))
        if len(calls) == 1:
            raise ProtocolError(
                "INITIAL_POSE_NOT_ACCEPTED",
                "weak match",
                details={"best_ndt_candidate": {
                    "eligible": True,
                    "stable_frames": 3,
                    "inlier_fraction": 0.60,
                    "matching_error": 0.22,
                    "matched_pose": {"x": 0.1, "y": 0.0, "z": 0.0, "yaw": 0.0},
                }},
            )
        if len(calls) == 2:
            raise ProtocolError(
                "INITIAL_POSE_NOT_ACCEPTED",
                "best match",
                details={"best_ndt_candidate": {
                    "eligible": True,
                    "stable_frames": 5,
                    "inlier_fraction": 0.91,
                    "matching_error": 0.08,
                    "matched_pose": {"x": 0.4, "y": 0.1, "z": 0.0, "yaw": 0.2},
                }},
            )
        if pose.get("x") == 0.4 and pose.get("require_absolute") is not False and len(calls) > 2:
            return {
                "localized_pose": {"x": 0.4, "y": 0.1, "yaw": 0.2},
                "localization_status": "normal",
            }
        raise ProtocolError("INITIAL_POSE_NOT_ACCEPTED", "no match", details={})

    adapter._set_initial_pose_once = set_once

    result = adapter.active_relocalize({
        "x": 0.0, "y": 0.0, "yaw": 0.0, "max_attempts": 3, "candidate_wait_seconds": 1.0,
    })

    assert len(calls) == 4
    assert calls[-1][0]["x"] == 0.4
    assert result["best_ndt_committed"] is True
    assert result["best_match_pose"]["x"] == 0.4
    assert [item["status"] for item in result["attempts"][:2]] == ["rejected", "accepted"]


def test_quick_then_global_stops_on_strict_optimal_ndt_candidate():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SimpleNamespace(
        localization_quick_search_seconds=30.0,
        localization_optimal_ndt_score=0.01,
    )
    adapter._start_localization_operation = lambda *_args, **_kwargs: 7
    adapter._assert_localization_operation = lambda _generation: None
    adapter._report_localization_attempts = lambda *_args, **_kwargs: None
    adapter.latest_trusted_pose = lambda: {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0}
    adapter._current_live_pose = lambda: None
    adapter._relocalization_candidates = lambda x, y, z, yaw: [
        {"x": x, "y": y, "z": z, "yaw": yaw},
        {"x": x, "y": y, "z": z, "yaw": yaw + 0.5},
    ]

    def probe(seed, _generation, **kwargs):
        return {
            "index": kwargs["index"],
            "status": "rejected",
            "seed_pose": seed,
            "stage": kwargs["extra"]["stage"],
            "ndt_candidate": {
                "eligible": True,
                "matching_error": 0.009,
                "stable_frames": 3,
                "required_stable_frames": 3,
                "matched_pose": {**seed},
            },
        }

    adapter._probe_localization_seed = probe
    adapter._commit_best_relocalization_candidate = (
        lambda _generation, _seed, candidate, _attempts: {
            "best_ndt_candidate": candidate,
            "best_match_pose": candidate["matched_pose"],
            "best_ndt_committed": True,
        }
    )
    adapter._global_relocalize_once = lambda *_args: pytest.fail("global search must stay idle")

    result = adapter.quick_then_global_relocalize(
        origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
        scene_scope="indoor",
        wait_seconds=60.0,
    )

    assert result["early_stopped"] is True
    assert result["global_search_started"] is False
    assert result["evaluated_candidate_count"] == 1
    assert result["attempts"][1]["status"] == "skipped"


def test_rtk_drift_verification_requires_fresh_consecutive_samples_below_threshold():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SimpleNamespace(
        localization_rtk_max_drift_m=0.30,
        localization_rtk_required_samples=3,
    )
    adapter._assert_localization_operation = lambda _generation: None
    samples = iter([
        {"rtk_usable": True, "rtk_quality": "fixed", "rtk_heading_usable": True,
         "rtk_drift": {"sample_stamp_ns": 10, "source": "aligned_fast_lio", "xy_m": 0.31}},
        {"rtk_usable": True, "rtk_quality": "fixed", "rtk_heading_usable": True,
         "rtk_drift": {"sample_stamp_ns": 11, "source": "aligned_fast_lio", "xy_m": 0.20}},
        {"rtk_usable": True, "rtk_quality": "fixed", "rtk_heading_usable": True,
         "rtk_drift": {"sample_stamp_ns": 12, "source": "aligned_fast_lio", "xy_m": 0.19}},
        {"rtk_usable": True, "rtk_quality": "fixed", "rtk_heading_usable": True,
         "rtk_drift": {"sample_stamp_ns": 13, "source": "aligned_fast_lio", "xy_m": 0.18}},
    ])
    adapter._localization_decision = lambda: next(samples)

    result = adapter._wait_for_verified_rtk_drift(
        timeout_seconds=1.0,
        generation=7,
        after_stamp_ns=9,
    )

    assert result["verified"] is True
    assert result["stable_frames"] == 3
    assert result["xy_m"] == pytest.approx(0.18)


def test_active_relocalize_executes_the_one_meter_candidates():
    adapter = object.__new__(RosAdapter)
    adapter._start_localization_operation = lambda _source: 12
    adapter._persist_relocalization_state = lambda _payload: None
    adapter._trusted_pose_cb = None
    adapter._last_trusted_pose_report_monotonic = 0.0
    adapter._last_trusted_pose = None
    adapter.telemetry = SimpleNamespace(latest_pose=lambda: None)
    calls = []

    def reject(pose, generation):
        calls.append((dict(pose), generation))
        raise ProtocolError("INITIAL_POSE_NOT_ACCEPTED", "no match", details={})

    adapter._set_initial_pose_once = reject

    with pytest.raises(ProtocolError) as exc:
        adapter.active_relocalize({
            "x": 4.0,
            "y": 5.0,
            "yaw": 0.0,
            "wait_seconds": 180.0,
            "candidate_wait_seconds": 1.0,
        })

    assert exc.value.code == "ACTIVE_RELOCALIZATION_FAILED"
    assert len(calls) == 20
    attempted_positions = {(call[0]["x"], call[0]["y"]) for call in calls}
    assert {(5.0, 5.0), (3.0, 5.0), (4.0, 6.0), (4.0, 4.0)} <= attempted_positions
    assert exc.value.details["candidate_count"] == 20
    assert exc.value.details["timed_out"] is False


def test_progressive_relocalize_runs_bounded_origin_then_each_waypoint_in_order():
    adapter = object.__new__(RosAdapter)
    adapter._start_localization_operation = lambda _source: 13
    adapter._assert_localization_operation = lambda _generation: None
    adapter._persist_relocalization_state = lambda _payload: None
    adapter._trusted_pose_cb = None
    adapter._last_trusted_pose_report_monotonic = 0.0
    adapter._last_trusted_pose = None
    adapter.telemetry = SimpleNamespace(latest_pose=lambda: None)
    calls = []

    def set_once(pose, generation):
        calls.append((dict(pose), generation))
        # All twenty origin candidates and the first route point fail.  The
        # second route point then supplies the accepted exact seed, and the
        # ranked winner is committed afterwards.
        if len(calls) <= 21:
            raise ProtocolError("INITIAL_POSE_NOT_ACCEPTED", "no match", details={})
        return {
            "localized_pose": {"x": pose["x"], "y": pose["y"], "yaw": pose["yaw"]},
            "localization_status": "normal",
            "best_ndt_candidate": {
                "eligible": True,
                "matching_error": 0.12,
                "inlier_fraction": 0.8,
                "matched_pose": {"x": pose["x"], "y": pose["y"], "yaw": pose["yaw"]},
            },
        }

    adapter._set_initial_pose_once = set_once

    result = adapter.progressive_relocalize(
        origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
        waypoints=[
            {"x": 1.0, "y": 2.0, "yaw": 0.1},
            {"x": 3.0, "y": 4.0, "yaw": 0.2},
        ],
        wait_seconds=120.0,
    )

    assert len(calls) == 23
    origin_positions = {(call[0]["x"], call[0]["y"]) for call in calls[:20]}
    assert {(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)} <= origin_positions
    assert [(call[0]["x"], call[0]["y"]) for call in calls[20:22]] == [
        (1.0, 2.0), (3.0, 4.0),
    ]
    assert calls[-1][0]["x"] == 3.0
    assert result["selected_stage"] == "route_waypoint"
    assert result["selected_waypoint_index"] == 1
    assert result["localized_pose"]["x"] == 3.0
    origin_stage = result["stages"][0]
    route_stage = result["stages"][1]
    assert origin_stage["stage"] == "mapping_origin_bounded"
    assert origin_stage["status"] == "rejected"
    assert origin_stage["started_at"] <= origin_stage["finished_at"]
    assert all(item["stage"] == "mapping_origin_bounded" for item in origin_stage["attempts"])
    assert route_stage["stage"] == "route_waypoints"
    assert route_stage["started_at"] <= route_stage["finished_at"]


def test_progressive_relocalize_falls_back_to_keyframe_global_match():
    adapter = object.__new__(RosAdapter)
    adapter._start_localization_operation = lambda _source: 14
    adapter._assert_localization_operation = lambda _generation: None
    adapter._persist_relocalization_state = lambda _payload: None
    adapter._trusted_pose_cb = None
    adapter._last_trusted_pose_report_monotonic = 0.0
    adapter.telemetry = SimpleNamespace(latest_pose=lambda: None)
    adapter._set_initial_pose_once = lambda _pose, _generation: (_ for _ in ()).throw(
        ProtocolError("INITIAL_POSE_NOT_ACCEPTED", "no local match", details={})
    )
    global_calls = []

    def global_once(wait_seconds, generation):
        global_calls.append((wait_seconds, generation))
        return {
            "localized_pose": {"x": 8.0, "y": 9.0, "yaw": 1.0},
            "localization_status": "normal",
            "message": "keyframe 42 verified by FastVGICP",
            "motion_commanded": False,
        }

    adapter._global_relocalize_once = global_once

    result = adapter.progressive_relocalize(
        origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
        waypoints=[{"x": 1.0, "y": 2.0, "yaw": 0.1}],
        wait_seconds=120.0,
    )

    assert global_calls[0][1] == 14
    assert result["selected_stage"] == "keyframe_global_match"
    assert result["localized_pose"]["x"] == 8.0
    assert result["stages"][-1]["status"] == "accepted"


def test_bounded_stage_progress_preserves_active_stage_and_timestamps():
    adapter = object.__new__(RosAdapter)
    adapter._start_localization_operation = lambda _source: 15
    adapter._assert_localization_operation = lambda _generation: None
    adapter._persist_relocalization_state = lambda _payload: None
    adapter._trusted_pose_cb = None
    adapter._last_trusted_pose_report_monotonic = 0.0
    adapter.telemetry = SimpleNamespace(latest_pose=lambda: None)
    progress = []
    adapter._attempt_progress_cb = lambda payload: progress.append(payload)
    adapter._set_initial_pose_once = lambda _pose, _generation: (_ for _ in ()).throw(
        ProtocolError("INITIAL_POSE_NOT_ACCEPTED", "no local match", details={})
    )
    with pytest.raises(ProtocolError):
        adapter._active_relocalize_once({
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "source": "mapping_origin",
            "stage": "mapping_origin_bounded",
            "stage_started_at": "2026-09-06T13:39:36.300Z",
            "max_attempts": 1,
            "wait_seconds": 1,
            "candidate_wait_seconds": 1,
        }, 15, persist_state=False)

    assert progress[0]["localization_attempts"]["selected_stage"] == "mapping_origin_bounded"
    assert progress[0]["localization_attempts"]["stages"][0]["status"] == "searching"
    final_stage = progress[-1]["localization_attempts"]["stages"][0]
    assert final_stage["status"] == "failed"
    assert final_stage["started_at"] == "2026-09-06T13:39:36.300Z"
    assert final_stage["finished_at"] >= final_stage["started_at"]


def test_operator_initial_pose_commits_verified_ndt_match():
    adapter = object.__new__(RosAdapter)
    adapter._start_localization_operation = lambda _source: 9
    calls = []
    candidate = {
        "eligible": True,
        "verified": True,
        "matched_pose": {"x": 1.2, "y": 2.1, "z": 0.0, "yaw": 0.4},
        "matching_error": 0.10,
        "inlier_fraction": 0.85,
    }

    def set_once(pose, generation):
        calls.append((dict(pose), generation))
        if len(calls) == 1:
            raise ProtocolError(
                "INITIAL_POSE_NOT_ACCEPTED",
                "lio handoff pending",
                details={"best_ndt_candidate": candidate},
            )
        return {
            "localized_pose": {"x": 1.2, "y": 2.1, "yaw": 0.4},
            "localization_status": "normal",
        }

    adapter._set_initial_pose_once = set_once

    result = adapter.set_initial_pose({"x": 1.0, "y": 2.0, "yaw": 0.3})

    assert len(calls) == 2
    assert calls[1][0]["x"] == 1.2
    assert calls[1][0]["y"] == 2.1
    assert result["best_ndt_committed"] is True


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
    adapter.telemetry.decision = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "policy_source_ready": False,
    }
    assert adapter._absolute_localization_stable() is False
    adapter.telemetry.decision = {
        "active_source": "rtk_imu",
        "absolute_stable": True,
        "rtk_good_for_navigation": True,
        "policy_source_ready": False,
    }
    assert adapter._absolute_localization_stable() is True


def test_localization_policy_preserves_ukf_mode():
    published = []
    adapter = object.__new__(RosAdapter)
    adapter._localization_policy_pub = SimpleNamespace(publish=published.append)

    result = adapter.set_localization_policy("UKF", "moving")

    assert result == {"topic": "/localization/policy", "source": "ukf", "phase": "moving"}
    assert published[0].data == "moving:ukf"


def test_good_rtk_ignores_ndt_degradation_before_active_source_switches():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry({
        "active_source": "unavailable",
        "rtk_good_for_navigation": True,
        "rtk_usable": True,
        "rtk_quality": "fixed",
        "rtk_heading_usable": True,
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


def test_lost_status_ignored_when_rtk_xy_is_fixed_without_heading():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry({
        "active_source": "lio_imu",
        "rtk_good_for_navigation": False,
        "rtk_usable": True,
        "rtk_quality": "fixed",
        "rtk_heading_usable": False,
        "rtk_position_good_for_navigation": True,
    })
    adapter.safety_config = SimpleNamespace(localization_loss_samples=1)
    adapter._localization_lost_count = 0
    adapter._localization_failure_notified = False
    adapter._localization_recovery_armed = False
    adapter._latest_speed = 0.0
    adapter._localization_sample_condition = threading.Condition()
    adapter._localization_sample_sequence = 0
    adapter._localization_status_samples = []
    adapter.operator_localization_active = lambda: False
    failures = []
    adapter._localization_failure_cb = failures.append

    adapter._on_localization(SimpleNamespace(status=4, speed=0.0))
    adapter._on_localization(SimpleNamespace(status=4, speed=0.0))

    assert failures == []
    assert adapter._localization_lost_count == 0


def test_lost_status_ignored_when_rtk_is_good_for_navigation():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry({
        "active_source": "unavailable",
        "rtk_good_for_navigation": True,
        "rtk_usable": True,
        "rtk_quality": "fixed",
        "rtk_heading_usable": True,
    })
    adapter.safety_config = SimpleNamespace(localization_loss_samples=1)
    adapter._localization_lost_count = 0
    adapter._localization_failure_notified = False
    adapter._localization_recovery_armed = False
    adapter._latest_speed = 0.0
    adapter._localization_sample_condition = threading.Condition()
    adapter._localization_sample_sequence = 0
    adapter._localization_status_samples = []
    adapter.operator_localization_active = lambda: False
    failures = []
    adapter._localization_failure_cb = failures.append

    adapter._on_localization(SimpleNamespace(status=4, speed=0.0))
    adapter._on_localization(SimpleNamespace(status=4, speed=0.0))

    assert failures == []
    assert adapter._localization_lost_count == 0
    assert adapter._localization_failure_notified is False


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


def test_fixed_rtk_ignores_ndt_degradation_during_heading_flicker():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry({
        "active_source": "lio_imu",
        "rtk_good_for_navigation": False,
        "rtk_usable": True,
        "rtk_quality": "fixed",
        "rtk_heading_usable": False,
        "correction_policy": "ndt",
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


def test_lio_primary_fixed_rtk_policy_ignores_unused_ndt_degradation():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry({
        "active_source": "lio_imu",
        "correction_policy": "rtk",
        "policy_source_ready": True,
        "rtk_usable": True,
        "rtk_quality": "fixed",
    })
    adapter.safety_config = SafetyConfig(ndt_failure_score=0.5, ndt_failure_samples=1)
    adapter._ndt_failure_count = 0
    adapter._ndt_failure_notified = False
    adapter._localization_recovery_armed = False
    triggered = threading.Event()
    adapter._localization_failure_cb = lambda reason: triggered.set()

    adapter._on_scan_matching_status(
        SimpleNamespace(has_converged=False, matching_error=1.2, inlier_fraction=0.0)
    )

    assert not triggered.wait(0.05)
    assert adapter._ndt_failure_count == 0


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
    assert final["FollowPath.vx_min"] == 0.0
    assert final["FollowPath.wz_max"] == 0.35
    assert final["FollowPath.PathAlignCritic.enabled"] is True
    assert final["FollowPath.PathAlignCritic.use_path_orientations"] is False
    assert final["FollowPath.PreferForwardCritic.enabled"] is False
    assert final["FollowPath.GoalCritic.enabled"] is True
    assert final["FollowPath.GoalAngleCritic.enabled"] is False

    heading_final = follow_path_patrol_params(
        final_approach=True,
        local_obstacles=False,
        require_yaw=True,
    )
    assert heading_final["FollowPath.GoalAngleCritic.enabled"] is True
    assert heading_final["FollowPath.PathAlignCritic.enabled"] is False

    with_obstacles = follow_path_patrol_params(final_approach=False, local_obstacles=True)
    assert with_obstacles["FollowPath.CostCritic.enabled"] is True
    assert with_obstacles["FollowPath.CostCritic.cost_weight"] == 18.0
    assert with_obstacles["FollowPath.PathAlignCritic.enabled"] is True
    assert with_obstacles["FollowPath.PathAlignCritic.cost_weight"] == 4.0

    outdoor_with_obstacles = follow_path_patrol_params(
        final_approach=False,
        local_obstacles=True,
        outdoor=True,
    )
    assert outdoor_with_obstacles["FollowPath.CostCritic.enabled"] is True
    assert outdoor_with_obstacles["FollowPath.CostCritic.cost_weight"] == 8.0
    assert outdoor_with_obstacles["FollowPath.PathAlignCritic.enabled"] is False


def test_outdoor_waypoint_profile_enables_local_detour_and_collision_monitor(monkeypatch):
    adapter = object.__new__(RosAdapter)
    adapter._goal_yaw_required_pub = SimpleNamespace(publish=lambda *_: None)
    monkeypatch.setattr(
        "roamerx_edge.ros_adapter.Bool",
        lambda: SimpleNamespace(data=False),
    )
    adapter._rtk_is_navigation_pose_source = lambda: False
    calls = []
    adapter._set_remote_parameters = lambda node, values, **kwargs: calls.append(
        (node, dict(values))
    )

    adapter.set_waypoint_profile(
        avoid_obstacles=True,
        require_yaw=False,
        outdoor=True,
    )

    assert calls[0][0] == "/controller_server"
    assert calls[0][1]["FollowPath.CostCritic.enabled"] is True
    assert calls[0][1]["FollowPath.CostCritic.cost_weight"] == 8.0
    assert (
        "/local_costmap/local_costmap",
        {"obstacle_layer.enabled": True},
    ) in calls
    assert ("/collision_monitor", {"PolygonStop.enabled": True}) in calls
    assert ("/collision_monitor", {"PolygonSlow.enabled": True}) in calls
    assert all(node != "/global_costmap/global_costmap" for node, _ in calls)


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
    # Safety profile uses a short retry budget; failures are swallowed so the
    # FollowPath write still wins the race against slow costmap services.
    assert all(item[1] <= 2 for item in costmap_calls)


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


def test_planner_profile_restores_indoor_defaults_and_enables_outdoor_rtk():
    adapter = object.__new__(RosAdapter)
    calls = []
    adapter._set_remote_parameters = lambda node, values, **kwargs: calls.append(
        (node, dict(values))
    )

    adapter.apply_outdoor_gps_profile(outdoor=False)
    assert calls[-1] == (
        "/planner_server",
        {
            "ThetaStar.allow_straight_line_fallback": False,
            "ThetaStar.prefer_straight_line": False,
            "ThetaStar.tolerance": 0.5,
        },
    )

    adapter.apply_outdoor_gps_profile(outdoor=True)
    assert calls[-1] == (
        "/planner_server",
        {
            "ThetaStar.allow_straight_line_fallback": True,
            "ThetaStar.prefer_straight_line": True,
            "ThetaStar.tolerance": 2.0,
        },
    )
    before = len(calls)
    adapter.apply_outdoor_gps_profile(outdoor=True)
    assert len(calls) == before


def test_global_controller_applies_verified_algorithm_before_publishing_selector():
    adapter = object.__new__(RosAdapter)
    adapter._active_global_controller = None
    adapter._planner_selector_pub = object()
    order = []
    adapter._set_remote_parameters = lambda node, values, **kwargs: order.append(
        ("parameters", node, dict(values), kwargs["code"])
    )
    adapter._publish_nav_selector = lambda publisher, plugin_id: order.append(
        ("selector", publisher, plugin_id)
    )

    adapter.set_global_controller("navfn")

    assert order == [
        (
            "parameters",
            "/planner_server",
            {"NavFn.use_astar": True},
            "GLOBAL_CONTROLLER_FAILED",
        ),
        ("selector", adapter._planner_selector_pub, "NavFn"),
    ]
    assert adapter._active_global_controller == "navfn"


def test_global_controller_parameter_failure_does_not_publish_or_cache_selection():
    adapter = object.__new__(RosAdapter)
    adapter._active_global_controller = None
    adapter._planner_selector_pub = object()
    published = []

    def reject_parameters(*_args, **_kwargs):
        raise ProtocolError("GLOBAL_CONTROLLER_FAILED", "readback mismatch")

    adapter._set_remote_parameters = reject_parameters
    adapter._publish_nav_selector = lambda *_args: published.append(True)

    with pytest.raises(ProtocolError, match="readback mismatch"):
        adapter.set_global_controller("theta_star")

    assert published == []
    assert adapter._active_global_controller is None


def test_waypoint_profile_skips_identical_rewrite(monkeypatch):
    adapter = object.__new__(RosAdapter)
    adapter._goal_yaw_required_pub = SimpleNamespace(publish=lambda *_: None)
    monkeypatch.setattr(
        "roamerx_edge.ros_adapter.Bool",
        lambda: SimpleNamespace(data=False),
    )
    adapter._rtk_is_navigation_pose_source = lambda: False
    adapter.set_local_controller = lambda *_args, **_kwargs: None
    calls = []
    adapter._set_remote_parameters = lambda node, values, **kwargs: calls.append(node)

    kwargs = dict(avoid_obstacles=True, require_yaw=False, outdoor=True)
    adapter.set_waypoint_profile(**kwargs)
    first = len(calls)
    assert first > 0
    adapter.set_waypoint_profile(**kwargs)
    assert len(calls) == first


def test_remote_param_cooldown_skips_unavailable_nodes():
    from roamerx_edge.protocol import ProtocolError

    adapter = object.__new__(RosAdapter)
    adapter._remote_param_cache = {}
    adapter._remote_param_unavailable_until = {}
    adapter._mark_remote_param_unavailable("/collision_monitor")
    try:
        adapter._set_remote_parameters(
            "/collision_monitor",
            {"PolygonStop.enabled": True},
            code="WAYPOINT_PROFILE_FAILED",
            attempts=1,
        )
        assert False, "expected cooldown ProtocolError"
    except ProtocolError as exc:
        assert "cooldown" in str(exc).lower() or "recently unavailable" in str(exc).lower()


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


def test_ready_probes_are_serialized_across_background_threads():
    adapter = object.__new__(RosAdapter)
    adapter._nav_ready_probe_lock = threading.Lock()
    first_entered = threading.Event()
    release_first = threading.Event()
    second_entered = threading.Event()
    calls = []

    def _probe(timeout_seconds):
        calls.append(timeout_seconds)
        if len(calls) == 1:
            first_entered.set()
            release_first.wait(1)
        else:
            second_entered.set()
        return True

    adapter._wait_until_ready_probe = _probe
    first = threading.Thread(target=adapter.wait_until_ready, args=(45.0,))
    second = threading.Thread(target=adapter.wait_until_ready, args=(0.5,))
    first.start()
    assert first_entered.wait(1)
    second.start()
    assert not second_entered.wait(0.1)
    release_first.set()
    first.join(1)
    second.join(1)

    assert calls == [45.0, 0.5]


def test_scan_reports_side_clearance_for_bypass():
    adapter = object.__new__(RosAdapter)
    adapter._front_obstacle_distance_m = None
    adapter._left_clearance_m = None
    adapter._right_clearance_m = None
    scan = SimpleNamespace(
        ranges=[0.5, 1.2, 1.5],
        range_min=0.05,
        angle_min=-0.8,
        angle_increment=0.8,
    )
    adapter._on_scan(scan)
    assert adapter._left_clearance_m == pytest.approx(1.5, abs=0.05)
    assert adapter._front_obstacle_distance_m is None


def test_global_plan_snapshot_exposes_fresh_points_and_stales_after_timeout():
    adapter = object.__new__(RosAdapter)
    adapter._raw_forward_command = 0.0
    adapter._actual_forward_command = 0.0
    adapter._raw_lateral_command = 0.0
    adapter._actual_lateral_command = 0.0
    adapter._raw_turn_command = 0.0
    adapter._actual_turn_command = 0.0
    adapter._latest_speed = 0.0
    adapter._front_obstacle_distance_m = None
    adapter._left_clearance_m = None
    adapter._right_clearance_m = None
    adapter._global_plan_points = []
    adapter._global_plan_updated_monotonic = 0.0
    adapter._global_plan_stale_seconds = 30.0

    adapter._on_global_plan(SimpleNamespace(poses=[
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0))),
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=4.0))),
    ]))

    fresh = adapter.obstacle_monitor_snapshot()["global_plan"]
    assert fresh["updated"] is True
    assert fresh["points"] == [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]

    adapter._global_plan_updated_monotonic = time.monotonic() - 31.0
    stale = adapter.obstacle_monitor_snapshot()["global_plan"]
    assert stale["updated"] is False
    assert stale["points"] == []
