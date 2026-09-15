import json
import math
import threading
import time
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from roamerx_edge.config import SafetyConfig
from roamerx_edge.protocol import ProtocolError
from roamerx_edge import ros_adapter as ros_adapter_module
from roamerx_edge.ros_adapter import RosAdapter, follow_path_patrol_params


class FakeTelemetry:
    def __init__(self, decision=None):
        self.decision = decision or {}
        self.scan_samples = 0

    def on_scan_matching_status(self, _msg, *, include_predictions=True):
        self.scan_samples += 1

    def on_localization(self, _msg):
        return None

    def localization_decision(self):
        return dict(self.decision)

    def on_localization_decision(self, decision):
        self.decision = dict(decision)


class _FakeRecoveryLeaseService:
    class Request:
        ACQUIRE = 1
        RELEASE = 2


def test_cancel_navigation_accepts_goal_that_reached_terminal_state():
    adapter = object.__new__(RosAdapter)

    class GoalHandle:
        status = 5  # action_msgs/GoalStatus.STATUS_CANCELED

        def cancel_goal_async(self):
            future = Future()
            future.set_result(SimpleNamespace(goals_canceling=[]))
            return future

    adapter._goal_handle = GoalHandle()
    adapter._nav_cancel_action = "/follow_waypoints"

    assert adapter.cancel_navigation(timeout_seconds=0.2) is True


def test_cancel_navigation_accepts_handle_cleared_during_cancel_request():
    adapter = object.__new__(RosAdapter)

    class GoalHandle:
        status = 0

        def cancel_goal_async(self):
            adapter._goal_handle = None
            future = Future()
            future.set_result(SimpleNamespace(goals_canceling=[]))
            return future

    adapter._goal_handle = GoalHandle()
    adapter._nav_cancel_action = "/follow_waypoints"

    assert adapter.cancel_navigation(timeout_seconds=0.2) is True


def test_recovery_lease_service_keeps_opaque_lease_by_generation(monkeypatch):
    monkeypatch.setattr(ros_adapter_module, "NavigationRecoveryLease", _FakeRecoveryLeaseService)
    adapter = object.__new__(RosAdapter)
    adapter._bt_recovery_leases = {}
    adapter._recovery_lease_acquire_cb = lambda owner, reason, distance_m: SimpleNamespace(
        owner=owner, reason=reason, generation=17, distance_m=distance_m
    )
    released = []
    adapter._recovery_lease_release_cb = lambda lease: released.append(lease) or True
    adapter._recovery_snapshot_cb = lambda: {
        "owner": "BT_NAVIGATOR",
        "recovery_generation": 17,
        "reason": "navigation_recovery_spin",
        "budget": {"attempts": 1, "max_attempts": 10, "distance_m": 0.0, "max_distance_m": 1.2},
    }
    response = SimpleNamespace()

    acquired = adapter._on_recovery_lease(
        SimpleNamespace(operation=1, owner="BT_NAVIGATOR", reason="navigation_recovery_spin", distance_m=0.0),
        response,
    )

    assert acquired.granted is True
    assert acquired.generation == 17
    released_response = adapter._on_recovery_lease(
        SimpleNamespace(operation=2, owner="forged", reason="", generation=17, distance_m=0.0),
        SimpleNamespace(),
    )
    assert released_response.granted is True
    assert [lease.generation for lease in released] == [17]


def test_recovery_lease_service_rejects_unknown_generation(monkeypatch):
    monkeypatch.setattr(ros_adapter_module, "NavigationRecoveryLease", _FakeRecoveryLeaseService)
    adapter = object.__new__(RosAdapter)
    adapter._bt_recovery_leases = {}
    adapter._recovery_lease_release_cb = lambda lease: True
    adapter._recovery_snapshot_cb = lambda: {"budget": {"exhausted": True}}

    response = adapter._on_recovery_lease(
        SimpleNamespace(operation=2, owner="BT_NAVIGATOR", reason="", generation=99, distance_m=0.0),
        SimpleNamespace(),
    )

    assert response.granted is False
    assert response.exhausted is True


def test_reclaim_recovery_lease_releases_only_the_matching_bt_generation():
    adapter = object.__new__(RosAdapter)
    lease = SimpleNamespace(owner="BT_NAVIGATOR", generation=17)
    adapter._bt_recovery_leases = {17: lease}
    released = []
    adapter._recovery_lease_release_cb = lambda value: released.append(value) or True

    assert adapter.reclaim_recovery_lease(17) is True
    assert released == [lease]
    assert adapter._bt_recovery_leases == {}
    assert adapter.reclaim_recovery_lease(17) is False


def test_fusion_profile_request_carries_generation_guard(monkeypatch):
    class FakeFusionProfileService:
        class Request:
            PROFILE_NOMINAL = 0
            PROFILE_LIO_HOLD = 1
            PROFILE_BALANCED = 2

    class FakeClient:
        def __init__(self):
            self.request = None

        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, request):
            self.request = request
            future = __import__("concurrent.futures").futures.Future()
            future.set_result(SimpleNamespace(
                accepted=True,
                applied_profile=request.profile,
                generation=42,
                profile_name="balanced",
                message="applied",
            ))
            return future

    monkeypatch.setattr(
        ros_adapter_module,
        "SetLocalizationFusionProfile",
        FakeFusionProfileService,
    )
    adapter = object.__new__(RosAdapter)
    client = FakeClient()
    adapter._localization_fusion_profile_client = client

    result = adapter.set_localization_fusion_profile(
        "balanced",
        reason="absolute_recovered",
        duration_seconds=5.0,
        expected_generation=41,
    )

    assert client.request.profile == client.request.PROFILE_BALANCED
    assert client.request.expected_generation == 41
    assert client.request.duration_seconds == 5.0
    assert result["generation"] == 42


def test_fresh_normal_streak_ignores_samples_before_candidate():
    samples = [(10, 3), (11, 3), (12, 3)]

    assert RosAdapter._fresh_normal_streak(samples, after_sequence=12) == 0


def test_lio_motion_anomaly_bypasses_localization_loss_debounce():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry()
    adapter._lio_motion_anomaly_notified = False
    adapter._lio_absolute_disagreement_notified = False
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


def test_lio_motion_anomaly_stops_navigation_even_when_rtk_xy_is_fixed():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry()
    adapter._lio_motion_anomaly_notified = False
    adapter._lio_absolute_disagreement_notified = False
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

    assert triggered.wait(0.2)
    assert adapter._localization_recovery_armed is True
    assert adapter._lio_motion_anomaly_notified is True


def test_fresh_normal_streak_requires_consecutive_successes():
    samples = [(20, 4), (21, 3), (22, 3), (23, 3)]

    assert RosAdapter._fresh_normal_streak(samples, after_sequence=20) == 3

    samples.append((24, 4))
    assert RosAdapter._fresh_normal_streak(samples, after_sequence=20) == 0


def test_relocalization_candidates_cover_full_yaw_and_nearby_positions():
    candidates = RosAdapter._relocalization_candidates(4.0, 5.0, 0.0, 0.0)

    assert candidates[0]["candidate_label"] == "中心点·原始航向"
    assert candidates[1]["candidate_label"] == "中心点·左转 45°"
    assert candidates[8]["candidate_label"] == "周边 +X 0.3 m"
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


def test_best_ndt_candidate_preserves_rejected_score_and_inlier_diagnostics():
    adapter = object.__new__(RosAdapter)
    adapter._scan_match_condition = threading.Condition()
    adapter._scan_match_records = {
        (1, 0): {
            "sequence": 1,
            "has_converged": False,
            "matching_error": 1.65,
            "inlier_fraction": 0.0,
            "geometric_rmse": 0.82,
            "matched_pose": {"x": 0.1, "y": -0.1, "z": 0.0, "yaw": 0.02},
        },
    }
    adapter.safety_config = SafetyConfig(ndt_failure_score=0.40)
    adapter.telemetry = FakeTelemetry({
        "initialization": {
            "verified": False,
            "stable_frames": 0,
            "required_stable_frames": 3,
        }
    })

    result = adapter._best_ndt_candidate(0, {"x": 0.0, "y": 0.0, "yaw": 0.0})

    assert result["eligible"] is False
    assert result["has_converged"] is False
    assert result["matching_error"] == pytest.approx(1.65)
    assert result["inlier_fraction"] == pytest.approx(0.0)
    assert result["reject_reason"] == "ndt_not_converged"
    assert "ndt_score_above_threshold" in result["quality_failures"]
    assert "ndt_inlier_fraction_below_threshold" in result["quality_failures"]


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
    progress = []
    adapter._attempt_progress_cb = lambda payload: progress.append(payload["localization_attempts"])
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
    assert result["best_candidate_index"] == 2
    assert result["best_candidate_label"] == "中心点·左转 45°"
    assert result["best_candidate_ndt"]["matching_error"] == pytest.approx(0.08)
    assert [item["status"] for item in result["attempts"][:2]] == ["rejected", "accepted"]
    assert [item.get("active_candidate_number") for item in progress if item.get("active_candidate_number")] == [1, 2, 3]
    first_completed = next(
        item for item in progress
        if item.get("active_candidate_number") is None
        and item["attempts"][0].get("status") == "qualified"
    )
    assert first_completed["evaluated_candidate_count"] == 1
    assert first_completed["attempts"][0]["finished_at"] >= first_completed["attempts"][0]["started_at"]
    assert first_completed["attempts"][0]["candidate_label"] == "中心点·原始航向"


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


def test_quick_then_global_failure_keeps_each_rejected_ndt_measurement():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SimpleNamespace(
        localization_quick_search_seconds=30.0,
        localization_optimal_ndt_score=0.01,
    )
    adapter._start_localization_operation = lambda *_args, **_kwargs: 7
    adapter._assert_localization_operation = lambda _generation: None
    adapter.latest_trusted_pose = lambda: None
    adapter._current_live_pose = lambda: None
    adapter._relocalization_candidates = lambda x, y, z, yaw: [
        {"x": x, "y": y, "z": z, "yaw": yaw},
        {"x": x, "y": y, "z": z, "yaw": yaw + 0.5},
    ]
    progress = []
    adapter._report_localization_attempts = lambda payload, **_kwargs: progress.append(payload)

    def reject(seed, _generation, **kwargs):
        index = kwargs["index"]
        score = 0.70 + index / 10
        return {
            "index": index,
            "status": "rejected",
            "seed_pose": seed,
            "stage": kwargs["extra"]["stage"],
            "reject_reason": "ndt_score_above_threshold",
            "ndt_candidate": {
                "eligible": False,
                "has_converged": True,
                "matching_error": score,
                "inlier_fraction": 0.30,
                "reject_reason": "ndt_score_above_threshold",
            },
        }

    adapter._probe_localization_seed = reject
    adapter._global_relocalize_once = lambda *_args: (_ for _ in ()).throw(
        ProtocolError("GLOBAL_RELOCALIZATION_NOT_VERIFIED", "global score 1.42 rejected")
    )

    with pytest.raises(ProtocolError) as captured:
        adapter.quick_then_global_relocalize(
            origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
            scene_scope="indoor",
            wait_seconds=60.0,
        )

    assert captured.value.code == "QUICK_THEN_GLOBAL_RELOCALIZATION_FAILED"
    assert [
        item["ndt_candidate"]["matching_error"]
        for item in captured.value.details["attempts"]
    ] == pytest.approx([0.8, 0.9])
    assert captured.value.details["best_ndt_candidate"]["matching_error"] == pytest.approx(0.8)
    assert captured.value.details["global_relocalization_error"]["error_code"] == "GLOBAL_RELOCALIZATION_NOT_VERIFIED"
    assert progress[-1]["state"] == "failed"


def test_quick_then_global_keeps_failed_rtk_verification_details_during_fallback():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SimpleNamespace(
        localization_quick_search_seconds=30.0,
        localization_optimal_ndt_score=0.01,
    )
    adapter._start_localization_operation = lambda *_args, **_kwargs: 7
    adapter._assert_localization_operation = lambda _generation: None
    verification = {
        "status": "rejected",
        "verified": False,
        "conclusion_code": "fixed_quality",
        "last_sample": {"quality": "float", "usable": True},
    }
    adapter._set_initial_pose_from_rtk_once = lambda *_args: (_ for _ in ()).throw(
        ProtocolError(
            "RTK_FIXED_NOT_STABLE",
            "RTK was not fixed",
            details={"rtk_verification": verification},
        )
    )
    adapter.latest_trusted_pose = lambda: None
    adapter._current_live_pose = lambda: None
    adapter._global_relocalize_once = lambda *_args: {
        "localized_pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        "motion_commanded": False,
    }
    progress = []
    adapter._report_localization_attempts = lambda payload, **_kwargs: progress.append(payload)

    result = adapter.quick_then_global_relocalize(
        origin=None,
        scene_scope="outdoor",
        coordinate_mode="rtk_fixed",
        wait_seconds=60.0,
    )

    assert result["stages"][0]["error_code"] == "RTK_FIXED_NOT_STABLE"
    assert result["stages"][0]["rtk_verification"] == verification
    fallback_progress = next(
        payload for payload in progress if payload.get("selected_stage") == "keyframe_global_match"
    )
    assert fallback_progress["rtk_verification"] == verification


def test_fixed_rtk_verification_uses_rtk_self_span_not_lio_drift():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SimpleNamespace(
        localization_rtk_max_drift_m=0.30,
        localization_rtk_required_samples=3,
    )
    adapter._assert_localization_operation = lambda _generation: None
    adapter.telemetry = SimpleNamespace(localization_diagnostics=lambda: {
        "raw_rtk": {
            "latitude": 31.1234567,
            "longitude": 121.7654321,
            "altitude": 8.2,
            "horizontal_std_m": 0.012,
            "solution_status": 0,
            "position_type": 50,
            "solution_satellites": 24,
            "heading": {
                "status": 0,
                "type": 50,
                "heading_deg": 93.2,
                "heading_std_deg": 0.4,
                "baseline_m": 0.8,
            },
        },
        "time_diagnostics": {"rtk": {"sample_age_seconds": 0.12}},
    })
    progress = []
    samples = iter([
        {"rtk_usable": True, "rtk_quality": "fixed", "rtk_heading_usable": True,
         "rtk_x": 10.00, "rtk_y": 2.00, "rtk_yaw": 1.62,
         "rtk_drift": {"sample_stamp_ns": 9, "xy_m": 8.0}},
        {"rtk_usable": True, "rtk_quality": "fixed", "rtk_heading_usable": True,
         "rtk_x": 10.00, "rtk_y": 2.00, "rtk_yaw": 1.62,
         "rtk_drift": {"sample_stamp_ns": 10, "xy_m": 8.0}},
        {"rtk_usable": True, "rtk_quality": "fixed", "rtk_heading_usable": True,
         "rtk_x": 10.08, "rtk_y": 2.03, "rtk_yaw": 1.63,
         "rtk_drift": {"sample_stamp_ns": 11, "xy_m": 8.2}},
        {"rtk_usable": True, "rtk_quality": "fixed", "rtk_heading_usable": True,
         "rtk_x": 10.05, "rtk_y": 2.06, "rtk_yaw": 1.625,
         "rtk_drift": {"sample_stamp_ns": 12, "xy_m": 8.1}},
    ])
    adapter._localization_decision = lambda: next(samples)

    result = adapter._wait_for_verified_fixed_rtk(
        timeout_seconds=1.0,
        generation=7,
        on_update=lambda payload: progress.append(payload),
    )

    assert result["verified"] is True
    assert result["stable_frames"] == 3
    assert result["span_m"] < 0.30
    assert result["source"] == "rtk_self_stability"
    assert result["conclusion_code"] == "fixed_rtk_verified"
    assert result["sample_count"] == 3
    assert result["last_sample"]["quality"] == "fixed"
    assert result["last_sample"]["map_x"] == pytest.approx(10.05)
    assert result["last_sample"]["latitude"] == pytest.approx(31.1234567)
    assert result["last_sample"]["horizontal_std_m"] == pytest.approx(0.012)
    assert result["last_sample"]["solution_satellites"] == 24
    assert result["last_sample"]["heading_std_deg"] == pytest.approx(0.4)
    assert result["last_sample"]["position_age_s"] == pytest.approx(0.12)
    assert result["last_sample"]["accepted"] is True
    assert len(result["sample_history"]) == 3
    assert progress[-1]["verified"] is True


def test_fixed_rtk_verification_reports_concrete_rejection_without_waiting_for_timeout():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SimpleNamespace(
        localization_rtk_max_drift_m=0.30,
        localization_rtk_required_samples=3,
    )
    adapter._assert_localization_operation = lambda _generation: None
    adapter._localization_decision = lambda: {
        "rtk_usable": True,
        "rtk_quality": "float",
        "rtk_heading_usable": False,
        "rtk_x": 3.2,
        "rtk_y": -1.4,
        "rtk_yaw": 0.2,
        "rtk_blocked_reason": "fixed_rtk_required",
        "rtk_drift": {"sample_stamp_ns": 20},
    }

    result = adapter._wait_for_verified_fixed_rtk(
        timeout_seconds=0.01,
        generation=7,
    )

    assert result["verified"] is False
    assert result["status"] == "rejected"
    assert result.get("timed_out", False) is False
    assert result["conclusion_code"] == "fixed_quality"
    assert result["last_sample"]["quality"] == "float"
    assert result["last_sample"]["blocked_reason"] == "fixed_rtk_required"
    assert result["last_sample"]["reject_reasons"] == [
        "fixed_quality",
        "heading_usable",
    ]


def test_lio_handoff_requires_a_new_ready_generation():
    adapter = object.__new__(RosAdapter)
    adapter._assert_localization_operation = lambda _generation: None
    adapter.telemetry = SimpleNamespace(
        latest_pose=lambda: SimpleNamespace(localization_status=3, x=1.0, y=2.0)
    )
    adapter._localization_decision = lambda: {
        "handoff_anchor_generation": 5,
        "handoff_state": "ready",
        "active_source": "lio_imu",
        "lio_healthy": True,
        "lio_anchored": True,
        "absolute_stable": True,
    }

    latest, decision = adapter._wait_for_lio_handoff(
        after_generation=4, timeout_seconds=0.01, generation=7
    )

    assert latest.x == 1.0
    assert decision["active_source"] == "lio_imu"


def test_lio_handoff_accepts_readable_normal_telemetry_status():
    adapter = object.__new__(RosAdapter)
    adapter._assert_localization_operation = lambda _generation: None
    adapter.telemetry = SimpleNamespace(
        latest_pose=lambda: SimpleNamespace(localization_status="normal", x=1.0, y=2.0)
    )
    adapter._localization_decision = lambda: {
        "handoff_anchor_generation": 6,
        "handoff_state": "ready",
        "active_source": "lio_imu",
        "lio_healthy": True,
        "lio_anchored": True,
        "absolute_stable": True,
    }

    latest, decision = adapter._wait_for_lio_handoff(
        after_generation=5, timeout_seconds=0.01, generation=7
    )

    assert latest.localization_status == "normal"
    assert decision["handoff_state"] == "ready"


@pytest.mark.parametrize("probe_result", [
    {
        "status": "qualified",
        "matching_error": 0.07,
        "inlier_fraction": 0.82,
        "has_converged": True,
        "eligible": True,
    },
    {
        "status": "rejected",
        "matching_error": 0.80,
        "inlier_fraction": 0.30,
        "has_converged": True,
        "eligible": False,
        "reject_reason": "ndt_score_above_threshold",
    },
    {
        "status": "rejected",
        "matching_error": 0.07,
        "inlier_fraction": 0.82,
        "has_converged": False,
        "eligible": False,
        "reject_reason": "ndt_not_converged",
    },
    {
        "status": "rejected",
        "matching_error": None,
        "inlier_fraction": None,
        "has_converged": None,
        "eligible": False,
        "reject_reason": "ndt_sample_unavailable",
    },
])
def test_fixed_rtk_runs_one_numbered_ndt_crosscheck_before_anchor_commit(
    monkeypatch,
    probe_result,
):
    monkeypatch.setattr(
        ros_adapter_module,
        "Trigger",
        SimpleNamespace(Request=lambda: SimpleNamespace()),
    )
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SimpleNamespace(
        localization_rtk_max_drift_m=0.30,
        localization_rtk_required_samples=3,
        localization_handoff_settle_seconds=8.0,
    )
    adapter._assert_localization_operation = lambda _generation: None
    adapter._trusted_pose_frozen = False
    latest = SimpleNamespace(
        x=10.04,
        y=2.05,
        z=0.0,
        yaw=1.62,
        localization_status="normal",
    )
    adapter.telemetry = SimpleNamespace(
        latest_pose=lambda: latest,
        localization_diagnostics=lambda: {},
    )
    fixed_decision = {
        "rtk_usable": True,
        "rtk_quality": "fixed",
        "rtk_heading_usable": True,
        "rtk_x": 10.05,
        "rtk_y": 2.06,
        "rtk_yaw": 1.625,
        "handoff_anchor_generation": 4,
        "sample_age_seconds": 0.08,
    }
    adapter._localization_decision = lambda: dict(fixed_decision)
    verification = {
        "status": "accepted",
        "verified": True,
        "started_at": "2026-09-14T09:00:00Z",
        "conclusion_code": "fixed_rtk_verified",
        "last_sample": {
            "quality": "fixed",
            "usable": True,
            "heading_usable": True,
            "map_x": 10.05,
            "map_y": 2.06,
            "map_yaw": 1.625,
        },
    }
    adapter._wait_for_verified_fixed_rtk = lambda **_kwargs: dict(verification)
    events = []

    def probe(seed, generation, **kwargs):
        events.append(("ndt_probe", kwargs.get("index"), kwargs.get("require_ndt_observation")))
        has_observation = any(
            probe_result.get(key) is not None
            for key in ("matching_error", "inlier_fraction", "has_converged")
        )
        candidate = {
            key: probe_result.get(key)
            for key in ("matching_error", "inlier_fraction", "has_converged", "eligible", "reject_reason")
            if key in probe_result
        }
        return {
            "index": 1,
            "candidate_number": 1,
            "candidate_label": seed["candidate_label"],
            "stage": "rtk_fixed",
            "source": "rtk_ndt_crosscheck",
            "status": probe_result["status"],
            "seed_pose": {"x": seed["x"], "y": seed["y"], "z": 0.0, "yaw": seed["yaw"]},
            "matched_pose": (
                {"x": 10.04, "y": 2.05, "yaw": 1.62}
                if has_observation else None
            ),
            "matching_error": probe_result.get("matching_error"),
            "inlier_fraction": probe_result.get("inlier_fraction"),
            "has_converged": probe_result.get("has_converged"),
            "stable_frames": 3,
            "required_stable_frames": 3,
            "eligible": probe_result["eligible"],
            "reject_reason": probe_result.get("reject_reason"),
            "ndt_candidate": ({
                **candidate,
                "matched_pose": {"x": 10.04, "y": 2.05, "yaw": 1.62},
            } if has_observation else None),
        }

    adapter._probe_localization_seed = probe
    progress = []
    adapter._report_localization_attempts = lambda payload, **_kwargs: progress.append(payload)

    class RtkClient:
        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, _request):
            events.append(("rtk_commit", None, None))
            future = Future()
            future.set_result(SimpleNamespace(success=True, message="accepted"))
            return future

    adapter._rtk_initial_pose_client = RtkClient()
    handoff = {
        "handoff_anchor_generation": 5,
        "handoff_state": "ready",
        "active_source": "lio_imu",
        "lio_healthy": True,
        "lio_anchored": True,
        "absolute_stable": True,
        "sample_age_seconds": 0.03,
    }
    adapter._wait_for_lio_handoff = lambda **_kwargs: (latest, handoff)

    result = adapter._set_initial_pose_from_rtk_once(30.0, 7)

    assert events == [("ndt_probe", 1, True), ("rtk_commit", None, None)]
    assert result["source"] == "rtk_fixed"
    assert result["rtk_fixed_committed"] is True
    assert result["best_candidate_index"] == 1
    assert result["best_candidate_label"] == "RTK固定解定位点"
    assert result["best_candidate_ndt"]["matching_error"] == probe_result.get("matching_error")
    assert result["ndt_crosscheck_passed"] is probe_result["eligible"]
    assert len(result["localization_attempts"]["attempts"]) == 1
    assert result["localization_attempts"]["attempts"][0]["rtk_fixed_committed"] is True
    assert progress[-1]["state"] == "accepted"


def test_active_relocalize_executes_the_one_meter_candidates():
    adapter = object.__new__(RosAdapter)
    adapter._start_localization_operation = lambda _source: 12
    adapter._persist_relocalization_state = lambda _payload: None
    adapter._trusted_pose_cb = None
    adapter._last_trusted_pose_report_monotonic = 0.0
    adapter._last_trusted_pose = None
    adapter.telemetry = SimpleNamespace(latest_pose=lambda: None)
    progress = []
    adapter._attempt_progress_cb = progress.append
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


def test_progressive_relocalize_reports_handoff_failure_without_false_ndt_rejection():
    adapter = object.__new__(RosAdapter)
    adapter._start_localization_operation = lambda _source: 16
    adapter._assert_localization_operation = lambda _generation: None
    adapter._persist_relocalization_state = lambda _payload: None
    adapter._trusted_pose_cb = None
    adapter._last_trusted_pose_report_monotonic = 0.0
    adapter._last_trusted_pose = None
    adapter.telemetry = SimpleNamespace(latest_pose=lambda: None)
    progress = []
    adapter._attempt_progress_cb = progress.append
    attempts = [
        {
            "index": index,
            "stage": "mapping_origin_bounded",
            "status": "failed" if index == 15 else "rejected",
            "seed_pose": {"x": float(index), "y": 0.0, "yaw": 0.0},
            "ndt_candidate": {
                "eligible": index == 15,
                "matching_error": 0.008 if index == 15 else 0.5,
                "matched_pose": {"x": 1.5, "y": 2.5, "yaw": 0.2} if index == 15 else None,
            },
        }
        for index in range(1, 19)
    ] + [
        {"index": 19, "stage": "mapping_origin_bounded", "status": "skipped", "reject_reason": "local_search_budget_exhausted"},
        {"index": 20, "stage": "mapping_origin_bounded", "status": "skipped", "reject_reason": "local_search_budget_exhausted"},
    ]

    def failed_origin(_seed, _generation, *, persist_state):
        raise ProtocolError(
            "RELOCALIZATION_HANDOFF_FAILED",
            "FAST-LIO did not become stable",
            details={
                "attempts": attempts,
                "best_ndt_candidate": attempts[14]["ndt_candidate"],
                "best_candidate_index": 15,
                "best_candidate_stage": "mapping_origin_bounded",
                "best_candidate_seed_pose": attempts[14]["seed_pose"],
                "best_candidate_label": "周边 -Y 0.6 m",
                "best_candidate_ndt": {
                    "matching_error": 0.008,
                    "inlier_fraction": 0.91,
                    "has_converged": True,
                },
                "best_ndt_committed": True,
                "handoff_pending": True,
            },
        )

    adapter._active_relocalize_once = failed_origin
    adapter._global_relocalize_once = lambda _wait, _generation: {
        "localized_pose": {"x": 8.0, "y": 9.0, "yaw": 1.0},
        "localization_status": "normal",
    }

    result = adapter.progressive_relocalize(
        origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
        waypoints=[],
        wait_seconds=120.0,
    )

    origin = result["stages"][0]
    assert origin["status"] == "failed"
    assert "候选 #15 已通过 NDT 质量门限" in origin["error_message"]
    assert "18 个" not in origin["error_message"]
    assert origin["best_candidate_index"] == 15
    origin_progress = next(
        item["localization_attempts"]
        for item in progress
        if item["localization_attempts"].get("stages", [{}])[0].get("status") == "failed"
    )
    assert origin_progress["evaluated_candidate_count"] == 18
    assert origin_progress["best_candidate_index"] == 15
    assert origin_progress["best_candidate_label"] == "周边 -Y 0.6 m"
    assert origin_progress["best_candidate_ndt"]["matching_error"] == pytest.approx(0.008)
    assert origin_progress["handoff_pending"] is True


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
            "wait_seconds": 2,
            "candidate_wait_seconds": 1,
        }, 15, persist_state=False)

    assert progress[0]["localization_attempts"]["selected_stage"] == "mapping_origin_bounded"
    assert progress[0]["localization_attempts"]["stages"][0]["status"] == "searching"
    running = next(
        item["localization_attempts"]
        for item in progress
        if item["localization_attempts"].get("active_candidate_number") == 1
    )
    assert running["evaluated_candidate_count"] == 0
    assert running["attempts"][0]["status"] == "verifying"
    assert running["attempts"][0]["started_at"]
    final_stage = progress[-1]["localization_attempts"]["stages"][0]
    assert final_stage["status"] == "failed"
    assert final_stage["started_at"] == "2026-09-06T13:39:36.300Z"
    assert final_stage["finished_at"] >= final_stage["started_at"]
    completed = progress[-1]["localization_attempts"]["attempts"][0]
    assert completed["status"] == "rejected"
    assert completed["finished_at"] >= completed["started_at"]


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


def test_task_startup_handoff_requires_fast_lio_imu_after_absolute_correction():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry({
        "active_source": "ndt_imu",
        "absolute_stable": True,
        "lio_healthy": True,
        "lio_anchored": True,
    })
    assert adapter._fast_lio_handoff_ready() is False

    adapter.telemetry.decision = {
        "active_source": "lio_imu",
        "absolute_stable": True,
        "lio_healthy": True,
        "lio_anchored": True,
    }
    assert adapter._fast_lio_handoff_ready() is True

    adapter.telemetry.decision["lio_anchored"] = False
    assert adapter._fast_lio_handoff_ready() is False


def test_localization_policy_preserves_ukf_mode():
    published = []
    adapter = object.__new__(RosAdapter)
    adapter._localization_policy_pub = SimpleNamespace(publish=published.append)

    result = adapter.set_localization_policy("UKF", "moving")

    assert result == {
        "topic": "/localization/policy",
        "source": "ukf",
        "phase": "moving",
        "anchor_preference": "balanced",
        "rtk_primary_allowed": False,
        "online_anchor_correction_allowed": False,
    }
    assert published[0].data == "moving:ukf:balanced:0:0"


def test_localization_policy_carries_online_anchor_opt_in_only_while_moving():
    published = []
    adapter = object.__new__(RosAdapter)
    adapter._localization_policy_pub = SimpleNamespace(publish=published.append)

    result = adapter.set_localization_policy("ndt", "moving", online_anchor_correction_allowed=True)

    assert result["online_anchor_correction_allowed"] is True
    assert published[0].data == "moving:ndt:balanced:0:1"

    result = adapter.set_localization_policy("ndt", "stationary", online_anchor_correction_allowed=True)
    assert result["online_anchor_correction_allowed"] is False
    assert published[1].data == "stationary:ndt:balanced:0:0"


def test_lio_absolute_disagreement_starts_relocalization_once():
    adapter = object.__new__(RosAdapter)
    adapter.telemetry = FakeTelemetry()
    adapter._lio_motion_anomaly_notified = False
    adapter._lio_absolute_disagreement_notified = False
    adapter._localization_failure_notified = False
    adapter._localization_recovery_armed = False
    triggered = threading.Event()
    reasons = []
    adapter._localization_failure_cb = lambda reason: (reasons.append(reason), triggered.set())
    message = SimpleNamespace(
        data='{"lio_large_absolute_disagreement":true,'
        '"lio_large_absolute_disagreement_reason":"high_quality_ndt_residual"}'
    )

    adapter._on_localization_decision(message)
    adapter._on_localization_decision(message)

    assert triggered.wait(1.0)
    assert reasons == ["lio_absolute_disagreement"]
    assert adapter._localization_recovery_armed is True


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
    assert cruise["FollowPath.vy_max"] == 0.225
    assert cruise["FollowPath.wz_max"] == 0.525
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
    assert final["FollowPath.PreferForwardCritic.enabled"] is True
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
    assert outdoor_with_obstacles["FollowPath.PathAlignCritic.enabled"] is True
    assert outdoor_with_obstacles["FollowPath.PathAlignCritic.use_path_orientations"] is False

    outdoor_final_with_obstacles = follow_path_patrol_params(
        final_approach=True,
        local_obstacles=True,
        outdoor=True,
    )
    assert outdoor_final_with_obstacles["FollowPath.CostCritic.enabled"] is True
    assert outdoor_final_with_obstacles["FollowPath.vx_min"] == 0.0
    assert outdoor_final_with_obstacles["FollowPath.PathAlignCritic.enabled"] is False
    assert outdoor_final_with_obstacles["FollowPath.PreferForwardCritic.enabled"] is True


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
    assert calls[0][1]["FollowPath.PathAlignCritic.enabled"] is True
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
    assert order[0][2]["FollowPath.PreferForwardCritic.enabled"] is True
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


def test_ilqr_waypoint_profile_and_boundary_limit_use_ilqr_parameters(monkeypatch):
    adapter = object.__new__(RosAdapter)
    adapter._goal_yaw_required_pub = SimpleNamespace(publish=lambda *_: None)
    adapter._active_local_controller = "ilqr"
    adapter._boundary_zone_speed_limit = None
    adapter._rtk_is_navigation_pose_source = lambda: False
    adapter.set_local_controller = lambda *_args, **_kwargs: None
    adapter.set_safety_profile = lambda **_kwargs: None
    monkeypatch.setattr(
        "roamerx_edge.ros_adapter.Bool",
        lambda: SimpleNamespace(data=False),
    )
    writes = []
    adapter._set_remote_parameters = lambda node, values, **kwargs: writes.append(
        (node, dict(values), kwargs["code"])
    )

    adapter.set_waypoint_profile(
        avoid_obstacles=True,
        require_yaw=False,
        final_approach=False,
        local_controller="ilqr",
    )
    assert writes[0] == (
        "/controller_server",
        {"ILQR.desired_linear_vel": 0.30, "ILQR.max_angular_vel": 0.525},
        "WAYPOINT_PROFILE_FAILED",
    )

    adapter.set_boundary_speed_limit(0.12)
    assert writes[-1] == (
        "/controller_server",
        {"ILQR.desired_linear_vel": 0.12},
        "BOUNDARY_SPEED_LIMIT_FAILED",
    )


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


def test_arrival_goal_tolerance_sets_xy_and_yaw_with_readback_path():
    adapter = object.__new__(RosAdapter)
    writes = []
    adapter._set_remote_parameters = lambda node, values, **kwargs: writes.append(
        (node, values, kwargs)
    )

    adapter.set_arrival_goal_tolerance(0.50, yaw_tolerance_rad=0.25)

    assert writes == [
        (
            "/controller_server",
            {
                "general_goal_checker.xy_goal_tolerance": 0.50,
                "general_goal_checker.required_yaw_goal_tolerance": 0.25,
            },
            {"code": "ARRIVAL_GOAL_TOLERANCE_FAILED", "attempts": 4},
        )
    ]


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


def test_scan_reports_verified_front_rear_left_and_right_ranges():
    adapter = object.__new__(RosAdapter)
    adapter._scan_geometry_key = None
    adapter._scan_geometry = []
    scan = SimpleNamespace(
        # rear, right, front, left
        ranges=[0.65, 1.10, 0.50, 1.20],
        range_min=0.05,
        range_max=4.0,
        angle_min=-math.pi,
        angle_increment=math.pi / 2,
    )

    adapter._process_scan(scan)

    assert adapter._front_obstacle_distance_m == pytest.approx(0.50)
    assert adapter._rear_clearance_m == pytest.approx(0.65)
    assert adapter._left_clearance_m == pytest.approx(1.20)
    assert adapter._right_clearance_m == pytest.approx(1.10)


def test_collision_monitor_diagnostic_is_retained_for_obstacle_snapshot():
    adapter = object.__new__(RosAdapter)
    adapter._emit_ros_diagnostic = lambda *args, **kwargs: None
    adapter._collision_monitor_state = {}
    adapter._collision_monitor_state_received_monotonic = 0.0

    adapter._on_collision_state(SimpleNamespace(data=json.dumps({
        "state": "STOP",
        "reason": "polygon",
        "zone": "left_stop",
        "motion_scope": "left",
        "points_inside": 5,
    })))

    assert adapter._collision_monitor_state["state"] == "STOP"
    assert adapter._collision_monitor_state["zone"] == "left_stop"
    assert adapter._collision_monitor_state_received_monotonic > 0


def test_behavior_recovery_builds_bounded_backup_and_lateral_goals(monkeypatch):
    class Goal:
        def __init__(self):
            self.target = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.speed = 0.0
            self.time_allowance = SimpleNamespace(sec=0, nanosec=0)

    monkeypatch.setattr(ros_adapter_module, "BackUp", SimpleNamespace(Goal=Goal), raising=False)
    monkeypatch.setattr(ros_adapter_module, "DriveOnHeading", SimpleNamespace(Goal=Goal), raising=False)
    adapter = object.__new__(RosAdapter)
    adapter._backup_client = object()
    adapter._drive_on_heading_client = object()
    calls = []

    def execute(client, goal, *, name, timeout_seconds):
        calls.append((client, goal, name, timeout_seconds))
        return {"action": name, "status": "succeeded", "success": True}

    adapter._execute_behavior_action = execute

    result = adapter.execute_obstacle_recovery(
        reverse_distance_m=0.25,
        lateral_distance_m=0.20,
        lateral_direction=-1,
        speed_mps=0.06,
        timeout_seconds=6.0,
    )

    assert result["success"] is True
    assert calls[0][1].target.x == pytest.approx(0.25)
    assert calls[0][1].speed == pytest.approx(0.06)
    assert calls[1][1].target.y == pytest.approx(-0.20)
    assert calls[1][1].speed == pytest.approx(-0.06)
    assert calls[1][1].time_allowance.sec == 6


def test_directional_clearance_uses_rear_and_side_scan_points():
    adapter = object.__new__(RosAdapter)
    adapter._scan_geometry_key = None
    adapter._scan_geometry = []
    adapter._latest_scan = SimpleNamespace(
        # -pi=rear obstacle, -pi/2=right clear, 0=front clear, pi/2=left obstacle
        ranges=[0.40, 4.0, 4.0, 0.35],
        range_min=0.05,
        angle_min=-math.pi,
        angle_increment=math.pi / 2,
    )
    adapter._latest_scan_received_monotonic = time.monotonic()

    assert adapter.directional_clearance(-0.08, 0.0, 0.3)["reason"] == "obstacle"
    assert adapter.directional_clearance(0.08, 0.0, 0.3)["clear"] is True
    assert adapter.directional_clearance(0.0, 0.08, 0.3)["reason"] == "obstacle"


def test_directional_clearance_rejects_stale_scan():
    adapter = object.__new__(RosAdapter)
    adapter._latest_scan = SimpleNamespace(
        ranges=[4.0], range_min=0.05, angle_min=0.0, angle_increment=1.0
    )
    adapter._latest_scan_received_monotonic = time.monotonic() - 0.6

    result = adapter.directional_clearance(0.08, 0.0, 0.2, max_scan_age_seconds=0.5)
    assert result["clear"] is False
    assert result["reason"] == "scan_stale"


def test_global_plan_snapshot_exposes_fresh_points_and_stales_after_timeout():
    adapter = object.__new__(RosAdapter)
    adapter._raw_forward_command = 0.0
    adapter._actual_forward_command = 0.0
    adapter._raw_lateral_command = 0.0
    adapter._actual_lateral_command = 0.0
    adapter._raw_turn_command = 0.0
    adapter._actual_turn_command = 0.0
    adapter._raw_velocity_updated_monotonic = time.monotonic()
    adapter._actual_velocity_updated_monotonic = time.monotonic()
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

    snapshot = adapter.obstacle_monitor_snapshot()
    assert snapshot["requested_velocity_sample_age_seconds"] is not None
    assert snapshot["actual_velocity_sample_age_seconds"] is not None
    assert snapshot["actual_velocity_sample_age_seconds"] < 0.1
    fresh = snapshot["global_plan"]
    assert fresh["updated"] is True
    assert fresh["points"] == [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]

    adapter._global_plan_updated_monotonic = time.monotonic() - 31.0
    stale = adapter.obstacle_monitor_snapshot()["global_plan"]
    assert stale["updated"] is False
    assert stale["points"] == []


def test_stopped_accepts_stale_zero_after_collision_monitor_quiet_period():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SafetyConfig()
    adapter._actual_forward_command = 0.0
    adapter._actual_lateral_command = 0.0
    adapter._actual_turn_command = 0.0
    adapter._actual_velocity_updated_monotonic = time.monotonic() - 10.0

    assert adapter._is_stopped_from_velocity(time.monotonic()) is True


def test_stopped_accepts_idle_when_collision_monitor_never_published():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SafetyConfig()
    adapter._actual_forward_command = 0.0
    adapter._actual_lateral_command = 0.0
    adapter._actual_turn_command = 0.0
    adapter._actual_velocity_updated_monotonic = 0.0
    adapter._raw_velocity_updated_monotonic = 0.0

    assert adapter._is_stopped_from_velocity(time.monotonic()) is True


def test_stopped_rejects_uninitialized_actual_when_raw_command_seen():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SafetyConfig()
    adapter._actual_forward_command = 0.0
    adapter._actual_lateral_command = 0.0
    adapter._actual_turn_command = 0.0
    adapter._actual_velocity_updated_monotonic = 0.0
    adapter._raw_velocity_updated_monotonic = time.monotonic()

    assert adapter._is_stopped_from_velocity(time.monotonic()) is False


def test_stopped_rejects_nonzero_command_even_when_feedback_is_stale():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SafetyConfig()
    adapter._actual_forward_command = 0.12
    adapter._actual_lateral_command = 0.0
    adapter._actual_turn_command = 0.0
    adapter._actual_velocity_updated_monotonic = time.monotonic() - 10.0

    assert adapter._is_stopped_from_velocity(time.monotonic()) is False


def test_nonzero_command_clears_stopped_state():
    adapter = object.__new__(RosAdapter)
    adapter.safety_config = SafetyConfig()
    adapter._actual_forward_command = 0.0
    adapter._actual_lateral_command = 0.0
    adapter._actual_turn_command = 0.0
    adapter._actual_velocity_updated_monotonic = time.monotonic() - 10.0
    assert adapter._is_stopped_from_velocity(time.monotonic()) is True

    adapter._on_cmd_vel(SimpleNamespace(
        linear=SimpleNamespace(x=0.12, y=0.0),
        angular=SimpleNamespace(z=0.0),
    ))
    assert adapter._is_stopped_from_velocity(time.monotonic()) is False
