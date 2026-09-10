import csv
import json
import math

import pytest

from roamerx_edge.map_optimization_summary import build_optimization_summary, summary_without_corrections


def _write_trajectory(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "index", "timestamp", "world_x", "world_y", "world_z",
            "world_qx", "world_qy", "world_qz", "world_qw",
        ])
        for row in rows:
            if len(row) == 5:
                index, stamp, x, y, yaw = row
                z = 0.0
            else:
                index, stamp, x, y, z, yaw = row
            writer.writerow([
                index, stamp, x, y, z, 0.0, 0.0,
                math.sin(yaw / 2.0), math.cos(yaw / 2.0),
            ])


def test_summary_aligns_keyframes_and_normalizes_yaw(tmp_path):
    raw_yaw = math.radians(179.0)
    optimized_yaw = math.radians(-179.0)
    _write_trajectory(tmp_path / "trajectory_raw.csv", [
        (0, 10.0, 0.0, 0.0, raw_yaw),
        (1, 11.0, 1.0, 1.0, 0.0),
    ])
    _write_trajectory(tmp_path / "trajectory_optimized.csv", [
        (0, 10.0, 0.1, 0.0, optimized_yaw),
        (1, 11.0, 1.18, 1.05, 0.04),
    ])
    (tmp_path / "trajectory_covariance.json").write_text(json.dumps({
        "factor_count": 9,
        "ndt_factor_count": 1,
        "imu_factor_count": 1,
        "imu_bias_factor_count": 1,
        "imu_velocity_prior_factor_count": 1,
        "rtk_position_factor_count": 1,
        "rtk_heading_factor_count": 1,
        "loop_closure_factor_count": 1,
        "error_before": 10.0,
        "error_after": 3.0,
    }))
    (tmp_path / "loop_closures.csv").write_text("from,to,score\n0,1,0.1\n")
    scan_context = tmp_path / "scan_context"
    scan_context.mkdir()
    (scan_context / "loop_candidates.csv").write_text(
        "query_index,match_index,accepted,rejection_reason\n1,0,true,\n1,2,false,geometric_verification\n"
    )

    summary = build_optimization_summary(tmp_path, mapping_type="outdoor")

    assert summary["stage"] == "completed"
    assert summary["corrections"][0]["delta"]["yaw_deg"] == pytest.approx(2.0, abs=1e-3)
    assert summary["corrections"][1]["delta"]["position_m"] == pytest.approx(
        math.hypot(0.18, 0.05), abs=1e-5
    )
    assert summary["graph_error"]["reduction_percent"] == pytest.approx(70.0)
    assert summary["candidate_count"] == 2
    assert summary["accepted_loop_count"] == 1
    assert summary["optimization_mode"] == "fast_lio2_slam_anchored"
    assert summary["factors"]["lio_between"] == 1
    assert summary["factors"]["ndt_registration"] == 0
    assert summary["factors"]["ndt_is_legacy_alias"] is True
    assert summary["factors"]["imu_velocity_prior"] == 1
    assert "corrections" not in summary_without_corrections(summary)


def test_indoor_summary_rejects_any_rtk_factor(tmp_path):
    _write_trajectory(tmp_path / "trajectory_raw.csv", [(0, 10.0, 0.0, 0.0, 0.0)])
    _write_trajectory(tmp_path / "trajectory_optimized.csv", [(0, 10.0, 0.1, 0.0, 0.0)])
    (tmp_path / "trajectory_covariance.json").write_text(json.dumps({
        "factor_count": 2,
        "rtk_position_factor_count": 1,
        "loop_closure_factor_count": 1,
    }))
    (tmp_path / "loop_closures.csv").write_text("from,to,score\n0,1,0.1\n")

    summary = build_optimization_summary(tmp_path, mapping_type="indoor")

    assert summary["use_gps"] is False
    assert summary["gps_factor_gate_valid"] is False
    assert summary["stage"] == "fallback"
    assert summary["auto_activation_allowed"] is False


def test_inertial_only_smoothing_passes_continuity_guard(tmp_path):
    _write_trajectory(tmp_path / "trajectory_raw.csv", [
        (0, 10.0, 0.0, 0.0, 0.0),
        (1, 11.0, 1.0, 0.0, 0.0),
        (2, 12.0, 2.0, 0.0, 0.0),
    ])
    _write_trajectory(tmp_path / "trajectory_optimized.csv", [
        (0, 10.0, 0.0, 0.0, 0.0),
        (1, 11.0, 1.04, 0.01, 0.01),
        (2, 12.0, 2.08, 0.02, 0.02),
    ])
    (tmp_path / "trajectory_covariance.json").write_text(json.dumps({
        "factor_count": 7,
        "lio_between_factor_count": 2,
        "ndt_factor_count": 2,
        "imu_factor_count": 2,
        "error_before": 8.0,
        "error_after": 2.0,
    }))

    summary = build_optimization_summary(tmp_path, mapping_type="indoor")

    assert summary["optimization_mode"] == "fast_lio2_inertial_smoothing"
    assert summary["inertial_smoothing_guard"]["checked"] is True
    assert summary["inertial_smoothing_guard"]["passed"] is True
    assert summary["auto_activation_allowed"] is True


def test_inertial_only_smoothing_blocks_large_vertical_correction(tmp_path):
    _write_trajectory(tmp_path / "trajectory_raw.csv", [
        (0, 10.0, 0.0, 0.0, 0.0, 0.0),
        (1, 11.0, 1.0, 0.0, 0.0, 0.0),
    ])
    _write_trajectory(tmp_path / "trajectory_optimized.csv", [
        (0, 10.0, 0.0, 0.0, 0.0, 0.0),
        (1, 11.0, 1.02, 0.0, 0.70, 0.0),
    ])
    (tmp_path / "trajectory_covariance.json").write_text(json.dumps({
        "factor_count": 4,
        "lio_between_factor_count": 1,
        "ndt_factor_count": 1,
        "imu_factor_count": 1,
        "error_before": 3.0,
        "error_after": 1.0,
    }))

    summary = build_optimization_summary(tmp_path, mapping_type="indoor")

    guard = summary["inertial_smoothing_guard"]
    assert guard["checked"] is True
    assert guard["passed"] is False
    assert "max_abs_z_correction_m_above_limit" in guard["reasons"]
    assert summary["auto_activation_allowed"] is False


def test_loop_anchored_optimization_does_not_bypass_correction_guard(tmp_path):
    _write_trajectory(tmp_path / "trajectory_raw.csv", [
        (0, 10.0, 0.0, 0.0, 0.0),
        (1, 11.0, 1.0, 0.0, 0.0),
    ])
    _write_trajectory(tmp_path / "trajectory_optimized.csv", [
        (0, 10.0, 0.0, 0.0, 0.0),
        (1, 11.0, 2.75, 0.0, math.radians(4.0)),
    ])
    (tmp_path / "trajectory_covariance.json").write_text(json.dumps({
        "factor_count": 5,
        "lio_between_factor_count": 1,
        "imu_factor_count": 1,
        "loop_closure_factor_count": 1,
        "error_before": 100.0,
        "error_after": 1.0,
    }))
    (tmp_path / "loop_closures.csv").write_text("from,to,score\n1,0,0.9\n")

    summary = build_optimization_summary(tmp_path, mapping_type="indoor")

    guard = summary["inertial_smoothing_guard"]
    assert summary["candidate_applied"] is True
    assert summary["applied"] is False
    assert summary["stage"] == "quality_rejected"
    assert guard["checked"] is True
    assert guard["passed"] is False
    assert "max_xy_correction_m_above_limit" in guard["reasons"]
    assert "max_yaw_correction_deg_above_limit" in guard["reasons"]
    assert summary["trajectory_source"] == "raw"
    assert summary["auto_activation_allowed"] is False


def test_continuity_guard_reports_offending_pair_and_normalized_rate(tmp_path):
    _write_trajectory(tmp_path / "trajectory_raw.csv", [
        (0, 10.0, 0.0, 0.0, 0.0),
        (1, 10.5, 0.3, 0.0, math.radians(20.0)),
        (2, 11.0, 0.6, 0.0, math.radians(40.0)),
    ])
    _write_trajectory(tmp_path / "trajectory_optimized.csv", [
        (0, 10.0, 0.0, 0.0, 0.0),
        (1, 10.5, 0.46, 0.0, math.radians(20.0)),
        (2, 11.0, 0.76, 0.0, math.radians(40.0)),
    ])
    (tmp_path / "trajectory_covariance.json").write_text(json.dumps({
        "factor_count": 7,
        "lio_between_factor_count": 2,
        "imu_factor_count": 2,
        "imu_kinematic_rejected_factor_count": 1,
        "max_imu_kinematic_residual_mps": 0.42,
        "error_before": 5.0,
        "error_after": 1.0,
    }))

    summary = build_optimization_summary(tmp_path, mapping_type="indoor")

    guard = summary["inertial_smoothing_guard"]
    assert summary["stage"] == "quality_rejected"
    assert "max_adjacent_xy_step_m_above_limit" in guard["reasons"]
    assert guard["worst_adjacent_xy_correction"] == {
        "from_index": 0,
        "to_index": 1,
        "from_stamp": 10.0,
        "to_stamp": 10.5,
        "delta_t_s": 0.5,
        "xy_correction_step_m": 0.16,
        "xy_correction_rate_mps": 0.32,
        "raw_xy_distance_m": 0.3,
        "raw_yaw_step_deg": pytest.approx(20.0, abs=5e-4),
    }
    assert summary["factors"]["imu_kinematic_rejected"] == 1
    assert summary["factors"]["max_imu_kinematic_residual_mps"] == pytest.approx(0.42)
    assert "adjacent_corrections" not in summary_without_corrections(summary)


def test_outdoor_rtk_anchor_applies_metre_scale_xy_correction(tmp_path):
    raw = [(index, 10.0 + index, index * 10.0, 0.0, 0.0) for index in range(8)]
    optimized = [
        (index, 10.0 + index, index * 10.0 + index * 0.275, index * 0.225, 0.0)
        for index in range(8)
    ]
    _write_trajectory(tmp_path / "trajectory_raw.csv", raw)
    _write_trajectory(tmp_path / "trajectory_optimized.csv", optimized)
    (tmp_path / "trajectory_covariance.json").write_text(json.dumps({
        "factor_count": 20,
        "lio_between_factor_count": 7,
        "imu_factor_count": 7,
        "rtk_position_factor_count": 8,
        "rtk_heading_factor_count": 8,
        "error_before": 40.0,
        "error_after": 8.0,
    }))

    summary = build_optimization_summary(tmp_path, mapping_type="outdoor")

    assert summary["optimization_mode"] == "fast_lio2_slam_anchored"
    assert summary["applied"] is True
    assert summary["stage"] == "no_valid_loop"
    assert summary["trajectory_source"] == "optimized"
    assert summary["auto_activation_allowed"] is True
    assert summary["inertial_smoothing_guard"]["passed"] is True
    assert summary["factors"]["rtk_position"] == 8
    assert summary["inertial_smoothing_guard"]["measurements"]["max_xy_correction_m"] == pytest.approx(
        math.hypot(7 * 0.275, 7 * 0.225), abs=1e-4
    )


def test_outdoor_rtk_heading_turn_allows_adjacent_yaw_correction(tmp_path):
    raw = [
        (0, 10.0, 0.0, 0.0, 0.0),
        (1, 10.5, 0.4, 0.0, math.radians(16.0)),
        (2, 11.0, 0.8, 0.1, math.radians(32.0)),
    ]
    optimized = [
        (0, 10.0, 0.0, 0.0, 0.0),
        (1, 10.5, 0.42, 0.0, math.radians(16.0 - 6.5)),
        (2, 11.0, 0.83, 0.1, math.radians(32.0 - 6.4)),
    ]
    _write_trajectory(tmp_path / "trajectory_raw.csv", raw)
    _write_trajectory(tmp_path / "trajectory_optimized.csv", optimized)
    (tmp_path / "trajectory_covariance.json").write_text(json.dumps({
        "factor_count": 8,
        "lio_between_factor_count": 2,
        "imu_factor_count": 2,
        "rtk_position_factor_count": 3,
        "rtk_heading_factor_count": 3,
        "error_before": 20.0,
        "error_after": 2.0,
    }))

    summary = build_optimization_summary(tmp_path, mapping_type="outdoor")
    guard = summary["inertial_smoothing_guard"]

    assert guard["limits"]["max_adjacent_yaw_step_deg"] == 10.0
    assert guard["measurements"]["max_adjacent_yaw_step_deg"] == pytest.approx(6.5, abs=0.05)
    assert "max_adjacent_yaw_step_deg_above_limit" not in guard["reasons"]
    assert summary["applied"] is True


def test_outdoor_rtk_rejects_discontinuous_adjacent_yaw_correction(tmp_path):
    raw = [
        (0, 10.0, 0.0, 0.0, 0.0),
        (1, 10.5, 0.4, 0.0, math.radians(16.0)),
    ]
    optimized = [
        (0, 10.0, 0.0, 0.0, 0.0),
        (1, 10.5, 0.42, 0.0, math.radians(16.0 - 12.0)),
    ]
    _write_trajectory(tmp_path / "trajectory_raw.csv", raw)
    _write_trajectory(tmp_path / "trajectory_optimized.csv", optimized)
    (tmp_path / "trajectory_covariance.json").write_text(json.dumps({
        "factor_count": 5,
        "lio_between_factor_count": 1,
        "imu_factor_count": 1,
        "rtk_position_factor_count": 2,
        "error_before": 10.0,
        "error_after": 1.0,
    }))

    summary = build_optimization_summary(tmp_path, mapping_type="outdoor")

    assert summary["applied"] is False
    assert summary["stage"] == "quality_rejected"
    assert "max_adjacent_yaw_step_deg_above_limit" in summary["inertial_smoothing_guard"]["reasons"]
