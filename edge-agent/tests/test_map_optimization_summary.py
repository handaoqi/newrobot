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
        for index, stamp, x, y, yaw in rows:
            writer.writerow([
                index, stamp, x, y, 0.0, 0.0, 0.0,
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
        (1, 11.0, 1.3, 1.4, 0.1),
    ])
    (tmp_path / "trajectory_covariance.json").write_text(json.dumps({
        "factor_count": 9,
        "ndt_factor_count": 1,
        "imu_factor_count": 0,
        "imu_bias_factor_count": 0,
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
    assert summary["corrections"][1]["delta"]["position_m"] == pytest.approx(0.5)
    assert summary["graph_error"]["reduction_percent"] == pytest.approx(70.0)
    assert summary["candidate_count"] == 2
    assert summary["accepted_loop_count"] == 1
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
