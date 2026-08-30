from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import json
import os
import subprocess
import sys


SCRIPT_DIR = Path(__file__).parent
SPEC = spec_from_file_location(
    "summarize_avoidance_bag", SCRIPT_DIR / "summarize_avoidance_bag.py"
)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
TwistSample = MODULE.TwistSample


def test_percentile_uses_nearest_rank():
    assert MODULE.percentile([5.0, 1.0, 3.0, 2.0, 4.0], 0.90) == 5.0
    assert MODULE.percentile([], 0.90) is None


def test_summary_detects_collision_monitor_stop_and_mppi_gate():
    topics = {
        topic: 1
        for topic in (
            "/front_lidar",
            "/laser_scan",
            "/odom/nav2",
            "/cmd_vel_nav",
            "/cmd_vel_raw",
            "/cmd_vel",
            "/local_costmap/costmap_raw",
            "/transformed_global_plan",
            "/sensor_health",
            "/mppi/performance",
        )
    }
    result = MODULE.summarize_records(
        bag_path="/tmp/example",
        topic_counts=topics,
        twists={
            "/cmd_vel_nav": [TwistSample(1_000_000_000, 0.2, 0.0, 0.0)],
            "/cmd_vel_raw": [TwistSample(1_000_000_000, 0.2, 0.0, 0.0)],
            "/cmd_vel": [TwistSample(1_010_000_000, 0.0, 0.0, 0.0)],
        },
        front_clearances_m=[0.6, 0.8],
        stop_zone_points=[0, 5],
        slow_zone_points=[2, 8],
        mppi_windows=[{"p99_ms": 31.0, "max_ms": 38.0}],
    )

    assert result["evidence"]["complete"] is True
    intervention = result["velocity"]["collision_monitor_intervention"]
    assert intervention["stopped_sample_count"] == 1
    assert result["mppi_performance"]["target_met"] is True
    assert result["stage_1_automatic_gates"][
        "requires_operator_collision_free_confirmation"
    ] is True


def test_summary_lists_missing_evidence_topics():
    result = MODULE.summarize_records(
        bag_path="/tmp/example",
        topic_counts={"/laser_scan": 2},
        twists={},
        front_clearances_m=[],
        stop_zone_points=[],
        slow_zone_points=[],
        mppi_windows=[],
    )
    assert result["evidence"]["complete"] is False
    assert "/cmd_vel_nav" in result["evidence"]["missing_topics"]


def test_navigation_recorder_contains_stage_zero_evidence_topics():
    source = (SCRIPT_DIR / "navigation_rosbag.sh").read_text(encoding="utf-8")
    for topic in (
        "/cmd_vel_nav",
        "/cmd_vel_raw",
        "/cmd_vel",
        "/local_costmap/costmap_raw",
        "/polygon_stop",
        "/polygon_slowdown",
        "/sensor_health",
        "/mppi/performance",
    ):
        assert topic in source


def test_shadow_replay_isolated_and_remaps_every_velocity_output():
    source = (SCRIPT_DIR / "avoidance_shadow_replay.sh").read_text(encoding="utf-8")
    assert 'SHADOW_ROS_DOMAIN_ID:-77' in source
    assert 'shadow replay refuses the production ROS domain 24' in source
    for topic in ("/cmd_vel", "/cmd_vel_raw", "/cmd_vel_nav", "/teleop_cmd_vel"):
        assert f"{topic}:=/shadow/recorded" in source


def test_baseline_scenarios_cover_stage_zero_and_one_cases():
    source = (SCRIPT_DIR / "avoidance_baseline.sh").read_text(encoding="utf-8")
    for scenario in (
        "straight",
        "turn",
        "narrow_passage",
        "static_box",
        "wall_corner",
        "person_crossing",
    ):
        assert scenario in source


def test_baseline_wrapper_persists_reviewed_outcome(tmp_path):
    state_dir = tmp_path / "state"
    bag_dir = tmp_path / "bag"
    state_dir.mkdir()
    bag_dir.mkdir()
    (bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    recorder = tmp_path / "fake_recorder.sh"
    recorder.write_text(
        "#!/bin/sh\n"
        f"printf 'BAG_DIR=%s\\n' '{bag_dir}' > '{state_dir}/session.env'\n"
        "printf '{\"running\":false,\"bag_dir\":\"%s\"}\\n' '"
        f"{bag_dir}'\n",
        encoding="utf-8",
    )
    recorder.chmod(0o755)
    summary = tmp_path / "fake_summary.py"
    summary.write_text(
        "#!/usr/bin/env python3\n"
        "from pathlib import Path\n"
        "import sys\n"
        "Path(sys.argv[3]).write_text('{}\\n', encoding='utf-8')\n",
        encoding="utf-8",
    )
    summary.chmod(0o755)
    env = os.environ.copy()
    env.update(
        {
            "NAVIGATION_ROSBAG_SCRIPT": str(recorder),
            "AVOIDANCE_SUMMARY_TOOL": str(summary),
            "STATE_DIR": str(state_dir),
        }
    )

    subprocess.run(
        [str(SCRIPT_DIR / "avoidance_baseline.sh"), "start", "static_box", "trial"],
        check=True,
        env=env,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            str(SCRIPT_DIR / "avoidance_baseline.sh"),
            "stop",
            "pass",
            "soft box; no contact; goal reached",
        ],
        check=True,
        env=env,
        capture_output=True,
        text=True,
    )

    manifest = json.loads((bag_dir / "avoidance_scenario.json").read_text())
    assert manifest["scenario"] == "static_box"
    assert manifest["outcome"] == "pass"
    assert "no contact" in manifest["operator_notes"]
    assert manifest["motion_started_by_recorder"] is False
