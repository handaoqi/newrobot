#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path

from replay_mapping_trajectory import (
    load_and_sparsify,
    load_mapping_trace,
    sparsify_points,
    yaw_from_xyzw,
)


def test_sparsify_keeps_start_end_and_turns():
    points = [
        {"index": 0, "source_index": 0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "stamp": 0.0},
        {"index": 1, "source_index": 1, "x": 0.2, "y": 0.0, "z": 0.0, "yaw": 0.0, "stamp": 1.0},
        {"index": 2, "source_index": 2, "x": 3.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "stamp": 2.0},
        {"index": 3, "source_index": 3, "x": 3.0, "y": 3.0, "z": 0.0, "yaw": 1.57, "stamp": 3.0},
        {"index": 4, "source_index": 4, "x": 3.05, "y": 3.02, "z": 0.0, "yaw": 1.57, "stamp": 4.0},
    ]
    waypoints = sparsify_points(points, min_distance_m=2.0, min_yaw_rad=0.5, skip_start_m=0.15, skip_end_m=0.0)
    assert waypoints[0]["x"] == 0.2
    assert waypoints[-1]["x"] == 3.05
    assert any(abs(item["x"] - 3.0) < 1e-6 and abs(item["y"]) < 1e-6 for item in waypoints)
    assert waypoints[0]["sequence"] == 0
    headings = [item["yaw"] for item in waypoints[:-1]]
    assert abs(headings[0]) < 0.05
    assert abs(headings[-1] - math.pi / 2) < 0.2


def test_load_mapping_trace_roundtrip(tmp_path: Path):
    payload = {
        "format": "roamerx.mapping-trace.v1",
        "samples": [
            {"index": 4, "stamp": 1.5, "slam": {"x": 1.2, "y": -0.4, "z": 0.1, "yaw": 0.3}},
            {"index": 5, "stamp": 2.5, "slam": {"x": 4.0, "y": -0.4, "z": 0.1, "yaw": 0.31}},
        ],
    }
    path = tmp_path / "mapping_trace.json"
    path.write_text(__import__("json").dumps(payload), encoding="utf-8")
    points = load_mapping_trace(path)
    assert [item["source_index"] for item in points] == [4, 5]
    assert points[0]["x"] == 1.2


def test_load_and_sparsify_prefers_trace(tmp_path: Path):
    (tmp_path / "mapping_trace.json").write_text(
        __import__("json").dumps({
            "samples": [
                {"index": 0, "stamp": 0, "slam": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}},
                {"index": 1, "stamp": 1, "slam": {"x": 5.0, "y": 0.0, "z": 0.0, "yaw": 0.0}},
                {"index": 2, "stamp": 2, "slam": {"x": 5.0, "y": 5.0, "z": 0.0, "yaw": 1.57}},
            ]
        }),
        encoding="utf-8",
    )
    result = load_and_sparsify(tmp_path, min_distance_m=2.0, skip_start_m=0.0, skip_end_m=0.0)
    assert result["source"] == "trace"
    assert result["waypoint_count"] >= 2
    assert result["waypoints"][-1]["y"] == 5.0


def test_yaw_from_identity_quaternion():
    assert abs(yaw_from_xyzw(0.0, 0.0, 0.0, 1.0)) < 1e-9
