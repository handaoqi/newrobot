import csv
import math
import struct
from pathlib import Path

import pytest

from roamerx_edge.map_loop_closure import (
    LOOP_MAX_SLAM_XY_M,
    LOOP_MIN_SLAM_XY_M,
    finalize_loop_closure,
    _should_accept_loop,
    _world_to_lidar_frame,
)


def _write_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
        "SIZE 4 4 4 4 4 4 4 4\n"
        "TYPE F F F F F F F F\n"
        "COUNT 1 1 1 1 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        for x, y, z in points:
            stream.write(struct.pack("<8f", x, y, z, 0.0, 0.0, 0.0, 0.0, 0.0))


def _l_shape() -> list[tuple[float, float, float]]:
    points = []
    for index in range(40):
        points.append((4.0, -2.0 + index * 0.1, 0.5))
        points.append((index * 0.1, 3.0, 0.5))
    return points


def _to_world(points, x, y, yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        (x + cosine * px - sine * py, y + sine * px + cosine * py, pz)
        for px, py, pz in points
    ]


def _unique_blob(seed: int) -> list[tuple[float, float, float]]:
    return [(float(seed), float(offset), 0.3) for offset in range(8)]


def _write_session(root: Path, poses: list[tuple[float, float, float]], clouds) -> None:
    keyframes = root / "keyframes"
    keyframes.mkdir(parents=True)
    raw_pcd = b"raw-lio-map"
    (root / "map.pcd").write_bytes(raw_pcd)
    with (keyframes / "keyframes.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "index", "stamp", "world_x", "world_y", "world_z",
                "world_qx", "world_qy", "world_qz", "world_qw",
                "lidar_x", "lidar_y", "lidar_z", "lidar_qx", "lidar_qy", "lidar_qz", "lidar_qw",
                "yaw", "point_count",
            ]
        )
        for index, ((x, y, yaw), cloud) in enumerate(zip(poses, clouds)):
            _write_pcd(keyframes / f"scan_{index:05d}.pcd", cloud)
            writer.writerow(
                [
                    index, 100.0 + index, x, y, 0.0,
                    0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0),
                    x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0),
                    yaw, len(cloud),
                ]
            )


def test_world_to_lidar_inverts_yaw_offset():
    yaw = 0.7
    world = _to_world([(1.0, 0.0, 0.2), (0.0, 2.0, 0.2)], 3.0, -1.0, yaw)
    local = _world_to_lidar_frame(
        world,
        {
            "lidar_x": 3.0,
            "lidar_y": -1.0,
            "lidar_z": 0.0,
            "lidar_qx": 0.0,
            "lidar_qy": 0.0,
            "lidar_qz": math.sin(yaw / 2.0),
            "lidar_qw": math.cos(yaw / 2.0),
        },
    )
    assert local[0][0] == pytest.approx(1.0, abs=1e-5)
    assert local[0][1] == pytest.approx(0.0, abs=1e-5)
    assert local[1][0] == pytest.approx(0.0, abs=1e-5)
    assert local[1][1] == pytest.approx(2.0, abs=1e-5)


def test_should_accept_loop_rejects_nearby_slam_and_identity_flood():
    assert not _should_accept_loop(
        rank=1, distance=0.1, geom_ok=True, geom_dx=0.2, geom_dy=-0.1,
        geom_dyaw=0.0, slam_xy=0.4, slam_dyaw=1.2,
    )
    assert not _should_accept_loop(
        rank=2, distance=0.1, geom_ok=True, geom_dx=0.2, geom_dy=0.1,
        geom_dyaw=0.0, slam_xy=8.0, slam_dyaw=0.0,
    )
    assert _should_accept_loop(
        rank=1, distance=0.1, geom_ok=True, geom_dx=0.2, geom_dy=0.1,
        geom_dyaw=0.05, slam_xy=8.0, slam_dyaw=0.0,
    )
    assert LOOP_MIN_SLAM_XY_M == 2.0
    assert LOOP_MAX_SLAM_XY_M == 20.0


def test_world_frame_overlap_does_not_flood_identity_loops(tmp_path):
    local = _l_shape()
    poses = []
    clouds = []
    for index in range(32):
        yaw = 0.2 * index
        poses.append((0.05 * index, 0.02 * index, yaw))
        clouds.append(_to_world(local, 0.05 * index, 0.02 * index, yaw))
    _write_session(tmp_path, poses, clouds)

    result = finalize_loop_closure(tmp_path)

    assert result["loop_status"] == "no_valid_loop"
    assert result["loop_closure_count"] == 0
    assert result["trajectory_source"] == "raw"
    assert (tmp_path / "map.pcd").read_bytes() == b"raw-lio-map"
    assert (tmp_path / "map_raw.pcd").read_bytes() == b"raw-lio-map"
    with (tmp_path / "loop_closures.csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert rows == []


def test_lidar_frame_revisit_with_odometry_gap_is_accepted(tmp_path):
    local = _l_shape()
    poses = []
    clouds = []
    for index in range(32):
        if index == 0:
            poses.append((0.0, 0.0, 0.0))
            clouds.append(_to_world(local, 0.0, 0.0, 0.0))
        elif index == 31:
            poses.append((8.0, 0.0, 0.0))
            clouds.append(_to_world(local, 8.0, 0.0, 0.0))
        else:
            poses.append((0.2 * index, 1.5, 0.4))
            clouds.append(_to_world(_unique_blob(index), 0.2 * index, 1.5, 0.4))
    _write_session(tmp_path, poses, clouds)

    result = finalize_loop_closure(tmp_path)

    assert result["loop_status"] == "accepted"
    assert result["loop_closure_count"] >= 1
    assert result["trajectory_source"] == "raw"
    assert (tmp_path / "map.pcd").read_bytes() == b"raw-lio-map"
    with (tmp_path / "loop_closures.csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    pairs = {(int(row["from"]), int(row["to"])) for row in rows}
    assert (0, 31) in pairs or (31, 0) in pairs


def test_manual_review_loop_detection_does_not_write_graph_closures(tmp_path):
    local = _l_shape()
    poses = []
    clouds = []
    for index in range(32):
        if index == 0:
            poses.append((0.0, 0.0, 0.0))
            clouds.append(_to_world(local, 0.0, 0.0, 0.0))
        elif index == 31:
            poses.append((8.0, 0.0, 0.0))
            clouds.append(_to_world(local, 8.0, 0.0, 0.0))
        else:
            poses.append((0.2 * index, 1.5, 0.4))
            clouds.append(_to_world(_unique_blob(index), 0.2 * index, 1.5, 0.4))
    _write_session(tmp_path, poses, clouds)

    result = finalize_loop_closure(tmp_path, apply_to_graph=False)

    assert result["apply_to_graph"] is False
    assert result["loop_status"] == "pending_manual_review"
    assert result["loop_closure_count"] == 0
    assert result["detected_loop_count"] >= 1
    assert result["candidate_count"] >= 1
    assert (tmp_path / "map.pcd").read_bytes() == b"raw-lio-map"
    with (tmp_path / "loop_closures.csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert rows == []
    with (tmp_path / "scan_context" / "loop_candidates.csv").open(encoding="utf-8") as stream:
        candidates = list(csv.DictReader(stream))
    assert any(row["accepted"] in {"True", "true", "1"} for row in candidates)
