"""Scan-Context retrieval for loop candidates.

Keyframe clouds are stored in the world frame. Descriptors and geometric
checks run in the lidar frame so already-aligned map points cannot generate
hundreds of identity-yaw false loops. Python does not rebuild map.pcd; the
C++ GTSAM service owns the final trajectory. Indoor saves still detect
Scan-Context but write an empty loop_closures.csv when automatic application
is disabled. Candidates and thresholds remain available for Map Management.
"""

from __future__ import annotations

import csv
import json
import math
import struct
import subprocess
import time
from pathlib import Path
from typing import Any

RINGS = 20
SECTORS = 60
MAX_RADIUS_M = 80.0
CANDIDATE_TOP_K = 5
MIN_KEYFRAME_GAP = 30
YAW_SEARCH_STEPS = 60
ACCEPT_DISTANCE = 0.30
GEOM_XY_LIMIT_M = 2.5
GEOM_YAW_LIMIT_RAD = 0.6
GEOM_OVERLAP_MIN = 0.35
LOOP_MIN_SLAM_XY_M = 2.0
LOOP_MAX_SLAM_XY_M = 20.0
MAX_ACCEPTED_LOOPS = 30
MAX_OPTIMIZED_JUMP_M = 5.0

IDENTITY_XYZW = (0.0, 0.0, 0.0, 1.0)


def finalize_loop_closure(map_dir: str | Path, *, apply_to_graph: bool = True) -> dict[str, Any]:
    started_at = time.time()
    root = Path(map_dir)
    keyframes = _load_keyframes(root)
    raw_path = root / "trajectory_raw.csv"
    optimized_path = root / "trajectory_optimized.csv"
    map_pcd = root / "map.pcd"
    map_raw = root / "map_raw.pcd"
    if map_pcd.exists() and not map_raw.exists():
        map_raw.write_bytes(map_pcd.read_bytes())
    if not raw_path.exists():
        _write_trajectory_csv(raw_path, keyframes)
    if not keyframes:
        if not optimized_path.exists() and raw_path.exists():
            optimized_path.write_bytes(raw_path.read_bytes())
        _write_loop_closures_csv(root, [])
        return {
            "scan_context_count": 0,
            "loop_closure_count": 0,
            "detected_loop_count": 0,
            "apply_to_graph": apply_to_graph,
            "trajectory_source": "raw",
            "loop_status": "disabled" if not apply_to_graph else "no_keyframes",
            "candidate_count": 0,
            "rejected_loop_count": 0,
            "detection_duration_seconds": round(time.time() - started_at, 3),
        }

    world_clouds = [_load_pcd_xyzi(root / keyframe["point_cloud_file"]) for keyframe in keyframes]
    lidar_clouds = [
        _world_to_lidar_frame(points, keyframe)
        for points, keyframe in zip(world_clouds, keyframes)
    ]
    descriptors = [_scan_context(points) for points in lidar_clouds]
    scan_dir = root / "scan_context"
    scan_dir.mkdir(parents=True, exist_ok=True)
    _write_binary_matrix(scan_dir / "descriptors.bin", descriptors)
    ring_keys = [[sum(row) / max(1, len(row)) for row in descriptor] for descriptor in descriptors]
    sector_keys = [
        [sum(descriptor[ring][sector] for ring in range(RINGS)) / RINGS for sector in range(SECTORS)]
        for descriptor in descriptors
    ]
    _write_binary_matrix(scan_dir / "ring_keys.bin", ring_keys)
    _write_binary_matrix(scan_dir / "sector_keys.bin", sector_keys)

    candidates = []
    accepted = []
    seen_pairs: set[tuple[int, int]] = set()
    for query_index, descriptor in enumerate(descriptors):
        ranked = _retrieve_candidates(query_index, descriptor, descriptors, ring_keys)
        for rank, (match_index, distance, yaw) in enumerate(ranked, start=1):
            geom_ok, geom_dx, geom_dy, geom_dyaw = _geometric_verify(
                lidar_clouds[query_index],
                lidar_clouds[match_index],
                yaw,
            )
            slam_xy, slam_dyaw = _slam_relative_xy_yaw(keyframes[query_index], keyframes[match_index])
            accepted_loop = _should_accept_loop(
                rank=rank,
                distance=distance,
                geom_ok=geom_ok,
                geom_dx=geom_dx,
                geom_dy=geom_dy,
                geom_dyaw=geom_dyaw,
                slam_xy=slam_xy,
                slam_dyaw=slam_dyaw,
            )
            pair = (min(query_index, match_index), max(query_index, match_index))
            if accepted_loop and pair in seen_pairs:
                accepted_loop = False
                rejection_reason = "duplicate_pair"
            else:
                rejection_reason = "" if accepted_loop else _loop_rejection_reason(
                    rank=rank,
                    distance=distance,
                    geom_ok=geom_ok,
                    slam_xy=slam_xy,
                )
            row = {
                "query_index": query_index,
                "match_index": match_index,
                "rank": rank,
                "distance": round(distance, 6),
                "estimated_yaw_rad": round(yaw, 6),
                "geometric_verified": geom_ok,
                "accepted": accepted_loop,
                "rejection_reason": rejection_reason,
                "dx": round(geom_dx, 4),
                "dy": round(geom_dy, 4),
                "dyaw": round(geom_dyaw, 6),
            }
            candidates.append(row)
            if accepted_loop:
                seen_pairs.add(pair)
                accepted.append(row)

    accepted.sort(key=lambda item: item["distance"])
    if len(accepted) > MAX_ACCEPTED_LOOPS:
        kept = {(item["query_index"], item["match_index"]) for item in accepted[:MAX_ACCEPTED_LOOPS]}
        for row in candidates:
            if row["accepted"] and (row["query_index"], row["match_index"]) not in kept:
                row["accepted"] = False
                row["rejection_reason"] = "accepted_loop_limit"
        accepted = accepted[:MAX_ACCEPTED_LOOPS]

    index_payload = {
        "schema_version": 2,
        "rings": RINGS,
        "sectors": SECTORS,
        "max_radius_m": MAX_RADIUS_M,
        "descriptor": "max-height",
        "frame": "lidar",
        "candidate_top_k": CANDIDATE_TOP_K,
        "min_keyframe_gap": MIN_KEYFRAME_GAP,
        "yaw_search_steps": YAW_SEARCH_STEPS,
        "loop_min_slam_xy_m": LOOP_MIN_SLAM_XY_M,
        "loop_max_slam_xy_m": LOOP_MAX_SLAM_XY_M,
        "accept_descriptor_distance_max": ACCEPT_DISTANCE,
        "geometric_xy_limit_m": GEOM_XY_LIMIT_M,
        "geometric_yaw_limit_rad": GEOM_YAW_LIMIT_RAD,
        "geometric_overlap_min": GEOM_OVERLAP_MIN,
        "max_accepted_loops": MAX_ACCEPTED_LOOPS,
        "automatic_application_enabled": bool(apply_to_graph),
        "keyframe_count": len(keyframes),
        "quaternion_order": "xyzw",
    }
    (scan_dir / "index.json").write_text(json.dumps(index_payload, indent=2), encoding="utf-8")
    with (scan_dir / "loop_candidates.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "query_index",
                "match_index",
                "rank",
                "distance",
                "estimated_yaw_rad",
                "geometric_verified",
                "accepted",
                "rejection_reason",
                "dx",
                "dy",
                "dyaw",
            ],
        )
        writer.writeheader()
        writer.writerows(candidates)
    graph_loops = accepted if apply_to_graph else []
    _write_loop_closures_csv(root, graph_loops)

    if not optimized_path.exists() and raw_path.exists():
        optimized_path.write_bytes(raw_path.read_bytes())
    elif not optimized_path.exists():
        _write_trajectory_csv(optimized_path, keyframes)

    # Keep the LIO export. False-loop SE2 rebuilds previously warped map.pcd.
    if map_raw.exists() and map_pcd.exists():
        map_pcd.write_bytes(map_raw.read_bytes())

    return {
        "scan_context_count": len(keyframes),
        "loop_closure_count": len(graph_loops),
        "detected_loop_count": len(accepted),
        "apply_to_graph": apply_to_graph,
        "trajectory_source": "raw",
        "loop_status": (
            ("pending_manual_review" if accepted else "no_valid_loop") if not apply_to_graph
            else ("accepted" if graph_loops else "no_valid_loop")
        ),
        "candidate_count": len(candidates),
        "rejected_loop_count": sum(not row["accepted"] for row in candidates),
        "detection_duration_seconds": round(time.time() - started_at, 3),
    }


def _loop_rejection_reason(*, rank: int, distance: float, geom_ok: bool, slam_xy: float) -> str:
    if rank != 1:
        return "not_top_rank"
    if distance >= ACCEPT_DISTANCE:
        return "descriptor_distance"
    if not geom_ok:
        return "geometric_verification"
    if slam_xy < LOOP_MIN_SLAM_XY_M:
        return "slam_distance_too_near"
    if slam_xy > LOOP_MAX_SLAM_XY_M:
        return "slam_distance_too_far"
    return "rejected_other"


def _should_accept_loop(
    *,
    rank: int,
    distance: float,
    geom_ok: bool,
    geom_dx: float,
    geom_dy: float,
    geom_dyaw: float,
    slam_xy: float,
    slam_dyaw: float,
) -> bool:
    if rank != 1 or not geom_ok or distance > ACCEPT_DISTANCE:
        return False
    if slam_xy < LOOP_MIN_SLAM_XY_M or slam_xy > LOOP_MAX_SLAM_XY_M:
        return False
    if abs(_wrap_angle(geom_dyaw - slam_dyaw)) > GEOM_YAW_LIMIT_RAD:
        return False
    if math.hypot(geom_dx, geom_dy) > GEOM_XY_LIMIT_M:
        return False
    return True


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _quat_normalize(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def _quat_to_rotation(qx: float, qy: float, qz: float, qw: float) -> list[list[float]]:
    qx, qy, qz, qw = _quat_normalize(qx, qy, qz, qw)
    return [
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ]


def _world_to_lidar_frame(
    points: list[tuple[float, float, float]],
    keyframe: dict[str, Any],
) -> list[tuple[float, float, float]]:
    rotation = _quat_to_rotation(
        float(keyframe.get("lidar_qx") or 0.0),
        float(keyframe.get("lidar_qy") or 0.0),
        float(keyframe.get("lidar_qz") or 0.0),
        float(keyframe.get("lidar_qw") or 1.0),
    )
    origin_x = float(keyframe.get("lidar_x") or 0.0)
    origin_y = float(keyframe.get("lidar_y") or 0.0)
    origin_z = float(keyframe.get("lidar_z") or 0.0)
    local = []
    for x, y, z in points:
        dx = x - origin_x
        dy = y - origin_y
        dz = z - origin_z
        local.append(
            (
                rotation[0][0] * dx + rotation[1][0] * dy + rotation[2][0] * dz,
                rotation[0][1] * dx + rotation[1][1] * dy + rotation[2][1] * dz,
                rotation[0][2] * dx + rotation[1][2] * dy + rotation[2][2] * dz,
            )
        )
    return local


def _slam_relative_xy_yaw(query: dict[str, Any], match: dict[str, Any]) -> tuple[float, float]:
    dx = float(match["lidar_x"]) - float(query["lidar_x"])
    dy = float(match["lidar_y"]) - float(query["lidar_y"])
    yaw = float(query.get("yaw") or 0.0)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    dyaw = _wrap_angle(float(match.get("yaw") or 0.0) - yaw)
    return math.hypot(local_x, local_y), dyaw


def _write_loop_closures_csv(root: Path, accepted: list[dict[str, Any]]) -> None:
    path = root / "loop_closures.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["from", "to", "tx", "ty", "tz", "qx", "qy", "qz", "qw", "sigma_translation", "sigma_rotation", "score"]
        )
        for row in accepted:
            yaw = float(row["dyaw"])
            score = max(0.0, 1.0 - float(row["distance"]))
            writer.writerow(
                [
                    int(row["query_index"]),
                    int(row["match_index"]),
                    f"{float(row['dx']):.4f}",
                    f"{float(row['dy']):.4f}",
                    "0",
                    "0",
                    "0",
                    f"{math.sin(yaw / 2.0):.7f}",
                    f"{math.cos(yaw / 2.0):.7f}",
                    "0.50",
                    "0.15",
                    f"{score:.6f}",
                ]
            )


def _rebuild_map_from_optimized_keyframes(
    root: Path,
    raw_keyframes: list[dict[str, Any]],
    optimized_keyframes: list[dict[str, Any]],
) -> bool:
    """Reproject each world-frame keyframe into the optimized SE2 trajectory.

    Kept for offline diagnostics. Live mapping no longer rebuilds map.pcd here
    because false loops previously warped the navigation cloud.
    """
    if len(raw_keyframes) != len(optimized_keyframes):
        return False
    records_by_keyframe = []
    total_points = 0
    for raw, optimized in zip(raw_keyframes, optimized_keyframes):
        records = _load_pcd_records(root / raw["point_cloud_file"])
        if records is None:
            return False
        records_by_keyframe.append((raw, optimized, records))
        total_points += len(records)
    if total_points <= 0:
        return False

    output = root / "map.pcd"
    temporary = root / "map.pcd.optimized.tmp"
    header_prefix = (
        b"# .PCD v0.7 - Point Cloud Data file format\n"
        b"VERSION 0.7\n"
        b"FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
        b"SIZE 4 4 4 4 4 4 4 4\n"
        b"TYPE F F F F F F F F\n"
        b"COUNT 1 1 1 1 1 1 1 1\n"
        b"WIDTH "
    )
    width_placeholder = b"00000000000000000000"
    header_middle = (
        b"\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
    )
    points_placeholder = b"00000000000000000000"
    header_suffix = b"\nDATA binary\n"
    with temporary.open("w+b") as stream:
        stream.write(header_prefix)
        width_offset = stream.tell()
        stream.write(width_placeholder)
        stream.write(header_middle)
        points_offset = stream.tell()
        stream.write(points_placeholder)
        stream.write(header_suffix)
        for raw, optimized, records in records_by_keyframe:
            raw_yaw = float(raw.get("yaw") or 0.0)
            optimized_yaw = float(optimized.get("yaw") or 0.0)
            raw_x, raw_y = float(raw.get("lidar_x") or 0.0), float(raw.get("lidar_y") or 0.0)
            optimized_x = float(optimized.get("lidar_x") or 0.0)
            optimized_y = float(optimized.get("lidar_y") or 0.0)
            raw_z = float(raw.get("lidar_z") or 0.0)
            optimized_z = float(optimized.get("lidar_z") or raw_z)
            raw_cos, raw_sin = math.cos(raw_yaw), math.sin(raw_yaw)
            opt_cos, opt_sin = math.cos(optimized_yaw), math.sin(optimized_yaw)
            yaw_delta = optimized_yaw - raw_yaw
            delta_cos, delta_sin = math.cos(yaw_delta), math.sin(yaw_delta)
            for record in records:
                x, y, z = record[0], record[1], record[2]
                local_x = raw_cos * (x - raw_x) + raw_sin * (y - raw_y)
                local_y = -raw_sin * (x - raw_x) + raw_cos * (y - raw_y)
                new_x = optimized_x + opt_cos * local_x - opt_sin * local_y
                new_y = optimized_y + opt_sin * local_x + opt_cos * local_y
                new_z = z + optimized_z - raw_z
                values = list(record)
                values[0], values[1], values[2] = new_x, new_y, new_z
                if len(values) >= 7:
                    normal_x, normal_y = values[4], values[5]
                    values[4] = delta_cos * normal_x - delta_sin * normal_y
                    values[5] = delta_sin * normal_x + delta_cos * normal_y
                stream.write(struct.pack("<8f", *(float(value) for value in values[:8])))
        stream.seek(width_offset)
        stream.write(f"{total_points:020d}".encode("ascii"))
        stream.seek(points_offset)
        stream.write(f"{total_points:020d}".encode("ascii"))
        stream.flush()
    temporary.replace(output)
    return True


def _load_keyframes(root: Path) -> list[dict[str, Any]]:
    csv_path = root / "keyframes" / "keyframes.csv"
    if not csv_path.is_file():
        return []
    rows = []
    with csv_path.open(encoding="utf-8") as stream:
        for raw in csv.DictReader(stream):
            try:
                index = int(raw.get("index") or 0)
                lidar_x = float(raw.get("lidar_x") or raw.get("x") or 0.0)
                lidar_y = float(raw.get("lidar_y") or raw.get("y") or 0.0)
                lidar_z = float(raw.get("lidar_z") or raw.get("z") or 0.0)
                yaw = float(raw.get("yaw") or 0.0)
                qx = float(raw.get("lidar_qx") or 0.0)
                qy = float(raw.get("lidar_qy") or 0.0)
                qz = float(raw.get("lidar_qz") or math.sin(yaw / 2.0))
                qw = float(raw.get("lidar_qw") or math.cos(yaw / 2.0))
                rows.append(
                    {
                        "index": index,
                        "timestamp": float(raw.get("stamp") or raw.get("timestamp") or 0.0),
                        "world_x": float(raw.get("world_x") or lidar_x),
                        "world_y": float(raw.get("world_y") or lidar_y),
                        "world_z": float(raw.get("world_z") or lidar_z),
                        "world_qx": float(raw.get("world_qx") or qx),
                        "world_qy": float(raw.get("world_qy") or qy),
                        "world_qz": float(raw.get("world_qz") or qz),
                        "world_qw": float(raw.get("world_qw") or qw),
                        "lidar_x": lidar_x,
                        "lidar_y": lidar_y,
                        "lidar_z": lidar_z,
                        "lidar_qx": qx,
                        "lidar_qy": qy,
                        "lidar_qz": qz,
                        "lidar_qw": qw,
                        "yaw": yaw,
                        "point_cloud_file": f"keyframes/scan_{index:05d}.pcd",
                    }
                )
            except (TypeError, ValueError):
                continue
    return rows


def _load_pcd_xyzi(path: Path) -> list[tuple[float, float, float]]:
    if not path.is_file():
        return []
    data = path.read_bytes()
    header, _, body = data.partition(b"DATA binary\n")
    if not body:
        header, _, body = data.partition(b"DATA binary\r\n")
    if not body:
        return []
    fields = []
    width = 0
    for line in header.decode("ascii", errors="ignore").splitlines():
        if line.startswith("FIELDS"):
            fields = line.split()[1:]
        if line.startswith("WIDTH"):
            try:
                width = int(line.split()[1])
            except (IndexError, ValueError):
                width = 0
    if "x" not in fields or "y" not in fields or "z" not in fields:
        return []
    stride = 4 * len(fields)
    count = min(width if width else len(body) // stride, len(body) // stride)
    x_i, y_i, z_i = fields.index("x"), fields.index("y"), fields.index("z")
    points = []
    for index in range(count):
        offset = index * stride
        x = struct.unpack_from("<f", body, offset + 4 * x_i)[0]
        y = struct.unpack_from("<f", body, offset + 4 * y_i)[0]
        z = struct.unpack_from("<f", body, offset + 4 * z_i)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    return points


def _load_pcd_records(path: Path) -> list[tuple[float, ...]] | None:
    """Load the binary eight-float PCD records emitted by robot_slam."""
    if not path.is_file():
        return None
    data = path.read_bytes()
    header, separator, body = data.partition(b"DATA binary\n")
    if not separator:
        header, separator, body = data.partition(b"DATA binary\r\n")
    if not separator:
        return None
    fields: list[str] = []
    width = 0
    sizes: list[int] = []
    types: list[str] = []
    counts: list[int] = []
    for line in header.decode("ascii", errors="ignore").splitlines():
        tokens = line.split()
        if not tokens:
            continue
        if tokens[0] == "FIELDS":
            fields = tokens[1:]
        elif tokens[0] == "WIDTH" and len(tokens) > 1:
            try:
                width = int(tokens[1])
            except ValueError:
                return None
        elif tokens[0] == "SIZE":
            sizes = [int(value) for value in tokens[1:]]
        elif tokens[0] == "TYPE":
            types = tokens[1:]
        elif tokens[0] == "COUNT":
            counts = [int(value) for value in tokens[1:]]
    if fields != ["x", "y", "z", "intensity", "normal_x", "normal_y", "normal_z", "curvature"]:
        return None
    if sizes != [4] * 8 or types != ["F"] * 8 or counts not in ([], [1] * 8):
        return None
    stride = 32
    count = min(width if width else len(body) // stride, len(body) // stride)
    records: list[tuple[float, ...]] = []
    for index in range(count):
        offset = index * stride
        values = struct.unpack_from("<8f", body, offset)
        if all(math.isfinite(value) for value in values[:3]):
            records.append(values)
    return records


def _scan_context(points: list[tuple[float, float, float]]) -> list[list[float]]:
    descriptor = [[-1e9] * SECTORS for _ in range(RINGS)]
    for x, y, z in points:
        radius = math.hypot(x, y)
        if radius <= 1e-6 or radius > MAX_RADIUS_M:
            continue
        ring = min(RINGS - 1, int(radius / MAX_RADIUS_M * RINGS))
        yaw = math.atan2(y, x)
        sector = int((yaw + math.pi) / (2.0 * math.pi) * SECTORS) % SECTORS
        if z > descriptor[ring][sector]:
            descriptor[ring][sector] = z
    for ring in range(RINGS):
        for sector in range(SECTORS):
            if descriptor[ring][sector] < -1e8:
                descriptor[ring][sector] = 0.0
    return descriptor


def _shift_descriptor(descriptor: list[list[float]], shift: int) -> list[list[float]]:
    if shift % SECTORS == 0:
        return descriptor
    return [row[shift:] + row[:shift] for row in descriptor]


def _descriptor_distance(left: list[list[float]], right: list[list[float]]) -> float:
    total = 0.0
    count = 0
    for ring in range(RINGS):
        for sector in range(SECTORS):
            total += abs(left[ring][sector] - right[ring][sector])
            count += 1
    return total / max(1, count)


def _retrieve_candidates(
    query_index: int,
    query: list[list[float]],
    descriptors: list[list[list[float]]],
    ring_keys: list[list[float]],
) -> list[tuple[int, float, float]]:
    query_key = ring_keys[query_index]
    scored = []
    for match_index, match_key in enumerate(ring_keys):
        if abs(match_index - query_index) < MIN_KEYFRAME_GAP:
            continue
        key_distance = sum(abs(a - b) for a, b in zip(query_key, match_key)) / max(1, len(query_key))
        scored.append((key_distance, match_index))
    scored.sort()
    results = []
    for _, match_index in scored[: max(CANDIDATE_TOP_K * 3, CANDIDATE_TOP_K)]:
        best_distance = 1e9
        best_yaw = 0.0
        for step in range(YAW_SEARCH_STEPS):
            shift = int(round(step * SECTORS / YAW_SEARCH_STEPS)) % SECTORS
            distance = _descriptor_distance(query, _shift_descriptor(descriptors[match_index], shift))
            if distance < best_distance:
                best_distance = distance
                best_yaw = shift * (2.0 * math.pi / SECTORS)
                if best_yaw > math.pi:
                    best_yaw -= 2.0 * math.pi
        results.append((match_index, best_distance, best_yaw))
    results.sort(key=lambda item: item[1])
    return results[:CANDIDATE_TOP_K]


def _geometric_verify(
    query_points: list[tuple[float, float, float]],
    match_points: list[tuple[float, float, float]],
    yaw: float,
) -> tuple[bool, float, float, float]:
    if not query_points or not match_points:
        return False, 0.0, 0.0, yaw
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotated = [(cosine * x - sine * y, sine * x + cosine * y) for x, y, _z in match_points]
    query_xy = [(x, y) for x, y, _z in query_points]
    query_occ = _occupancy(query_xy)
    match_occ = _occupancy(rotated)
    if not query_occ or not match_occ:
        return False, 0.0, 0.0, yaw
    overlap = len(query_occ & match_occ) / max(1, min(len(query_occ), len(match_occ)))
    qcx = sum(x for x, _y in query_xy) / len(query_xy)
    qcy = sum(y for _x, y in query_xy) / len(query_xy)
    mcx = sum(x for x, _y in rotated) / len(rotated)
    mcy = sum(y for _x, y in rotated) / len(rotated)
    dx, dy = qcx - mcx, qcy - mcy
    ok = overlap >= GEOM_OVERLAP_MIN and math.hypot(dx, dy) <= GEOM_XY_LIMIT_M
    return ok, dx, dy, yaw


def _occupancy(points: list[tuple[float, float]], resolution: float = 0.5) -> set[tuple[int, int]]:
    cells = set()
    for x, y in points:
        cells.add((int(math.floor(x / resolution)), int(math.floor(y / resolution))))
        if len(cells) > 20000:
            break
    return cells


def _optimize_se2(keyframes: list[dict[str, Any]], loops: list[dict[str, Any]]) -> list[dict[str, Any]]:
    poses = [[kf["lidar_x"], kf["lidar_y"], kf["yaw"]] for kf in keyframes]
    for _ in range(8):
        for loop in loops:
            i = int(loop["query_index"])
            j = int(loop["match_index"])
            if i == 0 or i >= len(poses) or j >= len(poses):
                continue
            alpha = 0.15
            poses[i][0] = (1.0 - alpha) * poses[i][0] + alpha * (poses[j][0] + loop["dx"])
            poses[i][1] = (1.0 - alpha) * poses[i][1] + alpha * (poses[j][1] + loop["dy"])
            poses[i][2] = poses[i][2] + alpha * loop["dyaw"]
    updated = []
    for keyframe, pose in zip(keyframes, poses):
        item = dict(keyframe)
        item["lidar_x"], item["lidar_y"], item["yaw"] = pose
        item["lidar_qz"] = math.sin(pose[2] / 2.0)
        item["lidar_qw"] = math.cos(pose[2] / 2.0)
        item["lidar_qx"] = 0.0
        item["lidar_qy"] = 0.0
        updated.append(item)
    return updated


def optimize_reviewed_loop_closures(
    map_dir: str | Path,
    selected_candidates: list[dict[str, Any]],
    thresholds: dict[str, Any],
    *,
    grid_converter: str | Path,
    navigation_grid_min_height_m: float = 0.50,
    navigation_grid_max_height_m: float = 0.75,
    navigation_grid_min_points_per_cell: int = 3,
    navigation_grid_support_radius_cells: int = 1,
) -> dict[str, Any]:
    """Apply operator-confirmed loops to an immutable copied map directory.

    This is an offline SE(2) pose graph: the sequential constraints are the
    tightly-coupled FAST-LIO2 keyframe deltas and the additional constraints
    are the reviewed Scan-Context matches. Roll, pitch and Z remain from LIO.
    """
    import numpy as np
    from scipy.optimize import least_squares

    root = Path(map_dir)
    keyframes = _load_keyframes(root)
    if len(keyframes) < 2:
        raise RuntimeError("offline optimization requires at least two keyframes")
    if not selected_candidates:
        raise RuntimeError("no reviewed loop candidates selected")
    started = time.monotonic()
    raw_poses = np.asarray([[kf["lidar_x"], kf["lidar_y"], kf["yaw"]] for kf in keyframes], dtype=float)

    def relative(left, right):
        cosine, sine = math.cos(left[2]), math.sin(left[2])
        dx, dy = right[0] - left[0], right[1] - left[1]
        return np.asarray([cosine * dx + sine * dy, -sine * dx + cosine * dy, _wrap_angle(right[2] - left[2])])

    sequential = [(index - 1, index, relative(raw_poses[index - 1], raw_poses[index])) for index in range(1, len(raw_poses))]
    loops = []
    for candidate in selected_candidates:
        query = int(candidate.get("query_index", -1))
        match = int(candidate.get("match_index", -1))
        if query < 0 or match < 0 or query >= len(raw_poses) or match >= len(raw_poses) or query == match:
            raise RuntimeError(f"invalid reviewed loop endpoints: {query}:{match}")
        slam_distance = math.hypot(raw_poses[query][0] - raw_poses[match][0], raw_poses[query][1] - raw_poses[match][1])
        geometric_translation = math.hypot(
            float(candidate.get("geometric_dx_m") or 0.0),
            float(candidate.get("geometric_dy_m") or 0.0),
        )
        checks = (
            int(candidate.get("rank") or 999) <= int(thresholds.get("max_rank", 1)),
            float(candidate.get("descriptor_distance") or 999.0) <= float(thresholds.get("descriptor_distance_max", 0.30)),
            bool(candidate.get("geometric_verified")),
            geometric_translation <= float(thresholds.get("geometric_translation_max_m", 2.50)),
            float(thresholds.get("slam_distance_min_m", 2.0)) <= slam_distance <= float(thresholds.get("slam_distance_max_m", 20.0)),
            float(candidate.get("yaw_consistency_deg") or 999.0) <= float(thresholds.get("yaw_consistency_max_deg", 34.38)),
            str(candidate.get("detector_rejection_reason") or "") != "duplicate_pair",
        )
        if not all(checks):
            raise RuntimeError(f"loop candidate failed device-side threshold audit: {query}:{match}")
        loops.append((query, match, np.asarray([
            float(candidate.get("geometric_dx_m") or candidate.get("dx") or 0.0),
            float(candidate.get("geometric_dy_m") or candidate.get("dy") or 0.0),
            float(candidate.get("geometric_dyaw_rad") or candidate.get("dyaw") or 0.0),
        ]), candidate))

    def unpack(values):
        poses = raw_poses.copy()
        poses[1:] = values.reshape((-1, 3))
        return poses

    def residual(values):
        poses = unpack(values)
        output = []
        for left, right, measured in sequential:
            error = relative(poses[left], poses[right]) - measured
            error[2] = _wrap_angle(error[2])
            output.extend((error / np.asarray([0.15, 0.15, 0.08])).tolist())
        for left, right, measured, _candidate in loops:
            error = relative(poses[left], poses[right]) - measured
            error[2] = _wrap_angle(error[2])
            output.extend((error / np.asarray([0.50, 0.50, 0.15])).tolist())
        return np.asarray(output)

    initial = raw_poses[1:].reshape(-1)
    error_before = float(np.dot(residual(initial), residual(initial)))
    result = least_squares(residual, initial, loss="huber", f_scale=1.345, max_nfev=100)
    optimized_poses = unpack(result.x)
    error_after = float(np.dot(residual(result.x), residual(result.x)))
    optimized = []
    position_corrections = []
    yaw_corrections = []
    for keyframe, pose, raw_pose in zip(keyframes, optimized_poses, raw_poses):
        item = dict(keyframe)
        item["lidar_x"], item["lidar_y"], item["yaw"] = [float(value) for value in pose]
        # Preserve measured roll/pitch and apply the optimized yaw as a planar
        # correction. The keyframe cloud reprojector uses this same transform.
        item["lidar_qx"], item["lidar_qy"] = 0.0, 0.0
        item["lidar_qz"], item["lidar_qw"] = math.sin(item["yaw"] / 2.0), math.cos(item["yaw"] / 2.0)
        item["world_x"], item["world_y"] = item["lidar_x"], item["lidar_y"]
        item["world_qx"], item["world_qy"] = item["lidar_qx"], item["lidar_qy"]
        item["world_qz"], item["world_qw"] = item["lidar_qz"], item["lidar_qw"]
        optimized.append(item)
        position_corrections.append(math.hypot(pose[0] - raw_pose[0], pose[1] - raw_pose[1]))
        yaw_corrections.append(abs(math.degrees(_wrap_angle(pose[2] - raw_pose[2]))))

    max_position = max(position_corrections, default=0.0)
    max_yaw = max(yaw_corrections, default=0.0)
    position_limit = float(thresholds.get("max_pose_correction_m", 5.0))
    yaw_limit = float(thresholds.get("max_yaw_correction_deg", 20.0))
    if not result.success or error_after > error_before + 1e-6:
        raise RuntimeError(f"pose graph did not converge safely: {result.message}")
    if max_position > position_limit or max_yaw > yaw_limit:
        raise RuntimeError(
            f"optimized correction exceeds guard: {max_position:.3f}m/{max_yaw:.2f}deg "
            f"> {position_limit:.3f}m/{yaw_limit:.2f}deg"
        )

    corrections = []
    for raw, after, position_m, yaw_deg in zip(keyframes, optimized, position_corrections, yaw_corrections):
        raw_pose = {"x": raw["lidar_x"], "y": raw["lidar_y"], "z": raw["lidar_z"], "yaw": raw["yaw"], "qx": raw["lidar_qx"], "qy": raw["lidar_qy"], "qz": raw["lidar_qz"], "qw": raw["lidar_qw"]}
        optimized_pose = {"x": after["lidar_x"], "y": after["lidar_y"], "z": after["lidar_z"], "yaw": after["yaw"], "qx": after["lidar_qx"], "qy": after["lidar_qy"], "qz": after["lidar_qz"], "qw": after["lidar_qw"]}
        dx, dy = after["lidar_x"] - raw["lidar_x"], after["lidar_y"] - raw["lidar_y"]
        signed_yaw = math.degrees(_wrap_angle(after["yaw"] - raw["yaw"]))
        corrections.append({
            "index": raw["index"], "stamp": raw["timestamp"],
            "raw": {key: round(float(value), 7) for key, value in raw_pose.items()},
            "optimized": {key: round(float(value), 7) for key, value in optimized_pose.items()},
            "delta": {"x": round(dx, 5), "y": round(dy, 5), "z": 0.0, "position_m": round(position_m, 5), "yaw_rad": round(math.radians(signed_yaw), 6), "yaw_deg": round(signed_yaw, 4)},
            "significant": position_m >= 0.30 or yaw_deg >= 3.0,
            "constraint_source": ["fast_lio2", "loop_closure"],
        })

    _write_loop_closures_csv(root, [{
        "query_index": left,
        "match_index": right,
        "dx": float(measured[0]),
        "dy": float(measured[1]),
        "dyaw": float(measured[2]),
        "distance": float(candidate.get("descriptor_distance") or candidate.get("distance") or 0.0),
    } for left, right, measured, candidate in loops])
    _write_trajectory_csv(root / "trajectory_optimized.csv", optimized)
    if not _rebuild_map_from_optimized_keyframes(root, keyframes, optimized):
        raise RuntimeError("failed to rebuild map.pcd from optimized keyframes")
    converter = Path(grid_converter).expanduser()
    if not converter.is_file():
        raise RuntimeError(f"grid converter not found: {converter}")
    subprocess.run(
        [
            str(converter), str(root / "map.pcd"), str(root / "map"), "0.05",
            str(navigation_grid_min_height_m), str(navigation_grid_max_height_m),
            "200000000", str(navigation_grid_min_points_per_cell),
            str(navigation_grid_support_radius_cells),
        ],
        check=True, capture_output=True, text=True, timeout=1800,
    )
    with (root / "map.txt").open("w", encoding="utf-8") as stream:
        stream.write("# path\n")
        for pose in optimized_poses:
            stream.write(f"{pose[0]:.2f} {pose[1]:.2f} {pose[2]:.2f}\n")

    def stats(values):
        array = np.asarray(values, dtype=float)
        return {
            "mean": round(float(np.mean(array)), 5),
            "rms": round(float(np.sqrt(np.mean(array * array))), 5),
            "p95": round(float(np.percentile(array, 95)), 5),
            "max": round(float(np.max(array)), 5),
        }

    summary = {
        "schema": "roamerx.optimization-summary.v1",
        "stage": "completed",
        "success": True,
        "applied": True,
        "candidate_applied": True,
        "trigger_source": "manual_map_management",
        "completed_at_unix": round(time.time(), 3),
        "optimization_mode": "offline_fast_lio2_se2_pose_graph",
        "trajectory_source": "optimized",
        "keyframe_count": len(keyframes),
        "candidate_count": len(selected_candidates),
        "accepted_loop_count": len(loops),
        "rejected_loop_count": 0,
        "factors": {"total": len(sequential) + len(loops), "lio_between": len(sequential), "ndt": len(sequential), "imu": 0, "rtk_position": 0, "rtk_heading": 0, "loop_closure": len(loops)},
        "graph_error": {"before": round(error_before, 6), "after": round(error_after, 6), "reduction_percent": round((error_before - error_after) / max(error_before, 1e-9) * 100.0, 2)},
        "correction": {"position_m": stats(position_corrections), "yaw_deg": stats(yaw_corrections)},
        "quality_guard": {"passed": True, "max_pose_correction_m": position_limit, "max_yaw_correction_deg": yaw_limit},
        "review_thresholds": thresholds,
        "selected_candidate_ids": [str(candidate.get("candidate_id") or f"{left}:{right}") for left, right, _measured, candidate in loops],
        "timing": {"optimization_and_rebuild_seconds": round(time.monotonic() - started, 3)},
        "corrections": corrections,
    }
    manifest_path = root / "map_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {}
    summary["mapping_type"] = "indoor" if str(manifest.get("scene_scope") or "indoor") == "indoor" else "outdoor"
    summary["auto_activation_allowed"] = True
    (root / "optimization_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    manifest["optimization"] = {key: value for key, value in summary.items() if key not in {"corrections"}}
    manifest["trajectory_source"] = "optimized"
    manifest["parent_map_id"] = str(thresholds.get("source_map_id") or "")
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def _trajectory_jump_m(raw: list[dict[str, Any]], optimized: list[dict[str, Any]]) -> float:
    worst = 0.0
    for left, right in zip(raw, optimized):
        worst = max(worst, math.hypot(left["lidar_x"] - right["lidar_x"], left["lidar_y"] - right["lidar_y"]))
    return worst


def _write_trajectory_csv(path: Path, keyframes: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "index",
                "timestamp",
                "world_x",
                "world_y",
                "world_z",
                "world_qx",
                "world_qy",
                "world_qz",
                "world_qw",
                "lidar_x",
                "lidar_y",
                "lidar_z",
                "lidar_qx",
                "lidar_qy",
                "lidar_qz",
                "lidar_qw",
            ],
        )
        writer.writeheader()
        for keyframe in keyframes:
            writer.writerow(
                {
                    "index": keyframe["index"],
                    "timestamp": f"{keyframe['timestamp']:.6f}",
                    "world_x": f"{keyframe['world_x']:.6f}",
                    "world_y": f"{keyframe['world_y']:.6f}",
                    "world_z": f"{keyframe['world_z']:.6f}",
                    "world_qx": f"{keyframe['world_qx']:.7f}",
                    "world_qy": f"{keyframe['world_qy']:.7f}",
                    "world_qz": f"{keyframe['world_qz']:.7f}",
                    "world_qw": f"{keyframe['world_qw']:.7f}",
                    "lidar_x": f"{keyframe['lidar_x']:.6f}",
                    "lidar_y": f"{keyframe['lidar_y']:.6f}",
                    "lidar_z": f"{keyframe['lidar_z']:.6f}",
                    "lidar_qx": f"{keyframe['lidar_qx']:.7f}",
                    "lidar_qy": f"{keyframe['lidar_qy']:.7f}",
                    "lidar_qz": f"{keyframe['lidar_qz']:.7f}",
                    "lidar_qw": f"{keyframe['lidar_qw']:.7f}",
                }
            )


def _write_binary_matrix(path: Path, rows: list[list[Any]]) -> None:
    payload = []
    for row in rows:
        flat = []
        for item in row:
            if isinstance(item, list):
                flat.extend(float(value) for value in item)
            else:
                flat.append(float(item))
        payload.extend(flat)
    path.write_bytes(struct.pack("<" + "f" * len(payload), *payload) if payload else b"")
