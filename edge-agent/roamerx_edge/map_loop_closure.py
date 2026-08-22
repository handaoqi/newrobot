"""Scan-Context retrieval and a conservative SE2 pose-graph update.

Loop acceptance requires both descriptor similarity and a geometric check.
Optimization failure leaves map_raw.pcd and trajectory_raw.csv untouched.
"""

from __future__ import annotations

import csv
import json
import math
import struct
from pathlib import Path
from typing import Any

RINGS = 20
SECTORS = 60
MAX_RADIUS_M = 80.0
CANDIDATE_TOP_K = 5
MIN_KEYFRAME_GAP = 30
YAW_SEARCH_STEPS = 60
ACCEPT_DISTANCE = 0.35
GEOM_XY_LIMIT_M = 2.5
GEOM_YAW_LIMIT_RAD = 0.6
MAX_OPTIMIZED_JUMP_M = 5.0

IDENTITY_XYZW = (0.0, 0.0, 0.0, 1.0)


def finalize_loop_closure(map_dir: str | Path) -> dict[str, Any]:
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
        return {
            "scan_context_count": 0,
            "loop_closure_count": 0,
            "trajectory_source": "raw",
            "loop_status": "no_keyframes",
        }

    clouds = [_load_pcd_xyzi(root / keyframe["point_cloud_file"]) for keyframe in keyframes]
    descriptors = [_scan_context(points) for points in clouds]
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
    for query_index, descriptor in enumerate(descriptors):
        ranked = _retrieve_candidates(query_index, descriptor, descriptors, ring_keys)
        for rank, (match_index, distance, yaw) in enumerate(ranked, start=1):
            geom_ok, geom_dx, geom_dy, geom_dyaw = _geometric_verify(
                clouds[query_index],
                clouds[match_index],
                yaw,
            )
            accepted_loop = geom_ok and distance <= ACCEPT_DISTANCE
            row = {
                "query_index": query_index,
                "match_index": match_index,
                "rank": rank,
                "distance": round(distance, 6),
                "estimated_yaw_rad": round(yaw, 6),
                "geometric_verified": geom_ok,
                "accepted": accepted_loop,
                "dx": round(geom_dx, 4),
                "dy": round(geom_dy, 4),
                "dyaw": round(geom_dyaw, 6),
            }
            candidates.append(row)
            if accepted_loop:
                accepted.append(row)

    index_payload = {
        "schema_version": 1,
        "rings": RINGS,
        "sectors": SECTORS,
        "max_radius_m": MAX_RADIUS_M,
        "descriptor": "max-height",
        "candidate_top_k": CANDIDATE_TOP_K,
        "min_keyframe_gap": MIN_KEYFRAME_GAP,
        "yaw_search_steps": YAW_SEARCH_STEPS,
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
                "dx",
                "dy",
                "dyaw",
            ],
        )
        writer.writeheader()
        writer.writerows(candidates)

    trajectory_source = "raw"
    loop_status = "no_valid_loop"
    optimized = [dict(item) for item in keyframes]
    if accepted:
        try:
            optimized = _optimize_se2(keyframes, accepted)
            if _trajectory_jump_m(keyframes, optimized) > MAX_OPTIMIZED_JUMP_M:
                raise ValueError("optimized trajectory jumped too far from raw")
            if not _rebuild_map_from_optimized_keyframes(root, keyframes, optimized):
                raise ValueError("optimized keyframe map rebuild failed")
            trajectory_source = "optimized"
            loop_status = "accepted"
        except (OSError, ValueError, ArithmeticError):
            optimized = [dict(item) for item in keyframes]
            trajectory_source = "raw"
            loop_status = "optimization_rejected"

    _write_trajectory_csv(optimized_path, optimized if trajectory_source == "optimized" else keyframes)
    if trajectory_source != "optimized" and map_raw.exists() and map_pcd.exists():
        # Keep the navigation map identical to the raw export.
        map_pcd.write_bytes(map_raw.read_bytes())
    return {
        "scan_context_count": len(keyframes),
        "loop_closure_count": len(accepted) if trajectory_source == "optimized" else 0,
        "trajectory_source": trajectory_source,
        "loop_status": loop_status,
    }


def _rebuild_map_from_optimized_keyframes(
    root: Path,
    raw_keyframes: list[dict[str, Any]],
    optimized_keyframes: list[dict[str, Any]],
) -> bool:
    """Reproject each world-frame keyframe into the optimized SE2 trajectory.

    The C++ GTSAM service is the authoritative optimizer in a live mapping
    session. This local rebuild keeps standalone/offline package finalization
    consistent and is also the safe fallback if the service is unavailable.
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
    ok = overlap >= 0.25 and math.hypot(dx, dy) <= GEOM_XY_LIMIT_M and abs(yaw) <= math.pi
    return ok, dx, dy, yaw


def _occupancy(points: list[tuple[float, float]], resolution: float = 1.0) -> set[tuple[int, int]]:
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
