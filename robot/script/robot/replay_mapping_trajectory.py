#!/usr/bin/env python3
"""Turn a mapping trajectory into sparse Nav2 waypoints.

Default mode only prints the sampled poses. Sending motion requires both
``--send`` and ``--i-confirm-path-clear``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_xyzw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def heading_between(from_xy: tuple[float, float], to_xy: tuple[float, float], minimum_distance: float = 0.05) -> float | None:
    dx = to_xy[0] - from_xy[0]
    dy = to_xy[1] - from_xy[1]
    if math.hypot(dx, dy) < minimum_distance:
        return None
    return math.atan2(dy, dx)


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def _point(index: int, x: float, y: float, yaw: float, z: float = 0.0, stamp: float = 0.0, source_index: int | None = None) -> dict[str, Any]:
    return {
        "index": int(index),
        "source_index": int(source_index if source_index is not None else index),
        "x": round(x, 4),
        "y": round(y, 4),
        "z": round(z, 4),
        "yaw": round(yaw, 5),
        "stamp": stamp,
    }


def load_mapping_trace(path: Path) -> list[dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    samples = payload.get("samples") if isinstance(payload, dict) else None
    if not isinstance(samples, list):
        raise ValueError(f"{path} is not a mapping_trace.json")
    points = []
    for sample in samples:
        slam = sample.get("slam") if isinstance(sample, dict) else None
        if not isinstance(slam, dict):
            continue
        points.append(_point(
            index=len(points),
            x=_finite(slam.get("x")),
            y=_finite(slam.get("y")),
            z=_finite(slam.get("z")),
            yaw=_finite(slam.get("yaw")),
            stamp=_finite(sample.get("stamp")),
            source_index=int(sample.get("index", len(points))),
        ))
    return points


def load_keyframes_csv(path: Path) -> list[dict[str, Any]]:
    points = []
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            yaw = row.get("yaw")
            if yaw in (None, ""):
                yaw = yaw_from_xyzw(
                    _finite(row.get("lidar_qx")),
                    _finite(row.get("lidar_qy")),
                    _finite(row.get("lidar_qz")),
                    _finite(row.get("lidar_qw"), 1.0),
                )
            points.append(_point(
                index=len(points),
                x=_finite(row.get("lidar_x", row.get("x"))),
                y=_finite(row.get("lidar_y", row.get("y"))),
                z=_finite(row.get("lidar_z", row.get("z"))),
                yaw=_finite(yaw),
                stamp=_finite(row.get("stamp", row.get("timestamp"))),
                source_index=int(float(row.get("index", len(points)))),
            ))
    return points


def load_trajectory_csv(path: Path, *, use_lidar: bool = True) -> list[dict[str, Any]]:
    points = []
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if use_lidar and row.get("lidar_x") not in (None, ""):
                x = _finite(row.get("lidar_x"))
                y = _finite(row.get("lidar_y"))
                z = _finite(row.get("lidar_z"))
                yaw = yaw_from_xyzw(
                    _finite(row.get("lidar_qx")),
                    _finite(row.get("lidar_qy")),
                    _finite(row.get("lidar_qz")),
                    _finite(row.get("lidar_qw"), 1.0),
                )
            else:
                x = _finite(row.get("world_x", row.get("x")))
                y = _finite(row.get("world_y", row.get("y")))
                z = _finite(row.get("world_z", row.get("z")))
                yaw = yaw_from_xyzw(
                    _finite(row.get("world_qx")),
                    _finite(row.get("world_qy")),
                    _finite(row.get("world_qz")),
                    _finite(row.get("world_qw"), 1.0),
                )
            points.append(_point(
                index=len(points),
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                stamp=_finite(row.get("timestamp", row.get("stamp"))),
                source_index=int(float(row.get("index", len(points)))),
            ))
    return points


def resolve_trajectory_file(map_dir: Path, source: str) -> tuple[Path, str]:
    candidates = {
        "trace": map_dir / "mapping_trace.json",
        "keyframes": map_dir / "keyframes" / "keyframes.csv",
        "raw": map_dir / "trajectory_raw.csv",
        "optimized": map_dir / "trajectory_optimized.csv",
    }
    if source != "auto":
        path = candidates[source]
        if not path.exists():
            raise FileNotFoundError(f"missing {path}")
        return path, source
    for name in ("trace", "keyframes", "raw"):
        if candidates[name].exists():
            return candidates[name], name
    raise FileNotFoundError(f"no mapping_trace.json, keyframes.csv or trajectory_raw.csv in {map_dir}")


def load_points(path: Path, source: str) -> list[dict[str, Any]]:
    if source == "trace" or path.suffix == ".json":
        return load_mapping_trace(path)
    if source == "keyframes" or path.name == "keyframes.csv":
        return load_keyframes_csv(path)
    if source == "optimized":
        return load_trajectory_csv(path, use_lidar=False)
    return load_trajectory_csv(path, use_lidar=True)


def planar_span(points: Iterable[dict[str, Any]]) -> dict[str, float]:
    items = list(points)
    if not items:
        return {"xy_m": 0.0, "z_m": 0.0, "path_m": 0.0}
    xs = [p["x"] for p in items]
    ys = [p["y"] for p in items]
    zs = [p["z"] for p in items]
    path = 0.0
    for previous, current in zip(items, items[1:]):
        path += math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
    return {
        "xy_m": math.hypot(max(xs) - min(xs), max(ys) - min(ys)),
        "z_m": max(zs) - min(zs),
        "path_m": path,
    }


def trim_stationary(points: list[dict[str, Any]], skip_start_m: float, skip_end_m: float) -> list[dict[str, Any]]:
    if len(points) < 2:
        return list(points)
    start = 0
    while start + 1 < len(points):
        if math.hypot(points[start]["x"] - points[0]["x"], points[start]["y"] - points[0]["y"]) >= skip_start_m:
            break
        start += 1
    end = len(points) - 1
    last = points[-1]
    while end > start:
        if math.hypot(points[end]["x"] - last["x"], points[end]["y"] - last["y"]) >= skip_end_m:
            break
        end -= 1
    trimmed = points[start:end + 1]
    return trimmed or points[:1]


def sparsify_points(
    points: list[dict[str, Any]],
    *,
    min_distance_m: float = 2.5,
    min_yaw_rad: float = 0.5,
    max_waypoints: int = 25,
    skip_start_m: float = 0.4,
    skip_end_m: float = 0.3,
    yaw_source: str = "path",
) -> list[dict[str, Any]]:
    if not points:
        return []
    work = trim_stationary(points, skip_start_m, skip_end_m)
    if len(work) == 1:
        return [dict(work[0], sequence=0, map_point_number=1, localization_mode="ndt")]

    def keep_with_thresholds(distance: float, yaw: float) -> list[dict[str, Any]]:
        kept = [work[0]]
        for point in work[1:-1]:
            last = kept[-1]
            moved = math.hypot(point["x"] - last["x"], point["y"] - last["y"])
            turned = abs(wrap_angle(point["yaw"] - last["yaw"]))
            if moved >= distance or turned >= yaw:
                kept.append(point)
        last_point = work[-1]
        if math.hypot(last_point["x"] - kept[-1]["x"], last_point["y"] - kept[-1]["y"]) >= 0.2:
            kept.append(last_point)
        elif kept[-1] is not last_point:
            kept[-1] = last_point
        return kept

    kept = keep_with_thresholds(min_distance_m, min_yaw_rad)
    distance = min_distance_m
    while len(kept) > max_waypoints and distance < 20.0:
        distance *= 1.25
        kept = keep_with_thresholds(distance, min_yaw_rad)

    if yaw_source == "path":
        for index, point in enumerate(kept[:-1]):
            heading = heading_between((point["x"], point["y"]), (kept[index + 1]["x"], kept[index + 1]["y"]))
            if heading is not None:
                point["yaw"] = round(heading, 5)
        if len(kept) >= 2:
            kept[-1]["yaw"] = kept[-2]["yaw"]

    waypoints = []
    for sequence, point in enumerate(kept):
        item = dict(point)
        item["sequence"] = sequence
        item["map_point_number"] = sequence + 1
        item["localization_mode"] = "ndt"
        waypoints.append(item)
    return waypoints


def load_and_sparsify(map_dir: Path, source: str = "auto", **kwargs: Any) -> dict[str, Any]:
    path, resolved_source = resolve_trajectory_file(map_dir, source)
    points = load_points(path, resolved_source)
    span = planar_span(points)
    warning = ""
    if resolved_source == "optimized" and span["z_m"] > max(2.0, 2.0 * span["xy_m"]):
        warning = (
            "trajectory_optimized.csv moves mostly in Z; Nav2 needs planar lidar x/y. "
            "Re-run with --source trace or --source raw."
        )
    waypoints = sparsify_points(points, **kwargs)
    return {
        "map_dir": str(map_dir),
        "source_file": str(path),
        "source": resolved_source,
        "input_count": len(points),
        "waypoint_count": len(waypoints),
        "span": span,
        "warning": warning,
        "waypoints": waypoints,
    }


def print_summary(result: dict[str, Any]) -> None:
    print(f"source: {result['source']}  {result['source_file']}")
    print(
        f"input: {result['input_count']} poses  "
        f"path {result['span']['path_m']:.1f} m  "
        f"xy span {result['span']['xy_m']:.1f} m  "
        f"z span {result['span']['z_m']:.1f} m"
    )
    if result["warning"]:
        print(f"warning: {result['warning']}")
    print(f"waypoints: {result['waypoint_count']}")
    for waypoint in result["waypoints"]:
        print(
            f"  {waypoint['map_point_number']:2d}  "
            f"src={waypoint['source_index']:3d}  "
            f"x={waypoint['x']:7.3f}  y={waypoint['y']:7.3f}  "
            f"yaw={math.degrees(waypoint['yaw']):7.1f} deg"
        )


def send_waypoints(waypoints: list[dict[str, Any]], timeout_seconds: float = 30.0) -> int:
    import time

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import FollowWaypoints
    from rclpy.action import ActionClient

    rclpy.init(args=None)
    node = rclpy.create_node("replay_mapping_trajectory")
    client = ActionClient(node, FollowWaypoints, "/follow_waypoints")
    try:
        if not client.wait_for_server(timeout_sec=timeout_seconds):
            print("FollowWaypoints action server is unavailable", file=sys.stderr)
            return 2
        goal = FollowWaypoints.Goal()
        now = node.get_clock().now().to_msg()
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now
            pose.pose.position.x = float(waypoint["x"])
            pose.pose.position.y = float(waypoint["y"])
            yaw = float(waypoint["yaw"])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal.poses.append(pose)
        future = client.send_goal_async(goal)
        deadline = time.monotonic() + timeout_seconds
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done() or future.result() is None or not future.result().accepted:
            print("FollowWaypoints goal was rejected", file=sys.stderr)
            return 3
        print(f"sent {len(waypoints)} waypoints to /follow_waypoints")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "map_dir",
        nargs="?",
        type=Path,
        default=Path("/home/dogrobot/runtime/nx-edge/data/jszr/map"),
        help="Map session directory containing mapping_trace.json or keyframes.csv",
    )
    parser.add_argument("--source", choices=("auto", "trace", "keyframes", "raw", "optimized"), default="auto")
    parser.add_argument("--min-distance-m", type=float, default=2.5)
    parser.add_argument("--min-yaw-rad", type=float, default=0.5)
    parser.add_argument("--max-waypoints", type=int, default=25)
    parser.add_argument("--skip-start-m", type=float, default=0.4)
    parser.add_argument("--skip-end-m", type=float, default=0.3)
    parser.add_argument("--yaw-source", choices=("path", "recorded"), default="path")
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--send", action="store_true", help="Send FollowWaypoints after sampling")
    parser.add_argument(
        "--i-confirm-path-clear",
        action="store_true",
        help="Required with --send; confirms the physical path is clear",
    )
    args = parser.parse_args()
    result = load_and_sparsify(
        args.map_dir.resolve(),
        source=args.source,
        min_distance_m=args.min_distance_m,
        min_yaw_rad=args.min_yaw_rad,
        max_waypoints=args.max_waypoints,
        skip_start_m=args.skip_start_m,
        skip_end_m=args.skip_end_m,
        yaw_source=args.yaw_source,
    )
    print_summary(result)
    if args.json_out:
        args.json_out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"wrote {args.json_out}")
    if result["warning"] and args.send:
        print(result["warning"], file=sys.stderr)
        return 4
    if args.send:
        if not args.i_confirm_path_clear:
            print("refusing to send motion without --i-confirm-path-clear", file=sys.stderr)
            return 5
        if not result["waypoints"]:
            print("no waypoints to send", file=sys.stderr)
            return 6
        return send_waypoints(result["waypoints"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
