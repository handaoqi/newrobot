#!/usr/bin/env python3
"""Build deterministic stage-0/1 avoidance metrics from a navigation rosbag."""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class TwistSample:
    stamp_ns: int
    vx: float
    vy: float
    wz: float

    @property
    def planar(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def combined(self) -> float:
        # Convert yaw to an approximate footprint-edge speed for comparisons.
        return max(self.planar, abs(self.wz) * 0.25)


def percentile(values: list[float], quantile: float) -> float | None:
    clean = sorted(value for value in values if math.isfinite(value))
    if not clean:
        return None
    rank = max(1, math.ceil(float(quantile) * len(clean)))
    return clean[min(len(clean), rank) - 1]


def _signal_summary(samples: list[TwistSample]) -> dict[str, Any]:
    return {
        "sample_count": len(samples),
        "max_planar_mps": max((sample.planar for sample in samples), default=0.0),
        "p90_planar_mps": percentile([sample.planar for sample in samples], 0.90),
        "max_abs_yaw_rate_rps": max((abs(sample.wz) for sample in samples), default=0.0),
    }


def _latest_pairs(
    requested: list[TwistSample], output: list[TwistSample], max_age_seconds: float = 0.25
) -> list[tuple[TwistSample, TwistSample]]:
    pairs: list[tuple[TwistSample, TwistSample]] = []
    requested = sorted(requested, key=lambda sample: sample.stamp_ns)
    output = sorted(output, key=lambda sample: sample.stamp_ns)
    index = 0
    latest: TwistSample | None = None
    max_age_ns = int(max_age_seconds * 1e9)
    for safe_sample in output:
        while index < len(requested) and requested[index].stamp_ns <= safe_sample.stamp_ns:
            latest = requested[index]
            index += 1
        if latest and safe_sample.stamp_ns - latest.stamp_ns <= max_age_ns:
            pairs.append((latest, safe_sample))
    return pairs


def _intervention_summary(
    requested: list[TwistSample], safe: list[TwistSample]
) -> dict[str, Any]:
    pairs = _latest_pairs(requested, safe)
    requested_motion = [pair for pair in pairs if pair[0].combined >= 0.04]
    limited = [
        pair
        for pair in requested_motion
        if pair[1].combined <= pair[0].combined * 0.75
    ]
    stopped = [pair for pair in requested_motion if pair[1].combined <= 0.01]
    ratios = [
        pair[1].combined / pair[0].combined
        for pair in requested_motion
        if pair[0].combined > 1e-6
    ]
    return {
        "paired_sample_count": len(pairs),
        "requested_motion_sample_count": len(requested_motion),
        "limited_sample_count": len(limited),
        "stopped_sample_count": len(stopped),
        "limited_ratio": len(limited) / len(requested_motion) if requested_motion else None,
        "stopped_ratio": len(stopped) / len(requested_motion) if requested_motion else None,
        "output_to_request_ratio_p50": percentile(ratios, 0.50),
        "output_to_request_ratio_p10": percentile(ratios, 0.10),
    }


def _yaw_reversal_count(samples: list[TwistSample], deadband: float = 0.05) -> int:
    previous_sign = 0
    reversals = 0
    for sample in sorted(samples, key=lambda value: value.stamp_ns):
        sign = 1 if sample.wz > deadband else (-1 if sample.wz < -deadband else 0)
        if sign and previous_sign and sign != previous_sign:
            reversals += 1
        if sign:
            previous_sign = sign
    return reversals


def summarize_records(
    *,
    bag_path: str,
    topic_counts: dict[str, int],
    twists: dict[str, list[TwistSample]],
    front_clearances_m: list[float],
    stop_zone_points: list[int],
    slow_zone_points: list[int],
    mppi_windows: list[dict[str, Any]],
) -> dict[str, Any]:
    required_topics = [
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
    ]
    missing = [topic for topic in required_topics if topic_counts.get(topic, 0) == 0]
    p99_values = [
        float(window["p99_ms"])
        for window in mppi_windows
        if isinstance(window.get("p99_ms"), (int, float))
    ]
    max_values = [
        float(window["max_ms"])
        for window in mppi_windows
        if isinstance(window.get("max_ms"), (int, float))
    ]
    safe = twists.get("/cmd_vel", [])
    result = {
        "schema": "roamerx.avoidance-diagnostics.v1",
        "bag_path": str(Path(bag_path).resolve()),
        "topic_counts": dict(sorted(topic_counts.items())),
        "evidence": {
            "required_topics": required_topics,
            "missing_topics": missing,
            "complete": not missing,
        },
        "velocity": {
            "nav": _signal_summary(twists.get("/cmd_vel_nav", [])),
            "candidate": _signal_summary(twists.get("/cmd_vel_raw", [])),
            "safe": _signal_summary(safe),
            "collision_monitor_intervention": _intervention_summary(
                twists.get("/cmd_vel_raw", []), safe
            ),
            "safe_yaw_direction_reversals": _yaw_reversal_count(safe),
        },
        "clearance": {
            "front_sample_count": len(front_clearances_m),
            "minimum_front_m": min(front_clearances_m) if front_clearances_m else None,
            "p10_front_m": percentile(front_clearances_m, 0.10),
            "stop_zone_points_p90": percentile(
                [float(value) for value in stop_zone_points], 0.90
            ),
            "slow_zone_points_p90": percentile(
                [float(value) for value in slow_zone_points], 0.90
            ),
        },
        "mppi_performance": {
            "window_count": len(mppi_windows),
            "worst_window_p99_ms": max(p99_values) if p99_values else None,
            "maximum_cycle_ms": max(max_values) if max_values else None,
            "target_p99_ms": 40.0,
            "target_met": bool(p99_values) and max(p99_values) < 40.0,
        },
    }
    result["stage_1_automatic_gates"] = {
        "evidence_complete": result["evidence"]["complete"],
        "mppi_p99_under_40_ms": result["mppi_performance"]["target_met"],
        "has_requested_motion": result["velocity"]["candidate"]["max_planar_mps"] >= 0.04,
        # A collision-free result and successful arrival need operator/task
        # evidence; point clouds alone must never certify those safety claims.
        "requires_operator_collision_free_confirmation": True,
        "requires_task_goal_success_confirmation": True,
    }
    return result


def _twist_from_message(message: Any, stamp_ns: int) -> TwistSample:
    twist = getattr(message, "twist", message)
    return TwistSample(
        stamp_ns=stamp_ns,
        vx=float(twist.linear.x),
        vy=float(twist.linear.y),
        wz=float(twist.angular.z),
    )


def read_bag(bag: Path) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_types: dict[str, Any] = {}
    topic_counts: dict[str, int] = {}
    twists: dict[str, list[TwistSample]] = {
        "/cmd_vel_nav": [],
        "/cmd_vel_raw": [],
        "/cmd_vel": [],
    }
    front_clearances_m: list[float] = []
    stop_zone_points: list[int] = []
    slow_zone_points: list[int] = []
    mppi_windows: list[dict[str, Any]] = []
    scan_index = 0

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        if topic not in twists and topic not in {"/laser_scan", "/mppi/performance"}:
            continue
        type_name = topic_types.get(topic)
        if not type_name:
            continue
        if type_name not in message_types:
            message_types[type_name] = get_message(type_name)
        message = deserialize_message(data, message_types[type_name])
        if topic in twists:
            twists[topic].append(_twist_from_message(message, stamp_ns))
            continue
        if topic == "/mppi/performance":
            try:
                payload = json.loads(message.data)
            except (AttributeError, TypeError, ValueError):
                continue
            if isinstance(payload, dict):
                mppi_windows.append(payload)
            continue
        scan_index += 1
        # 2 Hz is sufficient for clearance distributions and bounds bag-analysis CPU.
        if scan_index % 5:
            continue
        nearest = math.inf
        stop_count = 0
        slow_count = 0
        angle = float(message.angle_min)
        for distance in message.ranges:
            value = float(distance)
            if math.isfinite(value) and value >= float(message.range_min):
                x = value * math.cos(angle)
                y = value * math.sin(angle)
                if 0.0 <= x <= 4.0 and abs(y) <= 0.60:
                    nearest = min(nearest, math.hypot(x, y))
                if 0.55 <= x <= 0.90 and abs(y) <= 0.22:
                    stop_count += 1
                if 0.55 <= x <= 1.20 and abs(y) <= 0.30:
                    slow_count += 1
            angle += float(message.angle_increment)
        if math.isfinite(nearest):
            front_clearances_m.append(nearest)
        stop_zone_points.append(stop_count)
        slow_zone_points.append(slow_count)

    result = summarize_records(
        bag_path=str(bag),
        topic_counts=topic_counts,
        twists=twists,
        front_clearances_m=front_clearances_m,
        stop_zone_points=stop_zone_points,
        slow_zone_points=slow_zone_points,
        mppi_windows=mppi_windows,
    )
    scenario_path = bag / "avoidance_scenario.json"
    if scenario_path.is_file():
        try:
            scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            scenario = {"outcome": "invalid_manifest"}
        result["scenario"] = scenario
        automatic = result["stage_1_automatic_gates"]
        eligible = scenario.get("scenario") in {
            "straight",
            "turn",
            "narrow_passage",
            "static_box",
            "wall_corner",
        }
        result["stage_1_acceptance"] = {
            "eligible_static_scenario": eligible,
            "operator_outcome": scenario.get("outcome"),
            "passed": bool(
                eligible
                and scenario.get("outcome") == "pass"
                and automatic["evidence_complete"]
                and automatic["mppi_p99_under_40_ms"]
                and automatic["has_requested_motion"]
            ),
        }
    return result


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not (args.bag / "metadata.yaml").is_file():
        parser.error(f"not a rosbag directory: {args.bag}")
    result = read_bag(args.bag)
    output = args.output or args.bag / "avoidance_summary.json"
    write_json(output, result)
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
