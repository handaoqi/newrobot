from __future__ import annotations

import math
from collections import defaultdict
from pathlib import Path
from typing import Any


def _check(
    rule_id: str,
    title: str,
    category: str,
    status: str,
    *,
    message: str = "",
    hard_failure: bool = False,
    metric_name: str = "",
    actual_value: Any = None,
    expected_value: Any = None,
    topics: list[str] | None = None,
    start_time_ns: int | None = None,
    end_time_ns: int | None = None,
    evidence: dict | None = None,
) -> dict:
    return {
        "rule_id": rule_id,
        "title": title,
        "category": category,
        "status": status,
        "severity": "error" if status == "FAIL" else "warning" if status == "WARN" else "info",
        "hard_failure": hard_failure,
        "metric_name": metric_name,
        "actual_value": actual_value,
        "expected_value": expected_value,
        "topics": topics or [],
        "start_time_ns": start_time_ns,
        "end_time_ns": end_time_ns,
        "evidence": evidence or {},
        "message": message,
    }


def _get_message_class(type_name: str):
    try:
        from rosidl_runtime_py.utilities import get_message

        return get_message(type_name)
    except Exception:
        return None


def _deserialize(data: bytes, message_class):
    if message_class is None:
        return None
    try:
        from rclpy.serialization import deserialize_message

        return deserialize_message(data, message_class)
    except Exception:
        return None


def _angular_z(message) -> float | None:
    if message is None:
        return None
    twist = getattr(message, "twist", message)
    twist = getattr(twist, "twist", twist)
    angular = getattr(twist, "angular", None)
    return float(angular.z) if angular is not None else None


def _odom_pose(message) -> tuple[float, float, float] | None:
    try:
        position = message.pose.pose.position
        return float(position.x), float(position.y), float(position.z)
    except (AttributeError, TypeError, ValueError):
        return None


def _tf_pairs(message) -> list[tuple[str, str]]:
    pairs = []
    for transform in getattr(message, "transforms", []):
        parent = str(getattr(transform.header, "frame_id", "")).lstrip("/")
        child = str(getattr(transform, "child_frame_id", "")).lstrip("/")
        if parent and child:
            pairs.append((parent, child))
    return pairs


def inspect_mcap(path: Path, profile: dict, baseline_summary: dict | None = None) -> tuple[list[dict], dict]:
    """Read a ROS2 MCAP once and emit bounded, JSON-safe health evidence."""
    try:
        import rosbag2_py
    except ImportError as exc:
        raise RuntimeError("rosbag2_py is required by the validation runner") from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_classes = {topic: _get_message_class(type_name) for topic, type_name in topic_types.items()}
    counts: dict[str, int] = defaultdict(int)
    first_stamp: dict[str, int] = {}
    last_stamp: dict[str, int] = {}
    regressions: dict[str, int] = defaultdict(int)
    cmd_values: dict[str, list[tuple[int, float]]] = defaultdict(list)
    odom_last: dict[str, tuple[float, float, float]] = {}
    odom_max_jump: dict[str, float] = defaultdict(float)
    tf_parents: dict[str, set[str]] = defaultdict(set)
    global_start = None
    global_end = None

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        stamp = int(stamp)
        counts[topic] += 1
        first_stamp.setdefault(topic, stamp)
        previous_stamp = last_stamp.get(topic)
        if previous_stamp is not None and stamp < previous_stamp:
            regressions[topic] += 1
        last_stamp[topic] = stamp
        global_start = stamp if global_start is None else min(global_start, stamp)
        global_end = stamp if global_end is None else max(global_end, stamp)
        if topic not in {"/cmd_vel", "/cmd_vel_raw", "/tf", "/tf_static", "/odom/nav2", "/odom/localization_odom"}:
            continue
        message = _deserialize(data, message_classes.get(topic))
        if topic in {"/cmd_vel", "/cmd_vel_raw"}:
            value = _angular_z(message)
            if value is not None:
                cmd_values[topic].append((stamp, value))
        elif topic in {"/tf", "/tf_static"}:
            for parent, child in _tf_pairs(message):
                tf_parents[child].add(parent)
        else:
            pose = _odom_pose(message)
            if pose is not None and topic in odom_last:
                previous = odom_last[topic]
                jump = math.sqrt(sum((a - b) ** 2 for a, b in zip(pose, previous)))
                odom_max_jump[topic] = max(odom_max_jump[topic], jump)
            if pose is not None:
                odom_last[topic] = pose

    thresholds = profile.get("thresholds") or {}
    required_topics = profile.get("required_topics") or []
    checks: list[dict] = []
    missing = [topic for topic in required_topics if counts.get(topic, 0) == 0]
    checks.append(
        _check(
            "input.required_topics",
            "必需话题完整性",
            "input",
            "FAIL" if missing else "PASS",
            hard_failure=bool(missing),
            actual_value={"missing": missing, "present": len(counts)},
            expected_value={"required": required_topics},
            topics=missing,
            message=f"缺少必需话题：{', '.join(missing)}" if missing else "全部必需话题均有消息",
        )
    )
    regression_total = sum(regressions.values())
    max_regressions = int(thresholds.get("max_timestamp_regressions", 0))
    checks.append(
        _check(
            "input.timestamp_monotonic",
            "话题时间戳单调性",
            "input",
            "FAIL" if regression_total > max_regressions else "PASS",
            hard_failure=regression_total > max_regressions,
            metric_name="timestamp_regressions",
            actual_value=regression_total,
            expected_value={"max": max_regressions, "by_topic": dict(regressions)},
        )
    )
    multi_parent = {child: sorted(parents) for child, parents in tf_parents.items() if len(parents) > 1}
    checks.append(
        _check(
            "tf.single_parent",
            "TF 子坐标系单父节点",
            "tf",
            "FAIL" if multi_parent else "PASS",
            hard_failure=bool(multi_parent),
            actual_value=multi_parent,
            expected_value="每个 child_frame 只有一个 parent_frame",
            topics=["/tf", "/tf_static"],
        )
    )

    max_pose_jump = float(thresholds.get("max_pose_jump_m", 2.0))
    for topic in ("/odom/localization_odom", "/odom/nav2"):
        actual = round(float(odom_max_jump.get(topic, 0.0)), 6)
        status = "NOT_EVALUATED" if counts.get(topic, 0) < 2 else "FAIL" if actual > max_pose_jump else "PASS"
        checks.append(
            _check(
                f"localization.pose_jump.{topic.rsplit('/', 1)[-1]}",
                f"{topic} 相邻位姿跳变",
                "localization",
                status,
                metric_name=f"{topic}.max_pose_jump_m",
                actual_value=actual,
                expected_value={"max": max_pose_jump},
                topics=[topic],
            )
        )

    flip_metrics = {}
    max_flips = float(thresholds.get("max_cmd_sign_flips_per_second", 2.0))
    for topic, samples in cmd_values.items():
        flips = 0
        last_sign = 0
        for _, value in samples:
            sign = 1 if value > 0.02 else -1 if value < -0.02 else 0
            if sign and last_sign and sign != last_sign:
                flips += 1
            if sign:
                last_sign = sign
        duration = max(1e-9, (samples[-1][0] - samples[0][0]) / 1e9) if len(samples) > 1 else 0.0
        rate = flips / duration if duration else 0.0
        flip_metrics[topic] = round(rate, 6)
    raw_flip_rate = flip_metrics.get("/cmd_vel_raw")
    checks.append(
        _check(
            "control.angular_sign_flips",
            "角速度指令正负翻转",
            "control",
            "NOT_EVALUATED" if raw_flip_rate is None else "FAIL" if raw_flip_rate > max_flips else "PASS",
            metric_name="cmd_vel_raw.angular_sign_flips_per_second",
            actual_value=raw_flip_rate,
            expected_value={"max": max_flips},
            topics=["/cmd_vel_raw"],
        )
    )

    duration_seconds = ((global_end or 0) - (global_start or 0)) / 1e9 if global_start is not None else 0.0
    metrics = {
        "duration_seconds": round(duration_seconds, 6),
        "message_count": sum(counts.values()),
        "topic_count": len(counts),
        "topic_counts": dict(sorted(counts.items())),
        "timestamp_regressions": regression_total,
        "max_pose_jump_m": dict(odom_max_jump),
        "cmd_sign_flips_per_second": flip_metrics,
        "start_time_ns": global_start,
        "end_time_ns": global_end,
    }
    checks.extend(_baseline_checks(metrics, baseline_summary, thresholds))
    summary = {
        "metrics": metrics,
        "failed_rules": [item["rule_id"] for item in checks if item["status"] == "FAIL"],
        "warning_rules": [item["rule_id"] for item in checks if item["status"] == "WARN"],
        "not_evaluated_rules": [item["rule_id"] for item in checks if item["status"] == "NOT_EVALUATED"],
    }
    return checks, summary


def _baseline_checks(metrics: dict, baseline_summary: dict | None, thresholds: dict) -> list[dict]:
    if not baseline_summary:
        return [
            _check(
                "baseline.available",
                "黄金基线可用性",
                "regression",
                "NOT_EVALUATED",
                message="当前地图、路线和 Profile 尚无黄金基线",
            )
        ]
    baseline_metrics = baseline_summary.get("metrics") or {}
    warn_ratio = float(thresholds.get("baseline_warn_ratio", 0.10))
    fail_ratio = float(thresholds.get("baseline_fail_ratio", 0.20))
    results = []
    for name in ("message_count", "topic_count", "duration_seconds"):
        current = float(metrics.get(name) or 0)
        baseline = float(baseline_metrics.get(name) or 0)
        if baseline <= 0:
            status = "NOT_EVALUATED"
            ratio = None
        else:
            ratio = abs(current - baseline) / baseline
            status = "FAIL" if ratio > fail_ratio else "WARN" if ratio > warn_ratio else "PASS"
        results.append(
            _check(
                f"baseline.{name}",
                f"{name} 与黄金基线偏差",
                "regression",
                status,
                metric_name=name,
                actual_value={"value": current, "relative_delta": ratio},
                expected_value={"baseline": baseline, "warn_ratio": warn_ratio, "fail_ratio": fail_ratio},
            )
        )
    return results
