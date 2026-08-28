"""Summarize post-mapping global trajectory optimization results."""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path
from typing import Any


def build_optimization_summary(
    map_dir: str | Path,
    *,
    mapping_type: str = "indoor",
    trigger_source: str = "automatic_save",
    timing: dict[str, Any] | None = None,
    fallback_error: str = "",
) -> dict[str, Any]:
    root = Path(map_dir)
    raw = _load_trajectory(root / "trajectory_raw.csv")
    optimized = _load_trajectory(root / "trajectory_optimized.csv")
    covariance = _load_json(root / "trajectory_covariance.json")
    loop_rows = _load_csv(root / "loop_closures.csv")
    candidate_rows = _load_csv(root / "scan_context" / "loop_candidates.csv")
    legacy_ndt_count = int(covariance.get("ndt_factor_count") or 0)
    lio_between_count = int(
        covariance.get("lio_between_factor_count")
        if covariance.get("lio_between_factor_count") is not None
        else legacy_ndt_count
    )
    ndt_registration_count = int(covariance.get("ndt_registration_factor_count") or 0)
    imu_factor_count = int(covariance.get("imu_factor_count") or 0)
    imu_kinematic_rejected_count = int(
        covariance.get("imu_kinematic_rejected_factor_count") or 0
    )
    rtk_position_factor_count = int(covariance.get("rtk_position_factor_count") or 0)
    rtk_heading_factor_count = int(covariance.get("rtk_heading_factor_count") or 0)
    loop_factor_count = int(covariance.get("loop_closure_factor_count") or 0)
    external_anchor_factor_count = (
        rtk_position_factor_count + rtk_heading_factor_count + loop_factor_count
    )
    constraint_sources = ["lidar"]
    if loop_factor_count:
        constraint_sources.append("loop_closure")
    if imu_factor_count:
        constraint_sources.append("imu")
    if rtk_position_factor_count:
        constraint_sources.append("gps")
    if rtk_heading_factor_count:
        constraint_sources.append("heading")

    corrections = []
    for index in sorted(set(raw).intersection(optimized)):
        before = raw[index]
        after = optimized[index]
        dx = after["x"] - before["x"]
        dy = after["y"] - before["y"]
        dz = after["z"] - before["z"]
        position_m = math.hypot(dx, dy)
        yaw_delta = _normalize_angle(after["yaw"] - before["yaw"])
        corrections.append({
            "index": index,
            "stamp": after.get("stamp") if after.get("stamp") is not None else before.get("stamp"),
            "raw": _rounded_pose(before),
            "optimized": _rounded_pose(after),
            "delta": {
                "x": round(dx, 5),
                "y": round(dy, 5),
                "z": round(dz, 5),
                "position_m": round(position_m, 5),
                "yaw_rad": round(yaw_delta, 6),
                "yaw_deg": round(math.degrees(yaw_delta), 4),
            },
            "significant": position_m >= 0.30 or abs(math.degrees(yaw_delta)) >= 3.0,
            "constraint_source": constraint_sources,
        })

    position_values = [item["delta"]["position_m"] for item in corrections]
    yaw_values = [abs(item["delta"]["yaw_deg"]) for item in corrections]
    z_values = [abs(item["delta"]["z"]) for item in corrections]
    adjacent_corrections = []
    for previous, current in zip(corrections, corrections[1:]):
        xy_step = math.hypot(
            current["delta"]["x"] - previous["delta"]["x"],
            current["delta"]["y"] - previous["delta"]["y"],
        )
        previous_stamp = _finite_number(previous.get("stamp"))
        current_stamp = _finite_number(current.get("stamp"))
        delta_t = (
            current_stamp - previous_stamp
            if previous_stamp is not None and current_stamp is not None
            else None
        )
        if delta_t is not None and delta_t <= 0.0:
            delta_t = None
        raw_xy_distance = math.hypot(
            current["raw"]["x"] - previous["raw"]["x"],
            current["raw"]["y"] - previous["raw"]["y"],
        )
        raw_yaw_step_deg = abs(math.degrees(_normalize_angle(
            current["raw"]["yaw"] - previous["raw"]["yaw"]
        )))
        adjacent_corrections.append({
            "from_index": previous["index"],
            "to_index": current["index"],
            "from_stamp": previous_stamp,
            "to_stamp": current_stamp,
            "delta_t_s": None if delta_t is None else round(delta_t, 6),
            "xy_correction_step_m": round(xy_step, 5),
            "xy_correction_rate_mps": (
                None if delta_t is None else round(xy_step / delta_t, 5)
            ),
            "raw_xy_distance_m": round(raw_xy_distance, 5),
            "raw_yaw_step_deg": round(raw_yaw_step_deg, 4),
        })
    adjacent_position_values = [
        item["xy_correction_step_m"] for item in adjacent_corrections
    ]
    adjacent_rate_values = [
        item["xy_correction_rate_mps"]
        for item in adjacent_corrections
        if item["xy_correction_rate_mps"] is not None
    ]
    adjacent_yaw_values = [
        abs(math.degrees(_normalize_angle(
            math.radians(current["delta"]["yaw_deg"] - previous["delta"]["yaw_deg"])
        )))
        for previous, current in zip(corrections, corrections[1:])
    ]
    accepted_count = len(loop_rows)
    candidate_count = len(candidate_rows)
    rejected_count = sum(not _as_bool(row.get("accepted")) for row in candidate_rows)
    has_optimized_output = bool(covariance) and bool(corrections)
    candidate_applied = has_optimized_output and int(covariance.get("factor_count") or 0) > 0
    if fallback_error:
        stage = "fallback"
    elif accepted_count == 0:
        stage = "no_valid_loop"
    elif candidate_applied:
        stage = "completed"
    else:
        stage = "fallback"
        fallback_error = "accepted loops exist but no optimized factor-graph output was produced"
    indoor_gps_factor_violation = mapping_type == "indoor" and (
        rtk_position_factor_count > 0 or rtk_heading_factor_count > 0
    )
    if indoor_gps_factor_violation:
        stage = "fallback"
        fallback_error = "indoor useGPS=false gate violated: RTK factors were present"
        candidate_applied = False

    error_before = _finite_number(covariance.get("error_before"))
    error_after = _finite_number(covariance.get("error_after"))
    error_reduction_percent = None
    if error_before is not None and error_after is not None and error_before > 0:
        error_reduction_percent = round((error_before - error_after) / error_before * 100.0, 2)

    optimization_mode = (
        "fast_lio2_slam_anchored"
        if external_anchor_factor_count > 0
        else "fast_lio2_inertial_smoothing"
        if imu_factor_count > 0
        else "fast_lio2_pose_chain"
    )
    expected_between_factors = max(0, len(corrections) - 1)
    lio_coverage = (
        min(1.0, lio_between_count / expected_between_factors)
        if expected_between_factors else 1.0
    )
    imu_coverage = (
        min(1.0, imu_factor_count / expected_between_factors)
        if expected_between_factors else 1.0
    )
    inertial_limits = {
        "lio_between_coverage_min": 0.95,
        "imu_coverage_min": 0.80,
        "max_xy_correction_m": 0.50,
        "max_abs_z_correction_m": 0.60,
        "max_yaw_correction_deg": 3.0,
        "max_adjacent_xy_step_m": 0.15,
        "max_adjacent_xy_correction_rate_mps": 0.35,
        "max_adjacent_yaw_step_deg": 1.0,
    }
    inertial_measurements = {
        "lio_between_coverage": round(lio_coverage, 4),
        "imu_coverage": round(imu_coverage, 4),
        "max_xy_correction_m": round(max(position_values, default=0.0), 5),
        "max_abs_z_correction_m": round(max(z_values, default=0.0), 5),
        "max_yaw_correction_deg": round(max(yaw_values, default=0.0), 5),
        "max_adjacent_xy_step_m": round(max(adjacent_position_values, default=0.0), 5),
        "max_adjacent_xy_correction_rate_mps": round(
            max(adjacent_rate_values, default=0.0), 5
        ),
        "max_adjacent_yaw_step_deg": round(max(adjacent_yaw_values, default=0.0), 5),
        "graph_error_non_increasing": (
            None if error_before is None or error_after is None
            else error_after <= error_before + max(1e-6, abs(error_before) * 1e-6)
        ),
    }
    # Every optimized trajectory must pass the same displacement and continuity
    # checks.  A loop closure or RTK factor is evidence, not a waiver: a false
    # external anchor can produce an internally tiny graph error while warping
    # the navigation map by metres.
    trajectory_guard_checked = bool(candidate_applied)
    inertial_guard_reasons = []
    if trajectory_guard_checked:
        for measurement, limit in (
            ("lio_between_coverage", "lio_between_coverage_min"),
            ("imu_coverage", "imu_coverage_min"),
        ):
            if inertial_measurements[measurement] < inertial_limits[limit]:
                inertial_guard_reasons.append(f"{measurement}_below_limit")
        for measurement, limit in (
            ("max_xy_correction_m", "max_xy_correction_m"),
            ("max_abs_z_correction_m", "max_abs_z_correction_m"),
            ("max_yaw_correction_deg", "max_yaw_correction_deg"),
            ("max_adjacent_xy_step_m", "max_adjacent_xy_step_m"),
            (
                "max_adjacent_xy_correction_rate_mps",
                "max_adjacent_xy_correction_rate_mps",
            ),
            ("max_adjacent_yaw_step_deg", "max_adjacent_yaw_step_deg"),
        ):
            if inertial_measurements[measurement] > inertial_limits[limit]:
                inertial_guard_reasons.append(f"{measurement}_above_limit")
        if inertial_measurements["graph_error_non_increasing"] is False:
            inertial_guard_reasons.append("graph_error_increased")
    inertial_smoothing_guard = {
        "checked": trajectory_guard_checked,
        "passed": not inertial_guard_reasons,
        "external_anchor_factor_count": external_anchor_factor_count,
        "limits": inertial_limits,
        "measurements": inertial_measurements,
        "worst_adjacent_xy_correction": max(
            adjacent_corrections,
            key=lambda item: item["xy_correction_step_m"],
            default=None,
        ),
        "reasons": inertial_guard_reasons,
    }

    now = time.time()
    start_delta = corrections[0]["delta"] if corrections else {}
    enu_guard = {
        "checked": mapping_type == "outdoor",
        "start_translation_limit_m": 0.20,
        "global_yaw_limit_deg": 2.0,
        "start_translation_m": float(start_delta.get("position_m") or 0.0),
        "global_yaw_deg": abs(float(start_delta.get("yaw_deg") or 0.0)),
    }
    enu_guard["passed"] = not enu_guard["checked"] or (
        enu_guard["start_translation_m"] <= enu_guard["start_translation_limit_m"]
        and enu_guard["global_yaw_deg"] <= enu_guard["global_yaw_limit_deg"]
    )
    optimization_output_accepted = bool(
        candidate_applied
        and inertial_smoothing_guard["passed"]
        and enu_guard["passed"]
        and stage not in {"fallback", "failed"}
    )
    if candidate_applied and not optimization_output_accepted and stage not in {"fallback", "failed"}:
        stage = "quality_rejected"
        quality_reasons = list(inertial_smoothing_guard["reasons"])
        if not enu_guard["passed"]:
            quality_reasons.append("enu_guard_failed")
        fallback_error = "optimized trajectory rejected: " + ", ".join(quality_reasons)
    applied = optimization_output_accepted
    summary = {
        "schema": "roamerx.optimization-summary.v1",
        "stage": stage,
        # quality_rejected means the map save itself succeeded and the caller
        # must select the preserved raw products instead of the candidate.
        "success": stage in {"completed", "no_valid_loop", "quality_rejected"},
        "applied": applied,
        "candidate_applied": candidate_applied,
        "trigger_source": trigger_source,
        "completed_at_unix": round(now, 3),
        "mapping_type": mapping_type,
        "use_gps": mapping_type == "outdoor",
        "optimization_mode": optimization_mode,
        "external_anchor_factor_count": external_anchor_factor_count,
        "trajectory_source": "optimized" if applied else "raw",
        "keyframe_count": len(corrections) or len(raw),
        "candidate_count": candidate_count,
        "accepted_loop_count": accepted_count,
        "rejected_loop_count": rejected_count,
        "rejection_reasons": _rejection_reason_counts(candidate_rows),
        "loop_endpoints": [
            {
                "from": _integer(row.get("from")),
                "to": _integer(row.get("to")),
                "score": _finite_number(row.get("score")),
            }
            for row in loop_rows
        ],
        "factors": {
            "total": int(covariance.get("factor_count") or 0),
            "lio_between": lio_between_count,
            "ndt_registration": ndt_registration_count,
            # Compatibility field for the current MapsPage frontend. It is a
            # FAST-LIO2-between count in old artifacts, not an NDT result.
            "ndt": legacy_ndt_count,
            "ndt_is_legacy_alias": bool(
                covariance.get("ndt_factor_count_legacy_alias", True)
            ),
            "imu": imu_factor_count,
            "imu_bias": int(covariance.get("imu_bias_factor_count") or 0),
            "imu_velocity_prior": int(covariance.get("imu_velocity_prior_factor_count") or 0),
            "imu_kinematic_rejected": imu_kinematic_rejected_count,
            "max_imu_kinematic_residual_mps": _finite_number(
                covariance.get("max_imu_kinematic_residual_mps")
            ),
            "rtk_position": rtk_position_factor_count,
            "rtk_heading": rtk_heading_factor_count,
            "loop_closure": loop_factor_count,
        },
        "graph_error": {
            "before": error_before,
            "after": error_after,
            "reduction_percent": error_reduction_percent,
        },
        "imu_state": {
            "final_velocity": covariance.get("final_velocity") or [0.0, 0.0, 0.0],
            "final_accel_bias": covariance.get("final_accel_bias") or [0.0, 0.0, 0.0],
            "final_gyro_bias": covariance.get("final_gyro_bias") or [0.0, 0.0, 0.0],
        },
        "correction": {
            "position_m": _statistics(position_values, corrections, "position_m"),
            "z_m": _statistics(z_values, corrections, "z_m"),
            "yaw_deg": _statistics(yaw_values, corrections, "yaw_deg"),
            "start": corrections[0]["delta"] if corrections else {},
            "end": corrections[-1]["delta"] if corrections else {},
        },
        "timing": dict(timing or {}),
        "enu_guard": enu_guard,
        "inertial_smoothing_guard": inertial_smoothing_guard,
        "gps_factor_gate_valid": not indoor_gps_factor_violation,
        "auto_activation_allowed": bool(applied),
        "fallback_error": fallback_error,
        "corrections": corrections,
        "adjacent_corrections": adjacent_corrections,
    }
    (root / "optimization_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summary


def summary_without_corrections(summary: dict[str, Any]) -> dict[str, Any]:
    compact = dict(summary)
    compact.pop("corrections", None)
    compact.pop("adjacent_corrections", None)
    return compact


def _load_trajectory(path: Path) -> dict[int, dict[str, float | None]]:
    result: dict[int, dict[str, float | None]] = {}
    for row in _load_csv(path):
        try:
            index = int(row.get("index") or len(result))
            qx = float(row.get("world_qx") or row.get("qx") or 0.0)
            qy = float(row.get("world_qy") or row.get("qy") or 0.0)
            qz = float(row.get("world_qz") or row.get("qz") or 0.0)
            qw = float(row.get("world_qw") or row.get("qw") or 1.0)
            result[index] = {
                "x": float(row.get("world_x") or row.get("x") or 0.0),
                "y": float(row.get("world_y") or row.get("y") or 0.0),
                "z": float(row.get("world_z") or row.get("z") or 0.0),
                "yaw": _quaternion_yaw(qx, qy, qz, qw),
                "qx": qx,
                "qy": qy,
                "qz": qz,
                "qw": qw,
                "stamp": _finite_number(row.get("timestamp") or row.get("stamp")),
            }
        except (TypeError, ValueError):
            continue
    return result


def _rounded_pose(pose: dict[str, Any]) -> dict[str, Any]:
    return {
        key: round(float(pose[key]), 7 if key.startswith("q") else 5)
        for key in ("x", "y", "z", "yaw", "qx", "qy", "qz", "qw")
    }


def _quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _statistics(values: list[float], corrections: list[dict[str, Any]], field: str) -> dict[str, Any]:
    if not values:
        return {"mean": 0.0, "rms": 0.0, "p95": 0.0, "max": 0.0, "max_keyframe": None}
    ordered = sorted(values)
    p95_index = max(0, math.ceil(len(ordered) * 0.95) - 1)
    max_index = max(range(len(values)), key=values.__getitem__)
    return {
        "mean": round(sum(values) / len(values), 5),
        "rms": round(math.sqrt(sum(value * value for value in values) / len(values)), 5),
        "p95": round(ordered[p95_index], 5),
        "max": round(values[max_index], 5),
        "max_keyframe": corrections[max_index]["index"],
        "unit": "deg" if field == "yaw_deg" else "m",
    }


def _rejection_reason_counts(rows: list[dict[str, str]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in rows:
        if _as_bool(row.get("accepted")):
            continue
        reason = str(row.get("rejection_reason") or "rejected_other")
        counts[reason] = counts.get(reason, 0) + 1
    return counts


def _load_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    try:
        with path.open(encoding="utf-8", newline="") as stream:
            return list(csv.DictReader(stream))
    except OSError:
        return []


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _finite_number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _integer(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _as_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}
