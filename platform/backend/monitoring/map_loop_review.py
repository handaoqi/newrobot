"""Read and audit offline Scan-Context loop candidates from a map package."""

from __future__ import annotations

import csv
import io
import json
import math
import zipfile
from typing import Any


DEFAULT_THRESHOLDS = {
    "max_rank": 1,
    "descriptor_distance_max": 0.30,
    "geometric_translation_max_m": 2.50,
    "yaw_consistency_max_deg": 34.38,
    "slam_distance_min_m": 2.0,
    "slam_distance_max_m": 20.0,
    "max_selected_loops": 30,
    "max_pose_correction_m": 5.0,
    "max_yaw_correction_deg": 20.0,
}


class LoopReviewError(ValueError):
    pass


def _number(value: Any, default: float = 0.0) -> float:
    try:
        parsed = float(value)
        return parsed if math.isfinite(parsed) else default
    except (TypeError, ValueError):
        return default


def _boolean(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def normalize_thresholds(raw: Any) -> dict[str, float | int]:
    raw = raw if isinstance(raw, dict) else {}
    result = dict(DEFAULT_THRESHOLDS)
    integer_keys = {"max_rank", "max_selected_loops"}
    for key, default in DEFAULT_THRESHOLDS.items():
        if key not in raw:
            continue
        value = _number(raw.get(key), float(default))
        result[key] = max(1, int(value)) if key in integer_keys else max(0.0, value)
    if result["slam_distance_max_m"] <= result["slam_distance_min_m"]:
        raise LoopReviewError("SLAM 最大间距必须大于最小间距")
    return result


def _read_csv(archive: zipfile.ZipFile, name: str) -> list[dict[str, str]]:
    try:
        content = archive.read(name).decode("utf-8-sig")
    except KeyError:
        return []
    return list(csv.DictReader(io.StringIO(content)))


def _keyframe_poses(archive: zipfile.ZipFile) -> dict[int, tuple[float, float, float]]:
    rows = _read_csv(archive, "keyframes/keyframes.csv")
    result = {}
    for row in rows:
        try:
            index = int(row.get("index") or 0)
            result[index] = (
                _number(row.get("lidar_x") or row.get("x")),
                _number(row.get("lidar_y") or row.get("y")),
                _number(row.get("yaw")),
            )
        except (TypeError, ValueError):
            continue
    return result


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def audit_map_package(package_path: str, raw_thresholds: Any = None) -> dict[str, Any]:
    thresholds = normalize_thresholds(raw_thresholds)
    try:
        with zipfile.ZipFile(package_path) as archive:
            candidates = _read_csv(archive, "scan_context/loop_candidates.csv")
            poses = _keyframe_poses(archive)
            try:
                index = json.loads(archive.read("scan_context/index.json"))
            except (KeyError, json.JSONDecodeError, UnicodeDecodeError):
                index = {}
            try:
                validation = json.loads(archive.read("localization_validation.json"))
            except (KeyError, json.JSONDecodeError, UnicodeDecodeError):
                validation = {}
    except (OSError, zipfile.BadZipFile) as exc:
        raise LoopReviewError(f"无法读取地图 ZIP 包: {exc}") from exc

    audited = []
    for row in candidates:
        query = int(_number(row.get("query_index"), -1))
        match = int(_number(row.get("match_index"), -1))
        query_pose, match_pose = poses.get(query), poses.get(match)
        slam_distance = _number(row.get("slam_xy_m"), -1.0)
        slam_dyaw = _number(row.get("slam_dyaw_rad"), float("nan"))
        if query_pose and match_pose:
            slam_distance = math.hypot(query_pose[0] - match_pose[0], query_pose[1] - match_pose[1])
            slam_dyaw = _wrap_angle(match_pose[2] - query_pose[2])
        geometric_yaw = _number(row.get("dyaw") or row.get("estimated_yaw_rad"))
        yaw_consistency_deg = abs(math.degrees(_wrap_angle(geometric_yaw - slam_dyaw))) if math.isfinite(slam_dyaw) else 999.0
        geometric_translation = math.hypot(_number(row.get("dx")), _number(row.get("dy")))
        checks = {
            "rank": int(_number(row.get("rank"), 999)) <= thresholds["max_rank"],
            "descriptor": _number(row.get("distance"), 999.0) <= thresholds["descriptor_distance_max"],
            "geometric_verified": _boolean(row.get("geometric_verified")),
            "geometric_translation": geometric_translation <= thresholds["geometric_translation_max_m"],
            "slam_distance": thresholds["slam_distance_min_m"] <= slam_distance <= thresholds["slam_distance_max_m"],
            "yaw_consistency": yaw_consistency_deg <= thresholds["yaw_consistency_max_deg"],
            "unique_pair": str(row.get("rejection_reason") or "") != "duplicate_pair",
        }
        failed = [name for name, passed in checks.items() if not passed]
        audited.append({
            "candidate_id": f"{query}:{match}",
            "query_index": query,
            "match_index": match,
            "rank": int(_number(row.get("rank"), 0)),
            "descriptor_distance": round(_number(row.get("distance")), 6),
            "geometric_verified": checks["geometric_verified"],
            "geometric_translation_m": round(geometric_translation, 4),
            "geometric_dx_m": round(_number(row.get("dx")), 4),
            "geometric_dy_m": round(_number(row.get("dy")), 4),
            "geometric_dyaw_rad": round(geometric_yaw, 6),
            "slam_distance_m": round(slam_distance, 4),
            "slam_dyaw_rad": round(slam_dyaw, 6) if math.isfinite(slam_dyaw) else None,
            "yaw_consistency_deg": round(yaw_consistency_deg, 3),
            "detector_accepted": _boolean(row.get("accepted")),
            "detector_rejection_reason": row.get("rejection_reason") or "",
            "checks": checks,
            "eligible": not failed,
            "failed_checks": failed,
        })
    eligible = [item for item in audited if item["eligible"]]
    return {
        "schema": "roamerx.map-loop-review.v1",
        "status": "review_ready" if audited else "no_candidates",
        "thresholds": thresholds,
        "keyframe_count": int(index.get("keyframe_count") or len(poses)),
        "candidate_count": len(audited),
        "eligible_count": len(eligible),
        "detector_accepted_count": sum(item["detector_accepted"] for item in audited),
        "candidates": audited,
        "localization_validation": validation,
        "source_index": index,
    }
