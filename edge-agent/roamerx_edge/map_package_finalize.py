"""Complete a mapping session directory after SLAM export."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

import yaml

from .map_coordinate import SCHEMA_VERSION, MapConstraintError, new_map_constraints
from .map_loop_closure import finalize_loop_closure
from .recording_manifest import write_recording_manifest

LOGGER = logging.getLogger(__name__)


def finalize_map_package(
    map_dir: str | Path,
    *,
    requested_scene_scope: str = "indoor",
    bag_dir: str | Path | None = None,
    raw_recording: str = "",
) -> dict[str, Any]:
    root = Path(map_dir)
    root.mkdir(parents=True, exist_ok=True)
    gnss_origin = _load_yaml(root / "gnss_origin.yaml")
    try:
        constraints = new_map_constraints(gnss_origin=gnss_origin, requested_scene_scope=requested_scene_scope)
    except MapConstraintError:
        constraints = new_map_constraints(gnss_origin=gnss_origin, requested_scene_scope="indoor")
    recording = {}
    if bag_dir:
        try:
            recording = write_recording_manifest(bag_dir, root / "recording_manifest.yaml")
        except (OSError, FileNotFoundError, ValueError) as exc:
            LOGGER.warning("recording manifest not written for %s: %s", bag_dir, exc)

    loop_result = {"scan_context_count": 0, "loop_closure_count": 0, "trajectory_source": "raw"}
    try:
        loop_result = finalize_loop_closure(root)
    except Exception:
        LOGGER.exception("loop closure failed for %s; keeping raw map", root)

    keyframe_count = _count_keyframe_rows(root / "keyframes" / "keyframes.csv")
    preintegration_count = len(list((root / "imu_preintegration").glob("preint_*.json"))) if (root / "imu_preintegration").is_dir() else 0
    scan_context_count = int(loop_result.get("scan_context_count") or 0)
    completeness = "complete"
    if keyframe_count == 0:
        completeness = "legacy_incomplete"
    elif preintegration_count and preintegration_count != keyframe_count:
        completeness = "incomplete"
    elif scan_context_count and scan_context_count != keyframe_count:
        completeness = "incomplete"

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "coordinate_mode": constraints["coordinate_mode"],
        "scene_scope": constraints["scene_scope"],
        "localization_mode": constraints["localization_mode"],
        "origin_status": constraints["origin_status"],
        "rtk_origin_required": constraints["rtk_origin_required"],
        "completeness": completeness,
        "frame_id": "map",
        "quaternion_order": "xyzw",
        "keyframe_count": keyframe_count,
        "point_cloud_count": len(list((root / "keyframes").glob("scan_*.pcd"))) if (root / "keyframes").is_dir() else 0,
        "preintegration_count": preintegration_count,
        "scan_context_count": scan_context_count,
        "loop_closure_count": int(loop_result.get("loop_closure_count") or 0),
        "trajectory_source": loop_result.get("trajectory_source") or "raw",
        "loop_status": loop_result.get("loop_status") or "skipped",
        "raw_recording": raw_recording or str(bag_dir or recording.get("bag_dir") or ""),
        "recording_topics_complete": not bool(recording.get("missing_required_topics")),
    }
    (root / "map_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return manifest


def load_map_manifest(map_dir: str | Path) -> dict[str, Any]:
    path = Path(map_dir) / "map_manifest.json"
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _load_yaml(path: Path) -> dict:
    if not path.is_file():
        return {}
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _count_keyframe_rows(path: Path) -> int:
    if not path.is_file():
        return 0
    count = 0
    with path.open(encoding="utf-8") as stream:
        next(stream, None)
        for line in stream:
            if line.strip():
                count += 1
    return count
