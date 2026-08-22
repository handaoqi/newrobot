"""Map coordinate mode, scene scope, and localization capability checks.

Waypoint localization keeps the existing values ``ndt`` and ``rtk``.
Map-level capability uses ``ndt`` or ``rtk_ndt``. JSON quaternions use ROS
``qx, qy, qz, qw`` and field names ending in ``_xyzw``.
"""

from __future__ import annotations

from typing import Any

import json

SCHEMA_VERSION = 2
COORDINATE_MODES = ("rtk_fixed", "local_only")
SCENE_SCOPES = ("indoor", "transition", "outdoor")
MAP_LOCALIZATION_MODES = ("ndt", "rtk_ndt")
WAYPOINT_LOCALIZATION_MODES = ("ndt", "rtk")
ORIGIN_STATUSES = ("fixed", "local_only", "legacy_incomplete")

MAP_RTK_ORIGIN_REQUIRED = "MAP_RTK_ORIGIN_REQUIRED"
MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN = "MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN"
MAP_LOCAL_ONLY_TRANSITION_FORBIDDEN = "MAP_LOCAL_ONLY_TRANSITION_FORBIDDEN"
MAP_LOCAL_ONLY_NDT_ONLY = "MAP_LOCAL_ONLY_NDT_ONLY"
MAP_CONSTRAINT_INVALID = "MAP_CONSTRAINT_INVALID"


class MapConstraintError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def normalize_text(value: Any, default: str = "") -> str:
    text = str(value or "").strip().lower()
    return text or default


def waypoint_localization_mode(value: Any) -> str:
    mode = normalize_text(value, "ndt")
    if mode in {"rtk", "rtk_ndt"}:
        return "rtk"
    return "ndt"


def map_localization_mode(value: Any, *, coordinate_mode: str) -> str:
    mode = normalize_text(value)
    if coordinate_mode == "local_only":
        return "ndt"
    if mode in {"rtk", "rtk_ndt"}:
        return "rtk_ndt"
    if mode == "ndt":
        return "ndt"
    return "rtk_ndt" if coordinate_mode == "rtk_fixed" else "ndt"


def gnss_origin_is_fixed(gnss_origin: dict | None) -> bool:
    if not isinstance(gnss_origin, dict) or not gnss_origin:
        return False
    locked = gnss_origin.get("alignment_locked")
    if locked in (True, 1, "1", "true", "True"):
        return True
    return str(gnss_origin.get("rtk_enabled") or "").lower() == "true"


def constraints_from_manifest(
    manifest: dict | None,
    *,
    gnss_origin: dict | None = None,
    requested_scene_scope: str = "",
) -> dict[str, Any]:
    data = dict(manifest or {})
    completeness = normalize_text(data.get("completeness") or data.get("map_completeness"))
    explicit_mode = normalize_text(data.get("coordinate_mode"))
    if explicit_mode in COORDINATE_MODES:
        coordinate_mode = explicit_mode
        origin_status = normalize_text(data.get("origin_status"), "fixed" if coordinate_mode == "rtk_fixed" else "local_only")
        if origin_status not in ORIGIN_STATUSES:
            origin_status = "fixed" if coordinate_mode == "rtk_fixed" else "local_only"
    elif completeness == "legacy_incomplete" or not data:
        if gnss_origin_is_fixed(gnss_origin):
            coordinate_mode = "rtk_fixed"
            origin_status = "legacy_incomplete"
        else:
            coordinate_mode = ""
            origin_status = "legacy_incomplete"
    elif gnss_origin_is_fixed(gnss_origin):
        coordinate_mode = "rtk_fixed"
        origin_status = normalize_text(data.get("origin_status"), "fixed")
    else:
        coordinate_mode = "local_only"
        origin_status = "local_only"

    scene_scope = normalize_text(data.get("scene_scope") or requested_scene_scope)
    if coordinate_mode == "local_only":
        scene_scope = "indoor"
    elif scene_scope not in SCENE_SCOPES:
        scene_scope = "indoor" if scene_scope else ""

    localization_mode = map_localization_mode(
        data.get("localization_mode"),
        coordinate_mode=coordinate_mode or "local_only",
    )
    if coordinate_mode == "local_only":
        localization_mode = "ndt"
    elif not coordinate_mode:
        localization_mode = normalize_text(data.get("localization_mode")) or ""

    return {
        "schema_version": int(data.get("schema_version") or SCHEMA_VERSION),
        "coordinate_mode": coordinate_mode,
        "scene_scope": scene_scope,
        "localization_mode": localization_mode,
        "origin_status": origin_status or "legacy_incomplete",
        "rtk_origin_required": coordinate_mode == "rtk_fixed",
        "completeness": completeness or origin_status or "legacy_incomplete",
    }


def new_map_constraints(
    *,
    gnss_origin: dict | None,
    requested_scene_scope: str = "indoor",
) -> dict[str, Any]:
    fixed = gnss_origin_is_fixed(gnss_origin)
    coordinate_mode = "rtk_fixed" if fixed else "local_only"
    scene_scope = normalize_text(requested_scene_scope, "indoor")
    if scene_scope not in SCENE_SCOPES:
        scene_scope = "indoor"
    if coordinate_mode == "local_only":
        scene_scope = "indoor"
    constraints = {
        "schema_version": SCHEMA_VERSION,
        "coordinate_mode": coordinate_mode,
        "scene_scope": scene_scope,
        "localization_mode": "rtk_ndt" if fixed else "ndt",
        "origin_status": "fixed" if fixed else "local_only",
        "rtk_origin_required": fixed,
        "completeness": "complete",
    }
    validate_map_constraints(constraints)
    return constraints


def validate_map_constraints(constraints: dict[str, Any]) -> dict[str, Any]:
    mode = normalize_text(constraints.get("coordinate_mode"))
    origin = normalize_text(constraints.get("origin_status"))
    if origin == "legacy_incomplete" and not mode:
        return constraints
    if mode not in COORDINATE_MODES:
        raise MapConstraintError(MAP_CONSTRAINT_INVALID, "coordinate_mode must be rtk_fixed or local_only")
    scene = normalize_text(constraints.get("scene_scope"), "indoor")
    localization = map_localization_mode(constraints.get("localization_mode"), coordinate_mode=mode)
    if mode == "local_only":
        if scene == "outdoor":
            raise MapConstraintError(MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN, "local_only maps cannot use outdoor scene_scope")
        if scene == "transition":
            raise MapConstraintError(MAP_LOCAL_ONLY_TRANSITION_FORBIDDEN, "local_only maps cannot use transition scene_scope")
        if scene != "indoor":
            raise MapConstraintError(MAP_CONSTRAINT_INVALID, "local_only maps must use indoor scene_scope")
        if localization != "ndt":
            raise MapConstraintError(MAP_LOCAL_ONLY_NDT_ONLY, "local_only maps can only use NDT localization")
        origin = "local_only"
    else:
        if scene not in SCENE_SCOPES:
            raise MapConstraintError(MAP_CONSTRAINT_INVALID, "rtk_fixed maps require indoor, transition, or outdoor scene_scope")
        origin = origin if origin in ORIGIN_STATUSES else "fixed"
    constraints.update(
        {
            "coordinate_mode": mode,
            "scene_scope": scene,
            "localization_mode": localization,
            "origin_status": origin,
            "rtk_origin_required": mode == "rtk_fixed",
        }
    )
    return constraints


def validate_route_against_map(
    map_constraints: dict[str, Any],
    *,
    scene_scope: str = "",
    waypoints: list | None = None,
    require_rtk_origin: bool = False,
) -> None:
    constraints = validate_map_constraints(dict(map_constraints or {}))
    mode = constraints.get("coordinate_mode") or ""
    if not mode:
        return
    requested_scene = normalize_text(scene_scope or constraints.get("scene_scope"), "indoor")
    if mode == "local_only":
        if requested_scene == "outdoor":
            raise MapConstraintError(MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN, "local_only maps cannot bind outdoor tasks")
        if requested_scene == "transition":
            raise MapConstraintError(MAP_LOCAL_ONLY_TRANSITION_FORBIDDEN, "local_only maps cannot bind transition tasks")
        for index, waypoint in enumerate(waypoints or []):
            if not isinstance(waypoint, dict):
                continue
            if waypoint_localization_mode(waypoint.get("localization_mode")) == "rtk":
                raise MapConstraintError(
                    MAP_LOCAL_ONLY_NDT_ONLY,
                    f"local_only maps cannot use RTK localization at waypoint {index + 1}",
                )
    if (require_rtk_origin or requested_scene in {"outdoor", "transition"}) and mode != "rtk_fixed":
        raise MapConstraintError(MAP_RTK_ORIGIN_REQUIRED, "outdoor and transition scenes require an RTK-fixed map origin")


def constraints_from_map_data(map_data) -> dict[str, Any]:
    description = {}
    raw_description = getattr(map_data, "description", "") or ""
    if isinstance(raw_description, dict):
        description = raw_description
    else:
        try:
            parsed = json.loads(raw_description or "{}")
            description = parsed if isinstance(parsed, dict) else {}
        except (TypeError, json.JSONDecodeError):
            description = {}
    manifest = description.get("map_manifest") if isinstance(description.get("map_manifest"), dict) else {}
    coordinate_mode = str(getattr(map_data, "coordinate_mode", "") or manifest.get("coordinate_mode") or "")
    if coordinate_mode:
        manifest = {
            **manifest,
            "coordinate_mode": coordinate_mode,
            "scene_scope": getattr(map_data, "scene_scope", "") or manifest.get("scene_scope") or "",
            "localization_mode": getattr(map_data, "localization_mode", "") or manifest.get("localization_mode") or "",
            "origin_status": getattr(map_data, "origin_status", "") or manifest.get("origin_status") or "",
            "completeness": getattr(map_data, "map_completeness", "") or manifest.get("completeness") or "",
        }
    gnss_text = str(description.get("gnss_origin_yaml") or "")
    gnss_origin = {}
    if "alignment_locked:" in gnss_text:
        locked = any(token in gnss_text for token in ("alignment_locked: 1", "alignment_locked: true", "alignment_locked: True"))
        gnss_origin = {"alignment_locked": locked}
    return constraints_from_manifest(manifest, gnss_origin=gnss_origin)
