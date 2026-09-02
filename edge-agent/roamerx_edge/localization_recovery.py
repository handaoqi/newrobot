from __future__ import annotations

import logging
import math
from typing import Any

LOGGER = logging.getLogger(__name__)


def planar_distance_m(left: dict[str, Any], right: dict[str, Any]) -> float:
    return math.hypot(float(left["x"]) - float(right["x"]), float(left["y"]) - float(right["y"]))


def pose_dict_from_latest(latest) -> dict[str, float] | None:
    if latest is None:
        return None
    return {
        "x": float(latest.x),
        "y": float(latest.y),
        "z": float(getattr(latest, "z", 0.0) or 0.0),
        "yaw": float(getattr(latest, "yaw", 0.0) or 0.0),
    }


def seed_is_plausible(
    seed: dict[str, Any] | None,
    anchor: dict[str, Any] | None,
    *,
    max_drift_m: float,
) -> bool:
    if not seed or not anchor:
        return True
    if seed.get("x") is None or seed.get("y") is None:
        return True
    if anchor.get("x") is None or anchor.get("y") is None:
        return True
    drift = planar_distance_m(seed, anchor)
    if drift <= max_drift_m:
        return True
    LOGGER.warning(
        "rejecting localization seed source=%s drift=%.1fm exceeds %.1fm (anchor=%s)",
        seed.get("source"),
        drift,
        max_drift_m,
        anchor.get("source"),
    )
    return False


def select_recovery_seed(
    *,
    latest_pose,
    memory_trusted: dict[str, Any] | None,
    disk_trusted: dict[str, Any] | None,
    waypoint: dict[str, Any] | None,
    max_drift_m: float,
) -> dict[str, Any] | None:
    """Pick the safest automatic relocalization seed.

    Prefer the live published pose over persisted last_trusted values. A brief
    RTK fixed->float transition can leave a stale indoor-scale trusted pose on
    disk while the robot is still outdoors at the last good GPS pose.
    """
    anchor = pose_dict_from_latest(latest_pose)
    if anchor is not None:
        anchor = {**anchor, "source": "latest_pose"}

    candidates: list[dict[str, Any]] = []
    if anchor is not None:
        candidates.append(anchor)
    if memory_trusted and seed_is_plausible(memory_trusted, anchor, max_drift_m=max_drift_m):
        seed = dict(memory_trusted)
        seed.setdefault("source", "last_trusted")
        candidates.append(seed)
    if disk_trusted and seed_is_plausible(disk_trusted, anchor, max_drift_m=max_drift_m):
        seed = dict(disk_trusted)
        seed.setdefault("source", "last_trusted_persisted")
        if not any(
            planar_distance_m(seed, existing) < 0.05
            for existing in candidates
            if existing.get("x") is not None and existing.get("y") is not None
        ):
            candidates.append(seed)
    if waypoint and waypoint.get("x") is not None and waypoint.get("y") is not None:
        seed = {
            "x": float(waypoint["x"]),
            "y": float(waypoint["y"]),
            "z": float(waypoint.get("z", 0.0) or 0.0),
            "yaw": float(waypoint.get("yaw", 0.0) or 0.0),
            "source": "current_waypoint",
        }
        if waypoint.get("waypoint_index") is not None:
            seed["waypoint_index"] = waypoint["waypoint_index"]
        if seed_is_plausible(seed, anchor, max_drift_m=max_drift_m):
            candidates.append(seed)

    return candidates[0] if candidates else None
