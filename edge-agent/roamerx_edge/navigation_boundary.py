from __future__ import annotations

import math
import logging
from pathlib import Path

from .boundary_filter_mask import ensure_permissive_mask, write_keepout_mask
from .protocol import ProtocolError


LOGGER = logging.getLogger(__name__)


def _point_in_polygon(x: float, y: float, polygon: list) -> bool:
    if len(polygon) < 3:
        return False
    inside = False
    previous = len(polygon) - 1
    for index, current in enumerate(polygon):
        xi, yi = float(current[0]), float(current[1])
        xj, yj = float(polygon[previous][0]), float(polygon[previous][1])
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi:
            inside = not inside
        previous = index
    return inside


def _point_to_segment_distance(x: float, y: float, start, end) -> float:
    ax, ay = float(start[0]), float(start[1])
    bx, by = float(end[0]), float(end[1])
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return math.hypot(x - ax, y - ay)
    ratio = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_squared))
    return math.hypot(x - (ax + ratio * dx), y - (ay + ratio * dy))


def _distance_to_polygon(x: float, y: float, polygon: list) -> float:
    if len(polygon) < 2:
        return math.inf
    return min(
        _point_to_segment_distance(x, y, polygon[index], polygon[(index + 1) % len(polygon)])
        for index in range(len(polygon))
    )


class NavigationBoundaryManager:
    """Persist and enforce the active cloud geofence at Edge admission time."""

    METADATA_KEY = "navigation_boundary:active"

    def __init__(self, store, structured_logs=None, *, mask_dir: str = "", map_yaml_path: str = "") -> None:
        self.store = store
        self.structured_logs = structured_logs
        self.mask_dir = mask_dir
        self.map_yaml_path = map_yaml_path
        saved = store.get_metadata(self.METADATA_KEY)
        self.active = saved if isinstance(saved, dict) else {}
        self._last_region_keys: set[str] = set()
        self._last_warning_near_keys: set[str] = set()
        self._last_violation = ""
        self._last_speed_limit = None
        try:
            if self.mask_dir:
                if self.active and (self.active.get("map") or {}).get("width"):
                    write_keepout_mask(self.active, self.mask_dir, source_map_yaml=self.map_yaml_path)
                elif self.map_yaml_path:
                    ensure_permissive_mask(self.map_yaml_path, self.mask_dir)
        except Exception as exc:
            # The command path will report a hard failure when applying a real
            # boundary; bootstrap remains compatible with maps not yet present.
            LOGGER.warning("could not bootstrap navigation boundary mask: %s", exc)
            self.active = {}
            self.store.set_metadata(self.METADATA_KEY, self.active)
            for filename in ("keepout_mask.yaml", "keepout_mask.pgm", "keepout_mask.metadata.json"):
                try:
                    (Path(self.mask_dir) / filename).unlink(missing_ok=True)
                except OSError:
                    LOGGER.warning("could not invalidate stale boundary mask file: %s", filename)
            try:
                if self.mask_dir and self.map_yaml_path:
                    ensure_permissive_mask(self.map_yaml_path, self.mask_dir)
            except Exception as fallback_exc:
                LOGGER.warning("could not create permissive navigation mask: %s", fallback_exc)

    def apply(self, command: dict) -> dict:
        boundary = command.get("boundary") or {}
        map_id = str(command.get("map_id") or "")
        revision = int(boundary.get("revision") or 0)
        outer = boundary.get("outer_polygon") or []
        if not map_id or revision <= 0 or len(outer) < 3:
            raise ProtocolError("BOUNDARY_INVALID", "导航边界缺少地图、版本或外边界")
        for point in outer:
            if not isinstance(point, list) or len(point) != 2 or not all(math.isfinite(float(v)) for v in point):
                raise ProtocolError("BOUNDARY_INVALID", "导航边界坐标无效")
        self.active = {
            "map_id": map_id,
            "map_version": str(command.get("map_version") or ""),
            "revision": revision,
            "outer_polygon": outer,
            "safety_margin_m": float(boundary.get("safety_margin_m") or 0.0),
            "zones": [zone for zone in boundary.get("zones") or [] if zone.get("active", True)],
            "map": boundary.get("map") or {},
        }
        mask = None
        if self.mask_dir:
            mask = write_keepout_mask(self.active, self.mask_dir, source_map_yaml=self.map_yaml_path)
        self.store.set_metadata(self.METADATA_KEY, self.active)
        if self.structured_logs:
            self.structured_logs.emit(
                "INFO", "boundary", "boundary.applied", f"导航边界版本 {revision} 已在 Edge 生效",
                data={"map_id": map_id, "revision": revision, "zone_count": len(self.active["zones"])},
                map_id=map_id,
            )
            self.structured_logs.flush()
        return {
            "map_id": map_id,
            "active_revision": revision,
            "zone_count": len(self.active["zones"]),
            "enforcement": "nav2_keepout_filter_and_edge_watchdog" if mask else "edge_admission_and_runtime_watchdog",
            "keepout_mask": mask,
        }

    def activate_map(self, map_id: str, map_version: str = "") -> dict:
        """Fail open for a newly activated legacy map until its own boundary is published."""
        if str(self.active.get("map_id") or "") == str(map_id):
            return {"changed": False, "map_id": str(map_id)}
        self.active = {}
        self.store.set_metadata(self.METADATA_KEY, self.active)
        self._last_region_keys.clear()
        self._last_warning_near_keys.clear()
        self._last_violation = ""
        self._last_speed_limit = None
        mask = ensure_permissive_mask(self.map_yaml_path, self.mask_dir) if self.mask_dir and self.map_yaml_path else None
        return {"changed": True, "map_id": str(map_id), "map_version": str(map_version), "keepout_mask": mask}

    def validate_expected(self, map_id: str, expected_revision=None) -> None:
        if expected_revision in (None, 0, "0", ""):
            return
        if str(self.active.get("map_id") or "") != str(map_id) or int(self.active.get("revision") or 0) != int(expected_revision):
            raise ProtocolError(
                "BOUNDARY_REVISION_MISMATCH",
                f"导航边界版本不一致，期望 {expected_revision}，Edge 为 {self.active.get('revision') or 0}",
            )

    def validate_point(self, map_id: str, x: float, y: float, *, expected_revision=None) -> None:
        self.validate_expected(map_id, expected_revision)
        if str(self.active.get("map_id") or "") != str(map_id):
            return
        outer = self.active.get("outer_polygon") or []
        x, y = float(x), float(y)
        margin = max(0.0, float(self.active.get("safety_margin_m") or 0.0))
        if outer and (not _point_in_polygon(x, y, outer) or _distance_to_polygon(x, y, outer) < margin):
            raise ProtocolError("GOAL_OUTSIDE_NAVIGATION_BOUNDARY", "导航目标位于可行驶外边界之外")
        for zone in self.active.get("zones") or []:
            polygon = zone.get("polygon") or []
            if zone.get("zone_type") == "forbidden" and (
                _point_in_polygon(x, y, polygon) or _distance_to_polygon(x, y, polygon) < margin
            ):
                raise ProtocolError("GOAL_IN_FORBIDDEN_ZONE", f"导航目标位于禁入区“{zone.get('name') or ''}”内")

    def validate_route(self, route: dict) -> None:
        map_info = route.get("map") or {}
        map_id = str(map_info.get("map_id") or "")
        expected = route.get("boundary_revision")
        self.validate_expected(map_id, expected)
        waypoints = route.get("waypoints") or []
        for waypoint in waypoints:
            self.validate_point(map_id, waypoint["x"], waypoint["y"], expected_revision=expected)
        # Validate each route leg as well as its endpoints. This prevents a
        # straight route segment from cutting across a forbidden island or a
        # concave outer boundary before Nav2 receives it.
        for start, end in zip(waypoints, waypoints[1:]):
            distance = math.hypot(float(end["x"]) - float(start["x"]), float(end["y"]) - float(start["y"]))
            samples = min(2_000, max(1, int(math.ceil(distance / 0.05))))
            for index in range(1, samples):
                ratio = index / samples
                self.validate_point(
                    map_id,
                    float(start["x"]) + ratio * (float(end["x"]) - float(start["x"])),
                    float(start["y"]) + ratio * (float(end["y"]) - float(start["y"])),
                    expected_revision=expected,
                )

    def snapshot(self) -> dict:
        return dict(self.active)

    def retry_speed_application(self) -> None:
        self._last_speed_limit = object()

    def retry_violation_stop(self) -> None:
        self._last_violation = ""

    def observe_pose(self, map_id: str, x: float, y: float) -> dict:
        if not self.active or str(self.active.get("map_id") or "") != str(map_id):
            return {"violation": "", "speed_limit_mps": None, "events": []}
        violation = ""
        margin = max(0.0, float(self.active.get("safety_margin_m") or 0.0))
        outer = self.active.get("outer_polygon") or []
        if not _point_in_polygon(x, y, outer) or _distance_to_polygon(x, y, outer) < margin:
            violation = "outside_boundary"
        current_regions = set()
        warning_near = set()
        speed_limits = []
        for zone in self.active.get("zones") or []:
            polygon = zone.get("polygon") or []
            inside_zone = _point_in_polygon(x, y, polygon)
            key = str(zone.get("id") or zone.get("name") or "zone")
            if (
                zone.get("zone_type") == "warning"
                and not inside_zone
                and _distance_to_polygon(x, y, polygon) <= max(0.0, float(zone.get("warning_distance_m") or 0.0))
            ):
                warning_near.add(key)
            if not inside_zone:
                continue
            current_regions.add(key)
            if zone.get("zone_type") == "forbidden":
                violation = f"forbidden:{key}"
            elif zone.get("zone_type") == "restricted" and zone.get("speed_limit_mps") is not None:
                speed_limits.append(float(zone["speed_limit_mps"]))
        if not violation:
            for zone in self.active.get("zones") or []:
                if zone.get("zone_type") != "forbidden":
                    continue
                polygon = zone.get("polygon") or []
                if _distance_to_polygon(x, y, polygon) < margin:
                    violation = f"forbidden:{zone.get('id') or zone.get('name') or 'zone'}"
                    break
        events = []
        for key in current_regions - self._last_region_keys:
            events.append(("entered", key))
        for key in self._last_region_keys - current_regions:
            events.append(("exited", key))
        self._last_region_keys = current_regions
        for key in warning_near - self._last_warning_near_keys:
            events.append(("approaching", key))
        for key in self._last_warning_near_keys - warning_near - current_regions:
            events.append(("approach_cleared", key))
        self._last_warning_near_keys = warning_near
        changed_violation = violation != self._last_violation
        self._last_violation = violation
        speed_limit = min(speed_limits) if speed_limits else None
        speed_changed = speed_limit != self._last_speed_limit
        self._last_speed_limit = speed_limit
        return {
            "violation": violation,
            "violation_changed": changed_violation,
            "speed_limit_mps": speed_limit,
            "speed_changed": speed_changed,
            "events": events,
        }
