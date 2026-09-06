from __future__ import annotations

import math
from typing import Any

from rest_framework.exceptions import ValidationError

from ..models import MapNavigationBoundary, Zone


ZONE_TYPES = {choice[0] for choice in Zone.ZONE_TYPE_CHOICES}


def _point(value: Any, label: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValidationError(f"{label}必须是 [x, y]")
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError) as exc:
        raise ValidationError(f"{label}坐标必须是数值") from exc
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValidationError(f"{label}坐标必须是有限数值")
    return [round(x, 6), round(y, 6)]


def normalize_polygon(value: Any, label: str, *, required: bool = True) -> list[list[float]]:
    if value in (None, []) and not required:
        return []
    if not isinstance(value, list) or len(value) < 3:
        raise ValidationError(f"{label}至少需要三个顶点")
    polygon = [_point(item, f"{label}第 {index + 1} 个点") for index, item in enumerate(value)]
    if polygon[0] == polygon[-1]:
        polygon.pop()
    if len(polygon) < 3:
        raise ValidationError(f"{label}至少需要三个不同顶点")
    if abs(polygon_area(polygon)) < 1e-6:
        raise ValidationError(f"{label}面积不能为零")
    if polygon_self_intersects(polygon):
        raise ValidationError(f"{label}不能自相交")
    return polygon


def polygon_area(polygon: list[list[float]]) -> float:
    return 0.5 * sum(
        polygon[index][0] * polygon[(index + 1) % len(polygon)][1]
        - polygon[(index + 1) % len(polygon)][0] * polygon[index][1]
        for index in range(len(polygon))
    )


def _orientation(a, b, c) -> int:
    value = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
    if abs(value) < 1e-9:
        return 0
    return 1 if value > 0 else 2


def _on_segment(a, b, c) -> bool:
    return min(a[0], c[0]) - 1e-9 <= b[0] <= max(a[0], c[0]) + 1e-9 and min(a[1], c[1]) - 1e-9 <= b[1] <= max(a[1], c[1]) + 1e-9


def segments_intersect(a, b, c, d) -> bool:
    o1, o2, o3, o4 = _orientation(a, b, c), _orientation(a, b, d), _orientation(c, d, a), _orientation(c, d, b)
    if o1 != o2 and o3 != o4:
        return True
    return (
        (o1 == 0 and _on_segment(a, c, b))
        or (o2 == 0 and _on_segment(a, d, b))
        or (o3 == 0 and _on_segment(c, a, d))
        or (o4 == 0 and _on_segment(c, b, d))
    )


def polygon_self_intersects(polygon: list[list[float]]) -> bool:
    size = len(polygon)
    for index in range(size):
        a, b = polygon[index], polygon[(index + 1) % size]
        for other in range(index + 1, size):
            if other in {index, (index + 1) % size} or (other + 1) % size == index:
                continue
            if segments_intersect(a, b, polygon[other], polygon[(other + 1) % size]):
                return True
    return False


def point_in_polygon(point: list[float], polygon: list[list[float]]) -> bool:
    if len(polygon) < 3:
        return False
    x, y = point
    inside = False
    previous = len(polygon) - 1
    for index, current in enumerate(polygon):
        xi, yi = current
        xj, yj = polygon[previous]
        if _orientation(polygon[previous], point, current) == 0 and _on_segment(polygon[previous], point, current):
            return True
        if (yi > y) != (yj > y):
            intersection_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < intersection_x:
                inside = not inside
        previous = index
    return inside


def point_to_segment_distance(point, start, end) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def distance_to_polygon(point, polygon) -> float:
    if len(polygon) < 2:
        return math.inf
    return min(
        point_to_segment_distance(point, polygon[index], polygon[(index + 1) % len(polygon)])
        for index in range(len(polygon))
    )


def point_allowed_by_boundary(point, payload: dict) -> tuple[bool, str]:
    """Check hard geometry using the active safety margin.

    The margin shrinks the drivable outer polygon and expands forbidden zones;
    warning and speed-limit zones remain informational/soft constraints.
    """
    outer = payload.get("outer_polygon") or []
    margin = max(0.0, float(payload.get("safety_margin_m") or 0.0))
    if outer and (not point_in_polygon(point, outer) or distance_to_polygon(point, outer) < margin):
        return False, "outside"
    for zone in payload.get("zones") or []:
        if not zone.get("active", True) or zone.get("zone_type") != "forbidden":
            continue
        polygon = zone.get("polygon") or []
        if point_in_polygon(point, polygon) or distance_to_polygon(point, polygon) < margin:
            return False, "forbidden"
    return True, ""


def normalize_boundary_payload(payload: dict) -> dict:
    outer = normalize_polygon(payload.get("outer_polygon"), "可行驶外边界")
    try:
        margin = float(payload.get("safety_margin_m", 0.2))
    except (TypeError, ValueError) as exc:
        raise ValidationError("安全距离必须是数值") from exc
    if not math.isfinite(margin) or not 0 <= margin <= 5:
        raise ValidationError("安全距离必须在 0 到 5 米之间")

    zones = []
    for index, raw in enumerate(payload.get("zones") or []):
        if not isinstance(raw, dict):
            raise ValidationError(f"第 {index + 1} 个内部区域格式无效")
        zone_type = str(raw.get("zone_type") or "forbidden")
        if zone_type not in ZONE_TYPES:
            raise ValidationError(f"第 {index + 1} 个内部区域类型无效")
        polygon = normalize_polygon(raw.get("polygon"), f"第 {index + 1} 个内部区域")
        if any(not point_in_polygon(point, outer) for point in polygon):
            raise ValidationError(f"第 {index + 1} 个内部区域必须完全位于可行驶外边界内")
        speed = raw.get("speed_limit_mps")
        if zone_type == "restricted":
            try:
                speed = float(speed if speed not in (None, "") else 0.15)
            except (TypeError, ValueError) as exc:
                raise ValidationError(f"第 {index + 1} 个限速值必须是数值") from exc
            if not 0 < speed <= 0.3:
                raise ValidationError(f"第 {index + 1} 个限速值必须大于 0 且不超过 0.30 m/s")
        else:
            speed = None
        try:
            warning_distance = float(raw.get("warning_distance_m", 0.5))
        except (TypeError, ValueError) as exc:
            raise ValidationError(f"第 {index + 1} 个提前警告距离必须是数值") from exc
        if not 0 <= warning_distance <= 10:
            raise ValidationError(f"第 {index + 1} 个提前警告距离必须在 0 到 10 米之间")
        zones.append({
            "id": raw.get("id"),
            "name": str(raw.get("name") or f"区域{index + 1}")[:128],
            "zone_type": zone_type,
            "polygon": polygon,
            "description": str(raw.get("description") or ""),
            "active": bool(raw.get("active", True)),
            "speed_limit_mps": speed,
            "warning_distance_m": warning_distance,
        })
    return {"outer_polygon": outer, "safety_margin_m": round(margin, 3), "zones": zones}


def boundary_payload(boundary: MapNavigationBoundary, *, active: bool = False) -> dict:
    if active and boundary.active_revision > 0 and boundary.active_payload:
        payload = dict(boundary.active_payload)
        payload.update({
            "map_id": boundary.map_data_id,
            "revision": boundary.active_revision,
            "active_revision": boundary.active_revision,
            "apply_status": boundary.apply_status,
            "apply_error": boundary.apply_error,
            "updated_at": boundary.updated_at.isoformat(),
        })
        return payload
    zones = boundary.map_data.zones.order_by("id")
    return {
        "map_id": boundary.map_data_id,
        "map": {
            "resolution": boundary.map_data.resolution,
            "width": boundary.map_data.width,
            "height": boundary.map_data.height,
            "origin": boundary.map_data.origin,
        },
        "revision": boundary.revision,
        "active_revision": boundary.active_revision,
        "apply_status": boundary.apply_status,
        "apply_error": boundary.apply_error,
        "outer_polygon": boundary.outer_polygon or [],
        "safety_margin_m": boundary.safety_margin_m,
        "zones": [
            {
                "id": zone.id,
                "name": zone.name,
                "zone_type": zone.zone_type,
                "polygon": zone.polygon,
                "description": zone.description,
                "active": zone.active,
                "speed_limit_mps": zone.speed_limit_mps,
                "warning_distance_m": zone.warning_distance_m,
            }
            for zone in zones
        ],
        "updated_at": boundary.updated_at.isoformat(),
    }


def validate_waypoints_against_boundary(map_data, waypoints: list[dict]) -> None:
    try:
        boundary = map_data.navigation_boundary
    except MapNavigationBoundary.DoesNotExist:
        return
    if not boundary.outer_polygon:
        return
    points = []
    for index, waypoint in enumerate(waypoints or []):
        try:
            point = [float(waypoint["x"]), float(waypoint["y"])]
        except (KeyError, TypeError, ValueError):
            continue
        points.append(point)
        payload = boundary_payload(boundary, active=boundary.active_revision > 0)
        allowed, reason = point_allowed_by_boundary(point, payload)
        if reason == "outside":
            raise ValidationError(f"途经点 {index + 1} 位于可行驶外边界之外")
        if reason == "forbidden":
            raise ValidationError(f"途经点 {index + 1} 位于禁入区内")
    for index, (start, end) in enumerate(zip(points, points[1:])):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        samples = min(2_000, max(1, int(math.ceil(distance / 0.05))))
        for sample_index in range(1, samples):
            ratio = sample_index / samples
            sample = [
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            ]
            allowed, _ = point_allowed_by_boundary(sample, payload)
            if not allowed:
                raise ValidationError(f"途经点 {index + 1} 到 {index + 2} 的连线穿越硬边界")
