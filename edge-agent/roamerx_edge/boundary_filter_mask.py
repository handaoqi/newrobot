from __future__ import annotations

import json
import math
import os
import tempfile
from pathlib import Path

import yaml

from .protocol import ProtocolError


def _inside(x: float, y: float, polygon: list) -> bool:
    inside = False
    previous = len(polygon) - 1
    for index, current in enumerate(polygon):
        xi, yi = float(current[0]), float(current[1])
        xj, yj = float(polygon[previous][0]), float(polygon[previous][1])
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi:
            inside = not inside
        previous = index
    return inside


def _segment_distance(x: float, y: float, start, end) -> float:
    ax, ay = float(start[0]), float(start[1])
    bx, by = float(end[0]), float(end[1])
    dx, dy = bx - ax, by - ay
    squared = dx * dx + dy * dy
    if squared <= 1e-18:
        return math.hypot(x - ax, y - ay)
    ratio = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / squared))
    return math.hypot(x - ax - ratio * dx, y - ay - ratio * dy)


def _polygon_distance(x: float, y: float, polygon: list) -> float:
    return min(
        _segment_distance(x, y, polygon[index], polygon[(index + 1) % len(polygon)])
        for index in range(len(polygon))
    ) if len(polygon) >= 2 else math.inf


def _atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def write_keepout_mask(boundary: dict, output_dir: str, *, source_map_yaml: str = "") -> dict:
    """Rasterize hard cloud boundaries into a Nav2 keepout filter mask."""
    map_info = boundary.get("map") or {}
    try:
        width = int(map_info["width"])
        height = int(map_info["height"])
        resolution = float(map_info["resolution"])
        origin = list(map_info.get("origin") or [0.0, 0.0, 0.0])
        origin = [float(origin[0]), float(origin[1]), float(origin[2] if len(origin) > 2 else 0.0)]
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise ProtocolError("BOUNDARY_MAP_METADATA_INVALID", "导航边界缺少栅格地图尺寸、分辨率或原点") from exc
    if width <= 0 or height <= 0 or resolution <= 0 or width * height > 100_000_000:
        raise ProtocolError("BOUNDARY_MAP_METADATA_INVALID", "导航边界栅格地图尺寸无效或过大")
    if source_map_yaml and Path(source_map_yaml).exists():
        local_yaml = Path(source_map_yaml)
        local = yaml.safe_load(local_yaml.read_text(encoding="utf-8")) or {}
        local_image = Path(str(local.get("image") or ""))
        if not local_image.is_absolute():
            local_image = local_yaml.parent / local_image
        local_width, local_height = _pgm_dimensions(local_image)
        local_resolution = float(local.get("resolution") or 0.0)
        local_origin = [float(item) for item in (local.get("origin") or [0, 0, 0])]
        if (
            (local_width, local_height) != (width, height)
            or abs(local_resolution - resolution) > 1e-9
            or any(abs((local_origin + [0, 0, 0])[index] - origin[index]) > 1e-6 for index in range(3))
        ):
            raise ProtocolError(
                "BOUNDARY_MAP_METADATA_MISMATCH",
                "云端边界所用地图与机器人当前栅格地图尺寸、分辨率或原点不一致",
            )
    outer = boundary.get("outer_polygon") or []
    forbidden = [
        zone.get("polygon") or [] for zone in boundary.get("zones") or []
        if zone.get("active", True) and zone.get("zone_type") == "forbidden"
    ]
    margin = max(0.0, float(boundary.get("safety_margin_m") or 0.0))
    cos_yaw, sin_yaw = math.cos(origin[2]), math.sin(origin[2])
    pixels = bytearray(width * height)
    occupied_cells = 0
    for row in range(height):
        local_y = (height - row - 0.5) * resolution
        for column in range(width):
            local_x = (column + 0.5) * resolution
            x = origin[0] + cos_yaw * local_x - sin_yaw * local_y
            y = origin[1] + sin_yaw * local_x + cos_yaw * local_y
            blocked = not _inside(x, y, outer) or _polygon_distance(x, y, outer) < margin
            if not blocked:
                blocked = any(
                    _inside(x, y, polygon) or _polygon_distance(x, y, polygon) < margin
                    for polygon in forbidden
                )
            # Occupancy map convention: black=occupied, white=free.
            pixels[row * width + column] = 0 if blocked else 255
            occupied_cells += int(blocked)
    directory = Path(output_dir)
    pgm_path = directory / "keepout_mask.pgm"
    yaml_path = directory / "keepout_mask.yaml"
    metadata_path = directory / "keepout_mask.metadata.json"
    _atomic_write(pgm_path, f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(pixels))
    yaml_document = (
        f"image: {pgm_path}\n"
        f"mode: trinary\nresolution: {resolution}\n"
        f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    _atomic_write(yaml_path, yaml_document.encode("utf-8"))
    metadata = {
        "map_id": str(boundary.get("map_id") or ""),
        "revision": int(boundary.get("revision") or 0),
        "width": width, "height": height, "resolution": resolution, "origin": origin,
        "source_map_yaml": str(Path(source_map_yaml).resolve()) if source_map_yaml else "",
        "occupied_cells": occupied_cells,
    }
    _atomic_write(metadata_path, json.dumps(metadata, ensure_ascii=False, indent=2).encode("utf-8"))
    return {"yaml_path": str(yaml_path), "pgm_path": str(pgm_path), **metadata}


def ensure_permissive_mask(map_yaml_path: str, output_dir: str) -> dict | None:
    """Create a zero-cost filter mask when a legacy map has no boundary."""
    source = Path(map_yaml_path)
    if not source.exists():
        return None
    raw = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    image = Path(str(raw.get("image") or ""))
    if not image.is_absolute():
        image = source.parent / image
    width, height = _pgm_dimensions(image)
    boundary = {
        "map_id": "",
        "revision": 0,
        "map": {
            "width": width, "height": height,
            "resolution": float(raw.get("resolution") or 0.05),
            "origin": raw.get("origin") or [0.0, 0.0, 0.0],
        },
        "outer_polygon": _map_extent_polygon(width, height, float(raw.get("resolution") or 0.05), raw.get("origin") or [0, 0, 0]),
        "safety_margin_m": 0.0,
        "zones": [],
    }
    return write_keepout_mask(boundary, output_dir, source_map_yaml=str(source))


def _pgm_dimensions(path: Path) -> tuple[int, int]:
    with path.open("rb") as handle:
        tokens = []
        while len(tokens) < 4:
            line = handle.readline()
            if not line:
                break
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
    if len(tokens) < 4 or tokens[0] not in {b"P2", b"P5"}:
        raise ProtocolError("BOUNDARY_MAP_METADATA_INVALID", f"无法读取地图 PGM 尺寸: {path}")
    return int(tokens[1]), int(tokens[2])


def _map_extent_polygon(width: int, height: int, resolution: float, origin: list) -> list[list[float]]:
    ox, oy = float(origin[0]), float(origin[1])
    yaw = float(origin[2] if len(origin) > 2 else 0.0)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    result = []
    for local_x, local_y in ((0, 0), (width * resolution, 0), (width * resolution, height * resolution), (0, height * resolution)):
        result.append([ox + cos_yaw * local_x - sin_yaw * local_y, oy + sin_yaw * local_x + cos_yaw * local_y])
    return result
