"""Read-only scene artifacts derived from uploaded navigation map packages."""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import zipfile
from pathlib import Path, PurePosixPath

from django.conf import settings


SCENE_SCHEMA = "roamerx.scene-manifest.v1"
SCENE_POINT_CAP = 600_000
SCENE_ASSET_CATALOG_SCHEMA = "roamerx.scene-assets.v1"
SCENE_ASSET_CATALOG_URL = "/scene-assets/catalog.json"
_PCD_NAMES = ("scene_preview.pcd", "map.pcd")


class SceneArtifactError(ValueError):
    pass


def _description(map_data) -> dict:
    try:
        value = json.loads(map_data.description or "{}")
    except (TypeError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _safe_member(name: str) -> bool:
    path = PurePosixPath(name)
    return not path.is_absolute() and ".." not in path.parts


def _cloud_member(archive: zipfile.ZipFile) -> str | None:
    safe_names = [name for name in archive.namelist() if _safe_member(name)]
    for preferred in _PCD_NAMES:
        exact = next((name for name in safe_names if name == preferred), None)
        if exact:
            return exact
        nested = next((name for name in safe_names if PurePosixPath(name).name == preferred), None)
        if nested:
            return nested
    return None


def _parse_pcd_header(stream) -> tuple[list[bytes], dict[str, str]]:
    lines: list[bytes] = []
    values: dict[str, str] = {}
    total = 0
    while total < 64 * 1024:
        line = stream.readline()
        if not line:
            break
        total += len(line)
        lines.append(line)
        text = line.decode("ascii", errors="ignore").strip()
        if text and not text.startswith("#"):
            key, _, value = text.partition(" ")
            values[key.upper()] = value.strip()
        if text.upper().startswith("DATA "):
            return lines, values
    raise SceneArtifactError("PCD header is missing or too large")


def _point_step(header: dict[str, str]) -> int:
    sizes = [int(item) for item in header.get("SIZE", "").split()]
    counts = [int(item) for item in header.get("COUNT", "").split()] or [1] * len(sizes)
    if not sizes or len(sizes) != len(counts):
        raise SceneArtifactError("PCD SIZE/COUNT fields are invalid")
    step = sum(size * count for size, count in zip(sizes, counts))
    if step <= 0 or step > 4096:
        raise SceneArtifactError("PCD point step is invalid")
    return step


def _rewrite_header(lines: list[bytes], point_count: int) -> bytes:
    output = []
    for raw in lines:
        text = raw.decode("ascii", errors="ignore")
        key = text.strip().split(" ", 1)[0].upper() if text.strip() else ""
        if key == "WIDTH":
            raw = f"WIDTH {point_count}\n".encode()
        elif key == "HEIGHT":
            raw = b"HEIGHT 1\n"
        elif key == "POINTS":
            raw = f"POINTS {point_count}\n".encode()
        output.append(raw)
    return b"".join(output)


def _copy_binary_sampled(source, target, header_lines, header, max_points: int) -> int:
    points = int(header.get("POINTS") or header.get("WIDTH") or 0)
    if points <= 0:
        raise SceneArtifactError("PCD point count is invalid")
    point_step = _point_step(header)
    stride = max(1, math.ceil(points / max_points))
    selected = math.ceil(points / stride)
    target.write(_rewrite_header(header_lines, selected))
    block_points = max(1, (2 * 1024 * 1024) // point_step)
    global_index = 0
    written = 0
    while global_index < points:
        wanted = min(block_points, points - global_index)
        chunk = source.read(wanted * point_step)
        records = len(chunk) // point_step
        if not records:
            break
        first = (-global_index) % stride
        for local_index in range(first, records, stride):
            offset = local_index * point_step
            target.write(chunk[offset:offset + point_step])
            written += 1
        global_index += records
    if global_index != points or written != selected:
        raise SceneArtifactError("PCD binary payload is truncated")
    return written


def _copy_ascii_sampled(source, target, header_lines, header, max_points: int) -> int:
    points = int(header.get("POINTS") or header.get("WIDTH") or 0)
    stride = max(1, math.ceil(points / max_points))
    selected = math.ceil(points / stride)
    target.write(_rewrite_header(header_lines, selected))
    written = 0
    for index, line in enumerate(source):
        if index % stride == 0:
            target.write(line)
            written += 1
    if written != selected:
        raise SceneArtifactError("PCD ASCII payload is truncated")
    return written


def _package_sha256(map_data) -> str:
    description = _description(map_data)
    stored = str(description.get("package_sha256") or "")
    if len(stored) == 64:
        return stored
    digest = hashlib.sha256()
    with map_data.package_file.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite_number(value, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _vector3(value, default: tuple[float, float, float]) -> dict[str, float]:
    if isinstance(value, dict):
        values = [value.get(axis) for axis in ("x", "y", "z")]
    elif isinstance(value, (list, tuple)):
        values = list(value[:3])
    else:
        values = []
    return {axis: _finite_number(values[index] if index < len(values) else None, fallback) for index, (axis, fallback) in enumerate(zip(("x", "y", "z"), default))}


def _scene_static_assets(scene: dict) -> list[dict]:
    raw_assets = scene.get("static_assets") if isinstance(scene.get("static_assets"), list) else []
    assets = []
    for index, raw in enumerate(raw_assets):
        if not isinstance(raw, dict):
            continue
        asset_id = str(raw.get("asset_id") or raw.get("asset") or raw.get("class_name") or "").strip()
        if not asset_id:
            continue
        item = dict(raw)
        item.setdefault("id", f"static-{index}")
        item["asset_id"] = asset_id
        item["position"] = _vector3(raw.get("position"), (0.0, 0.0, 0.0))
        if raw.get("orientation") is not None:
            orientation = raw.get("orientation")
            if isinstance(orientation, dict):
                item["orientation"] = {
                    "x": _finite_number(orientation.get("x"), 0.0),
                    "y": _finite_number(orientation.get("y"), 0.0),
                    "z": _finite_number(orientation.get("z"), 0.0),
                    "w": _finite_number(orientation.get("w"), 1.0),
                }
        scale = raw.get("scale")
        if isinstance(scale, (int, float)) and math.isfinite(float(scale)) and float(scale) > 0:
            item["scale"] = float(scale)
        elif isinstance(scale, (dict, list, tuple)):
            item["scale"] = _vector3(scale, (1.0, 1.0, 1.0)) if isinstance(scale, (dict, list, tuple)) else scale
        assets.append(item)
    return assets


def scene_cloud_path(map_data, *, max_points: int = SCENE_POINT_CAP) -> tuple[Path, str, int]:
    if not map_data.package_file:
        raise SceneArtifactError("地图没有三维点云包")
    checksum = _package_sha256(map_data)
    cache_dir = Path(settings.MEDIA_ROOT) / "maps" / "scene-cache" / str(map_data.pk) / checksum
    cache_path = cache_dir / f"scene-{max_points}.pcd"
    meta_path = cache_dir / f"scene-{max_points}.json"
    if cache_path.is_file() and meta_path.is_file():
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        return cache_path, checksum, int(meta.get("point_count") or 0)

    cache_dir.mkdir(parents=True, exist_ok=True)
    temporary = cache_path.with_suffix(".partial")
    try:
        with zipfile.ZipFile(map_data.package_file.path) as archive:
            member = _cloud_member(archive)
            if not member:
                raise SceneArtifactError("地图包缺少 map.pcd 或 scene_preview.pcd")
            with archive.open(member) as source, temporary.open("wb") as target:
                header_lines, header = _parse_pcd_header(source)
                data_kind = header.get("DATA", "").lower()
                if data_kind == "binary":
                    point_count = _copy_binary_sampled(source, target, header_lines, header, max_points)
                elif data_kind == "ascii":
                    point_count = _copy_ascii_sampled(source, target, header_lines, header, max_points)
                elif data_kind == "binary_compressed" and PurePosixPath(member).name == "scene_preview.pcd":
                    point_count = int(header.get("POINTS") or header.get("WIDTH") or 0)
                    if point_count <= 0 or point_count > max_points:
                        raise SceneArtifactError(f"scene_preview.pcd 点数必须在 1 到 {max_points} 之间")
                    target.write(b"".join(header_lines))
                    shutil.copyfileobj(source, target, length=2 * 1024 * 1024)
                else:
                    raise SceneArtifactError("压缩PCD必须先生成 scene_preview.pcd")
        temporary.replace(cache_path)
        meta_path.write_text(json.dumps({"point_count": point_count, "source": member}), encoding="utf-8")
        return cache_path, checksum, point_count
    finally:
        temporary.unlink(missing_ok=True)


def build_scene_manifest(map_data) -> dict:
    description = _description(map_data)
    map_manifest = description.get("map_manifest") if isinstance(description.get("map_manifest"), dict) else {}
    scene = description.get("scene_manifest") if isinstance(description.get("scene_manifest"), dict) else {}
    if not scene and isinstance(map_manifest.get("scene"), dict):
        scene = map_manifest["scene"]
    origin = list(map_data.origin or [0.0, 0.0, 0.0])
    origin += [0.0] * (3 - len(origin))
    min_x, min_y = float(origin[0]), float(origin[1])
    max_x = min_x + float(map_data.width or 0) * float(map_data.resolution or 0.05)
    max_y = min_y + float(map_data.height or 0) * float(map_data.resolution or 0.05)
    package_names = description.get("package_files") if isinstance(description.get("package_files"), list) else []
    cloud_available = any(PurePosixPath(str(name)).name in _PCD_NAMES for name in package_names)
    if not cloud_available and map_data.package_file:
        try:
            with zipfile.ZipFile(map_data.package_file.path) as archive:
                cloud_available = _cloud_member(archive) is not None
        except (OSError, zipfile.BadZipFile):
            cloud_available = False
    boundary_record = getattr(map_data, "navigation_boundary", None)
    boundary = scene.get("boundary") if isinstance(scene.get("boundary"), (list, dict)) else []
    if boundary_record is not None:
        active = boundary_record.active_payload if isinstance(boundary_record.active_payload, dict) else {}
        use_active = boundary_record.active_revision > 0 and bool(active.get("outer_polygon"))
        boundary = {
            "points": active.get("outer_polygon", []) if use_active else boundary_record.outer_polygon,
            "zones": active.get("zones", []) if use_active else [
                {"id": zone.pk, "name": zone.name, "zone_type": zone.zone_type, "polygon": zone.polygon}
                for zone in map_data.zones.filter(active=True)
            ],
            "revision": boundary_record.revision,
            "active_revision": boundary_record.active_revision,
            "apply_status": boundary_record.apply_status,
            "safety_margin_m": boundary_record.safety_margin_m,
        }
    asset_catalog_url = str(scene.get("asset_catalog_url") or SCENE_ASSET_CATALOG_URL)
    return {
        "schema": SCENE_SCHEMA,
        "map_id": map_data.pk,
        "name": map_data.name,
        "frame_id": "map",
        "coordinate_mode": map_data.coordinate_mode or "local_only",
        "origin": origin[:3],
        "bounds": {"min_x": min_x, "min_y": min_y, "min_z": -1.0, "max_x": max_x, "max_y": max_y, "max_z": 3.0},
        "resolution": float(map_data.resolution or 0.05),
        "occupancy_url": f"/api/maps/{map_data.pk}/preview/?raw=1",
        "cloud": {
            "available": cloud_available,
            "url": f"/api/maps/{map_data.pk}/scene-cloud/" if cloud_available else None,
            "point_cap": SCENE_POINT_CAP,
            "color_mode": str(scene.get("color_mode") or "intensity"),
        },
        "asset_catalog": {
            "schema": SCENE_ASSET_CATALOG_SCHEMA,
            "url": asset_catalog_url,
        },
        "asset_catalog_url": asset_catalog_url,
        "static_assets": _scene_static_assets(scene),
        "boundary": boundary,
        "package_checksum": str(description.get("package_sha256") or ""),
    }
