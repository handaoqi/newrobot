"""Deterministic PCD-to-street-block conversion used by the cloud worker.

The converter deliberately treats learned semantics as an optional hint.  Its
authoritative geometry comes from the point cloud: flat connected cells become
roads and elevated connected components become independent street objects.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import struct
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


SCHEMA = "roamerx.street-block.v1"
DEFAULT_COLORS = {
    "road": (72, 82, 92),
    "building": (184, 168, 145),
    "wall": (154, 145, 132),
    "tree": (66, 135, 76),
    "vegetation": (80, 145, 75),
    "barrier": (217, 147, 63),
    "traffic_cone": (239, 105, 32),
}


class SceneConversionError(ValueError):
    pass


def _pcd_header(path: Path) -> tuple[int, dict[str, str], int]:
    values: dict[str, str] = {}
    size = 0
    with path.open("rb") as stream:
        while size < 64 * 1024:
            line = stream.readline()
            if not line:
                break
            size += len(line)
            text = line.decode("ascii", errors="ignore").strip()
            if text and not text.startswith("#"):
                key, _, value = text.partition(" ")
                values[key.upper()] = value.strip()
            if text.upper().startswith("DATA "):
                return size, values, int(values.get("POINTS") or values.get("WIDTH") or 0)
    raise SceneConversionError("PCD 文件头无效")


def _header_line_count(path: Path) -> int:
    with path.open("rb") as stream:
        for index, line in enumerate(stream, 1):
            if line.decode("ascii", errors="ignore").strip().upper().startswith("DATA "):
                return index
    raise SceneConversionError("PCD 缺少 DATA 声明")


def read_pcd_xyz(path: str | Path, max_points: int = 1_000_000) -> np.ndarray:
    path = Path(path)
    header_size, header, point_count = _pcd_header(path)
    if point_count <= 0:
        raise SceneConversionError("PCD 中没有点")
    fields = header.get("FIELDS", "").split()
    sizes = [int(value) for value in header.get("SIZE", "").split()]
    types = header.get("TYPE", "").split()
    counts = [int(value) for value in header.get("COUNT", "").split()] or [1] * len(fields)
    if not fields or not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise SceneConversionError("PCD 字段声明不一致")
    try:
        xyz_indices = [fields.index(axis) for axis in ("x", "y", "z")]
    except ValueError as exc:
        raise SceneConversionError("PCD 必须包含 x、y、z") from exc
    stride = max(1, math.ceil(point_count / max(1, max_points)))
    data_kind = header.get("DATA", "").lower()
    if data_kind == "ascii":
        raw = np.loadtxt(path, dtype=np.float32, skiprows=_header_line_count(path))
        raw = np.atleast_2d(raw)[::stride]
        points = raw[:, xyz_indices]
    elif data_kind == "binary":
        type_map = {
            ("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2",
            ("U", 4): "u4", ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4",
        }
        dtype_fields = []
        for index, (name, size, kind, count) in enumerate(zip(fields, sizes, types, counts)):
            dtype = type_map.get((kind.upper(), size))
            if dtype is None or count != 1:
                raise SceneConversionError(f"暂不支持 PCD 字段 {name}")
            dtype_fields.append((f"f{index}", "<" + dtype))
        records = np.memmap(path, dtype=np.dtype(dtype_fields), mode="r", offset=header_size, shape=(point_count,))[::stride]
        points = np.column_stack([records[f"f{index}"] for index in xyz_indices]).astype(np.float32)
    else:
        raise SceneConversionError("MVP 仅支持 ASCII 或 Binary PCD")
    points = np.asarray(points, dtype=np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 20:
        raise SceneConversionError("有效点数量不足 20")
    return points


def sha256_path(path: str | Path) -> str:
    path = Path(path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _largest_component(mask: np.ndarray) -> np.ndarray:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    if count <= 1:
        return mask.astype(bool)
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == index


def _trajectory_points(path: str | Path | None) -> np.ndarray:
    if not path:
        return np.empty((0, 2), dtype=np.float32)
    rows = []
    try:
        with Path(path).open(newline="", encoding="utf-8-sig") as stream:
            for row in csv.DictReader(stream):
                rows.append((float(row["x"]), float(row["y"])))
    except (OSError, KeyError, TypeError, ValueError):
        return np.empty((0, 2), dtype=np.float32)
    return np.asarray(rows, dtype=np.float32)


def _mask_rectangles(mask: np.ndarray, min_x: float, min_y: float, resolution: float) -> list[dict]:
    active: dict[tuple[int, int], tuple[int, int]] = {}
    completed: list[tuple[int, int, int, int]] = []
    for row_index, row in enumerate(mask):
        padded = np.pad(row.astype(np.int8), (1, 1))
        changes = np.diff(padded)
        starts = np.flatnonzero(changes == 1)
        ends = np.flatnonzero(changes == -1)
        spans = {(int(start), int(end)) for start, end in zip(starts, ends) if end - start >= 2}
        next_active = {}
        for span in spans:
            start_row, _ = active.get(span, (row_index, row_index))
            next_active[span] = (start_row, row_index)
        for span, (start_row, end_row) in active.items():
            if span not in spans:
                completed.append((span[0], span[1], start_row, end_row))
        active = next_active
    for span, (start_row, end_row) in active.items():
        completed.append((span[0], span[1], start_row, end_row))
    rectangles = []
    for start, end, row_start, row_end in completed:
        width = (end - start) * resolution
        depth = (row_end - row_start + 1) * resolution
        if width * depth < 0.8:
            continue
        rectangles.append({
            "position": (min_x + (start + end) * resolution / 2, min_y + (row_start + row_end + 1) * resolution / 2, 0.0),
            "dimensions": (width, depth, 0.08),
        })
    return rectangles[:1200]


def _component_nodes(obstacle_mask: np.ndarray, max_z: np.ndarray, ground_z: float, min_x: float, min_y: float, resolution: float) -> list[dict]:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(obstacle_mask.astype(np.uint8), 8)
    nodes = []
    for label in range(1, count):
        cells = np.column_stack(np.nonzero(labels == label)).astype(np.int32)
        if len(cells) < 3:
            continue
        coordinates = np.column_stack((cells[:, 1], cells[:, 0])).astype(np.float32)
        (cx, cy), (width_cells, depth_cells), angle = cv2.minAreaRect(coordinates)
        width = max(resolution, width_cells * resolution + resolution)
        depth = max(resolution, depth_cells * resolution + resolution)
        height = float(np.percentile(max_z[labels == label], 90) - ground_z)
        long_side, short_side = max(width, depth), min(width, depth)
        if height < 0.45:
            continue
        if long_side >= 3.0 and short_side <= 1.1 and long_side / max(short_side, 0.1) >= 3.5:
            category = "wall"
        elif long_side <= 3.5 and short_side <= 3.5 and height >= 1.8:
            category = "tree"
        elif height >= 2.2 and (long_side >= 2.0 or width * depth >= 4.0):
            category = "building"
        else:
            continue
        nodes.append({
            "id": f"{category}-{len(nodes) + 1}",
            "category": category,
            "position": (min_x + (cx + 0.5) * resolution, min_y + (cy + 0.5) * resolution, ground_z),
            "dimensions": (width, depth, max(0.6, min(height, 30.0))),
            "yaw": math.radians(angle),
            "confidence": 0.72 if category == "building" else 0.62,
            "source": "geometry",
        })
    return nodes


def _overlaps(a: dict, b: dict, margin: float = 0.0) -> bool:
    ax, ay, _ = a["position"]
    bx, by, _ = b["position"]
    aw, ad, _ = a["dimensions"]
    bw, bd, _ = b["dimensions"]
    return abs(ax - bx) < (aw + bw) / 2 + margin and abs(ay - by) < (ad + bd) / 2 + margin


def _extract_geometry(points: np.ndarray, trajectory: np.ndarray, resolution: float) -> tuple[list[dict], dict]:
    low = np.percentile(points, 0.5, axis=0)
    high = np.percentile(points, 99.5, axis=0)
    min_x, min_y = float(low[0]), float(low[1])
    span_x, span_y = max(1.0, float(high[0] - low[0])), max(1.0, float(high[1] - low[1]))
    resolution = max(float(resolution), math.sqrt(span_x * span_y / 3_000_000))
    width = int(math.ceil(span_x / resolution)) + 1
    height = int(math.ceil(span_y / resolution)) + 1
    inside = (
        (points[:, 0] >= min_x) & (points[:, 0] <= min_x + width * resolution) &
        (points[:, 1] >= min_y) & (points[:, 1] <= min_y + height * resolution)
    )
    points = points[inside]
    columns = np.clip(((points[:, 0] - min_x) / resolution).astype(np.int32), 0, width - 1)
    rows = np.clip(((points[:, 1] - min_y) / resolution).astype(np.int32), 0, height - 1)
    min_z = np.full((height, width), np.inf, dtype=np.float32)
    max_z = np.full((height, width), -np.inf, dtype=np.float32)
    counts = np.zeros((height, width), dtype=np.int32)
    np.minimum.at(min_z, (rows, columns), points[:, 2])
    np.maximum.at(max_z, (rows, columns), points[:, 2])
    np.add.at(counts, (rows, columns), 1)
    occupied = counts > 0
    vertical_span = max_z - min_z
    ground_cells = occupied & (vertical_span <= 0.4)
    ground_z = float(np.percentile(min_z[ground_cells], 25)) if ground_cells.any() else float(np.percentile(points[:, 2], 10))
    obstacle = occupied & (vertical_span >= 0.55)
    obstacle = cv2.morphologyEx(obstacle.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)) > 0
    objects = _component_nodes(obstacle, max_z, ground_z, min_x, min_y, resolution)

    road = cv2.morphologyEx(ground_cells.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)) > 0
    if len(trajectory):
        seed = np.zeros_like(road, dtype=np.uint8)
        for x, y in trajectory:
            column = int((x - min_x) / resolution)
            row = int((y - min_y) / resolution)
            if 0 <= row < height and 0 <= column < width:
                cv2.circle(seed, (column, row), max(2, int(1.6 / resolution)), 1, -1)
        seeded = road & (cv2.dilate(seed, np.ones((9, 9), np.uint8)) > 0)
        road = cv2.morphologyEx(seeded.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8)) > 0
    else:
        road = _largest_component(road)
    road &= cv2.dilate(obstacle.astype(np.uint8), np.ones((3, 3), np.uint8)) == 0
    road_nodes = []
    for index, item in enumerate(_mask_rectangles(road, min_x, min_y, resolution), 1):
        road_nodes.append({
            "id": f"road-{index}", "category": "road", **item, "yaw": 0.0,
            "confidence": 0.76, "source": "geometry",
        })

    accepted = []
    conflicts = 0
    for node in sorted(objects, key=lambda item: item["category"] == "tree"):
        if node["category"] == "tree" and any(_overlaps(node, other, 0.15) for other in accepted + road_nodes):
            conflicts += 1
            continue
        accepted.append(node)
    metrics = {
        "input_point_count": int(len(points)),
        "grid_resolution_m": round(resolution, 3),
        "ground_z_m": round(ground_z, 3),
        "road_area_m2": round(float(road.sum()) * resolution * resolution, 2),
        "geometry_conflicts": conflicts,
    }
    return road_nodes + accepted, metrics


def _semantic_category(item: dict) -> str:
    value = str(item.get("class_name") or item.get("asset_id") or "").lower().replace("-", "_")
    value = value.split(".", 1)[0]
    return {"drivable_flat": "road", "non_drivable_flat": "road", "vertical_thin": "wall", "groundcover": "vegetation"}.get(value, value)


def _semantic_center(item: dict) -> tuple[float, float, float]:
    bbox = item.get("bbox") or {}
    minimum, maximum = bbox.get("min"), bbox.get("max")
    if isinstance(minimum, list) and isinstance(maximum, list) and len(minimum) >= 3 and len(maximum) >= 3:
        return tuple((float(minimum[index]) + float(maximum[index])) / 2 for index in range(3))
    value = item.get("position") or {}
    if isinstance(value, (list, tuple)):
        return tuple(float(value[index]) if index < len(value) else 0.0 for index in range(3))
    return tuple(float(value.get(axis) or 0.0) for axis in ("x", "y", "z"))


def _merge_semantics(nodes: list[dict], semantics: dict | None) -> tuple[list[dict], int]:
    if not semantics:
        return nodes, 0
    applied = 0
    for item in semantics.get("instances", []):
        confidence = float(item.get("confidence") or 0.0)
        category = _semantic_category(item)
        if confidence < 0.8 or category not in DEFAULT_COLORS:
            continue
        x, y, _ = _semantic_center(item)
        candidates = [node for node in nodes if node["category"] != "road"]
        nearest = min(candidates, key=lambda node: math.hypot(node["position"][0] - x, node["position"][1] - y), default=None)
        if nearest and math.hypot(nearest["position"][0] - x, nearest["position"][1] - y) <= 3.0:
            nearest["category"] = category
            nearest["confidence"] = confidence
            nearest["source"] = "geometry+ptv3"
            if item.get("asset_id"):
                nearest["asset_id"] = str(item["asset_id"])
            applied += 1
    return nodes, applied


def _palette_from_image(path: Path) -> dict[str, tuple[int, int, int]]:
    try:
        with Image.open(path) as image:
            image.thumbnail((320, 240))
            values = np.asarray(image.convert("RGB"), dtype=np.uint8)
    except (OSError, ValueError):
        return {}
    pixels = values.reshape(-1, 3)
    brightness = pixels.mean(axis=1)
    pixels = pixels[(brightness > 25) & (brightness < 235)]
    if not len(pixels):
        return {}
    green = pixels[(pixels[:, 1] > pixels[:, 0] * 1.08) & (pixels[:, 1] > pixels[:, 2] * 1.05)]
    spread = pixels.max(axis=1) - pixels.min(axis=1)
    neutral = pixels[spread < 55]
    dark = neutral[neutral.mean(axis=1) < 125] if len(neutral) else neutral
    building_source = neutral if len(neutral) else pixels
    building = building_source[(building_source.mean(axis=1) >= 70) & (building_source.mean(axis=1) < 220)]
    palette = {}
    if len(dark):
        palette["road"] = tuple(int(value) for value in np.median(dark, axis=0))
    if len(building):
        palette["building"] = tuple(int(value) for value in np.median(building, axis=0))
        palette["wall"] = palette["building"]
    if len(green):
        palette["tree"] = tuple(int(value) for value in np.median(green, axis=0))
        palette["vegetation"] = palette["tree"]
    return palette


def _media_palettes(references: list[str | Path]) -> list[dict[str, tuple[int, int, int]]]:
    palettes = []
    for value in references:
        path = Path(value)
        if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".webp"}:
            palette = _palette_from_image(path)
            if palette:
                palettes.append(palette)
            continue
        if path.suffix.lower() not in {".mp4", ".mov", ".mkv"}:
            continue
        capture = cv2.VideoCapture(str(path))
        fps = max(1.0, float(capture.get(cv2.CAP_PROP_FPS) or 1.0))
        frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        for frame_index in range(0, frames, max(1, int(fps * 2))):
            capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, frame = capture.read()
            if not ok:
                continue
            temporary = path.parent / f".{path.stem}-{frame_index}.jpg"
            try:
                cv2.imwrite(str(temporary), frame)
                palette = _palette_from_image(temporary)
                if palette:
                    palettes.append(palette)
            finally:
                temporary.unlink(missing_ok=True)
        capture.release()
    return palettes


def _apply_colors(nodes: list[dict], references: list[str | Path]) -> float:
    palettes = _media_palettes(references)
    measured = {}
    for category in DEFAULT_COLORS:
        samples = [palette[category] for palette in palettes if category in palette]
        if samples:
            measured[category] = tuple(int(np.median([sample[index] for sample in samples])) for index in range(3))
    coloured = 0
    for node in nodes:
        category = node["category"]
        node["color"] = measured.get(category, DEFAULT_COLORS.get(category, (150, 150, 150)))
        node["color_source"] = "reference_media" if category in measured else "default_material"
        coloured += category in measured
    return coloured / max(1, len(nodes))


def _write_glb(path: Path, nodes: list[dict]) -> None:
    vertices = np.asarray([
        [-.5, -.5, 0], [.5, -.5, 0], [.5, .5, 0], [-.5, .5, 0],
        [-.5, -.5, 1], [.5, -.5, 1], [.5, .5, 1], [-.5, .5, 1],
    ], dtype="<f4")
    indices = np.asarray([
        0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
        1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7,
    ], dtype="<u2")
    binary = vertices.tobytes() + indices.tobytes()
    binary += b"\0" * (-len(binary) % 4)
    materials, material_indices = [], {}
    gltf_nodes = []
    for node in nodes:
        color = tuple(node["color"])
        if color not in material_indices:
            material_indices[color] = len(materials)
            materials.append({
                "pbrMetallicRoughness": {
                    "baseColorFactor": [value / 255 for value in color] + [1],
                    "metallicFactor": 0,
                    "roughnessFactor": 0.9,
                },
                "extensions": {"KHR_materials_unlit": {}},
            })
        x, y, z = node["position"]
        half_yaw = node.get("yaw", 0.0) / 2
        gltf_nodes.append({
            "name": node["id"], "mesh": material_indices[color],
            "translation": [x, y, z],
            "rotation": [0, 0, math.sin(half_yaw), math.cos(half_yaw)],
            "scale": list(node["dimensions"]),
            "extras": {key: node.get(key) for key in ("category", "confidence", "source", "color_source")},
        })
    meshes = [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1, "material": index}]} for index in range(max(1, len(materials)))]
    document = {
        "asset": {"version": "2.0", "generator": "RoamerX code scene converter"},
        "extensionsUsed": ["KHR_materials_unlit"],
        "scene": 0,
        "scenes": [{"nodes": list(range(len(gltf_nodes)))}],
        "nodes": gltf_nodes,
        "meshes": meshes,
        "materials": materials or [{"pbrMetallicRoughness": {"baseColorFactor": [.6, .6, .6, 1]}}],
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": vertices.nbytes, "target": 34962},
            {"buffer": 0, "byteOffset": vertices.nbytes, "byteLength": indices.nbytes, "target": 34963},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": 8, "type": "VEC3", "min": [-.5, -.5, 0], "max": [.5, .5, 1]},
            {"bufferView": 1, "componentType": 5123, "count": len(indices), "type": "SCALAR"},
        ],
    }
    encoded = json.dumps(document, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    encoded += b" " * (-len(encoded) % 4)
    total = 12 + 8 + len(encoded) + 8 + len(binary)
    with path.open("wb") as stream:
        stream.write(struct.pack("<4sII", b"glTF", 2, total))
        stream.write(struct.pack("<I4s", len(encoded), b"JSON"))
        stream.write(encoded)
        stream.write(struct.pack("<I4s", len(binary), b"BIN\0"))
        stream.write(binary)


def convert_scene(
    point_cloud: str | Path,
    output_dir: str | Path,
    *,
    references: list[str | Path] | None = None,
    trajectory: str | Path | None = None,
    semantics: dict | None = None,
    grid_resolution_m: float = 0.35,
    max_points: int = 1_000_000,
) -> tuple[Path, dict]:
    cloud_path = Path(point_cloud)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    points = read_pcd_xyz(cloud_path, max_points=max_points)
    nodes, metrics = _extract_geometry(points, _trajectory_points(trajectory), grid_resolution_m)
    nodes, semantic_matches = _merge_semantics(nodes, semantics)
    color_coverage = _apply_colors(nodes, list(references or []))
    artifact = output / "street_block.glb"
    _write_glb(artifact, nodes)
    metrics.update({
        "node_count": len(nodes),
        "road_mesh_count": sum(node["category"] == "road" for node in nodes),
        "building_count": sum(node["category"] == "building" for node in nodes),
        "tree_count": sum(node["category"] == "tree" for node in nodes),
        "ptv3_matches": semantic_matches,
        "color_coverage": color_coverage,
    })
    manifest = {
        "schema": SCHEMA,
        "generator": "server_code",
        "source_cloud_sha256": sha256_path(cloud_path),
        "semantic_source": "ptv3+geometry" if semantic_matches else "geometry",
        "node_count": len(nodes),
        "building_count": metrics["building_count"],
        "road_mesh_count": metrics["road_mesh_count"],
        "review_count": 0,
        "quality": metrics,
        "nodes": [
            {
                **node,
                "position": list(node["position"]),
                "dimensions": list(node["dimensions"]),
                "color": list(node["color"]),
            }
            for node in nodes
        ],
    }
    (output / "street_block_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return artifact, manifest
