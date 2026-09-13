"""Build a compact, human-facing street-block GLB from PTv3 instances."""

from __future__ import annotations

import csv
import json
import math
import os
import struct
import subprocess
from pathlib import Path

import numpy as np


DEFAULT_COLORS = {
    "road": (74, 85, 96),
    "building": (184, 168, 145),
    "tree": (66, 135, 76),
    "vegetation": (80, 145, 75),
    "wall": (154, 145, 132),
    "barrier": (217, 147, 63),
    "traffic_cone": (239, 105, 32),
    "debris": (112, 102, 90),
}
DEFAULT_DIMS = {
    "road": (4.0, 3.0, 0.08),
    "building": (4.5, 3.5, 3.2),
    "tree": (2.8, 2.8, 4.6),
    "vegetation": (1.2, 1.2, 0.25),
    "wall": (2.5, 0.18, 1.8),
    "barrier": (1.8, 0.35, 0.9),
    "traffic_cone": (0.35, 0.35, 0.7),
    "debris": (0.8, 0.8, 0.35),
}


def _category(item: dict) -> str:
    value = str(item.get("class_name") or item.get("asset_id") or "").lower().replace("-", "_")
    aliases = {
        "drivable_flat": "road", "non_drivable_flat": "road", "vertical_thin": "wall",
        "groundcover": "vegetation", "static_clutter": "debris",
    }
    head = value.split(".", 1)[0]
    return aliases.get(value, aliases.get(head, head))


def _position(item: dict) -> tuple[float, float, float]:
    bbox = item.get("bbox") or {}
    minimum, maximum = bbox.get("min"), bbox.get("max")
    if isinstance(minimum, list) and isinstance(maximum, list) and len(minimum) >= 3 and len(maximum) >= 3:
        return ((float(minimum[0]) + float(maximum[0])) / 2, (float(minimum[1]) + float(maximum[1])) / 2, float(minimum[2]))
    value = item.get("position") or {}
    return tuple(float(value.get(axis) or 0.0) for axis in ("x", "y", "z"))


def _yaw(item: dict) -> float:
    q = item.get("orientation") or {}
    z, w = float(q.get("z") or 0.0), float(q.get("w") or 1.0)
    return 2.0 * math.atan2(z, w)


def _dimensions(item: dict, category: str) -> tuple[float, float, float]:
    bbox = item.get("bbox") or {}
    minimum, maximum = bbox.get("min"), bbox.get("max")
    if isinstance(minimum, list) and isinstance(maximum, list) and len(minimum) >= 3 and len(maximum) >= 3:
        dims = tuple(max(0.05, float(maximum[i]) - float(minimum[i])) for i in range(3))
    else:
        dims = DEFAULT_DIMS.get(category, (1.0, 1.0, 1.0))
    if category == "road":
        length, width = max(dims[0], dims[1]), min(dims[0], dims[1])
        dims = (max(1.2, min(12.0, length)), max(1.2, min(8.0, width)), 0.08)
    elif category == "building":
        dims = (max(2.0, min(40.0, dims[0])), max(2.0, min(40.0, dims[1])), max(2.5, min(30.0, dims[2])))
    elif category == "vegetation":
        dims = (max(0.5, min(5.0, dims[0])), max(0.5, min(5.0, dims[1])), min(0.35, dims[2]))
    elif category == "wall":
        length, width = max(dims[0], dims[1]), min(dims[0], dims[1])
        dims = (max(0.5, min(20.0, length)), max(0.12, min(0.5, width)), max(0.5, min(4.0, dims[2])))
    scale = item.get("scale", 1.0)
    if isinstance(scale, (int, float)):
        return tuple(value * max(0.05, float(scale)) for value in dims)
    return dims


def _dominant_color(path: Path) -> tuple[int, int, int] | None:
    try:
        from PIL import Image, ImageStat
        with Image.open(path) as image:
            image.thumbnail((160, 160))
            rgb = image.convert("RGB")
            values = np.asarray(rgb, dtype=np.uint8).reshape(-1, 3)
            brightness = values.mean(axis=1)
            values = values[(brightness > 25) & (brightness < 235)]
            if not len(values):
                return None
            median = np.median(values, axis=0)
            return tuple(int(value) for value in median)
    except (OSError, ValueError):
        return None


def _trajectory(path: Path) -> list[tuple[float, float, float]]:
    if not path.is_file():
        return []
    rows = []
    try:
        with path.open(newline="", encoding="utf-8-sig") as stream:
            for raw in csv.DictReader(stream):
                rows.append((float(raw.get("timestamp") or len(rows)), float(raw["x"]), float(raw["y"])))
    except (OSError, KeyError, TypeError, ValueError):
        return []
    return rows


def _tum_trajectory(path: Path) -> list[tuple[float, float, float]]:
    rows = []
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            values = line.strip().split()
            if len(values) >= 4 and not line.lstrip().startswith("#"):
                rows.append((float(values[0]), float(values[1]), float(values[2])))
    except (OSError, ValueError):
        return []
    return rows


def _export_rtab_references(root: Path) -> Path | None:
    databases = [path for path in (root / "references").glob("*") if path.suffix.lower() in {".db", ".db3"}]
    if not databases:
        return None
    binary = Path("/opt/ros/humble/bin/rtabmap-export")
    if not binary.is_file():
        return None
    output = root / "rtab-export"
    output.mkdir(exist_ok=True)
    environment = os.environ.copy()
    environment.update({"QT_QPA_PLATFORM": "offscreen", "DISPLAY": ""})
    try:
        subprocess.run([
            str(binary), "--output", "scene_reference", "--output_dir", str(output),
            "--opt", "2", "--poses_camera", "--poses_format", "10", "--images", str(databases[0]),
        ], env=environment, capture_output=True, text=True, timeout=1800, check=True)
    except (OSError, subprocess.SubprocessError):
        return None
    return next(iter(sorted(output.rglob("*.txt"))), None)


def _color_samples(root: Path) -> list[tuple[float, float, tuple[int, int, int]]]:
    exported_poses = _export_rtab_references(root)
    references = sorted((root / "references").glob("*")) + sorted((root / "rtab-export").rglob("*"))
    poses = _trajectory(root / "trajectory.csv")
    if not poses and exported_poses:
        poses = _tum_trajectory(exported_poses)
    samples = []
    images = [path for path in references if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".webp"}]
    for index, path in enumerate(images):
        color = _dominant_color(path)
        if color:
            pose = poses[min(index, len(poses) - 1)] if poses else (0.0, math.nan, math.nan)
            samples.append((pose[1], pose[2], color))
    for video in (path for path in references if path.suffix.lower() in {".mp4", ".mov", ".mkv"}):
        try:
            import cv2
            capture = cv2.VideoCapture(str(video))
            fps = max(1.0, float(capture.get(cv2.CAP_PROP_FPS) or 1.0))
            frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            for frame_index in range(0, frames, max(1, int(fps * 2.0))):
                capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
                ok, frame = capture.read()
                if not ok:
                    continue
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                values = rgb.reshape(-1, 3)
                brightness = values.mean(axis=1)
                values = values[(brightness > 25) & (brightness < 235)]
                if not len(values):
                    continue
                color = tuple(int(value) for value in np.median(values, axis=0))
                pose = poses[min(int((frame_index / max(1, frames - 1)) * (len(poses) - 1)), len(poses) - 1)] if poses else (0.0, math.nan, math.nan)
                samples.append((pose[1], pose[2], color))
            capture.release()
        except (ImportError, OSError, ValueError):
            continue
    return samples


def _material_color(category: str, x: float, y: float, samples) -> tuple[int, int, int]:
    base = DEFAULT_COLORS.get(category, (150, 150, 150))
    if not samples:
        return base
    positioned = [sample for sample in samples if math.isfinite(sample[0]) and math.isfinite(sample[1])]
    if positioned:
        nearest = min(positioned, key=lambda sample: math.hypot(sample[0] - x, sample[1] - y))
        if math.hypot(nearest[0] - x, nearest[1] - y) > 25.0:
            return base
    else:
        nearest = (math.nan, math.nan, tuple(int(np.median([sample[2][i] for sample in samples])) for i in range(3)))
    # Preserve category readability while using the measured local scene colour.
    return tuple(int(base[i] * 0.35 + nearest[2][i] * 0.65) for i in range(3))


def _overlaps(a: dict, b: dict, margin: float = 0.0) -> bool:
    ax, ay, _ = a["position"]
    bx, by, _ = b["position"]
    adx, ady, _ = a["dimensions"]
    bdx, bdy, _ = b["dimensions"]
    return abs(ax - bx) < (adx + bdx) / 2 + margin and abs(ay - by) < (ady + bdy) / 2 + margin


def _nodes(semantics: dict, root: Path) -> tuple[list[dict], int]:
    samples = _color_samples(root)
    road_ground = []
    for item in semantics.get("instances", []):
        if _category(item) != "road":
            continue
        minimum = (item.get("bbox") or {}).get("min")
        if isinstance(minimum, list) and len(minimum) >= 3:
            road_ground.append(float(minimum[2]))
    ground_z = float(np.percentile(road_ground, 10)) if road_ground else 0.0
    nodes = []
    conflicts = 0
    for item in semantics.get("instances", []):
        category = _category(item)
        if category not in DEFAULT_DIMS:
            continue
        position = _position(item)
        if category == "road":
            position = (position[0], position[1], ground_z)
        node = {
            "id": str(item.get("id") or f"{category}-{len(nodes) + 1}"),
            "category": category,
            "position": position,
            "dimensions": _dimensions(item, category),
            "yaw": _yaw(item),
            "confidence": float(item.get("confidence") or 0.0),
            "color": _material_color(category, position[0], position[1], samples),
            "color_source": "reference_media" if samples else "default_asset",
        }
        if category in {"tree", "vegetation"} and any(
            other["category"] in {"road", "building", "tree"} and _overlaps(node, other, 0.1)
            for other in nodes
        ):
            conflicts += 1
            continue
        nodes.append(node)

    # Join nearby, direction-compatible road observations with explicit meshes.
    roads = [node for node in nodes if node["category"] == "road"]
    joined = set()
    for road in roads:
        candidates = sorted(
            (other for other in roads if other is not road),
            key=lambda other: math.hypot(other["position"][0] - road["position"][0], other["position"][1] - road["position"][1]),
        )[:2]
        for other in candidates:
            pair = tuple(sorted((road["id"], other["id"])))
            dx = other["position"][0] - road["position"][0]
            dy = other["position"][1] - road["position"][1]
            distance = math.hypot(dx, dy)
            line_yaw = math.atan2(dy, dx)
            alignment = abs(math.cos(line_yaw - road["yaw"]))
            if pair in joined or not 2.5 < distance <= 8.0 or alignment < 0.7:
                continue
            joined.add(pair)
            nodes.append({
                "id": f"road-link-{len(joined)}", "category": "road",
                "position": ((road["position"][0] + other["position"][0]) / 2, (road["position"][1] + other["position"][1]) / 2, min(road["position"][2], other["position"][2])),
                "dimensions": (distance + 0.2, min(road["dimensions"][1], other["dimensions"][1]), 0.08),
                "yaw": line_yaw, "confidence": min(road["confidence"], other["confidence"]),
                "color": road["color"], "color_source": road["color_source"],
            })
    return nodes, conflicts


def _write_glb(path: Path, nodes: list[dict]) -> None:
    vertices = np.asarray([
        [-.5, -.5, 0], [.5, -.5, 0], [.5, .5, 0], [-.5, .5, 0],
        [-.5, -.5, 1], [.5, -.5, 1], [.5, .5, 1], [-.5, .5, 1],
    ], dtype="<f4")
    indices = np.asarray([
        0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
        0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5,
        2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7,
    ], dtype="<u2")
    binary = vertices.tobytes() + indices.tobytes()
    while len(binary) % 4:
        binary += b"\0"
    materials, material_index = [], {}
    gltf_nodes = []
    for node in nodes:
        color = node["color"]
        if color not in material_index:
            material_index[color] = len(materials)
            materials.append({"pbrMetallicRoughness": {"baseColorFactor": [value / 255 for value in color] + [1], "metallicFactor": 0, "roughnessFactor": .9}, "extensions": {"KHR_materials_unlit": {}}})
        x, y, z = node["position"]
        half = node["yaw"] / 2
        gltf_nodes.append({
            "name": node["id"], "mesh": 0, "translation": [x, y, z],
            "rotation": [0, 0, math.sin(half), math.cos(half)], "scale": list(node["dimensions"]),
            "extras": {key: node[key] for key in ("category", "confidence", "color_source")},
        })
    meshes = [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1, "material": index}]} for index in range(max(1, len(materials)))]
    for index, node in enumerate(gltf_nodes):
        node["mesh"] = material_index[nodes[index]["color"]]
    document = {
        "asset": {"version": "2.0", "generator": "RoamerX street-block builder"},
        "extensionsUsed": ["KHR_materials_unlit"],
        "scene": 0, "scenes": [{"nodes": list(range(len(gltf_nodes)))}], "nodes": gltf_nodes,
        "meshes": meshes, "materials": materials or [{"pbrMetallicRoughness": {"baseColorFactor": [.6, .6, .6, 1]}}],
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": vertices.nbytes, "target": 34962}, {"buffer": 0, "byteOffset": vertices.nbytes, "byteLength": indices.nbytes, "target": 34963}],
        "accessors": [{"bufferView": 0, "componentType": 5126, "count": 8, "type": "VEC3", "min": [-.5, -.5, 0], "max": [.5, .5, 1]}, {"bufferView": 1, "componentType": 5123, "count": len(indices), "type": "SCALAR"}],
    }
    encoded = json.dumps(document, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    encoded += b" " * (-len(encoded) % 4)
    total = 12 + 8 + len(encoded) + 8 + len(binary)
    with path.open("wb") as stream:
        stream.write(struct.pack("<4sII", b"glTF", 2, total))
        stream.write(struct.pack("<I4s", len(encoded), b"JSON")); stream.write(encoded)
        stream.write(struct.pack("<I4s", len(binary), b"BIN\0")); stream.write(binary)


def build_street_block(map_dir: str | Path, semantics: dict) -> tuple[Path, dict]:
    root = Path(map_dir)
    nodes, conflicts = _nodes(semantics, root)
    destination = root / "street_block.glb"
    _write_glb(destination, nodes)
    buildings = sum(node["category"] == "building" for node in nodes)
    roads = sum(node["category"] == "road" for node in nodes)
    coloured = sum(node["color_source"] == "reference_media" for node in nodes)
    manifest = {
        "schema": "roamerx.street-block.v1", "node_count": len(nodes),
        "building_count": buildings, "road_mesh_count": roads,
        "review_count": conflicts + len(semantics.get("review_candidates", [])),
        "quality": {
            "publish_precision_target": 0.90,
            "publish_precision_verified": False,
            "geometry_conflicts": conflicts,
            "color_coverage": coloured / max(1, len(nodes)),
        },
        "nodes": [{key: value for key, value in node.items() if key not in {"position", "dimensions", "yaw"}} for node in nodes],
    }
    return destination, manifest
