from __future__ import annotations

import csv
import json
import math
import shutil
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .map_cleaner import _read_pcd_header, _read_pgm, _write_binary_pcd, _write_pgm
from .map_preview import generate_map_preview


@dataclass(frozen=True)
class SubmapBuildConfig:
    segment_length_m: float = 250.0
    step_length_m: float = 190.0
    margin_m: float = 12.0


def build_map_set(source: Path, output: Path, *, config: SubmapBuildConfig) -> dict:
    """Create overlapping navigation submaps from a completed SLAM session.

    Keyframe positions define distance along the manually driven trajectory.
    The final PCD/PGM are cropped in the existing SLAM map frame, so existing
    NDT and Nav2 still consume the standard map files without a new format.
    """
    source = source.resolve()
    output.mkdir(parents=True, exist_ok=True)
    keyframes = _load_keyframes(source / "keyframes" / "keyframes.csv")
    if len(keyframes) < 2:
        raise RuntimeError("at least two keyframes are required to build a map set")
    if config.segment_length_m <= 0 or config.step_length_m <= 0 or config.step_length_m > config.segment_length_m:
        raise RuntimeError("invalid submap segment/step configuration")

    _, dtype, offset = _read_pcd_header(source / "map.pcd")
    points = np.fromfile(source / "map.pcd", dtype=dtype, offset=offset)
    pgm, maxval, _ = _read_pgm(source / "map.pgm")
    yaml_data = yaml.safe_load((source / "map.yaml").read_text(encoding="utf-8")) or {}
    resolution = float(yaml_data.get("resolution") or 0.05)
    origin = list(yaml_data.get("origin") or [0.0, 0.0, 0.0])
    if len(origin) < 3:
        origin = [*origin, *([0.0] * (3 - len(origin)))]

    total_distance = keyframes[-1]["distance_m"]
    starts = list(np.arange(0.0, total_distance + 0.001, config.step_length_m))
    if starts and starts[-1] + config.segment_length_m < total_distance:
        starts.append(max(0.0, total_distance - config.segment_length_m))
    submaps = []
    for index, start_m in enumerate(starts, start=1):
        end_m = min(total_distance, start_m + config.segment_length_m)
        frame_indexes = [item["index"] for item in keyframes if start_m <= item["distance_m"] <= end_m]
        if not frame_indexes:
            continue
        selected = [item for item in keyframes if item["index"] in frame_indexes]
        xs = [item["x"] for item in selected]
        ys = [item["y"] for item in selected]
        bounds = {
            "min_x": min(xs) - config.margin_m,
            "max_x": max(xs) + config.margin_m,
            "min_y": min(ys) - config.margin_m,
            "max_y": max(ys) + config.margin_m,
        }
        submap_id = f"submap_{index:03d}"
        subdir = output / submap_id
        subdir.mkdir(parents=True, exist_ok=True)
        keep = (
            np.isfinite(points["x"])
            & np.isfinite(points["y"])
            & (points["x"] >= bounds["min_x"])
            & (points["x"] <= bounds["max_x"])
            & (points["y"] >= bounds["min_y"])
            & (points["y"] <= bounds["max_y"])
        )
        header, _, _ = _read_pcd_header(source / "map.pcd")
        _write_binary_pcd(subdir / "map.pcd", source / "map.pcd", header["lines"], points[keep])
        _crop_grid(pgm, maxval, resolution, origin, bounds, subdir / "map.pgm", subdir / "map.yaml")
        _copy_keyframes(source, subdir, selected)
        generate_map_preview(subdir / "map.pgm", subdir / "map_preview.png")
        metadata = {
            "submap_id": submap_id,
            "frame_id": "map",
            "distance_range_m": [round(start_m, 3), round(end_m, 3)],
            "bounds": {key: round(value, 3) for key, value in bounds.items()},
            "keyframe_indexes": frame_indexes,
            "point_count": int(keep.sum()),
            "origin_enu": _gnss_origin(source),
            "neighbors": [f"submap_{index - 1:03d}" if index > 1 else None, f"submap_{index + 1:03d}" if end_m < total_distance else None],
        }
        metadata["neighbors"] = [item for item in metadata["neighbors"] if item]
        (subdir / "submap.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        submaps.append(metadata)

    manifest = {
        "format": "roamerx.map_set.v1",
        "frame_id": "map",
        "source_map_dir": str(source),
        "total_distance_m": round(total_distance, 3),
        "segment_length_m": config.segment_length_m,
        "step_length_m": config.step_length_m,
        "overlap_m": round(config.segment_length_m - config.step_length_m, 3),
        "origin_enu": _gnss_origin(source),
        "submaps": submaps,
    }
    (output / "map_set_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def _load_keyframes(path: Path) -> list[dict]:
    rows = []
    cumulative = 0.0
    previous = None
    with path.open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            item = {"index": int(row["index"]), "stamp": float(row["stamp"]), "x": float(row["x"]), "y": float(row["y"]), "z": float(row["z"])}
            if previous:
                cumulative += math.hypot(item["x"] - previous["x"], item["y"] - previous["y"])
            item["distance_m"] = cumulative
            rows.append(item)
            previous = item
    return rows


def _crop_grid(image, maxval: int, resolution: float, origin: list[float], bounds: dict, pgm_path: Path, yaml_path: Path) -> None:
    height, width = image.shape
    col0 = max(0, int(math.floor((bounds["min_x"] - origin[0]) / resolution)))
    col1 = min(width, int(math.ceil((bounds["max_x"] - origin[0]) / resolution)))
    bottom0 = max(0, int(math.floor((bounds["min_y"] - origin[1]) / resolution)))
    bottom1 = min(height, int(math.ceil((bounds["max_y"] - origin[1]) / resolution)))
    if col1 <= col0 or bottom1 <= bottom0:
        raise RuntimeError("submap bounds do not intersect occupancy grid")
    row0, row1 = height - bottom1, height - bottom0
    cropped = image[row0:row1, col0:col1]
    _write_pgm(pgm_path, cropped, maxval, b"P5\n%d %d\n%d\n" % (cropped.shape[1], cropped.shape[0], maxval))
    yaml_path.write_text(
        "image: map.pgm\nresolution: %.8f\norigin: [%.8f, %.8f, %.8f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        % (resolution, origin[0] + col0 * resolution, origin[1] + bottom0 * resolution, origin[2]),
        encoding="utf-8",
    )


def _copy_keyframes(source: Path, destination: Path, keyframes: list[dict]) -> None:
    source_dir = source / "keyframes"
    destination_dir = destination / "keyframes"
    destination_dir.mkdir(exist_ok=True)
    with (destination_dir / "keyframes.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["index", "stamp", "x", "y", "z"])
        writer.writeheader()
        for item in keyframes:
            writer.writerow({key: item[key] for key in writer.fieldnames})
            shutil.copy2(source_dir / f"scan_{item['index']:05d}.pcd", destination_dir / f"scan_{item['index']:05d}.pcd")


def _gnss_origin(source: Path) -> dict:
    path = source / "gnss_origin.yaml"
    if not path.exists():
        return {}
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
