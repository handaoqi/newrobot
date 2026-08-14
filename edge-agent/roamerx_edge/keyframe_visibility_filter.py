from __future__ import annotations

import csv
import json
import shutil
from collections import Counter
from pathlib import Path

import numpy as np

from .map_cleaner import _clear_removed_points_from_pgm, _read_pcd_header, _write_binary_pcd
from .map_preview import generate_map_preview


def filter_with_keyframe_visibility(source: Path, output: Path, *, voxel_size_m: float = 0.3, min_free_observations: int = 1, max_hit_observations: int = 10) -> dict:
    """Remove map voxels hit once but traversed by later keyframe rays."""
    source = source.resolve()
    output.mkdir(parents=True, exist_ok=True)
    keyframe_dir = source / "keyframes"
    pose_rows = list(csv.DictReader((keyframe_dir / "keyframes.csv").open(encoding="utf-8")))
    if not pose_rows:
        raise RuntimeError("keyframes/keyframes.csv is empty or missing")

    pcd_source = source / "map.pcd"
    header, dtype, offset = _read_pcd_header(pcd_source)
    points = np.fromfile(pcd_source, dtype=dtype, offset=offset)
    hits: Counter[tuple[int, int, int]] = Counter()
    frees: Counter[tuple[int, int, int]] = Counter()

    for row in pose_rows:
        cloud_path = keyframe_dir / f"scan_{int(row['index']):05d}.pcd"
        cloud_header, cloud_dtype, cloud_offset = _read_pcd_header(cloud_path)
        cloud = np.fromfile(cloud_path, dtype=cloud_dtype, offset=cloud_offset)
        origin = np.array([float(row["x"]), float(row["y"]), float(row["z"])], dtype=np.float32)
        xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"]))
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        # One ray per occupied voxel is sufficient and bounds offline processing time.
        endpoint_voxels = np.floor(xyz / voxel_size_m).astype(np.int32)
        _, indexes = np.unique(endpoint_voxels, axis=0, return_index=True)
        frame_hits: set[tuple[int, int, int]] = set()
        frame_frees: set[tuple[int, int, int]] = set()
        for endpoint in xyz[indexes]:
            delta = endpoint - origin
            distance = float(np.linalg.norm(delta))
            if distance < voxel_size_m:
                continue
            steps = int(distance / voxel_size_m)
            for step in range(1, steps):
                sample = origin + delta * (step / steps)
                frame_frees.add(tuple(np.floor(sample / voxel_size_m).astype(int)))
            frame_hits.add(tuple(np.floor(endpoint / voxel_size_m).astype(int)))
        hits.update(frame_hits)
        frees.update(frame_frees - frame_hits)

    map_voxels = np.floor(np.column_stack((points["x"], points["y"], points["z"])) / voxel_size_m).astype(np.int32)
    keep = np.array(
        [not (frees[tuple(voxel)] >= min_free_observations and hits[tuple(voxel)] <= max_hit_observations) for voxel in map_voxels],
        dtype=bool,
    )
    for name in ("map.yaml", "map.txt", "gnss_origin.yaml"):
        if (source / name).exists():
            shutil.copy2(source / name, output / name)
    shutil.copy2(source / "map.pgm", output / "map.pgm")
    _write_binary_pcd(output / "map.pcd", pcd_source, header["lines"], points[keep])
    _clear_removed_points_from_pgm(source / "map.pgm", output / "map.pgm", output / "map.yaml", points, keep, clear_radius_cells=1)
    generate_map_preview(output / "map.pgm", output / "map_preview.png")
    result = {
        "source": str(source), "output": str(output), "keyframes": len(pose_rows),
        "voxel_size_m": voxel_size_m, "min_free_observations": min_free_observations, "max_hit_observations": max_hit_observations,
        "source_points": int(points.size), "kept_points": int(keep.sum()),
        "removed_points": int((~keep).sum()), "removed_ratio": round(float((~keep).mean()), 4),
        "visibility_voxels": {"hit": len(hits), "free": len(frees)},
    }
    (output / "visibility_filter_report.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result
