from __future__ import annotations

import hashlib
import json
import math
import shutil
import tempfile
from pathlib import Path
from urllib.parse import urlparse

import numpy as np
import requests
import yaml

from .map_cleaner import _read_pcd_header, _read_pgm, _write_binary_pcd


class ManualMapCleanupError(RuntimeError):
    pass


def build_manual_cleanup_map(
    source_dir: Path,
    output_dir: Path,
    *,
    pgm_url: str,
    yaml_url: str,
    pgm_sha256: str = "",
    yaml_sha256: str = "",
) -> dict:
    """Build an edited local map without modifying the original SLAM output."""
    for name in ("map.pgm", "map.yaml", "map.pcd"):
        if not (source_dir / name).is_file():
            raise ManualMapCleanupError(f"source map is missing {name}: {source_dir}")

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temp_dir = Path(tempfile.mkdtemp(prefix=f".{output_dir.name}-", dir=output_dir.parent))
    try:
        _download(pgm_url, temp_dir / "map.pgm", pgm_sha256)
        _download(yaml_url, temp_dir / "map.yaml", yaml_sha256)
        result = _filter_point_cloud(source_dir, temp_dir)
        for name in ("map.txt", "gnss_origin.yaml"):
            if (source_dir / name).is_file():
                shutil.copy2(source_dir / name, temp_dir / name)
        (temp_dir / "manual_cleanup.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        backup = output_dir.with_name(f".{output_dir.name}.previous")
        if backup.exists():
            shutil.rmtree(backup)
        if output_dir.exists():
            output_dir.replace(backup)
        temp_dir.replace(output_dir)
        if backup.exists():
            shutil.rmtree(backup)
        return result
    except ManualMapCleanupError:
        shutil.rmtree(temp_dir, ignore_errors=True)
        raise
    except Exception as exc:
        shutil.rmtree(temp_dir, ignore_errors=True)
        raise ManualMapCleanupError(f"failed to build manual cleanup map: {exc}") from exc


def _download(url: str, destination: Path, expected_sha256: str) -> None:
    parsed = urlparse(url)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ManualMapCleanupError(f"invalid map URL: {url}")
    digest = hashlib.sha256()
    try:
        with requests.get(url, stream=True, timeout=(10, 120)) as response:
            response.raise_for_status()
            with destination.open("wb") as stream:
                for chunk in response.iter_content(chunk_size=1024 * 1024):
                    if chunk:
                        stream.write(chunk)
                        digest.update(chunk)
    except requests.RequestException as exc:
        destination.unlink(missing_ok=True)
        raise ManualMapCleanupError(f"failed to download {url}: {exc}") from exc
    if expected_sha256 and digest.hexdigest().lower() != expected_sha256.lower():
        destination.unlink(missing_ok=True)
        raise ManualMapCleanupError(f"SHA256 mismatch for {url}")


def _filter_point_cloud(source_dir: Path, output_dir: Path) -> dict:
    source_image, source_maxval, _ = _read_pgm(source_dir / "map.pgm")
    edited_image, edited_maxval, _ = _read_pgm(output_dir / "map.pgm")
    if source_image.shape != edited_image.shape:
        raise ManualMapCleanupError(
            f"edited PGM dimensions {edited_image.shape} do not match source {source_image.shape}"
        )

    metadata = yaml.safe_load((output_dir / "map.yaml").read_text(encoding="utf-8")) or {}
    resolution = float(metadata.get("resolution") or 0.0)
    origin = metadata.get("origin") or [0.0, 0.0, 0.0]
    if resolution <= 0 or len(origin) < 3:
        raise ManualMapCleanupError("edited map YAML has invalid resolution or origin")

    free_floor = max(1, int(edited_maxval * 0.95))
    erased = (edited_image >= free_floor) & (source_image < edited_image)
    header, dtype, offset = _read_pcd_header(source_dir / "map.pcd")
    if header["data"] != "binary":
        raise ManualMapCleanupError(f"unsupported PCD DATA {header['data']}")
    points = np.fromfile(source_dir / "map.pcd", dtype=dtype, offset=offset)
    for field in ("x", "y", "z"):
        if field not in points.dtype.names:
            raise ManualMapCleanupError(f"map.pcd missing required field {field}")

    keep = np.ones(points.shape[0], dtype=bool)
    finite = np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])
    indexes = np.flatnonzero(finite)
    if indexes.size and np.any(erased):
        origin_x, origin_y, origin_yaw = (float(origin[0]), float(origin[1]), float(origin[2]))
        dx = points["x"][indexes].astype(np.float64) - origin_x
        dy = points["y"][indexes].astype(np.float64) - origin_y
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        cols = np.floor(local_x / resolution).astype(np.int64)
        rows = edited_image.shape[0] - 1 - np.floor(local_y / resolution).astype(np.int64)
        valid = (
            (rows >= 0)
            & (rows < edited_image.shape[0])
            & (cols >= 0)
            & (cols < edited_image.shape[1])
        )
        projected = indexes[valid]
        keep[projected] = ~erased[rows[valid], cols[valid]]

    _write_binary_pcd(output_dir / "map.pcd", source_dir / "map.pcd", header["lines"], points[keep])
    return {
        "mode": "manual_cleanup",
        "source_dir": str(source_dir),
        "output_dir": str(output_dir),
        "erased_cells": int(np.count_nonzero(erased)),
        "source_points": int(points.size),
        "kept_points": int(np.count_nonzero(keep)),
        "removed_points": int(np.count_nonzero(~keep)),
        "source_maxval": int(source_maxval),
    }
