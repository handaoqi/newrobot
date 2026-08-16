from __future__ import annotations

import logging
import shutil
from pathlib import Path
from typing import Any

import numpy as np
import yaml

LOGGER = logging.getLogger(__name__)


class MapCleanError(RuntimeError):
    pass


def clean_dynamic_points(
    map_dir: Path,
    *,
    voxel_size_m: float = 0.12,
    min_points_per_voxel: int = 3,
    clear_radius_cells: int = 1,
) -> dict[str, Any]:
    """Conservatively remove sparse dynamic artifacts from saved map files.

    The SLAM stack only gives us the final aggregate map here, not per-frame
    observations, so this is not a full dynamic-object segmentation algorithm.
    It removes low-repeatability 3D voxels and clears matching occupied cells
    from the 2D occupancy image.
    """
    pcd_path = map_dir / "map.pcd"
    pgm_path = map_dir / "map.pgm"
    yaml_path = map_dir / "map.yaml"
    if not pcd_path.exists():
        return {"enabled": True, "skipped": "map.pcd missing"}
    if voxel_size_m <= 0 or min_points_per_voxel <= 1:
        return {"enabled": True, "skipped": "invalid filter parameters"}

    raw_pcd_path = _backup_once(pcd_path, "map.raw_dynamic_unfiltered.pcd")
    header, dtype, data_offset = _read_pcd_header(raw_pcd_path)
    if header["data"] != "binary":
        return {"enabled": True, "skipped": f"unsupported PCD DATA {header['data']}"}

    points = np.fromfile(raw_pcd_path, dtype=dtype, offset=data_offset)
    if points.size == 0:
        return {"enabled": True, "skipped": "empty point cloud"}
    for field in ("x", "y", "z"):
        if field not in points.dtype.names:
            raise MapCleanError(f"map.pcd missing required field {field}")

    finite_mask = np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])
    ix = np.floor(points["x"][finite_mask] / voxel_size_m).astype(np.int64)
    iy = np.floor(points["y"][finite_mask] / voxel_size_m).astype(np.int64)
    iz = np.floor(points["z"][finite_mask] / voxel_size_m).astype(np.int64)
    voxel_key = _voxel_keys(ix, iy, iz)
    _, inverse, counts = np.unique(voxel_key, return_inverse=True, return_counts=True)
    finite_keep = counts[inverse] >= int(min_points_per_voxel)

    keep_mask = np.zeros(points.shape[0], dtype=bool)
    keep_mask[np.flatnonzero(finite_mask)] = finite_keep
    removed_count = int(points.shape[0] - np.count_nonzero(keep_mask))

    _write_binary_pcd(pcd_path, raw_pcd_path, header["lines"], points[keep_mask])
    cleared_cells = 0
    if pgm_path.exists() and yaml_path.exists():
        raw_pgm_path = _backup_once(pgm_path, "map.raw_dynamic_unfiltered.pgm")
        cleared_cells = _clear_removed_points_from_pgm(
            raw_pgm_path,
            pgm_path,
            yaml_path,
            points,
            keep_mask,
            clear_radius_cells=clear_radius_cells,
        )

    result = {
        "enabled": True,
        "source_points": int(points.shape[0]),
        "kept_points": int(np.count_nonzero(keep_mask)),
        "removed_points": removed_count,
        "removed_ratio": round(removed_count / max(points.shape[0], 1), 4),
        "voxel_size_m": voxel_size_m,
        "min_points_per_voxel": int(min_points_per_voxel),
        "cleared_grid_cells": int(cleared_cells),
        "raw_backups": {
            "pcd": str(raw_pcd_path),
            "pgm": str(map_dir / "map.raw_dynamic_unfiltered.pgm") if pgm_path.exists() else None,
        },
    }
    LOGGER.info("Map dynamic-point filter result: %s", result)
    return result


def _backup_once(path: Path, backup_name: str) -> Path:
    backup = path.with_name(backup_name)
    if not backup.exists():
        shutil.copy2(path, backup)
    return backup


def _read_pcd_header(path: Path) -> tuple[dict[str, Any], np.dtype, int]:
    lines: list[str] = []
    offset = 0
    with path.open("rb") as fh:
        while True:
            line = fh.readline()
            if not line:
                raise MapCleanError("PCD header missing DATA line")
            offset += len(line)
            text = line.decode("ascii", errors="strict").strip()
            lines.append(text)
            if text.startswith("DATA "):
                break

    fields: list[str] = []
    sizes: list[int] = []
    types: list[str] = []
    counts: list[int] = []
    data = ""
    for line in lines:
        if not line or line.startswith("#"):
            continue
        key, *values = line.split()
        if key == "FIELDS":
            fields = values
        elif key == "SIZE":
            sizes = [int(value) for value in values]
        elif key == "TYPE":
            types = values
        elif key == "COUNT":
            counts = [int(value) for value in values]
        elif key == "DATA":
            data = values[0].lower()
    if not counts:
        counts = [1] * len(fields)
    if not fields or len({len(fields), len(sizes), len(types), len(counts)}) != 1:
        raise MapCleanError("unsupported or malformed PCD header")

    dtype_fields: list[tuple[str, Any]] = []
    for name, size, field_type, count in zip(fields, sizes, types, counts):
        scalar = _pcd_scalar_dtype(field_type, size)
        if count == 1:
            dtype_fields.append((name, scalar))
        else:
            dtype_fields.append((name, scalar, (count,)))
    return {"lines": lines, "data": data}, np.dtype(dtype_fields), offset


def _pcd_scalar_dtype(field_type: str, size: int) -> str:
    if field_type == "F" and size == 4:
        return "<f4"
    if field_type == "F" and size == 8:
        return "<f8"
    if field_type == "I" and size in {1, 2, 4, 8}:
        return f"<i{size}"
    if field_type == "U" and size in {1, 2, 4, 8}:
        return f"<u{size}"
    raise MapCleanError(f"unsupported PCD field type {field_type}{size}")


def _voxel_keys(ix: np.ndarray, iy: np.ndarray, iz: np.ndarray) -> np.ndarray:
    if ix.size == 0:
        return np.array([], dtype=np.int64)
    min_x, min_y, min_z = int(ix.min()), int(iy.min()), int(iz.min())
    nx = int(ix.max()) - min_x + 1
    ny = int(iy.max()) - min_y + 1
    nz = int(iz.max()) - min_z + 1
    if nx <= 0 or ny <= 0 or nz <= 0 or nx * ny * nz >= np.iinfo(np.int64).max:
        structured = np.empty(ix.shape[0], dtype=[("x", "<i8"), ("y", "<i8"), ("z", "<i8")])
        structured["x"] = ix
        structured["y"] = iy
        structured["z"] = iz
        return structured
    return (ix - min_x) + nx * ((iy - min_y) + ny * (iz - min_z))


def _write_binary_pcd(path: Path, source_path: Path, header_lines: list[str], points: np.ndarray) -> None:
    updated_lines: list[str] = []
    for line in header_lines:
        if line.startswith("WIDTH "):
            updated_lines.append(f"WIDTH {points.shape[0]}")
        elif line.startswith("HEIGHT "):
            updated_lines.append("HEIGHT 1")
        elif line.startswith("POINTS "):
            updated_lines.append(f"POINTS {points.shape[0]}")
        else:
            updated_lines.append(line)

    tmp_path = path.with_suffix(".pcd.tmp")
    with tmp_path.open("wb") as fh:
        fh.write(("\n".join(updated_lines) + "\n").encode("ascii"))
        points.tofile(fh)
    tmp_path.replace(path)
    LOGGER.debug("Wrote cleaned PCD %s from raw backup %s", path, source_path)


def _clear_removed_points_from_pgm(
    raw_pgm_path: Path,
    output_pgm_path: Path,
    yaml_path: Path,
    points: np.ndarray,
    keep_mask: np.ndarray,
    *,
    clear_radius_cells: int,
) -> int:
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    resolution = float(metadata.get("resolution") or 0.05)
    origin = metadata.get("origin") or [0.0, 0.0, 0.0]
    origin_x, origin_y = float(origin[0]), float(origin[1])

    image, maxval, header = _read_pgm(raw_pgm_path)
    height, width = image.shape

    kept_cells = _project_points_to_cells(points[keep_mask], width, height, resolution, origin_x, origin_y)
    removed_cells = _project_points_to_cells(points[~keep_mask], width, height, resolution, origin_x, origin_y)
    if removed_cells.size == 0:
        _write_pgm(output_pgm_path, image, maxval, header)
        return 0

    cells_to_clear = np.setdiff1d(removed_cells, kept_cells, assume_unique=False)
    if cells_to_clear.size == 0:
        _write_pgm(output_pgm_path, image, maxval, header)
        return 0

    rows = cells_to_clear // width
    cols = cells_to_clear % width
    radius = max(0, int(clear_radius_cells))
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            rr = rows + dy
            cc = cols + dx
            valid = (rr >= 0) & (rr < height) & (cc >= 0) & (cc < width)
            image[rr[valid], cc[valid]] = maxval

    _write_pgm(output_pgm_path, image, maxval, header)
    return int(cells_to_clear.size)


def _project_points_to_cells(
    points: np.ndarray,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> np.ndarray:
    if points.size == 0:
        return np.array([], dtype=np.int64)
    cols = np.floor((points["x"] - origin_x) / resolution).astype(np.int64)
    rows_from_bottom = np.floor((points["y"] - origin_y) / resolution).astype(np.int64)
    rows = height - 1 - rows_from_bottom
    valid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    return np.unique(rows[valid] * width + cols[valid])


def _read_pgm(path: Path) -> tuple[np.ndarray, int, bytes]:
    with path.open("rb") as fh:
        magic = fh.readline()
        if magic.strip() != b"P5":
            raise MapCleanError("unsupported PGM format")
        header_parts = [magic]
        line = fh.readline()
        while line.startswith(b"#"):
            header_parts.append(line)
            line = fh.readline()
        width, height = [int(value) for value in line.split()]
        maxval_line = fh.readline()
        maxval = int(maxval_line.strip())
        if maxval > 255:
            raise MapCleanError("unsupported 16-bit PGM")
        header_parts.extend([line, maxval_line])
        data = np.frombuffer(fh.read(width * height), dtype=np.uint8).copy()
    return data.reshape((height, width)), maxval, b"".join(header_parts)


def _write_pgm(path: Path, image: np.ndarray, maxval: int, header: bytes) -> None:
    tmp_path = path.with_suffix(".pgm.tmp")
    with tmp_path.open("wb") as fh:
        fh.write(header)
        fh.write(image.astype(np.uint8, copy=False).tobytes())
    tmp_path.replace(path)
