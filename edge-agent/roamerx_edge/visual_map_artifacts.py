"""Create human-facing RGB replay artifacts from an RGB point cloud.

This module is deliberately outside the navigation map path.  It consumes a
PCD exported by RTAB-Map (or another RGB-D mapper), creates a top-down PNG,
and writes a manifest that the platform can expose as an optional visual
layer.  It refuses intensity-only clouds so that display colors are never
mistaken for sensor RGB.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


SCHEMA = "roamerx.visual-map.v1"


class VisualMapError(RuntimeError):
    pass


def _header(path: Path) -> tuple[int, dict[str, str], int]:
    values: dict[str, str] = {}
    offset = 0
    with path.open("rb") as stream:
        while offset < 64 * 1024:
            line = stream.readline()
            if not line:
                break
            offset += len(line)
            text = line.decode("ascii", errors="ignore").strip()
            if text and not text.startswith("#"):
                key, _, value = text.partition(" ")
                values[key.upper()] = value.strip()
            if text.upper().startswith("DATA "):
                count = int(values.get("POINTS") or values.get("WIDTH") or 0)
                return offset, values, count
    raise VisualMapError(f"PCD header is invalid: {path}")


def _unpack_rgb(value: np.ndarray, field_type: str, field_size: int) -> np.ndarray:
    if field_type.upper() == "F":
        packed = value.astype(np.float32, copy=False).view(np.uint32)
    else:
        packed = value.astype(np.uint32, copy=False)
    return np.column_stack(((packed >> 16) & 255, (packed >> 8) & 255, packed & 255)).astype(np.uint8)


def read_pcd_xyz_rgb(path: str | Path, max_points: int = 2_000_000) -> tuple[np.ndarray, np.ndarray]:
    """Read sampled XYZ and true RGB values from ASCII or binary PCD."""
    path = Path(path).expanduser().resolve()
    offset, header, point_count = _header(path)
    fields = header.get("FIELDS", "").split()
    sizes = [int(value) for value in header.get("SIZE", "").split()]
    types = header.get("TYPE", "").split()
    counts = [int(value) for value in header.get("COUNT", "").split()] or [1] * len(fields)
    if len(fields) != len(sizes) or len(fields) != len(types) or len(fields) != len(counts):
        raise VisualMapError("PCD fields are inconsistent")
    if not all(name in fields for name in ("x", "y", "z")):
        raise VisualMapError("PCD must contain x, y and z")
    rgb_field = next((name for name in ("rgb", "rgba") if name in fields), None)
    separate = all(name in fields for name in ("r", "g", "b"))
    if not rgb_field and not separate:
        raise VisualMapError("PCD has no RGB fields; refusing to fabricate a color layer")
    stride = max(1, math.ceil(point_count / max(1, max_points)))
    wanted = [fields.index(name) for name in ("x", "y", "z")]
    data_kind = header.get("DATA", "").lower()
    if data_kind == "ascii":
        rows = np.loadtxt(path, dtype=np.float32, skiprows=sum(1 for _ in _header_lines(path)))
        rows = np.atleast_2d(rows)[::stride]
        points = rows[:, wanted].astype(np.float32)
        if separate:
            colors = np.clip(rows[:, [fields.index("r"), fields.index("g"), fields.index("b")],], 0, 255).astype(np.uint8)
        else:
            colors = _unpack_rgb(rows[:, fields.index(rgb_field)], types[fields.index(rgb_field)], sizes[fields.index(rgb_field)])
        return points, colors
    if data_kind != "binary":
        raise VisualMapError("only ASCII and binary PCD are supported")
    type_map = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}
    dtype_fields = []
    for index, (name, size, kind, count) in enumerate(zip(fields, sizes, types, counts)):
        np_kind = type_map.get((kind.upper(), size))
        if not np_kind or count != 1:
            raise VisualMapError(f"unsupported PCD field: {name}")
        dtype_fields.append((f"f{index}", "<" + np_kind))
    data = np.fromfile(path, dtype=np.dtype(dtype_fields), offset=offset, count=point_count)[::stride]
    if len(data) != math.ceil(point_count / stride):
        raise VisualMapError("PCD binary payload is truncated")
    points = np.column_stack([data[f"f{index}"] for index in wanted]).astype(np.float32)
    if separate:
        colors = np.column_stack([data[f"f{fields.index(name)}"] for name in ("r", "g", "b")])
        colors = np.clip(colors, 0, 255).astype(np.uint8)
    else:
        index = fields.index(rgb_field)
        colors = _unpack_rgb(data[f"f{index}"], types[index], sizes[index])
    return points, colors


def _header_lines(path: Path):
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                return
            yield line
            if line.decode("ascii", errors="ignore").strip().upper().startswith("DATA "):
                return


def build_rgb_orthophoto(points: np.ndarray, colors: np.ndarray, output: str | Path, resolution_m: float = 0.05) -> dict:
    """Rasterize the highest point per top-down cell and return its metadata."""
    if len(points) == 0 or len(points) != len(colors):
        raise VisualMapError("RGB point cloud is empty or inconsistent")
    if resolution_m <= 0:
        raise VisualMapError("resolution_m must be positive")
    finite = np.isfinite(points).all(axis=1)
    points, colors = points[finite], colors[finite]
    min_xy = points[:, :2].min(axis=0)
    cells = np.floor((points[:, :2] - min_xy) / resolution_m).astype(np.int64)
    width, height = int(cells[:, 0].max() + 1), int(cells[:, 1].max() + 1)
    if width * height > 100_000_000:
        raise VisualMapError("requested orthophoto is too large")
    image = np.zeros((height, width, 3), dtype=np.uint8)
    # Image row zero is north/top; local Y increases upward.
    image[height - 1 - cells[:, 1], cells[:, 0]] = colors
    output = Path(output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image
    except ImportError as exc:
        raise VisualMapError("Pillow is required to create RGB orthophotos") from exc
    Image.fromarray(image, mode="RGB").save(output, format="PNG", optimize=True)
    return {"url": output.name, "path": str(output), "width_px": width, "height_px": height, "resolution_m": resolution_m,
            "bounds": {"min_x": float(min_xy[0]), "min_y": float(min_xy[1]), "max_x": float(min_xy[0] + width * resolution_m), "max_y": float(min_xy[1] + height * resolution_m)}}


def create_visual_map_artifacts(pcd: str | Path, output_dir: str | Path, resolution_m: float = 0.05, max_points: int = 2_000_000) -> dict:
    output_dir = Path(output_dir).expanduser().resolve()
    points, colors = read_pcd_xyz_rgb(pcd, max_points=max_points)
    artifact = build_rgb_orthophoto(points, colors, output_dir / "rgb_orthophoto.png", resolution_m)
    manifest = {"schema": SCHEMA, "navigation_authoritative": False, "source": {"pcd": str(Path(pcd).expanduser().resolve()), "point_count": len(points)}, "artifacts": {"rgb_orthophoto": artifact}}
    (output_dir / "visual_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a human-facing RGB replay map from an RGB PCD")
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--resolution-m", type=float, default=0.05)
    parser.add_argument("--max-points", type=int, default=2_000_000)
    args = parser.parse_args()
    print(json.dumps(create_visual_map_artifacts(args.pcd, args.output_dir, args.resolution_m, args.max_points), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
