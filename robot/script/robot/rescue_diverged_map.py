#!/usr/bin/env python3
"""Build a clearly marked recovery map from keyframes recorded before SLAM divergence."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "edge_agent"))

from roamerx_edge.map_cleaner import _read_pcd_header, _write_binary_pcd  # noqa: E402
from roamerx_edge.map_preview import generate_map_preview  # noqa: E402


def load_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        for key in ("stamp", "x", "y", "z", "yaw"):
            row[key] = float(row.get(key) or 0.0)
        row["index"] = int(row["index"])
    if not rows:
        raise RuntimeError("keyframes.csv is empty")
    return rows


def detect_cutoff(rows: list[dict], speed_limit: float, z_jump_limit: float) -> tuple[int, str]:
    previous = rows[0]
    for row in rows[1:]:
        dt = row["stamp"] - previous["stamp"]
        distance = math.dist(
            (row["x"], row["y"], row["z"]),
            (previous["x"], previous["y"], previous["z"]),
        )
        speed = distance / dt if dt > 1e-3 else float("inf")
        z_jump = abs(row["z"] - previous["z"])
        if speed > speed_limit or z_jump > z_jump_limit:
            return previous["index"], (
                f"first keyframe anomaly at {row['index']}: "
                f"speed={speed:.2f}m/s z_jump={z_jump:.2f}m"
            )
        previous = row
    fallback = rows[max(0, len(rows) - 6)]["index"]
    return fallback, "no discrete anomaly found; excluded the final five keyframes"


def merge_keyframes(
    keyframe_dir: Path,
    rows: list[dict],
    output_path: Path,
    *,
    voxel_size: float,
) -> tuple[int, int]:
    reference_z = rows[0]["z"]
    clouds = []
    source_path = None
    source_header = None
    for row in rows:
        scan_path = keyframe_dir / f"scan_{row['index']:05d}.pcd"
        if not scan_path.exists():
            continue
        header, dtype, offset = _read_pcd_header(scan_path)
        if header["data"] != "binary":
            raise RuntimeError(f"unsupported PCD encoding: {scan_path}")
        points = np.fromfile(scan_path, dtype=dtype, offset=offset).copy()
        points["z"] -= row["z"] - reference_z
        clouds.append(points)
        source_path = source_path or scan_path
        source_header = source_header or header
    if not clouds or source_path is None or source_header is None:
        raise RuntimeError("no readable keyframe scans")

    merged = np.concatenate(clouds)
    source_count = int(merged.size)
    finite = np.isfinite(merged["x"]) & np.isfinite(merged["y"]) & np.isfinite(merged["z"])
    merged = merged[finite]
    voxels = np.empty(merged.size, dtype=[("x", "<i8"), ("y", "<i8"), ("z", "<i8")])
    voxels["x"] = np.floor(merged["x"] / voxel_size).astype(np.int64)
    voxels["y"] = np.floor(merged["y"] / voxel_size).astype(np.int64)
    voxels["z"] = np.floor(merged["z"] / voxel_size).astype(np.int64)
    _, indexes = np.unique(voxels, return_index=True)
    merged = merged[np.sort(indexes)]
    _write_binary_pcd(output_path, source_path, source_header["lines"], merged)
    return source_count, int(merged.size)


def write_metadata(source: Path, output: Path, rows: list[dict], cutoff: int, reason: str, counts: tuple[int, int]) -> None:
    adjusted_rows = []
    reference_z = rows[0]["z"]
    fieldnames = list(rows[0].keys())
    keyframe_dir = output / "keyframes"
    keyframe_dir.mkdir(exist_ok=True)
    with (keyframe_dir / "keyframes.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            item = dict(row)
            item["z"] = reference_z
            writer.writerow(item)
            adjusted_rows.append(item)

    with (output / "map.txt").open("w", encoding="utf-8") as stream:
        stream.write("# rescued path\n")
        for row in adjusted_rows:
            stream.write(f"{row['x']:.2f} {row['y']:.2f} {row['yaw']:.6f}\n")

    source_progress = json.loads((source / "save_progress.json").read_text(encoding="utf-8"))
    alignment = source_progress.get("rtk_alignment") or {}
    first = rows[0]
    gnss = {
        "rtk_enabled": bool(alignment.get("locked")),
        "datum": "CGCS2000",
        "origin_latitude": float(first.get("rtk_latitude") or 0.0),
        "origin_longitude": float(first.get("rtk_longitude") or 0.0),
        "origin_altitude": float(first.get("rtk_altitude") or 0.0),
        "alignment_locked": int(bool(alignment.get("locked"))),
        "enu_to_map_yaw": math.radians(float(alignment.get("yaw_deg") or 0.0)),
        "alignment_rms": float(alignment.get("rms") or -1.0),
        "alignment_samples": int(alignment.get("samples") or 0),
    }
    (output / "gnss_origin.yaml").write_text(yaml.safe_dump(gnss, sort_keys=False), encoding="utf-8")

    source_points, kept_points = counts
    rescue = {
        "format": "roamerx.diverged-map-rescue.v1",
        "source_map_dir": str(source),
        "cutoff_keyframe": cutoff,
        "keyframe_count": len(rows),
        "cutoff_reason": reason,
        "z_normalized": True,
        "source_points": source_points,
        "kept_points": kept_points,
        "created_at_unix": int(time.time()),
        "warning": "Recovered from pre-divergence keyframes; verify alignment before navigation.",
    }
    (output / "rescue_metadata.json").write_text(
        json.dumps(rescue, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    progress = {
        "format": "roamerx.streaming-map-progress.v1",
        "stage": "completed",
        "progress_percent": 100.0,
        "keyframe_count": len(rows),
        "written_keyframes": len(rows),
        "queued_keyframes": 0,
        "dropped_keyframes": 0,
        "written_points": kept_points,
        "estimated_output_bytes": (output / "map.pcd").stat().st_size,
        "trajectory_m": round(sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(rows, rows[1:])), 3),
        "slam_health": {"state": "rescued_degraded", "source_error": "SLAM_DIVERGED"},
        "updated_at_unix": int(time.time()),
        "recoverable": False,
        "error_code": "",
        "error": "",
        "rescue": rescue,
    }
    (output / "save_progress.json").write_text(
        json.dumps(progress, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--cutoff-index", type=int)
    parser.add_argument("--speed-limit", type=float, default=2.0)
    parser.add_argument("--z-jump-limit", type=float, default=0.2)
    parser.add_argument("--voxel-size", type=float, default=0.10)
    args = parser.parse_args()

    source = args.source.resolve()
    rows = load_rows(source / "keyframes" / "keyframes.csv")
    detected_cutoff, reason = detect_cutoff(rows, args.speed_limit, args.z_jump_limit)
    cutoff = args.cutoff_index if args.cutoff_index is not None else detected_cutoff
    rows = [row for row in rows if row["index"] <= cutoff]
    if len(rows) < 10:
        raise RuntimeError("too few healthy keyframes remain for rescue")
    output = args.output or source.with_name(f"{source.name}_rescue_{int(time.time())}")
    output.mkdir(parents=True, exist_ok=False)
    try:
        counts = merge_keyframes(source / "keyframes", rows, output / "map.pcd", voxel_size=args.voxel_size)
        converter = WORKSPACE / "install" / "robot_slam" / "lib" / "robot_slam" / "pcd2grid_streaming"
        subprocess.run(
            [str(converter), str(output / "map.pcd"), str(output / "map"), "0.05", "0.05", "0.75", "200000000"],
            check=True,
        )
        generate_map_preview(output / "map.pgm", output / "map_preview.png")
        shutil.copy2(output / "map_preview.png", output / "preview.png")
        write_metadata(source, output, rows, cutoff, reason, counts)
    except Exception:
        shutil.rmtree(output, ignore_errors=True)
        raise
    print(json.dumps({"output": str(output), "cutoff": cutoff, "reason": reason, "keyframes": len(rows)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
