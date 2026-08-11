#!/usr/bin/env python3
"""Generate comparable saved-map dynamic-filter variants from one raw map."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import zipfile
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_DIR / "edge_agent"))

from roamerx_edge.map_cleaner import clean_dynamic_points


REQUIRED_FILES = ("map.yaml", "map.pgm", "map.pcd")
OPTIONAL_FILES = ("map.txt",)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_dir", type=Path, help="saved map directory containing map.raw_dynamic_unfiltered.pcd")
    parser.add_argument("--thresholds", default="3,5,7", help="comma-separated minimum points per 12 cm voxel")
    parser.add_argument("--voxel-size", type=float, default=0.12, help="post-save voxel size in metres")
    parser.add_argument("--clear-radius", type=int, default=1, help="occupied cells cleared around removed points")
    parser.add_argument("--output-root", type=Path, help="defaults to <map_dir>/filter_variants")
    return parser.parse_args()


def create_variant(source: Path, output_root: Path, threshold: int, voxel_size: float, clear_radius: int) -> dict:
    target = output_root / f"{source.name}_postfilter_{threshold}"
    if target.exists():
        raise FileExistsError(f"refusing to overwrite existing variant: {target}")
    target.mkdir(parents=True)

    raw_pcd = source / "map.raw_dynamic_unfiltered.pcd"
    raw_pgm = source / "map.raw_dynamic_unfiltered.pgm"
    if not raw_pcd.exists():
        raise FileNotFoundError(f"missing raw point cloud: {raw_pcd}")

    shutil.copy2(raw_pcd, target / "map.pcd")
    shutil.copy2(raw_pcd, target / raw_pcd.name)
    shutil.copy2(raw_pgm if raw_pgm.exists() else source / "map.pgm", target / "map.pgm")
    if raw_pgm.exists():
        shutil.copy2(raw_pgm, target / raw_pgm.name)
    for name in ("map.yaml",) + OPTIONAL_FILES:
        file_path = source / name
        if file_path.exists():
            shutil.copy2(file_path, target / name)

    result = clean_dynamic_points(
        target,
        voxel_size_m=voxel_size,
        min_points_per_voxel=threshold,
        clear_radius_cells=clear_radius,
    )
    report = {
        "source_map": str(source),
        "variant": target.name,
        "post_filter": result,
        "note": "This compares saved-map density filtering. It does not replace the live per-scan dynamic filter used during the next mapping run.",
    }
    (target / "filter_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    archive_path = target.with_suffix(".zip")
    with zipfile.ZipFile(archive_path, "w", zipfile.ZIP_DEFLATED) as archive:
        for name in REQUIRED_FILES + OPTIONAL_FILES:
            file_path = target / name
            if file_path.exists():
                archive.write(file_path, arcname=name)
        archive.write(target / "filter_report.json", arcname="filter_report.json")
    report["archive"] = str(archive_path)
    return report


def main() -> int:
    args = parse_args()
    source = args.map_dir.expanduser().resolve()
    output_root = (args.output_root or source / "filter_variants").expanduser().resolve()
    thresholds = [int(value.strip()) for value in args.thresholds.split(",") if value.strip()]
    if not source.is_dir() or not thresholds or any(value < 2 for value in thresholds):
        raise ValueError("map_dir must exist and thresholds must be integers >= 2")

    output_root.mkdir(parents=True, exist_ok=True)
    reports = [create_variant(source, output_root, value, args.voxel_size, args.clear_radius) for value in thresholds]
    print(json.dumps(reports, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
