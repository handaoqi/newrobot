import csv
from pathlib import Path

import numpy as np
import yaml

from roamerx_edge.map_cleaner import _write_binary_pcd
from roamerx_edge.submap_builder import SubmapBuildConfig, build_map_set


def _write_session(root: Path):
    root.mkdir()
    keyframes = root / "keyframes"
    keyframes.mkdir()
    points = np.array([(float(x), 0.0, 0.1, 1.0) for x in range(0, 501, 10)], dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    header = ["# .PCD v0.7", "VERSION 0.7", "FIELDS x y z intensity", "SIZE 4 4 4 4", "TYPE F F F F", "COUNT 1 1 1 1", "WIDTH 0", "HEIGHT 1", "POINTS 0", "DATA binary"]
    _write_binary_pcd(root / "map.pcd", root / "map.pcd", header, points)
    image = np.full((40, 520), 254, dtype=np.uint8)
    with (root / "map.pgm").open("wb") as stream:
        stream.write(b"P5\n520 40\n255\n")
        stream.write(image.tobytes())
    (root / "map.yaml").write_text(yaml.safe_dump({"image": "map.pgm", "resolution": 1.0, "origin": [0.0, -20.0, 0.0]}), encoding="utf-8")
    with (keyframes / "keyframes.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["index", "stamp", "x", "y", "z"])
        writer.writeheader()
        for index, x in enumerate(range(0, 501, 50)):
            writer.writerow({"index": index, "stamp": index, "x": x, "y": 0, "z": 0})
            _write_binary_pcd(keyframes / f"scan_{index:05d}.pcd", root / "map.pcd", header, points[index:index + 1])


def test_build_map_set_creates_overlapping_submaps(tmp_path):
    source = tmp_path / "session"
    _write_session(source)
    manifest = build_map_set(source, tmp_path / "map_set", config=SubmapBuildConfig(segment_length_m=250, step_length_m=190, margin_m=10))
    assert manifest["format"] == "roamerx.map_set.v1"
    assert len(manifest["submaps"]) == 3
    assert manifest["overlap_m"] == 60.0
    assert (tmp_path / "map_set" / "submap_001" / "map.pcd").exists()
    assert (tmp_path / "map_set" / "map_set_manifest.json").exists()
