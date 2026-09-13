import json
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from roamerx_edge import scene_semantic_runner
from roamerx_edge.scene_semantic_runner import LABELS, read_pcd_xyz_intensity, run_autoware_scene_semantics, run_scene_semantics


def write_ascii_pcd(path: Path, points):
    lines = [
        "# .PCD v0.7", "VERSION 0.7", "FIELDS x y z intensity",
        "SIZE 4 4 4 4", "TYPE F F F F", "COUNT 1 1 1 1",
        f"WIDTH {len(points)}", "HEIGHT 1", "POINTS " + str(len(points)), "DATA ascii",
    ]
    path.write_text("\n".join(lines) + "\n" + "\n".join("%s %s %s %s" % point for point in points) + "\n", encoding="ascii")


def test_runner_reads_ascii_pcd_and_keeps_low_confidence_for_review():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        points = [(0.00, 0.00, 0.0, 1.0), (0.05, 0.00, 0.0, 1.0), (0.00, 0.05, 0.0, 1.0),
                  (5.00, 0.00, 1.0, 2.0), (5.05, 0.00, 1.0, 2.0), (5.00, 0.05, 1.0, 2.0)]
        write_ascii_pcd(root / "map.pcd", points)
        (root / "keyframes").mkdir()
        for index in range(3):
            (root / "keyframes" / f"scan_{index:03d}.pcd").touch()
        catalog = root / "catalog.json"
        catalog.write_text(json.dumps({"schema": "roamerx.scene-assets.v1", "assets": [
            {"asset_id": "road.straight", "category": "road", "dimensions_m": {"x": 2, "y": 2, "z": 0.1}},
            {"asset_id": "tree.shrub", "category": "tree", "dimensions_m": {"x": 1, "y": 1, "z": 1}},
        ]}), encoding="utf-8")

        def predictor(features):
            result = np.full((len(features), len(LABELS)), -4.0, dtype=np.float32)
            result[:, 0] = np.where(features[:, 0] < 1, 2.0, -4.0)
            result[:, 1] = np.where(features[:, 0] >= 1, 2.0, -4.0)
            return result

        payload = run_scene_semantics(root, model_path="unused.onnx", model_version="test", asset_catalog_path=catalog, predictor=predictor)
        assert len(payload["instances"]) == 2
        assert payload["review_candidates"] == []
        assert {item["asset_id"] for item in payload["instances"]} == {"road.straight", "tree.shrub"}
        assert np.allclose(read_pcd_xyz_intensity(root / "map.pcd", 3)[:, 3], [1, 1, 2])


def test_autoware_runner_normalizes_tensor_rt_cluster_output(monkeypatch):
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        write_ascii_pcd(root / "map.pcd", [(0.0, 0.0, 0.0, 1.0), (0.1, 0.0, 0.0, 1.0), (0.0, 0.1, 0.0, 1.0)])
        (root / "keyframes").mkdir()
        for index in range(3):
            (root / "keyframes" / f"scan_{index:03d}.pcd").touch()
        bundle = root / "bundle"
        bundle.mkdir()
        binary = root / "runner"
        binary.touch()
        plugin = root / "plugin.so"
        plugin.touch()
        catalog = root / "catalog.json"
        catalog.write_text(json.dumps({"schema": "roamerx.scene-assets.v1", "assets": [
            {"asset_id": "road.straight", "category": "road", "dimensions_m": {"x": 2, "y": 2, "z": 0.1}},
        ]}), encoding="utf-8")

        commands = []

        def fake_run(command, **_kwargs):
            commands.append(command)
            output = Path(command[command.index("--output") + 1])
            output.write_text(json.dumps({
                "instances": [{
                    "id": "ptv3-road-1", "asset_id": "road.straight", "class_name": "drivable_flat",
                    "position": {"x": 0, "y": 0, "z": 0}, "confidence": 0.92,
                }],
                "detections": [{"id": "ptv3-detection-1", "asset_id": "person.adult", "class_name": "pedestrian", "confidence": 0.91}],
            }), encoding="utf-8")
            return SimpleNamespace(returncode=0, stdout="", stderr="")

        monkeypatch.setattr(scene_semantic_runner.subprocess, "run", fake_run)
        payload = run_autoware_scene_semantics(
            root, bundle_path=bundle, plugin_path=plugin, inference_binary_path=binary,
            model_version="autoware-ptv3-v4", asset_catalog_path=catalog,
        )
        assert payload["status"] == "ready"
        assert payload["instances"][0]["support_frames"] == 3
        assert payload["instances"][0]["review_state"] == "generated"
        assert payload["metrics"]["detection_count"] == 1
        assert payload["dynamic_detections"][0]["class_name"] == "pedestrian"
        assert "--with-detection" in commands[0]
