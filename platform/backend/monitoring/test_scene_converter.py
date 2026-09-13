import json
import struct

import numpy as np
from PIL import Image
from django.test import SimpleTestCase

from .services.scene_converter import convert_scene


def write_pcd(path, points):
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
        "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n"
    ).encode()
    with path.open("wb") as stream:
        stream.write(header)
        for x, y, z in points:
            stream.write(struct.pack("<ffff", x, y, z, 1.0))


def synthetic_street():
    points = []
    for x in np.arange(0, 14, 0.3):
        for y in np.arange(0, 5, 0.3):
            points.append((x, y, 0.0))
    for offset in (0, 8):
        for x in np.arange(1 + offset, 5 + offset, 0.3):
            for y in np.arange(7, 10, 0.3):
                points.extend([(x, y, 0.0), (x, y, 4.0)])
    return points


class SceneConverterTests(SimpleTestCase):
    def test_code_converter_writes_valid_glb_continuous_road_and_independent_buildings(self):
        with self.subTest("binary pcd"):
            from tempfile import TemporaryDirectory
            with TemporaryDirectory() as folder:
                from pathlib import Path
                root = Path(folder)
                cloud = root / "street.pcd"
                write_pcd(cloud, synthetic_street())
                artifact, manifest = convert_scene(cloud, root / "out")
                raw = artifact.read_bytes()
                magic, version, length = struct.unpack("<4sII", raw[:12])
                self.assertEqual((magic, version, length), (b"glTF", 2, len(raw)))
                self.assertGreaterEqual(manifest["road_mesh_count"], 1)
                self.assertEqual(manifest["building_count"], 2)
                building_ids = [node["id"] for node in manifest["nodes"] if node["category"] == "building"]
                self.assertEqual(len(building_ids), len(set(building_ids)))

    def test_reference_image_colours_buildings_and_ptv3_is_only_a_hint(self):
        from tempfile import TemporaryDirectory
        from pathlib import Path
        with TemporaryDirectory() as folder:
            root = Path(folder)
            cloud = root / "street.pcd"
            write_pcd(cloud, synthetic_street())
            image = root / "reference.jpg"
            Image.new("RGB", (80, 60), (180, 150, 120)).save(image)
            _, manifest = convert_scene(
                cloud,
                root / "out",
                references=[image],
                semantics={
                    "instances": [{
                        "class_name": "wall", "confidence": 0.96,
                        "position": {"x": 3, "y": 8, "z": 0},
                    }]
                },
            )
            self.assertEqual(manifest["semantic_source"], "ptv3+geometry")
            self.assertGreater(manifest["quality"]["ptv3_matches"], 0)
            self.assertGreater(manifest["quality"]["color_coverage"], 0)
            self.assertGreater(manifest["quality"]["road_area_m2"], 1)
