import json
import struct

from roamerx_edge.street_block_builder import build_street_block


def test_builder_connects_roads_keeps_buildings_independent_and_writes_glb(tmp_path):
    artifact, manifest = build_street_block(tmp_path, {
        "instances": [
            {"id": "road-1", "class_name": "drivable_flat", "position": {"x": 0, "y": 0, "z": 0}, "confidence": .96},
            {"id": "road-2", "class_name": "drivable_flat", "position": {"x": 5, "y": 0, "z": 0}, "confidence": .95},
            {"id": "building-1", "class_name": "building", "position": {"x": 10, "y": 4, "z": 0}, "confidence": .97},
            {"id": "building-2", "class_name": "building", "position": {"x": 16, "y": 4, "z": 0}, "confidence": .97},
        ],
        "review_candidates": [],
    })
    raw = artifact.read_bytes()
    magic, version, total = struct.unpack("<4sII", raw[:12])
    json_size, _ = struct.unpack("<I4s", raw[12:20])
    document = json.loads(raw[20:20 + json_size])
    assert (magic, version, total) == (b"glTF", 2, len(raw))
    assert manifest["building_count"] == 2
    assert manifest["road_mesh_count"] == 3
    assert {node["name"] for node in document["nodes"]} >= {"building-1", "building-2", "road-link-1"}


def test_builder_rejects_tree_collision(tmp_path):
    _, manifest = build_street_block(tmp_path, {
        "instances": [
            {"id": "building", "class_name": "building", "position": {"x": 0, "y": 0, "z": 0}, "confidence": .96},
            {"id": "tree", "class_name": "tree", "position": {"x": 0, "y": 0, "z": 0}, "confidence": .96},
        ],
        "review_candidates": [],
    })
    assert manifest["quality"]["geometry_conflicts"] == 1
    assert manifest["node_count"] == 1


def test_builder_uses_unposed_reference_image_as_approximate_material_colour(tmp_path):
    from PIL import Image

    references = tmp_path / "references"
    references.mkdir()
    Image.new("RGB", (20, 20), (190, 65, 45)).save(references / "building.jpg")
    _, manifest = build_street_block(tmp_path, {
        "instances": [{"id": "building", "class_name": "building", "position": {"x": 0, "y": 0, "z": 0}, "confidence": .96}],
        "review_candidates": [],
    })
    assert manifest["quality"]["color_coverage"] == 1.0
    assert manifest["nodes"][0]["color_source"] == "reference_media"
