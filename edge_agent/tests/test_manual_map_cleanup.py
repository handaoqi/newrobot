import numpy as np
import yaml

from roamerx_edge.manual_map_cleanup import _filter_point_cloud, _normalize_map_yaml_image
from roamerx_edge.map_cleaner import _read_pcd_header


def _write_test_pcd(path, points):
    header = (
        "# .PCD v0.7\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
    cloud = np.array(points, dtype=dtype)
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        cloud.tofile(stream)


def test_filters_only_points_projected_into_erased_cells(tmp_path):
    source = tmp_path / "source"
    output = tmp_path / "edited"
    source.mkdir()
    output.mkdir()
    source_pixels = bytes([
        255, 255, 255, 255,
        255, 0, 255, 255,
        255, 255, 255, 255,
    ])
    edited_pixels = bytearray(source_pixels)
    edited_pixels[5] = 255
    (source / "map.pgm").write_bytes(b"P5\n4 3\n255\n" + source_pixels)
    (output / "map.pgm").write_bytes(b"P5\n4 3\n255\n" + bytes(edited_pixels))
    (output / "map.yaml").write_text("resolution: 1.0\norigin: [0.0, 0.0, 0.0]\n")
    _write_test_pcd(source / "map.pcd", [(1.2, 1.2, 0.5), (2.2, 1.2, 0.5)])

    result = _filter_point_cloud(source, output)

    assert result["erased_cells"] == 1
    assert result["removed_points"] == 1
    _, dtype, offset = _read_pcd_header(output / "map.pcd")
    cleaned = np.fromfile(output / "map.pcd", dtype=dtype, offset=offset)
    assert cleaned.size == 1
    assert cleaned[0]["x"] == np.float32(2.2)


def test_projection_honors_map_origin_yaw(tmp_path):
    source = tmp_path / "source"
    output = tmp_path / "edited"
    source.mkdir()
    output.mkdir()
    source_pixels = bytes([255, 0, 255, 255])
    (source / "map.pgm").write_bytes(b"P5\n2 2\n255\n" + source_pixels)
    (output / "map.pgm").write_bytes(b"P5\n2 2\n255\n" + bytes([255] * 4))
    (output / "map.yaml").write_text("resolution: 1.0\norigin: [10.0, 20.0, 1.5707963267948966]\n")
    # Local map coordinate (1.2, 1.2) rotated +90 degrees and translated.
    _write_test_pcd(source / "map.pcd", [(8.8, 21.2, 0.5)])

    result = _filter_point_cloud(source, output)

    assert result["removed_points"] == 1


def test_normalizes_downloaded_yaml_to_local_pgm(tmp_path):
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        "image: /old/map/session/map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n",
        encoding="utf-8",
    )

    _normalize_map_yaml_image(yaml_path)

    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    assert metadata["image"] == "map.pgm"
    assert metadata["resolution"] == 0.05
