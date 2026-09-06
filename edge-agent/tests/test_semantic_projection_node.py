from types import SimpleNamespace

import numpy as np

from roamerx_edge.semantic_projection_node import load_calibration, pointcloud_xyz, transform_matrix


def test_load_calibration_and_decode_strided_pointcloud(tmp_path):
    calibration = tmp_path / "camera.yaml"
    calibration.write_text(
        """calibration:
  id: front-v2
  image_size: [640, 480]
  camera_matrix: [400, 0, 320, 0, 401, 240, 0, 0, 1]
  distortion: [0, 0, 0, 0, 0]
  lidar_to_camera:
    - [1, 0, 0, 0]
    - [0, 1, 0, 0]
    - [0, 0, 1, 0]
    - [0, 0, 0, 1]
""",
        encoding="utf-8",
    )
    loaded = load_calibration(calibration)
    assert loaded.calibration_id == "front-v2"
    assert loaded.fy == 401

    records = np.array([(1, 2, 3, 9), (4, 5, 6, 8)], dtype="<f4").tobytes()
    fields = [SimpleNamespace(name=name, offset=index * 4, datatype=7) for index, name in enumerate(("x", "y", "z", "intensity"))]
    message = SimpleNamespace(fields=fields, point_step=16, width=2, height=1, data=records, is_bigendian=False)
    assert pointcloud_xyz(message).tolist() == [[1, 2, 3], [4, 5, 6]]


def test_transform_matrix_normalizes_quaternion_and_keeps_translation():
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=2, y=-1, z=.5),
        rotation=SimpleNamespace(x=0, y=0, z=0, w=2),
    ))
    matrix = transform_matrix(transform)
    assert np.allclose(matrix[:3, :3], np.eye(3))
    assert matrix[:3, 3].tolist() == [2, -1, .5]
