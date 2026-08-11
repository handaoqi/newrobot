from types import SimpleNamespace

from roamerx_edge.map_activation_adapter import MapActivationAdapter
from roamerx_edge.safety_policy import RuntimeSafetyState


def _source_map(root, name, *, gnss=False):
    source = root / name
    source.mkdir()
    for filename in ("map.yaml", "map.pgm", "map.pcd"):
        (source / filename).write_text(filename)
    if gnss:
        (source / "gnss_origin.yaml").write_text("origin_latitude: 1.0\n")
    (source / "map.txt").write_text("# path\n1.25 -2.50 0.75\n2.0 -2.0 0.8\n")
    return source


def test_activation_removes_stale_map_scoped_gnss_metadata(tmp_path):
    source_with_gnss = _source_map(tmp_path, "with-gnss", gnss=True)
    source_without_gnss = _source_map(tmp_path, "without-gnss")
    config_path = tmp_path / "edge.yaml"
    config_path.write_text("robot: {}\n")
    config = SimpleNamespace(
        mapping=SimpleNamespace(map_dir=str(tmp_path)),
        robot=SimpleNamespace(current_map_id="", current_map_version=""),
    )
    adapter = MapActivationAdapter(config, RuntimeSafetyState(), str(config_path))

    adapter.activate(
        {"map_id": "1", "map_version": "v1", "local_map_dir": str(source_with_gnss)}
    )
    assert (tmp_path / "gnss_origin.yaml").resolve() == (source_with_gnss / "gnss_origin.yaml")

    result = adapter.activate(
        {"map_id": "2", "map_version": "v2", "local_map_dir": str(source_without_gnss)}
    )

    assert not (tmp_path / "gnss_origin.yaml").exists()
    assert "gnss_origin.yaml" not in result["switched_files"]


def test_mapping_start_pose_comes_from_first_trajectory_sample(tmp_path):
    source = _source_map(tmp_path, "source")
    config_path = tmp_path / "edge.yaml"
    config_path.write_text("robot: {}\n")
    config = SimpleNamespace(
        mapping=SimpleNamespace(map_dir=str(tmp_path)),
        robot=SimpleNamespace(current_map_id="", current_map_version=""),
    )
    adapter = MapActivationAdapter(config, RuntimeSafetyState(), str(config_path))
    adapter.activate({"map_id": "1", "map_version": "v1", "local_map_dir": str(source)})

    assert adapter.mapping_start_pose() == {
        "x": 1.25,
        "y": -2.5,
        "z": 0.0,
        "yaw": 0.75,
        "source": "mapping_start",
    }


def test_activation_restores_cloud_gnss_metadata(tmp_path):
    source = _source_map(tmp_path, "restored")
    config_path = tmp_path / "edge.yaml"
    config_path.write_text("robot: {}\n")
    config = SimpleNamespace(
        mapping=SimpleNamespace(map_dir=str(tmp_path)),
        robot=SimpleNamespace(current_map_id="", current_map_version=""),
    )
    adapter = MapActivationAdapter(config, RuntimeSafetyState(), str(config_path))
    metadata = (
        "origin_latitude: 39.0\n"
        "origin_longitude: 116.0\n"
        "alignment_locked: 1\n"
    )

    adapter.activate({
        "map_id": "3",
        "map_version": "v3",
        "local_map_dir": str(source),
        "gnss_origin_yaml": metadata,
    })

    assert (source / "gnss_origin.yaml").read_text() == metadata
    assert (tmp_path / "gnss_origin.yaml").resolve() == (source / "gnss_origin.yaml")
