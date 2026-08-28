import hashlib
import io
from pathlib import Path
import zipfile
from types import SimpleNamespace

import pytest

from roamerx_edge.map_activation_adapter import MapActivationAdapter
from roamerx_edge.protocol import ProtocolError
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


def test_activation_rejects_local_only_outdoor_scene(tmp_path):
    source = _source_map(tmp_path, "local-only")
    (source / "map_manifest.json").write_text(
        '{"schema_version": 2, "coordinate_mode": "local_only", "scene_scope": "indoor", '
        '"localization_mode": "ndt", "origin_status": "local_only"}'
    )
    config_path = tmp_path / "edge.yaml"
    config_path.write_text("robot: {}\n")
    config = SimpleNamespace(
        mapping=SimpleNamespace(map_dir=str(tmp_path)),
        robot=SimpleNamespace(current_map_id="", current_map_version=""),
    )
    adapter = MapActivationAdapter(config, RuntimeSafetyState(), str(config_path))

    with pytest.raises(ProtocolError) as error:
        adapter.activate({
            "map_id": "9",
            "map_version": "v9",
            "local_map_dir": str(source),
            "scene_scope": "outdoor",
        })
    assert error.value.code == "MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN"


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


def test_activation_switches_all_required_files_through_one_version_pointer(tmp_path):
    first = _source_map(tmp_path, "first")
    second = _source_map(tmp_path, "second")
    config_path = tmp_path / "edge.yaml"
    config_path.write_text("robot: {}\n")
    config = SimpleNamespace(
        mapping=SimpleNamespace(map_dir=str(tmp_path)),
        robot=SimpleNamespace(current_map_id="", current_map_version=""),
    )
    adapter = MapActivationAdapter(config, RuntimeSafetyState(), str(config_path))

    adapter.activate({"map_id": "1", "map_version": "v1", "local_map_dir": str(first)})
    adapter.activate({"map_id": "2", "map_version": "v2", "local_map_dir": str(second)})

    assert (tmp_path / "current").is_symlink()
    assert (tmp_path / "current").resolve() == second
    for name in adapter.REQUIRED_FILES:
        assert (tmp_path / name).readlink() == Path("current") / name
        assert (tmp_path / name).resolve() == second / name
    assert adapter.status()["version_pointer"] == str(second)


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


def test_manual_activation_reuses_complete_cached_revision(tmp_path):
    source = _source_map(tmp_path, "source")
    config_path = tmp_path / "edge.yaml"
    config_path.write_text("robot: {}\n")
    config = SimpleNamespace(
        mapping=SimpleNamespace(map_dir=str(tmp_path)),
        robot=SimpleNamespace(current_map_id="", current_map_version=""),
    )
    adapter = MapActivationAdapter(config, RuntimeSafetyState(), str(config_path))
    cached = tmp_path / "manual_edits" / "map_95_revision"
    cached.mkdir(parents=True)
    for filename in ("map.yaml", "map.pgm", "map.pcd"):
        (cached / filename).write_text(f"cached-{filename}")

    result = adapter.activate({
        "map_id": "95",
        "map_version": "revision",
        "local_map_dir": str(source),
        "manual_edit": True,
        "pgm_url": "http://unreachable/maps/95.pgm",
        "yaml_url": "http://unreachable/maps/95.yaml",
    })

    assert result["manual_cleanup"] == {"reused_cached_revision": True}
    assert (tmp_path / "map.pcd").resolve() == cached / "map.pcd"


def test_activation_downloads_complete_cloud_package_when_no_local_source(tmp_path, monkeypatch):
    package_buffer = io.BytesIO()
    with zipfile.ZipFile(package_buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("map.yaml", "image: map.pgm\nresolution: 0.05\n")
        archive.writestr("map.pgm", b"pgm")
        archive.writestr("map.pcd", b"pcd")
    package = package_buffer.getvalue()

    class Response:
        def raise_for_status(self):
            return None

        def iter_content(self, chunk_size):
            yield package

    monkeypatch.setattr("roamerx_edge.map_activation_adapter.requests.get", lambda *args, **kwargs: Response())
    config_path = tmp_path / "edge.yaml"
    config_path.write_text("robot: {}\n")
    config = SimpleNamespace(
        mapping=SimpleNamespace(map_dir=str(tmp_path)),
        robot=SimpleNamespace(current_map_id="", current_map_version=""),
    )
    adapter = MapActivationAdapter(config, RuntimeSafetyState(), str(config_path))

    result = adapter.activate({
        "map_id": "cloud-1",
        "map_version": "v1",
        "package_url": "https://cloud.example/maps/1.zip",
        "package_sha256": hashlib.sha256(package).hexdigest(),
    })

    source = tmp_path / ".cloud_packages" / "cloud-1_v1"
    assert source.is_dir()
    assert (source / "map.pcd").read_bytes() == b"pcd"
    assert result["current_map"]["local_state"] == "applied"
