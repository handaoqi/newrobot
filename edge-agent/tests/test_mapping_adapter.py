import json
import subprocess
import time
import zipfile
from types import SimpleNamespace

import pytest

from roamerx_edge.config import MappingConfig
from roamerx_edge.mapping_adapter import MappingAdapter, MappingSession
from roamerx_edge.protocol import ProtocolError


class RunningProcess:
    def poll(self):
        return None


def make_adapter(tmp_path, **overrides):
    overrides.setdefault("rosbag_script", str(tmp_path / "missing-rosbag-script"))
    config = MappingConfig(map_dir=str(tmp_path), **overrides)
    media = SimpleNamespace(robot_id="test-dog")
    return MappingAdapter(config, media)


def test_start_mapping_records_before_slam_start(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    calls = []
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors_ready"))
    monkeypatch.setattr(adapter, "_start_rosbag", lambda label: calls.append(("start_bag", label)))
    monkeypatch.setattr(adapter, "_ensure_slam_process", lambda: calls.append("start_slam"))
    monkeypatch.setattr(adapter, "_call_map_state", lambda data: calls.append(("map_state", data)))
    monkeypatch.setattr(adapter, "status", lambda: {"state": adapter.session.state})

    result = adapter.start_mapping({"map_name": "park", "record_rosbag": True})

    assert result["state"] == "mapping"
    assert calls == ["stop_nav", "sensors_ready", ("start_bag", "park"), "start_slam", ("map_state", 3)]


def test_start_mapping_stops_rosbag_when_slam_fails(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    calls = []
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda: None)
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: None)
    monkeypatch.setattr(adapter, "_start_rosbag", lambda label: calls.append("start_bag"))
    monkeypatch.setattr(adapter, "_ensure_slam_process", lambda: (_ for _ in ()).throw(RuntimeError("boom")))
    monkeypatch.setattr(adapter, "_stop_rosbag", lambda: calls.append("stop_bag"))

    with pytest.raises(RuntimeError, match="boom"):
        adapter.start_mapping({"map_name": "park", "record_rosbag": True})

    assert calls == ["start_bag", "stop_bag"]


def test_status_prefers_active_progress_directory(tmp_path):
    complete = tmp_path / "20260715_100000_001"
    complete.mkdir()
    (complete / "map.yaml").write_text("resolution: 0.05\n")
    (complete / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")

    active = tmp_path / "20260715_110000_002"
    active.mkdir()
    progress = {
        "stage": "writing_pcd",
        "progress_percent": 42.5,
        "keyframe_count": 120,
    }
    (active / "save_progress.json").write_text(json.dumps(progress))

    adapter = make_adapter(tmp_path)
    adapter._slam_process = RunningProcess()
    status = adapter.status()

    assert status["active_map_dir"] == str(active)
    assert status["save_progress"] == progress
    assert status["state"] == "saving"


def test_large_map_skips_whole_cloud_visibility_filter(tmp_path):
    session = tmp_path / "20260715_120000_003"
    session.mkdir()
    (session / "map.pcd").write_bytes(b"x" * 32)

    adapter = make_adapter(
        tmp_path,
        visibility_filter_enabled=True,
        visibility_filter_max_source_bytes=16,
    )
    output, result = adapter._filter_map_outputs(session)

    assert output == session
    assert result["skipped"] == "source_exceeds_memory_safe_visibility_limit"
    assert result["active_filter"] == "disk_sharded_cpp_keyframe_filter"


def test_disabled_visibility_filter_packages_raw_map(tmp_path):
    session = tmp_path / "20260715_121000_003"
    session.mkdir()
    (session / "map.pcd").write_bytes(b"raw-map")

    adapter = make_adapter(tmp_path, visibility_filter_enabled=False)
    output, result = adapter._filter_map_outputs(session)

    assert output == session
    assert result == {
        "enabled": False,
        "mode": "manual_cleanup",
        "source": str(session),
    }


def test_map_package_keeps_gnss_origin(tmp_path, monkeypatch):
    session = tmp_path / "20260715_122000_004"
    session.mkdir()
    (session / "map.yaml").write_text("resolution: 0.05\n")
    (session / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (session / "map.txt").write_text("0 0 0\n")
    (session / "gnss_origin.yaml").write_text(
        "origin_latitude: 39.0\norigin_longitude: 116.0\nalignment_locked: 1\n"
    )
    adapter = make_adapter(tmp_path, visibility_filter_enabled=False)
    monkeypatch.setattr(adapter, "_generate_map_preview", lambda _base: None)

    package, metadata = adapter._package_map({}, session)

    with zipfile.ZipFile(package) as archive:
        assert "gnss_origin.yaml" in archive.namelist()
    assert "gnss_origin.yaml" in metadata["files"]


def test_finds_newest_incomplete_recoverable_session(tmp_path):
    recoverable = tmp_path / "20260715_130000_004"
    keyframes = recoverable / "keyframes"
    keyframes.mkdir(parents=True)
    (keyframes / "keyframes.csv").write_text("index,stamp,x,y,z,point_count\n")
    (recoverable / "save_progress.json").write_text(
        json.dumps({"stage": "failed", "recoverable": True})
    )

    adapter = make_adapter(tmp_path)

    assert adapter._find_latest_recoverable_dir() == recoverable


def test_completed_session_is_not_recoverable(tmp_path):
    completed = tmp_path / "20260715_140000_005"
    keyframes = completed / "keyframes"
    keyframes.mkdir(parents=True)
    (keyframes / "keyframes.csv").write_text("index,stamp,x,y,z,point_count\n")
    (completed / "save_progress.json").write_text(
        json.dumps({"stage": "completed", "recoverable": True})
    )
    (completed / "map.yaml").write_text("resolution: 0.05\n")
    (completed / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")

    adapter = make_adapter(tmp_path)

    assert adapter._find_latest_recoverable_dir() is None


def test_map_state_false_response_is_rejected(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    monkeypatch.setattr(
        "roamerx_edge.mapping_adapter.subprocess.run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args=args,
            returncode=0,
            stdout="response: success=false, message='no keyframes'",
            stderr="",
        ),
    )

    with pytest.raises(ProtocolError) as error:
        adapter._call_map_state(5)

    assert error.value.code == "MAPPING_SERVICE_REJECTED"


def test_status_reports_imu_initialization_readiness(tmp_path):
    active = tmp_path / "20260717_120000_001"
    active.mkdir()
    (active / "save_progress.json").write_text(
        json.dumps(
            {
                "stage": "initializing_imu",
                "keyframe_count": 0,
                "updated_at_unix": time.time(),
                "slam_health": {
                    "state": "initializing",
                    "imu_initialized": False,
                    "imu_samples": 240,
                    "imu_required_samples": 600,
                },
            }
        )
    )
    adapter = make_adapter(tmp_path)
    adapter._slam_process = RunningProcess()

    status = adapter.status()

    assert status["readiness"]["state"] == "imu_initializing"
    assert status["readiness"]["imu_samples"] == 240
    assert status["ready_for_motion"] is False
    assert status["ready_for_save"] is False


def test_status_is_ready_after_first_written_keyframe(tmp_path):
    active = tmp_path / "20260717_120100_002"
    active.mkdir()
    (active / "save_progress.json").write_text(
        json.dumps(
            {
                "stage": "mapping",
                "keyframe_count": 1,
                "written_keyframes": 1,
                "updated_at_unix": time.time(),
                "slam_health": {"state": "healthy", "imu_initialized": True},
            }
        )
    )
    adapter = make_adapter(tmp_path)
    adapter._slam_process = RunningProcess()

    status = adapter.status()

    assert status["readiness"]["state"] == "ready"
    assert status["ready_for_motion"] is True
    assert status["ready_for_save"] is True


def test_save_before_first_keyframe_restores_mapping_state(tmp_path):
    active = tmp_path / "20260717_120200_003"
    active.mkdir()
    (active / "save_progress.json").write_text(
        json.dumps(
            {
                "stage": "initializing_imu",
                "keyframe_count": 0,
                "updated_at_unix": time.time(),
                "slam_health": {"state": "initializing", "imu_initialized": False},
            }
        )
    )
    adapter = make_adapter(tmp_path)
    adapter._slam_process = RunningProcess()
    adapter.session = MappingSession("session", "map", "", "mapping", "now", "now")

    with pytest.raises(ProtocolError) as error:
        adapter._save_active_mapping({})

    assert error.value.code == "MAPPING_NOT_READY"
    assert adapter.session.state == "mapping"


def test_cancelled_progress_is_not_recovered(tmp_path):
    session = tmp_path / "20260715_150000_006"
    keyframes = session / "keyframes"
    keyframes.mkdir(parents=True)
    (keyframes / "keyframes.csv").write_text("index,stamp,x,y,z,point_count\n")
    (session / "save_progress.json").write_text(
        json.dumps({"stage": "mapping", "recoverable": True})
    )
    adapter = make_adapter(tmp_path)

    adapter._mark_progress_cancelled(session)

    progress = json.loads((session / "save_progress.json").read_text())
    assert progress["stage"] == "cancelled"
    assert progress["recoverable"] is False
    assert adapter._find_latest_recoverable_dir() is None


def test_diverged_session_is_not_recovered_or_packaged(tmp_path):
    session = tmp_path / "20260715_160000_007"
    keyframes = session / "keyframes"
    keyframes.mkdir(parents=True)
    (keyframes / "keyframes.csv").write_text("index,stamp,x,y,z,point_count\n")
    (session / "map.yaml").write_text("resolution: 0.05\n")
    (session / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (session / "save_progress.json").write_text(
        json.dumps(
            {
                "stage": "failed",
                "recoverable": False,
                "error_code": "SLAM_DIVERGED",
                "error": "pose guard triggered",
                "slam_health": {"state": "diverged"},
            }
        )
    )
    adapter = make_adapter(tmp_path)

    assert adapter._find_latest_recoverable_dir() is None
    with pytest.raises(ProtocolError) as error:
        adapter._validate_map_files(session)
    assert error.value.code == "SLAM_DIVERGED"
