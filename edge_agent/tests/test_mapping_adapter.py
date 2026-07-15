import json
import subprocess
from types import SimpleNamespace

import pytest

from roamerx_edge.config import MappingConfig
from roamerx_edge.mapping_adapter import MappingAdapter
from roamerx_edge.protocol import ProtocolError


class RunningProcess:
    def poll(self):
        return None


def make_adapter(tmp_path, **overrides):
    config = MappingConfig(map_dir=str(tmp_path), **overrides)
    media = SimpleNamespace(robot_id="test-dog")
    return MappingAdapter(config, media)


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
