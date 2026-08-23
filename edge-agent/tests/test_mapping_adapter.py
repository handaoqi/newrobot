import json
import math
import subprocess
import time
import zipfile
from types import SimpleNamespace

import pytest
import yaml

from roamerx_edge.config import MappingConfig
from roamerx_edge.mapping_adapter import MappingAdapter, MappingSession
from roamerx_edge.protocol import ProtocolError


class RunningProcess:
    def poll(self):
        return None


def make_adapter(tmp_path, **overrides):
    overrides.setdefault("rosbag_script", str(tmp_path / "missing-rosbag-script"))
    overrides.setdefault("mapping_unit", "")
    overrides.setdefault("navigation_script", str(tmp_path / "missing-nav-script"))
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


def test_slam_warmup_does_not_enable_formal_capture(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    calls = []
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors"))
    monkeypatch.setattr(adapter, "_ensure_slam_process", lambda: calls.append("slam"))
    monkeypatch.setattr(adapter, "_call_map_state", lambda data: calls.append(("state", data)))
    monkeypatch.setattr(adapter, "_rosbag_status", lambda: {"running": False})
    monkeypatch.setattr(adapter, "status", lambda: {
        "state": adapter.session.state,
        "mapping_capture_enabled": adapter.session.mapping_capture_enabled,
    })

    result = adapter.start_slam_warmup({"map_name": "inside", "mapping_type": "indoor"})

    assert result == {"state": "slam_warmup", "mapping_capture_enabled": False}
    assert calls == ["stop_nav", "sensors", "slam", ("state", 7)]


def test_outdoor_warmup_requires_locked_origin(tmp_path):
    adapter = make_adapter(tmp_path)

    with pytest.raises(ProtocolError) as error:
        adapter.start_slam_warmup({"map_name": "outside", "mapping_type": "outdoor"})

    assert error.value.code == "MAPPING_ORIGIN_REQUIRED"


def test_outdoor_start_can_prepare_sensors_before_origin_lock(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    calls = []
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors"))
    monkeypatch.setattr(adapter, "status", lambda: {
        "state": adapter.session.state,
        "origin": adapter._origin_monitor.status(),
    })

    result = adapter.start_origin_lock({
        "map_name": "outside",
        "scene_scope": "outdoor",
        "mapping_type": "outdoor",
        "prepare_only": True,
    })

    assert result["state"] == "origin_waiting"
    assert result["origin"]["origin_status"] == "ready"
    assert calls == ["stop_nav", "sensors"]


def test_origin_lock_reuses_prepared_waiting_session(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    calls = []
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors"))
    monkeypatch.setattr(adapter, "status", lambda: {
        "state": adapter.session.state,
        "origin": adapter._origin_monitor.status(),
    })

    adapter.start_origin_lock({
        "map_name": "outside",
        "scene_scope": "outdoor",
        "mapping_type": "outdoor",
        "prepare_only": True,
    })
    session_id = adapter.session.session_id
    monkeypatch.setattr(adapter._origin_monitor, "start", lambda: adapter._origin_monitor.status())

    result = adapter.start_origin_lock({
        "mapping_session_id": session_id,
        "scene_scope": "outdoor",
        "mapping_type": "outdoor",
    })

    assert result["state"] == "origin_waiting"
    assert adapter.session.session_id == session_id
    assert calls == ["stop_nav", "sensors"]


def test_origin_lock_retries_after_quality_failure(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    calls = []
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors"))
    monkeypatch.setattr(adapter, "status", lambda: {
        "state": adapter.session.state,
        "origin": adapter._origin_monitor.status(),
    })

    adapter.start_origin_lock({
        "map_name": "outside",
        "scene_scope": "outdoor",
        "mapping_type": "outdoor",
        "prepare_only": True,
    })
    session_id = adapter.session.session_id
    adapter._origin_monitor._status.update(origin_status="failed", message="RTK_SIGNAL_TIMEOUT")

    def retry_start():
        adapter._origin_monitor._status.update(origin_status="waiting_quality", message="retrying")
        return adapter._origin_monitor.status()

    monkeypatch.setattr(adapter._origin_monitor, "start", retry_start)
    result = adapter.start_origin_lock({
        "mapping_session_id": session_id,
        "scene_scope": "outdoor",
        "mapping_type": "outdoor",
    })

    assert result["state"] == "origin_waiting"
    assert result["origin"]["origin_status"] == "waiting_quality"
    assert adapter.session.session_id == session_id
    assert calls == ["stop_nav", "sensors"]


def test_extract_global_enu_persists_locked_origin(tmp_path):
    adapter = make_adapter(tmp_path)
    result = adapter.extract_global_enu({
        "source_map_id": "map-7",
        "source_map_name": "outside",
        "global_enu": {
            "alignment_locked": 1,
            "origin_latitude": 39.9,
            "origin_longitude": 116.4,
            "origin_altitude": 42.0,
            "heading_deg": 12.5,
        },
    })

    assert result["global_enu"]["source_map_id"] == "map-7"
    assert adapter._global_enu_file.exists()
    assert adapter.status()["global_enu"]["origin_latitude"] == 39.9


def test_indoor_warmup_rejects_outdoor_scene_scope(tmp_path):
    adapter = make_adapter(tmp_path)

    with pytest.raises(ProtocolError) as error:
        adapter.start_slam_warmup({"mapping_type": "indoor", "scene_scope": "outdoor"})

    assert error.value.code == "MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN"


def test_origin_topic_sample_requires_position_and_heading_fixed(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    messages = {
        "/fix": {"status": {"status": 2}, "latitude": 39.9, "longitude": 116.4, "altitude": 42.0},
        "/rtk_pvh": {
            "bestnav": {
                "p_sol_status": 0, "pos_type": 48, "latitude_deg": 39.9,
                "longitude_deg": 116.4, "altitude_m": 42.0, "lat_std": 0.008, "lon_std": 0.009,
            },
            "heading": {"sol_status": 0, "heading_type": 4, "base_line": 0.8, "heading_deg": 90.0, "heading_std": 0.4},
        },
        "/rtk/ntrip_status": {"data": "quality: rtk_fixed\nage_sec: 0.1"},
    }
    monkeypatch.setattr(adapter, "_echo_topic_once", lambda topic, _timeout: messages[topic])

    sample = adapter._sample_origin_topics()

    assert sample.position_fixed is True
    assert sample.heading_fixed is True
    assert sample.horizontal_std_m == pytest.approx(0.009)


def test_outdoor_export_merges_lock_evidence_with_slam_alignment(tmp_path):
    adapter = make_adapter(tmp_path)
    adapter.session = MappingSession(
        "mapping-session", "outside", "", "saving", "now", "now",
        scene_scope="outdoor", mapping_type="outdoor",
    )
    adapter._origin_file.write_text(yaml.safe_dump({
        "alignment_locked": True,
        "origin_lock_session_id": "origin-session",
        "origin_latitude": 39.9,
        "origin_longitude": 116.4,
        "origin_altitude": 42.0,
        "position_spread_m": 0.012,
        "lock_duration_seconds": 60,
    }))
    work = tmp_path / "20260822_120000_001"
    work.mkdir()
    exported = work / "gnss_origin.yaml"
    exported.write_text(yaml.safe_dump({"alignment_locked": 1, "enu_to_map_yaw": 0.25}))

    adapter._merge_locked_origin_metadata(work)

    merged = yaml.safe_load(exported.read_text())
    assert merged["origin_lock_session_id"] == "origin-session"
    assert merged["position_spread_m"] == pytest.approx(0.012)
    assert merged["enu_to_map_yaw"] == pytest.approx(0.25)


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
    (session / "map.pcd").write_bytes(b"pcd")
    (session / "keyframes").mkdir()
    (session / "keyframes" / "keyframes.csv").write_text("index,x,y,yaw\n0,0,0,0\n")
    (session / "scan_context").mkdir()
    (session / "scan_context" / "index.json").write_text("{}")
    adapter = make_adapter(tmp_path, visibility_filter_enabled=False)
    monkeypatch.setattr(adapter, "_generate_map_preview", lambda _base: None)

    package, metadata = adapter._package_map({}, session)

    with zipfile.ZipFile(package) as archive:
        assert "gnss_origin.yaml" in archive.namelist()
        assert "map.pcd" in archive.namelist()
        assert "keyframes/keyframes.csv" in archive.namelist()
        assert "scan_context/index.json" in archive.namelist()
    assert "gnss_origin.yaml" in metadata["files"]


def test_map_package_contains_lightweight_slam_and_rtk_trace(tmp_path, monkeypatch):
    session = tmp_path / "20260715_122500_004"
    keyframes = session / "keyframes"
    keyframes.mkdir(parents=True)
    (session / "map.yaml").write_text("resolution: 0.05\n")
    (session / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (session / "map.txt").write_text("0 0 0\n")
    (session / "gnss_origin.yaml").write_text(
        "origin_latitude: 39.0\norigin_longitude: 116.0\nalignment_locked: 1\n"
        "enu_to_map_yaw: 0.0\nmap_offset_x: 0.0\nmap_offset_y: 0.0\n"
    )
    (keyframes / "keyframes.csv").write_text(
        "index,stamp,x,y,z,yaw,point_count,rtk_valid,rtk_status,rtk_latitude,rtk_longitude,rtk_altitude,rtk_horizontal_std,rtk_age_seconds,rtk_heading_valid,rtk_heading_deg,rtk_heading_std_deg,rtk_heading_age_seconds\n"
        "0,100.0,1.0,2.0,0.0,0.5,10,1,2,39.0,116.0,0.0,0.02,0.1,1,90.0,0.5,0.1\n"
    )
    adapter = make_adapter(tmp_path, visibility_filter_enabled=False)
    monkeypatch.setattr(adapter, "_generate_map_preview", lambda _base: None)

    package, metadata = adapter._package_map({}, session)

    with zipfile.ZipFile(package) as archive:
        trace = json.loads(archive.read("mapping_trace.json"))
    assert "mapping_trace.json" in metadata["files"]
    assert trace["samples"][0]["slam"]["x"] == 1.0
    assert trace["samples"][0]["slam"]["y"] == 2.0
    assert trace["samples"][0]["slam"]["yaw"] == 0.5
    assert trace["samples"][0]["rtk"]["yaw"] == pytest.approx(0.0)


def test_heading_locked_origin_writes_rtk_map_pose(tmp_path, monkeypatch):
    session = tmp_path / "20260821_170000_001"
    keyframes = session / "keyframes"
    keyframes.mkdir(parents=True)
    heading_deg = 223.17
    slam_yaw = 0.0
    yaw_enu = math.pi / 2.0 - math.radians(heading_deg)
    alignment_yaw = math.atan2(math.sin(slam_yaw - yaw_enu), math.cos(slam_yaw - yaw_enu))
    (session / "map.yaml").write_text("resolution: 0.05\n")
    (session / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (session / "map.txt").write_text("0 0 0\n")
    (session / "gnss_origin.yaml").write_text(
        "origin_latitude: 39.9714186649\norigin_longitude: 116.4483795514\n"
        f"alignment_locked: 1\nenu_to_map_yaw: {alignment_yaw}\n"
        "map_offset_x: 0.0\nmap_offset_y: 0.0\n"
    )
    (keyframes / "keyframes.csv").write_text(
        "index,stamp,x,y,z,yaw,point_count,rtk_valid,rtk_status,rtk_latitude,rtk_longitude,rtk_altitude,"
        "rtk_horizontal_std,rtk_age_seconds,rtk_heading_valid,rtk_heading_deg,rtk_heading_std_deg,rtk_heading_age_seconds\n"
        f"0,100.0,-0.0154,-0.0447,0.04,{slam_yaw},10,1,2,39.9714186649,116.4483795514,39.7,0.03,0.08,1,{heading_deg},2.4,0.09\n"
    )
    adapter = make_adapter(tmp_path, visibility_filter_enabled=False)
    monkeypatch.setattr(adapter, "_generate_map_preview", lambda _base: None)

    package, _metadata = adapter._package_map({}, session)

    with zipfile.ZipFile(package) as archive:
        trace = json.loads(archive.read("mapping_trace.json"))
    sample = trace["samples"][0]
    assert trace["alignment_locked"] is True
    assert sample["rtk"]["x"] == pytest.approx(0.0, abs=0.02)
    assert sample["rtk"]["y"] == pytest.approx(0.0, abs=0.02)
    assert sample["rtk"]["yaw"] == pytest.approx(slam_yaw, abs=1e-4)


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


def test_stop_nav_uses_configured_script(tmp_path, monkeypatch):
    script = tmp_path / "start_navigation_real.sh"
    script.write_text("#!/bin/bash\nexit 0\n")
    script.chmod(0o755)
    adapter = make_adapter(tmp_path, navigation_script=str(script))
    calls = []

    def fake_run(cmd, **kwargs):
        calls.append(cmd)
        return subprocess.CompletedProcess(cmd, 0, "", "")

    monkeypatch.setattr("roamerx_edge.mapping_adapter.subprocess.run", fake_run)
    adapter._stop_conflicting_navigation_stack()
    assert calls[0][:2] == [str(script), "full-stop"]


def test_ensure_slam_starts_systemd_unit(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path, mapping_unit="roamerx-mapping.service")
    calls = []
    active = {"value": False}

    def fake_systemctl(action):
        calls.append(action)
        if action == "start":
            active["value"] = True
        return {}

    monkeypatch.setattr(adapter, "_systemctl", fake_systemctl)
    monkeypatch.setattr(adapter, "_is_mapping_unit_active", lambda: active["value"])
    monkeypatch.setattr(adapter, "_wait_for_slam_services", lambda: calls.append("wait"))
    adapter._ensure_slam_process()
    assert calls == ["start", "wait"]


def test_wait_until_ready_for_motion(tmp_path):
    active = tmp_path / "20260717_140000_001"
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
    status = adapter.wait_until_ready_for_motion(timeout_seconds=1)
    assert status["ready_for_motion"] is True


def test_local_save_skips_upload_and_keeps_process(tmp_path, monkeypatch):
    session = tmp_path / "20260717_150000_001"
    session.mkdir()
    (session / "map.yaml").write_text("resolution: 0.05\n")
    (session / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (session / "save_progress.json").write_text(
        json.dumps(
            {
                "stage": "mapping",
                "keyframe_count": 2,
                "written_keyframes": 2,
                "updated_at_unix": time.time(),
                "slam_health": {"state": "healthy", "imu_initialized": True},
            }
        )
    )
    adapter = make_adapter(tmp_path)
    adapter._slam_process = RunningProcess()
    adapter.session = MappingSession("session", "map", "", "mapping", "now", "now")
    adapter.media_client = SimpleNamespace(
        upload_map_package=lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("upload"))
    )
    monkeypatch.setattr(adapter, "_call_map_state", lambda data: "ok")
    monkeypatch.setattr(adapter, "_wait_for_complete_map_dir", lambda *_args, **_kwargs: session)
    monkeypatch.setattr(
        adapter,
        "_stop_slam_process",
        lambda: (_ for _ in ()).throw(AssertionError("stop")),
    )
    result = adapter._save_active_mapping({"upload": False, "package": False, "stop_process": False})
    assert adapter.session.state == "saving"
    assert "upload_result" not in result
