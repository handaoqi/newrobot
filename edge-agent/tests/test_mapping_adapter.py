import json
import hashlib
import math
import subprocess
import threading
import time
import zipfile
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

import roamerx_edge.mapping_adapter as mapping_adapter_module
from roamerx_edge.config import MappingConfig
from roamerx_edge.mapping_adapter import (
    MappingAdapter,
    MappingSession,
    _parse_post_save_localization_check,
    _parse_scan_context_check,
)
from roamerx_edge.protocol import ProtocolError


class RunningProcess:
    def poll(self):
        return None


def make_adapter(tmp_path, **overrides):
    overrides.setdefault("rosbag_script", str(tmp_path / "missing-rosbag-script"))
    overrides.setdefault("mapping_unit", "")
    overrides.setdefault("navigation_script", str(tmp_path / "missing-nav-script"))
    overrides.setdefault("deployment_manifest", str(tmp_path / "missing-deployment.json"))
    overrides.setdefault("mapping_session_params_file", str(tmp_path / "mapping-origin-session.yaml"))
    overrides.setdefault("mapping_environment_file", str(tmp_path / "mapping-session.env"))
    overrides.setdefault(
        "localization_scan_context_check_binary",
        str(tmp_path / "missing-scan-context-check"),
    )
    config = MappingConfig(map_dir=str(tmp_path), **overrides)
    media = SimpleNamespace(robot_id="test-dog")
    return MappingAdapter(config, media)


def test_scan_context_validation_output_is_structured_for_manual_threshold_review():
    parsed = _parse_scan_context_check(
        "TOTAL: 76/81 answerable  top1 98.7%  topk 100.0%  "
        "top1_err_med 0.53 m  yaw_err_med 0.32 deg  yaw_err_p90 14.75 deg  "
        "dist_hit_p90 0.07  dist_miss_med 0.10\n"
    )

    assert parsed == {
        "queries": 81,
        "answerable": 76,
        "top1_hit_percent": 98.7,
        "topk_hit_percent": 100.0,
        "top1_position_error_median_m": 0.53,
        "yaw_error_median_deg": 0.32,
        "yaw_error_p90_deg": 14.75,
        "descriptor_hit_p90": 0.07,
        "descriptor_miss_median": 0.10,
    }


def test_post_save_localization_output_parser_ignores_ros_logs():
    result = _parse_post_save_localization_check(
        "[INFO] localization ready\n"
        + json.dumps({
            "schema": "roamerx.post-save-localization-check.v1",
            "state": "passed",
            "accurate": True,
            "pose": {"x": 1.2, "y": -0.5, "yaw_deg": 91.0},
        })
        + "\n"
    )

    assert result["state"] == "passed"
    assert result["accurate"] is True
    assert result["pose"]["x"] == 1.2


def test_post_save_validation_loads_exact_saved_map_and_persists_pose(tmp_path, monkeypatch):
    navigation_script = tmp_path / "start_navigation_real.sh"
    navigation_script.write_text("#!/bin/sh\n")
    saved_map = tmp_path / "20260830_120000_001"
    saved_map.mkdir()
    for name in ("map.pcd", "map.yaml", "map.txt", "map_manifest.json"):
        (saved_map / name).write_text("test")
    adapter = make_adapter(tmp_path, navigation_script=str(navigation_script))
    calls = []
    checker_payload = {
        "schema": "roamerx.post-save-localization-check.v1",
        "state": "passed",
        "accurate": True,
        "mapping_type": "outdoor",
        "map_dir": str(saved_map),
        "frame_id": "map",
        "pose": {"x": 4.25, "y": -1.5, "yaw_deg": 87.0},
        "position_error_m": 0.08,
        "yaw_error_deg": 1.2,
    }

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        if isinstance(command, list) and command[-1] == "restart-localization":
            return SimpleNamespace(returncode=0, stdout="ready", stderr="")
        return SimpleNamespace(returncode=0, stdout=json.dumps(checker_payload), stderr="")

    monkeypatch.setattr(mapping_adapter_module.subprocess, "run", fake_run)

    adapter._run_post_save_validation(str(saved_map), "outdoor")

    assert calls[0][0] == [str(navigation_script), "restart-localization"]
    assert calls[0][1]["env"]["PCD_MAP"] == str(saved_map / "map.pcd")
    assert calls[0][1]["env"]["MAP_YAML"] == str(saved_map / "map.yaml")
    assert calls[1][0][0] == "python3"
    assert calls[1][0][-4:] == ["--mapping-type", "outdoor", "--map-dir", str(saved_map)]
    assert calls[1][1]["shell"] is False
    persisted = json.loads(adapter._post_save_validation_file.read_text())
    assert persisted["state"] == "passed"
    assert persisted["mapping_type"] == "outdoor"
    assert persisted["result"]["pose"]["x"] == 4.25
    assert persisted["outdoor"]["attempts"] == 1
    assert persisted["outdoor"]["success"] == 1


def test_post_save_validation_rejects_unstructured_or_wrong_map_output(tmp_path, monkeypatch):
    navigation_script = tmp_path / "start_navigation_real.sh"
    navigation_script.write_text("#!/bin/sh\n")
    saved_map = tmp_path / "20260830_120000_002"
    saved_map.mkdir()
    for name in ("map.pcd", "map.yaml", "map.txt", "map_manifest.json"):
        (saved_map / name).write_text("test")
    adapter = make_adapter(tmp_path, navigation_script=str(navigation_script))
    checker_results = iter([
        SimpleNamespace(returncode=0, stdout="PASS", stderr=""),
        SimpleNamespace(returncode=0, stdout=json.dumps({
            "schema": "roamerx.post-save-localization-check.v1",
            "state": "passed",
            "accurate": True,
            "mapping_type": "indoor",
            "map_dir": str(tmp_path / "different-map"),
            "message": "静止定位准确",
        }), stderr=""),
    ])

    def fake_run(command, **_kwargs):
        if isinstance(command, list) and command[-1] in {"restart-localization", "stop-localization"}:
            return SimpleNamespace(returncode=0, stdout="ready", stderr="")
        return next(checker_results)

    monkeypatch.setattr(mapping_adapter_module.subprocess, "run", fake_run)

    adapter._run_post_save_validation(str(saved_map), "indoor")
    first = json.loads(adapter._post_save_validation_file.read_text())
    assert first["state"] == "failed"
    assert first["result"]["reason_code"] == "LOCALIZATION_CHECK_OUTPUT_INVALID"
    assert first["indoor"]["success"] == 0

    adapter._run_post_save_validation(str(saved_map), "indoor")
    second = json.loads(adapter._post_save_validation_file.read_text())
    assert second["state"] == "failed"
    assert second["result"]["reason_code"] == "LOCALIZATION_CHECK_IDENTITY_MISMATCH"
    assert second["indoor"]["attempts"] == 2
    assert second["indoor"]["success"] == 0


def test_finalize_calls_global_graph_without_loop_closure(tmp_path, monkeypatch):
    session_dir = tmp_path / "20260825_180000_001"
    session_dir.mkdir()
    adapter = make_adapter(tmp_path)
    adapter.session = MappingSession("session", "map", "", "saving", "now", "now")
    calls = []
    monkeypatch.setattr(mapping_adapter_module, "finalize_map_package", lambda *_args, **_kwargs: {
        "keyframe_count": 2,
        "loop_status": "no_valid_loop",
        "loop_closure_count": 0,
        "loop_candidate_count": 3,
        "completeness": "complete",
    })
    monkeypatch.setattr(mapping_adapter_module, "build_optimization_summary", lambda *_args, **_kwargs: {
        "stage": "no_valid_loop",
        "success": True,
        "trajectory_source": "optimized",
        "use_gps": False,
        "auto_activation_allowed": True,
        "corrections": [],
    })
    monkeypatch.setattr(adapter, "_call_ros_service", lambda *args: calls.append(args))

    manifest = adapter._finalize_session_package(session_dir, {})

    assert calls == [("/slam/global_optimize", "std_srvs/srv/Trigger", "{}")]
    assert manifest["trajectory_source"] == "optimized"


def test_mapping_deployment_hash_mismatch_blocks_start(tmp_path):
    slam_binary = tmp_path / "mapping"
    slam_params = tmp_path / "config.yaml"
    slam_binary.write_bytes(b"binary-v1")
    slam_params.write_text("config-v1")
    adapter = make_adapter(
        tmp_path,
        slam_binary=str(slam_binary),
        slam_params_file=str(slam_params),
        slam_command="exec test-mapping --params test-config",
        deployment_manifest=str(tmp_path / "deployment.json"),
        deployment_manifest_required=True,
    )
    mapping_adapter_path = Path(mapping_adapter_module.__file__).resolve()
    digest = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    (tmp_path / "deployment.json").write_text(json.dumps({
        "configuration": {
            "mapping_unit": "",
            "slam_command": adapter.config.slam_command,
        },
        "artifacts": {
            "slam_binary": {"path": str(slam_binary), "sha256": digest(slam_binary)},
            "slam_params_file": {"path": str(slam_params), "sha256": digest(slam_params)},
            "mapping_adapter": {"path": str(mapping_adapter_path), "sha256": digest(mapping_adapter_path)},
        },
    }))

    adapter._verify_mapping_deployment()
    slam_binary.write_bytes(b"binary-v2")

    with pytest.raises(ProtocolError, match="SHA256") as error:
        adapter._verify_mapping_deployment()
    assert error.value.code == "MAPPING_DEPLOYMENT_MISMATCH"


def test_outdoor_runtime_uses_locked_origin_and_records_hash(tmp_path):
    adapter = make_adapter(tmp_path)
    adapter._mapping_type = "outdoor"
    adapter.session = MappingSession(
        "mapping-session", "outside", "", "slam_warmup", "now", "now",
        mapping_type="outdoor",
    )
    adapter._origin_file.write_text(yaml.safe_dump({
        "alignment_locked": True,
        "origin_lock_session_id": "origin-session",
        "origin_latitude": 39.9042,
        "origin_longitude": 116.4074,
        "origin_altitude": 43.5,
    }))

    runtime = adapter._prepare_mapping_runtime()

    params = yaml.safe_load(Path(adapter.config.mapping_session_params_file).read_text())
    converter = params["slam_enu_converter"]["ros__parameters"]
    assert converter["lat0"] == 39.9042
    assert converter["lon0"] == 116.4074
    assert converter["alt0"] == 43.5
    assert converter["origin_session_id"] == "origin-session"
    assert converter["origin_sha256"] == runtime["origin_sha256"]
    assert adapter.session.origin_sha256 == runtime["origin_sha256"]
    assert "ROAMERX_MAPPING_TYPE=outdoor" in Path(
        adapter.config.mapping_environment_file
    ).read_text()
    assert "ROAMERX_AUTO_LOOP_OPTIMIZATION=false" in Path(
        adapter.config.mapping_environment_file
    ).read_text()


def test_outdoor_runtime_rejects_all_zero_origin(tmp_path):
    adapter = make_adapter(tmp_path)
    adapter._mapping_type = "outdoor"
    adapter.session = MappingSession(
        "mapping-session", "outside", "", "slam_warmup", "now", "now",
        mapping_type="outdoor",
    )
    adapter._origin_file.write_text(yaml.safe_dump({
        "alignment_locked": True,
        "origin_lock_session_id": "origin-session",
        "origin_latitude": 0.0,
        "origin_longitude": 0.0,
        "origin_altitude": 0.0,
    }))

    with pytest.raises(ProtocolError) as error:
        adapter._prepare_mapping_runtime()

    assert error.value.code == "MAPPING_ORIGIN_LOCK_FAILED"


def test_auto_rescue_keeps_post_trigger_rosbag_tail(tmp_path, monkeypatch):
    source = tmp_path / "20260825_190000_001"
    source.mkdir()
    (source / "save_progress.json").write_text(json.dumps({"error_code": "SLAM_DIVERGED"}))
    adapter = make_adapter(tmp_path, divergence_post_record_seconds=3.0)
    adapter.session = MappingSession("session", "map", "", "mapping", "now", "now")
    calls = []
    monkeypatch.setattr(mapping_adapter_module.time, "sleep", lambda seconds: calls.append(("sleep", seconds)))
    monkeypatch.setattr(adapter, "_stop_rosbag", lambda: calls.append(("stop_rosbag",)))
    monkeypatch.setattr(
        adapter,
        "_rescue_diverged_mapping",
        lambda command, directory: calls.append(("rescue", command, directory)) or {"rescued": True},
    )

    result = adapter.auto_rescue_diverged_mapping({"map_dir": str(source)})

    assert result == {"rescued": True}
    assert calls[0] == ("sleep", 3.0)
    assert calls[1] == ("stop_rosbag",)
    assert calls[2][0] == "rescue"


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
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda **_kwargs: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors"))
    monkeypatch.setattr(adapter, "_ensure_origin_odometry", lambda: calls.append("odom"))
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
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda **_kwargs: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors"))
    monkeypatch.setattr(adapter, "_ensure_origin_odometry", lambda: calls.append("odom"))
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
    assert calls == ["stop_nav", "sensors", "odom"]


def test_outdoor_origin_preparation_starts_diagnostic_bag(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    calls = []
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda **_kwargs: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors"))
    monkeypatch.setattr(adapter, "_ensure_origin_odometry", lambda: calls.append("odom"))
    monkeypatch.setattr(adapter, "_rosbag_status", lambda: {"running": False})
    monkeypatch.setattr(adapter, "_start_rosbag", lambda label: calls.append(("start_bag", label)))
    monkeypatch.setattr(adapter, "status", lambda: {
        "state": adapter.session.state,
        "origin": adapter._origin_monitor.status(),
    })

    result = adapter.start_origin_lock({
        "map_name": "outside",
        "scene_scope": "outdoor",
        "mapping_type": "outdoor",
        "record_rosbag": True,
        "prepare_only": True,
    })

    assert result["state"] == "origin_waiting"
    assert calls == ["stop_nav", "sensors", ("start_bag", "outside"), "odom"]


def test_indoor_slam_warmup_starts_diagnostic_bag_before_slam(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    calls = []
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda **_kwargs: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors"))
    monkeypatch.setattr(adapter, "_rosbag_status", lambda: {"running": False})
    monkeypatch.setattr(adapter, "_start_rosbag", lambda label: calls.append(("start_bag", label)))
    monkeypatch.setattr(adapter, "_ensure_slam_process", lambda: calls.append("slam"))
    monkeypatch.setattr(adapter, "_call_map_state", lambda data: calls.append(("state", data)))
    monkeypatch.setattr(adapter, "status", lambda: {
        "state": adapter.session.state,
        "mapping_capture_enabled": adapter.session.mapping_capture_enabled,
    })

    result = adapter.start_slam_warmup({
        "map_name": "inside",
        "mapping_type": "indoor",
        "record_rosbag": True,
    })

    assert result == {"state": "slam_warmup", "mapping_capture_enabled": False}
    assert calls == ["stop_nav", "sensors", ("start_bag", "inside"), "slam", ("state", 7)]


def test_origin_lock_reuses_prepared_waiting_session(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    calls = []
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda **_kwargs: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors"))
    monkeypatch.setattr(adapter, "_ensure_origin_odometry", lambda: calls.append("odom"))
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
    assert calls == ["stop_nav", "sensors", "odom"]


def test_origin_lock_retries_after_quality_failure(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    calls = []
    monkeypatch.setattr(adapter, "_stop_conflicting_navigation_stack", lambda **_kwargs: calls.append("stop_nav"))
    monkeypatch.setattr(adapter, "_ensure_mapping_sensors", lambda: calls.append("sensors"))
    monkeypatch.setattr(adapter, "_ensure_origin_odometry", lambda: calls.append("odom"))
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
        adapter._origin_monitor._status.update(origin_status="waiting_fix", message="retrying")
        return adapter._origin_monitor.status()

    monkeypatch.setattr(adapter._origin_monitor, "start", retry_start)
    result = adapter.start_origin_lock({
        "mapping_session_id": session_id,
        "scene_scope": "outdoor",
        "mapping_type": "outdoor",
    })

    assert result["state"] == "origin_waiting"
    assert result["origin"]["origin_status"] == "waiting_fix"
    assert adapter.session.session_id == session_id
    assert calls == ["stop_nav", "sensors", "odom"]


def test_workflow_persist_survives_concurrent_writers(tmp_path):
    adapter = make_adapter(tmp_path)
    adapter.session = MappingSession(
        session_id="session-race",
        map_name="outside",
        route_hint="",
        state="origin_waiting",
        started_at="2026-08-24T05:52:00+00:00",
        updated_at="2026-08-24T05:52:00+00:00",
        scene_scope="outdoor",
        mapping_type="outdoor",
    )
    errors = []

    def writer(state):
        try:
            adapter._set_state(state)
        except Exception as exc:
            errors.append(exc)

    threads = [
        threading.Thread(target=writer, args=("origin_waiting",)),
        threading.Thread(target=writer, args=("origin_locked",)),
        threading.Thread(target=writer, args=("failed",)),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert errors == []
    payload = json.loads(adapter._origin_state_file.read_text(encoding="utf-8"))
    assert payload["session"]["session_id"] == "session-race"
    assert payload["session"]["state"] in {"origin_waiting", "origin_locked", "failed"}
    leftover = list(tmp_path.glob("mapping_workflow.json*.tmp")) + list(tmp_path.glob(".mapping_workflow.json.*.tmp"))
    assert leftover == []


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
            "confirmed_heading_deg": 13.5,
            "heading_confirmed": True,
            "heading_confirmed_at_unix": 1787570000.0,
        },
    })

    assert result["global_enu"]["source_map_id"] == "map-7"
    assert result["global_enu"]["confirmed_heading_deg"] == pytest.approx(13.5)
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
                "hgt_std": 0.015, "diff_age_s": 0.2, "soln_svs_num": 18,
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
    assert sample.vertical_std_m == pytest.approx(0.015)
    assert sample.position_type == 48
    assert sample.solution_status == 0
    assert sample.solution_satellites == 18
    assert sample.differential_age_seconds == pytest.approx(0.2)


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
        "heading_confirmed": True,
        "confirmed_heading_deg": 93.2,
        "heading_confirmed_at_unix": 1787570000.0,
    }))
    work = tmp_path / "20260822_120000_001"
    work.mkdir()
    exported = work / "gnss_origin.yaml"
    exported.write_text(yaml.safe_dump({"alignment_locked": 1, "enu_to_map_yaw": 0.25}))

    adapter._merge_locked_origin_metadata(work)

    merged = yaml.safe_load(exported.read_text())
    assert merged["origin_lock_session_id"] == "origin-session"
    assert merged["position_spread_m"] == pytest.approx(0.012)
    assert merged["confirmed_heading_deg"] == pytest.approx(93.2)
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


def test_quality_rejected_optimization_restores_complete_raw_map_set(tmp_path):
    session = tmp_path / "20260828_190000_001"
    session.mkdir()
    raw_values = {
        "map_raw.pcd": b"raw-pcd",
        "map_raw.pgm": b"raw-pgm",
        "map_raw.yaml": b"raw-yaml",
        "map_raw.txt": b"raw-trajectory",
    }
    optimized_values = {
        "map.pcd": b"optimized-pcd",
        "map.pgm": b"optimized-pgm",
        "map.yaml": b"optimized-yaml",
        "map.txt": b"optimized-trajectory",
    }
    for name, value in {**raw_values, **optimized_values}.items():
        (session / name).write_bytes(value)

    result = MappingAdapter._restore_raw_map_products(session)

    assert result["restored"] is True
    for active_name, raw_name in (
        ("map.pcd", "map_raw.pcd"),
        ("map.pgm", "map_raw.pgm"),
        ("map.yaml", "map_raw.yaml"),
        ("map.txt", "map_raw.txt"),
    ):
        assert (session / active_name).read_bytes() == raw_values[raw_name]
    assert (session / "map_optimized_rejected.pcd").read_bytes() == b"optimized-pcd"


def test_quality_rejected_map_does_not_replace_local_active_links(tmp_path, monkeypatch):
    session = tmp_path / "20260828_191000_001"
    session.mkdir()
    (session / "map.yaml").write_text("resolution: 0.05\n")
    (session / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (session / "map.pcd").write_bytes(b"raw-fallback")
    (session / "optimization_summary.json").write_text(json.dumps({
        "stage": "quality_rejected",
        "auto_activation_allowed": False,
        "inertial_smoothing_guard": {
            "passed": False,
            "reasons": ["max_xy_correction_m_above_limit"],
        },
    }))
    adapter = make_adapter(tmp_path, visibility_filter_enabled=False)
    monkeypatch.setattr(adapter, "_generate_map_preview", lambda _base: None)

    _package, metadata = adapter._package_map({}, session)

    assert metadata["auto_activate"] is False
    assert not (tmp_path / "map.pcd").exists()
    assert not (tmp_path / "map.yaml").exists()


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
    preintegration = session / "imu_preintegration"
    preintegration.mkdir()
    (preintegration / "preint_00000.json").write_text(
        '{"schema_version":2,"measurements":[]}'
    )
    adapter = make_adapter(tmp_path, visibility_filter_enabled=False)
    monkeypatch.setattr(adapter, "_generate_map_preview", lambda _base: None)

    package, metadata = adapter._package_map({}, session)

    with zipfile.ZipFile(package) as archive:
        assert "gnss_origin.yaml" in archive.namelist()
        assert "map.pcd" in archive.namelist()
        assert "keyframes/keyframes.csv" in archive.namelist()
        assert "scan_context/index.json" in archive.namelist()
        assert "imu_preintegration/preint_00000.json" in archive.namelist()
    assert "gnss_origin.yaml" in metadata["files"]
    metrics = metadata["mapping_metrics"]
    assert metrics["package_size_bytes"] == package.stat().st_size
    assert metrics["robot_directory_size_bytes"] > 0
    assert metrics["keyframe_count"] == 1
    assert metrics["diagnostic_data_size_bytes"] >= 0


def test_map_package_keeps_recorded_rosbag_on_robot(tmp_path, monkeypatch):
    session = tmp_path / "20260824_173000_001"
    session.mkdir()
    (session / "map.yaml").write_text("resolution: 0.05\n")
    (session / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (session / "map.pcd").write_bytes(b"pcd")
    bag_dir = tmp_path / "diagnostic-bag"
    bag_dir.mkdir()
    (bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    (bag_dir / "mapping_0.db3").write_bytes(b"fake-db3-payload")
    adapter = make_adapter(tmp_path, visibility_filter_enabled=False)
    adapter._rosbag_dir = str(bag_dir)
    monkeypatch.setattr(adapter, "_generate_map_preview", lambda _base: None)

    package, metadata = adapter._package_map({}, session)

    with zipfile.ZipFile(package) as archive:
        names = archive.namelist()
    assert all(not name.startswith("diagnostics/rosbag/") for name in names)
    assert (bag_dir / "mapping_0.db3").read_bytes() == b"fake-db3-payload"
    assert metadata["local_rosbag_dir"] == str(bag_dir)
    assert "diagnostics/rosbag/mapping_0.db3" not in metadata["files"]


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
    heading_offset = math.pi
    yaw_enu = math.pi / 2.0 - math.radians(heading_deg)
    alignment_yaw = math.atan2(
        math.sin(slam_yaw - yaw_enu - heading_offset),
        math.cos(slam_yaw - yaw_enu - heading_offset),
    )
    (session / "map.yaml").write_text("resolution: 0.05\n")
    (session / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (session / "map.txt").write_text("0 0 0\n")
    (session / "gnss_origin.yaml").write_text(
        "origin_latitude: 39.9714186649\norigin_longitude: 116.4483795514\n"
        f"alignment_locked: 1\nenu_to_map_yaw: {alignment_yaw}\nheading_offset_deg: 180.0\n"
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
    assert sample["rtk"]["base_heading_deg"] == pytest.approx((heading_deg - 180.0) % 360.0)
    assert trace["heading_offset_deg"] == pytest.approx(180.0)


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


def test_complete_manifest_overrides_stale_post_optimization_progress(tmp_path):
    completed = tmp_path / "20260824_193739_951"
    completed.mkdir()
    (completed / "map.yaml").write_text("resolution: 0.05\n")
    (completed / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (completed / "map_manifest.json").write_text(json.dumps({"completeness": "complete"}))
    (completed / "save_progress.json").write_text(json.dumps({
        "stage": "writing_pcd",
        "progress_percent": 65.0,
        "recoverable": True,
    }))

    status = make_adapter(tmp_path).status()

    assert status["state"] == "idle"
    assert status["process_alive"] is False


def test_mark_progress_completed_closes_post_optimization_export(tmp_path):
    completed = tmp_path / "20260824_193739_951"
    completed.mkdir()
    (completed / "save_progress.json").write_text(json.dumps({
        "stage": "writing_pcd",
        "progress_percent": 65.0,
        "recoverable": True,
        "error": "",
    }))
    adapter = make_adapter(tmp_path)

    adapter._mark_progress_completed(completed)

    progress = json.loads((completed / "save_progress.json").read_text())
    assert progress["stage"] == "completed"
    assert progress["progress_percent"] == 100.0
    assert progress["recoverable"] is False


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


def test_origin_stop_preserves_localization_and_ensures_odom(tmp_path, monkeypatch):
    script = tmp_path / "start_navigation_real.sh"
    script.write_text("#!/bin/bash\nexit 0\n")
    script.chmod(0o755)
    adapter = make_adapter(tmp_path, navigation_script=str(script))
    calls = []

    def fake_run(cmd, **kwargs):
        calls.append(cmd)
        return subprocess.CompletedProcess(cmd, 0, "", "")

    monkeypatch.setattr("roamerx_edge.mapping_adapter.subprocess.run", fake_run)
    adapter._stop_conflicting_navigation_stack(preserve_localization=True)
    adapter._ensure_origin_odometry()
    adapter._stop_localization_for_slam()

    script_calls = [call for call in calls if call and call[0] == str(script)]
    assert [call[1] for call in script_calls] == ["stop", "ensure-localization-odom", "stop-localization"]


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
    monkeypatch.setattr(adapter, "_verify_mapping_publishers", lambda: calls.append("publishers"))
    adapter._ensure_slam_process()
    assert calls == ["start", "wait", "publishers"]


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


def test_lio_odometry_is_never_matched_when_ps_output_is_narrow(tmp_path, monkeypatch):
    # Navigation's LIO node is the same executable as the mapping node; only
    # "-r __node:=lio_odometry" separates them, ~70 chars into the line. ps
    # truncates args to $COLUMNS, so without -ww an 80-column caller loses the
    # marker and _stop_orphan_slam_processes() SIGKILLs live navigation.
    adapter = make_adapter(tmp_path)
    lio_args = (
        "/home/dogrobot/robot/install/robot_slam/lib/robot_slam/mapping "
        "--ros-args -r __node:=lio_odometry --params-file "
        "/home/dogrobot/robot/install/robot_slam/share/robot_slam/config/config.yaml"
    )
    seen_argv = {}

    def fake_run(argv, **_kwargs):
        seen_argv["argv"] = argv
        # Emulate ps honouring -ww: full args only when it is present.
        args = lio_args if "-ww" in argv else lio_args[:71]
        return SimpleNamespace(returncode=0, stdout=f"  57606 {args}\n")

    monkeypatch.setattr(mapping_adapter_module.subprocess, "run", fake_run)

    assert adapter._find_slam_process_pids() == []
    assert "-ww" in seen_argv["argv"]


def test_slam_process_scan_ignores_shell_that_only_mentions_mapping(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    shell_args = (
        "/bin/bash -c python3 -m roamerx_edge.mapping_cli status; "
        "pgrep -af '/robot_slam/lib/robot_slam/mapping'; "
        "rg 'ros2 launch robot_slam'"
    )

    monkeypatch.setattr(
        mapping_adapter_module.subprocess,
        "run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0,
            stdout=f"  57607 {shell_args}\n",
        ),
    )

    assert adapter._find_slam_process_pids() == []


def test_slam_process_scan_matches_mapping_binary_and_ros_launch(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    mapping_args = (
        "/home/dogrobot/robot/install/robot_slam/lib/robot_slam/mapping "
        "--ros-args -r __node:=mapping"
    )
    launch_args = (
        "/usr/bin/python3 /opt/ros/humble/bin/ros2 launch robot_slam "
        "unified_mapping.launch.py"
    )

    monkeypatch.setattr(
        mapping_adapter_module.subprocess,
        "run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0,
            stdout=f"  57608 {mapping_args}\n  57609 {launch_args}\n",
        ),
    )

    assert adapter._find_slam_process_pids() == [57608, 57609]


def _ready_to_save_adapter(tmp_path, monkeypatch):
    """An adapter parked one step before the export, with a healthy save_progress."""
    session = tmp_path / "20260828_120000_001"
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
    stopped = []
    monkeypatch.setattr(adapter, "_call_map_state", lambda data: "ok")
    monkeypatch.setattr(adapter, "_wait_for_complete_map_dir", lambda *_a, **_kw: session)
    monkeypatch.setattr(adapter, "_stop_slam_process", lambda: stopped.append("slam"))
    return adapter, session, stopped


def test_save_stops_slam_when_packaging_fails(tmp_path, monkeypatch):
    # The save service already returned, so the map is on disk and SLAM has
    # nothing left to do. A packaging failure used to leak the mapping node and
    # block the next start_mapping with MAPPING_ALREADY_ACTIVE.
    adapter, _session, stopped = _ready_to_save_adapter(tmp_path, monkeypatch)
    monkeypatch.setattr(
        adapter,
        "_package_map",
        lambda *_a, **_kw: (_ for _ in ()).throw(ProtocolError("MAP_PACKAGE_FAILED", "boom")),
    )

    with pytest.raises(ProtocolError) as excinfo:
        adapter._save_active_mapping({"upload": False, "package": True})

    assert excinfo.value.code == "MAP_PACKAGE_FAILED"
    assert stopped == ["slam"]
    assert adapter.session.state == "failed"


def test_save_stops_slam_when_upload_fails(tmp_path, monkeypatch):
    adapter, _session, stopped = _ready_to_save_adapter(tmp_path, monkeypatch)
    monkeypatch.setattr(adapter, "_package_map", lambda *_a, **_kw: (tmp_path / "map.zip", {}))
    adapter.media_client = SimpleNamespace(
        upload_map_package=lambda *_a, **_kw: (_ for _ in ()).throw(
            ProtocolError("MAP_UPLOAD_FAILED", "network")
        )
    )

    with pytest.raises(ProtocolError) as excinfo:
        adapter._save_active_mapping({"upload": True})

    assert excinfo.value.code == "MAP_UPLOAD_FAILED"
    assert stopped == ["slam"]
    assert adapter.session.state == "failed"


def test_save_teardown_failure_does_not_mask_the_export_error(tmp_path, monkeypatch):
    # The operator needs the reason the export failed, not whatever went wrong
    # while tearing SLAM down afterwards.
    adapter, _session, _stopped = _ready_to_save_adapter(tmp_path, monkeypatch)
    monkeypatch.setattr(
        adapter,
        "_package_map",
        lambda *_a, **_kw: (_ for _ in ()).throw(ProtocolError("MAP_PACKAGE_FAILED", "boom")),
    )
    monkeypatch.setattr(
        adapter,
        "_stop_slam_process",
        lambda: (_ for _ in ()).throw(RuntimeError("systemctl unavailable")),
    )

    with pytest.raises(ProtocolError) as excinfo:
        adapter._save_active_mapping({"upload": False, "package": True})

    assert excinfo.value.code == "MAP_PACKAGE_FAILED"


def test_save_defaults_to_stopping_slam(tmp_path, monkeypatch):
    # The platform never sends stop_process, so the default is what mapping.save
    # actually runs. "Mapping finished" has to mean the node exits.
    adapter, _session, stopped = _ready_to_save_adapter(tmp_path, monkeypatch)

    result = adapter._save_active_mapping({"upload": False, "package": False})

    # Not an equality check: the success path stops SLAM once itself and again
    # via _cleanup(). The repeat is a harmless no-op, so only "it stopped" matters.
    assert stopped
    assert result["state"] == "exited"


def test_outdoor_save_keeps_mapping_type_for_post_save_validation(tmp_path, monkeypatch):
    adapter, session_dir, _stopped = _ready_to_save_adapter(tmp_path, monkeypatch)
    adapter._mapping_type = "outdoor"
    adapter.session.mapping_type = "outdoor"
    validation_calls = []
    monkeypatch.setattr(adapter, "_finalize_session_package", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(
        adapter,
        "_start_post_save_validation",
        lambda map_dir, mapping_type: validation_calls.append((map_dir, mapping_type)),
    )

    result = adapter._save_active_mapping({"upload": False, "package": False})

    assert result["state"] == "exited"
    assert validation_calls == [(str(session_dir), "outdoor")]


def test_save_keeps_slam_when_the_save_service_itself_fails(tmp_path, monkeypatch):
    # Nothing was exported, so mapping can legitimately continue and be retried.
    adapter, _session, stopped = _ready_to_save_adapter(tmp_path, monkeypatch)
    monkeypatch.setattr(
        adapter,
        "_call_map_state",
        lambda data: (_ for _ in ()).throw(ProtocolError("MAP_SAVE_FAILED", "service down")),
    )

    with pytest.raises(ProtocolError) as excinfo:
        adapter._save_active_mapping({"upload": False, "package": False})

    assert excinfo.value.code == "MAP_SAVE_FAILED"
    assert stopped == []
    assert adapter.session.state == "mapping"


def test_save_keeps_slam_when_not_ready(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    adapter._slam_process = RunningProcess()
    adapter.session = MappingSession("session", "map", "", "mapping", "now", "now")
    stopped = []
    monkeypatch.setattr(adapter, "_stop_slam_process", lambda: stopped.append("slam"))

    with pytest.raises(ProtocolError) as excinfo:
        adapter._save_active_mapping({"upload": False, "package": False})

    assert excinfo.value.code == "MAPPING_NOT_READY"
    assert stopped == []


def test_begin_mapping_outdoor_requires_explicit_operator_confirmation(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    adapter.session = MappingSession(
        session_id="session-heading",
        map_name="outside",
        route_hint="",
        state="ready_to_map",
        started_at="2026-08-24T07:00:00+00:00",
        updated_at="2026-08-24T07:00:00+00:00",
        scene_scope="outdoor",
        mapping_type="outdoor",
    )
    monkeypatch.setattr(adapter, "status", lambda: {
        "readiness": {"imu_initialized": True, "slam_pose_ready": True, "message": "ok"},
        "origin": {
            "origin_status": "locked",
            "heading_review_status": "manual_confirmation",
            "heading_stable": False,
        },
    })

    with pytest.raises(ProtocolError) as error:
        adapter.begin_mapping({"heading_check_confirmed": False})

    assert error.value.code == "MAPPING_HEADING_NOT_CONFIRMED"


def test_begin_mapping_outdoor_locks_heading_after_confirm(tmp_path, monkeypatch):
    adapter = make_adapter(tmp_path)
    adapter.session = MappingSession(
        session_id="session-heading",
        map_name="outside",
        route_hint="",
        state="ready_to_map",
        started_at="2026-08-24T07:00:00+00:00",
        updated_at="2026-08-24T07:00:00+00:00",
        scene_scope="outdoor",
        mapping_type="outdoor",
    )
    calls = []
    monkeypatch.setattr(adapter, "status", lambda: {
        "readiness": {"imu_initialized": True, "slam_pose_ready": True, "message": "ok"},
        "origin": {
            "origin_status": "locked",
            "heading_review_status": "manual_confirmation",
            "heading_stable": False,
        },
        "state": adapter.session.state,
        "mapping_capture_enabled": adapter.session.mapping_capture_enabled,
        "rtk_alignment": {"locked": True, "source": "heading"},
    })
    monkeypatch.setattr(adapter, "_rosbag_status", lambda: {"running": False})
    monkeypatch.setattr(adapter, "_call_map_state", lambda data: calls.append(("state", data)))
    monkeypatch.setattr(adapter._origin_monitor, "confirm_heading_review", lambda: calls.append("confirm"))
    monkeypatch.setattr(adapter._origin_monitor, "stop", lambda: calls.append("stop"))
    monkeypatch.setattr(
        adapter,
        "_wait_for_heading_alignment",
        lambda timeout_seconds=2.5: {"locked": True, "source": "heading"},
    )

    result = adapter.begin_mapping({"heading_check_confirmed": True})

    assert calls[:2] == [("state", adapter.config.start_data), "confirm"]
    assert "stop" not in calls
    assert adapter.session.mapping_capture_enabled is True
    assert adapter.session.heading_check_confirmed is True
    assert adapter.session.state == "mapping"
    assert result["rtk_alignment"]["locked"] is True


def test_mapping_readiness_reports_dual_antenna_heading_lock():
    progress = {
        "stage": "mapping",
        "mapping_capture_enabled": True,
        "keyframe_count": 3,
        "written_keyframes": 3,
        "updated_at_unix": time.time(),
        "slam_health": {"state": "healthy", "imu_initialized": True, "slam_pose_ready": True},
        "rtk_alignment": {"locked": True, "source": "heading", "fusion_enabled": True},
    }

    readiness = MappingAdapter._mapping_readiness(progress, True)

    assert readiness["state"] == "ready"
    assert "双天线锁定" in readiness["message"]
