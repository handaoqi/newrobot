from types import SimpleNamespace

from roamerx_edge.safety_policy import RuntimeSafetyState
from roamerx_edge.telemetry_collector import TelemetryCollector, normalize_rtk_quality


def test_status_timestamp_advances_without_new_localization(monkeypatch):
    timestamps = iter(("pose-time", "status-time-1", "status-time-2"))
    monkeypatch.setattr("roamerx_edge.telemetry_collector.now_iso", lambda: next(timestamps))
    collector = TelemetryCollector(
        SimpleNamespace(current_map_id="1", current_map_version="v1"),
        RuntimeSafetyState(),
    )
    collector.on_localization(
        SimpleNamespace(
            status=3,
            pos=SimpleNamespace(x=1.0, y=2.0, z=0.0),
            rpy=SimpleNamespace(z=0.5),
            speed=0.0,
            coord_type=0,
        )
    )

    first = collector.build_status_snapshot()
    second = collector.build_status_snapshot()

    assert first["sampled_at"] == "status-time-1"
    assert second["sampled_at"] == "status-time-2"
    assert first["state_version"] == second["state_version"] == 1
    assert first["localization"]["sampled_at"] == "pose-time"
    assert first["localization"]["fresh"] is True


def test_wait_for_pose_update_requires_a_new_localization_frame(monkeypatch):
    timestamps = iter(("pose-1", "pose-2"))
    monkeypatch.setattr("roamerx_edge.telemetry_collector.now_iso", lambda: next(timestamps))
    collector = TelemetryCollector(
        SimpleNamespace(current_map_id="1", current_map_version="v1"),
        RuntimeSafetyState(),
    )

    def message(x):
        return SimpleNamespace(
            status=3,
            pos=SimpleNamespace(x=x, y=2.0, z=0.0),
            rpy=SimpleNamespace(z=0.5),
            speed=0.0,
            coord_type=0,
        )

    collector.on_localization(message(1.0))
    first = collector.latest_pose()
    assert collector.wait_for_pose_update(first.sampled_at, timeout_seconds=0.001) is None

    collector.on_localization(message(3.0))
    updated = collector.wait_for_pose_update(first.sampled_at, timeout_seconds=0.001)
    assert updated is not None
    assert updated.sampled_at == "pose-2"
    assert updated.x == 3.0


def test_localization_normal_timer_resets_on_every_non_normal_state(monkeypatch):
    monotonic_values = iter((10.0, 11.0, 12.0, 20.0))
    monkeypatch.setattr(
        "roamerx_edge.telemetry_collector.time.monotonic",
        lambda: next(monotonic_values),
    )
    state = RuntimeSafetyState()
    collector = TelemetryCollector(
        SimpleNamespace(current_map_id="1", current_map_version="v1"),
        state,
    )

    def message(status):
        return SimpleNamespace(
            status=status,
            pos=SimpleNamespace(x=1.0, y=2.0, z=0.0),
            rpy=SimpleNamespace(z=0.5),
            speed=0.0,
            coord_type=0,
        )

    collector.on_localization(message(3))
    assert state.localization_normal_since_monotonic == 10.0
    collector.on_localization(message(3))
    assert state.localization_normal_since_monotonic == 10.0
    collector.on_localization(message(0))
    assert state.localization_normal_since_monotonic == 0.0
    collector.on_localization(message(3))
    assert state.localization_normal_since_monotonic == 20.0


def test_stale_localization_sample_cannot_remain_normal(monkeypatch):
    monotonic = {"value": 100.0}
    monkeypatch.setattr(
        "roamerx_edge.telemetry_collector.time.monotonic",
        lambda: monotonic["value"],
    )
    state = RuntimeSafetyState(nav_ready=False)
    collector = TelemetryCollector(
        SimpleNamespace(current_map_id="1", current_map_version="v1"),
        state,
    )
    collector.on_localization(SimpleNamespace(
        status=3,
        pos=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        rpy=SimpleNamespace(z=0.5),
        speed=0.0,
        coord_type=0,
    ))

    fresh = collector.build_status_snapshot()
    assert fresh["localization"]["status"] == "normal"
    assert fresh["localization"]["fresh"] is True

    monotonic["value"] = 111.0
    stale = collector.build_status_snapshot()
    assert stale["localization"]["status"] == "unknown"
    assert stale["localization"]["source_status"] is None
    assert stale["localization"]["fresh"] is False
    assert stale["localization"]["sample_age_seconds"] == 11.0
    assert state.localization_status == "unknown"
    assert state.localization_normal_since_monotonic == 0.0


def test_power_and_network_are_only_reported_while_fresh(monkeypatch):
    monotonic = {"value": 10.0}
    monkeypatch.setattr(
        "roamerx_edge.telemetry_collector.time.monotonic",
        lambda: monotonic["value"],
    )
    collector = TelemetryCollector(
        SimpleNamespace(current_map_id="1", current_map_version="v1"),
        RuntimeSafetyState(),
    )
    collector.configure_system_probe_staleness(30)
    collector.on_battery(48, True, current_a=3.3)
    collector.on_network("5G SA", 68, csq=21)

    fresh = collector.build_status_snapshot()
    assert fresh["power"] == {
        "available": True,
        "percent": 48,
        "charging": True,
        "current_a": 3.3,
    }
    assert fresh["network"]["available"] is True
    assert fresh["network"]["signal_percent"] == 68

    monotonic["value"] = 41.0
    stale = collector.build_status_snapshot()
    assert stale["power"] == {"available": False, "percent": None, "charging": None}
    assert stale["network"] == {
        "available": False,
        "type": "",
        "signal_percent": None,
    }


def test_direct_odometry_samples_keep_source_and_frame_details(monkeypatch):
    now = {"value": 100.0}
    monkeypatch.setattr("roamerx_edge.telemetry_collector.time.time", lambda: now["value"])
    collector = TelemetryCollector(
        SimpleNamespace(current_map_id="1", current_map_version="v1"),
        RuntimeSafetyState(),
    )

    collector.on_sensor_message(
        "odometry",
        topic="/odom/localization_odom",
        frame_id="map",
        child_frame_id="body",
        source="slam",
        message_time_offset_seconds=0.02,
    )
    snapshot = collector.build_status_snapshot()["sensors"]["odometry"]

    assert snapshot["online"] is True
    assert snapshot["source"] == "slam"
    assert snapshot["frame_id"] == "map"
    assert snapshot["child_frame_id"] == "body"


def test_scan_matching_summary_can_skip_prediction_arrays():
    collector = TelemetryCollector(
        SimpleNamespace(current_map_id="1", current_map_version="v1"),
        RuntimeSafetyState(),
    )
    message = SimpleNamespace(
        has_converged=True,
        matching_error=0.1,
        inlier_fraction=0.9,
        relative_pose=SimpleNamespace(
            translation=SimpleNamespace(x=0.1, y=0.2, z=0.0)
        ),
        prediction_labels=[SimpleNamespace(data="constant_velocity")],
        prediction_errors=[SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=0.0, z=0.0)
        )],
    )

    collector.on_scan_matching_status(message, include_predictions=False)
    quality = collector.build_status_snapshot()["localization"]["quality"]

    assert quality["matching_error"] == 0.1
    assert quality["prediction_errors"] == []


def test_rtk_raw_details_and_cross_sensor_time_diagnostics(monkeypatch):
    now = {"value": 100.0}
    monkeypatch.setattr("roamerx_edge.telemetry_collector.time.time", lambda: now["value"])
    collector = TelemetryCollector(
        SimpleNamespace(current_map_id="1", current_map_version="v1"),
        RuntimeSafetyState(),
    )

    for name, stamp in (
        ("lidar", 99.900),
        ("imu", 99.905),
        ("rtk", 99.920),
        ("odometry", 99.910),
    ):
        collector.on_sensor_message(
            name,
            topic=f"/{name}",
            measurement_stamp=stamp,
            measurement_header_stamp=99.800 if name == "lidar" else None,
            measurement_time_basis="scan_end" if name == "lidar" else "header",
            scan_duration_ms=100.0 if name == "lidar" else None,
            measurement_time_valid=True,
            measurement_time_offset_ms=0.0,
        )
    collector.update_sensor_details(
        "rtk",
        quality="rtk_fixed",
        fix_status=2,
        solution_status=1,
        position_type=4,
        solution_satellites=18,
        heading={
            "status": 0,
            "type": 1,
            "heading_deg": 90.0,
            "heading_std_deg": 0.1,
            "measurement_stamp": 99.920,
        },
    )

    snapshot = collector.build_status_snapshot()
    raw_rtk = snapshot["localization"]["raw_rtk"]
    diagnostics = snapshot["localization"]["time_diagnostics"]

    assert normalize_rtk_quality("rtk_fixed") == "fixed"
    assert raw_rtk["quality"] == "fixed"
    assert raw_rtk["fix_status"] == 2
    assert raw_rtk["heading"]["heading_deg"] == 90.0
    assert diagnostics["lidar_to_rtk_delta_ms"] == 20.0
    assert diagnostics["lidar_to_odom_delta_ms"] == 10.0
    assert diagnostics["lidar"]["measurement_header_stamp"] == 99.800
    assert diagnostics["lidar"]["measurement_time_basis"] == "scan_end"
    assert diagnostics["lidar"]["scan_duration_ms"] == 100.0
    assert diagnostics["all_time_valid"] is True
