from types import SimpleNamespace

from roamerx_edge.safety_policy import RuntimeSafetyState
from roamerx_edge.telemetry_collector import TelemetryCollector


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


def test_localization_normal_timer_resets_on_every_non_normal_state(monkeypatch):
    monotonic_values = iter((10.0, 20.0))
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
