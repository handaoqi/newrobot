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
