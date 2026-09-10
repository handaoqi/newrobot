from types import SimpleNamespace

from bike_bot.runtime import RuntimeState


def _config():
    return SimpleNamespace(
        location=SimpleNamespace(name="test", latitude=None, longitude=None),
        runtime=SimpleNamespace(
            speed=0.0,
            heading=0.0,
            battery_level=100,
            charging=False,
            signal_strength=100,
            network_type="WiFi",
            mode="auto",
            status="online",
        ),
    )


def test_degraded_warning_persists_after_transient_status_clears():
    state = RuntimeState(_config())

    state.set_degraded("vision_cpu_fallback")
    state.update_status(runtime_status="warning")
    state.update_status(runtime_status="online")

    assert state.snapshot().runtime.status == "warning"


def test_clearing_degraded_reason_restores_requested_status():
    state = RuntimeState(_config())
    state.set_degraded("vision_cpu_fallback")

    state.set_degraded("vision_cpu_fallback", active=False)

    assert state.snapshot().runtime.status == "online"


def test_offline_status_has_priority_over_degraded_warning():
    state = RuntimeState(_config())
    state.set_degraded("vision_cpu_fallback")

    state.update_status(runtime_status="offline")

    assert state.snapshot().runtime.status == "offline"
