import time
from types import SimpleNamespace

from roamerx_edge.charge_control_adapter import ChargeControlAdapter
from roamerx_edge.config import ChargeControlConfig, PowerModeConfig
from roamerx_edge.power_mode_controller import PowerModeController


def result(returncode=0, stdout="", stderr=""):
    return SimpleNamespace(returncode=returncode, stdout=stdout, stderr=stderr)


def test_mode_transitions_run_expected_commands(tmp_path):
    commands = []

    def runner(command, _timeout):
        commands.append(command)
        if command[:3] == ["sudo", "nvpmodel", "-q"]:
            return result(stdout="NV Power Mode: MAXN\n0\n")
        return result()

    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )
    controller = PowerModeController(config, runner=runner)

    assert controller.enter_cooling()["mode"] == "cooling_standby"
    assert ["sudo", "systemctl", "disable", "--now", config.monitoring_service] in commands
    assert [config.navigation_script, "full-stop"] in commands
    assert ["sudo", "pkill", "-TERM", "-f", "[r]tk_ntrip_bridge.py|[s]ixents_gps_driver"] in commands
    assert controller.ensure_cooling_power_profile()["transition_state"] == "rebooting"
    assert any(
        command[:2] == ["sudo", "systemd-run"]
        and command[-4:] == ["/usr/sbin/nvpmodel", "--force", "-m", "1"]
        for command in commands
    )

    commands.clear()
    assert controller.restore_normal()["mode"] == "normal"
    assert [config.sensor_start_script] in commands
    assert ["sudo", "systemctl", "enable", "--now", config.monitoring_service] in commands
    assert [config.navigation_script, "start"] in commands


def test_service_status_is_compared_with_current_mode(tmp_path):
    def runner(command, _timeout):
        if command[:3] == ["sudo", "nvpmodel", "-q"]:
            return result(stdout="NV Power Mode: 10W\n1\n")
        if command[:3] == ["systemctl", "is-active", "roamerx-edge-agent.service"]:
            return result()
        return result(returncode=1)

    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )
    controller = PowerModeController(config, runner=runner)
    controller._update(mode="cooling_standby", transition_state="ready")

    services = controller.refresh_service_status({"charger_controller_active": True})["services"]

    assert services["edge_agent"]["matches_mode"] is True
    assert services["detection_video"]["matches_mode"] is True
    assert services["charge_controller"]["matches_mode"] is True
    assert services["power_profile"]["matches_mode"] is True


def test_cooling_schedules_forced_reboot(tmp_path):
    commands = []

    def runner(command, _timeout):
        commands.append(command)
        if command[:3] == ["sudo", "nvpmodel", "-q"]:
            return result(stdout="NV Power Mode: MAXN\n0\n")
        return result()

    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )
    controller = PowerModeController(config, runner=runner)
    controller.enter_cooling()
    state = controller.ensure_cooling_power_profile()

    assert state["transition_state"] == "rebooting"
    assert state["reboot_required"] is True
    reboot = next(command for command in commands if command[:2] == ["sudo", "systemd-run"])
    assert ["/usr/sbin/nvpmodel", "--force", "-m", "1"] == reboot[-4:]


def test_full_charge_stops_charger_and_restores_normal(tmp_path):
    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )
    controller = PowerModeController(config, runner=lambda *_args: result())
    controller.set_auto_charge_enabled(True)
    adapter = ChargeControlAdapter(
        ChargeControlConfig(full_battery_percent=100, full_confirmation_samples=3),
        controller,
    )
    calls = []
    adapter._stop_remote = lambda: calls.append("stop")
    controller.restore_normal = lambda: calls.append("normal")

    sample = {"available": True, "percent": 100, "charger_controller_active": True}
    for _ in range(3):
        adapter.observe_power(sample)
    for _ in range(50):
        if calls == ["stop", "normal"]:
            break
        time.sleep(0.01)

    assert calls == ["stop", "normal"]
