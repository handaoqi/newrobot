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
        return result()

    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )
    controller = PowerModeController(config, runner=runner)

    assert controller.enter_cooling()["mode"] == "cooling_standby"
    assert any(command[:2] == ["bash", "-lc"] and "/teleop_action" in command[-1] for command in commands)
    assert ["sleep", "2"] in commands
    assert ["sudo", "systemctl", "disable", "--now", config.monitoring_service] in commands
    assert [config.navigation_script, "full-stop"] in commands
    assert ["sudo", "pkill", "-TERM", "-f", "[r]tk_ntrip_bridge.py|[s]ixents_gps_driver"] in commands
    assert not any("nvpmodel" in " ".join(command) for command in commands)

    commands.clear()
    assert controller.restore_normal()["mode"] == "normal"
    assert any(
        command[:4] == ["sudo", "systemctl", "enable", "--now"]
        and config.monitoring_service in command
        and config.teleop_bridge_service in command
        for command in commands
    )
    assert any(config.navigation_script in " ".join(command) for command in commands)


def test_normal_mode_startup_reconciles_required_services(tmp_path):
    commands = []

    def runner(command, _timeout):
        commands.append(command)
        return result()

    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )
    controller = PowerModeController(config, runner=runner)

    state = controller.reconcile_startup()

    assert state["mode"] == "normal"
    assert state["transition_state"] == "ready"
    assert any(
        command[:4] == ["sudo", "systemctl", "enable", "--now"]
        and config.monitoring_service in command
        and config.teleop_bridge_service in command
        for command in commands
    )
    remote = next(command for command in commands if command[0] == "ssh")
    assert all(egg in remote[-1] for egg in (*config.controller_always_eggs, *config.controller_runtime_eggs))
    assert all(service in remote[-1] for service in (*config.controller_always_services, *config.controller_runtime_services))
    assert any(config.navigation_script in " ".join(command) for command in commands)


def test_navigation_start_failure_keeps_normal_mode_available(tmp_path):
    def runner(command, _timeout):
        if command[:2] == ["bash", "-lc"] and "start" in command[-1]:
            return result(returncode=1, stderr="localization not ready")
        return result()

    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )
    controller = PowerModeController(config, runner=runner)

    state = controller.reconcile_startup()

    assert state["mode"] == "normal"
    assert state["transition_state"] == "ready"
    assert "localization not ready" in state["last_warning"]


def test_service_status_is_compared_with_current_mode(tmp_path):
    def runner(command, _timeout):
        if command[:3] == ["systemctl", "is-active", "roamerx-edge-agent.service"]:
            return result()
        if command[:2] == ["systemctl", "is-active"] and command[-1] in PowerModeConfig().always_on_services:
            return result()
        if command[0] == "ssh":
            config = PowerModeConfig()
            eggs = (*config.controller_always_eggs, *config.controller_runtime_eggs)
            services = (*config.controller_always_services, *config.controller_runtime_services)
            values = [
                f"egg_{index}={1 if egg in config.controller_always_eggs else 0}"
                for index, egg in enumerate(eggs)
            ]
            values.extend(
                f"service_{index}={1 if service in config.controller_always_services else 0}"
                for index, service in enumerate(services)
            )
            return result(stdout="\n".join(values))
        return result(returncode=1)

    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )
    controller = PowerModeController(config, runner=runner)
    controller._update(mode="cooling_standby", transition_state="ready")

    services = controller.refresh_service_status({"charger_controller_active": True})["services"]

    assert services["edge_agent"]["matches_mode"] is True
    assert services["dev_agent"]["matches_mode"] is True
    assert services["detection_video"]["matches_mode"] is True
    assert services["controller_service_roamerx_charge_pile_service"]["matches_mode"] is True
    assert "power_profile" not in services
    assert "leg_power" not in services
    assert services["controller_egg_time_sync"]["matches_mode"] is True
    assert services["controller_egg_power_daemon"]["matches_mode"] is True
    assert services["controller_egg_motion_control"]["matches_mode"] is True
    assert services["controller_egg_dog_task"]["matches_mode"] is True
    assert services["controller_service_robot_launch_service"]["matches_mode"] is True
    assert services["controller_service_rknn_server_service"]["matches_mode"] is True


def test_normal_mode_reports_every_required_service_as_running(tmp_path):
    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )

    def runner(command, _timeout):
        if command[0] in {"systemctl", "pgrep"}:
            return result()
        if command[0] == "ssh":
            eggs = (*config.controller_always_eggs, *config.controller_runtime_eggs)
            services = (*config.controller_always_services, *config.controller_runtime_services)
            values = [f"egg_{index}=1" for index, _ in enumerate(eggs)]
            values.extend(f"service_{index}=1" for index, _ in enumerate(services))
            return result(stdout="\n".join(values))
        return result(returncode=1)

    controller = PowerModeController(config, runner=runner)
    services = controller.refresh_service_status({"charger_controller_active": True})["services"]

    assert len(services) == 27
    assert all(service["matches_mode"] for service in services.values())


def test_cooling_does_not_switch_nx_power_profile(tmp_path):
    commands = []

    def runner(command, _timeout):
        commands.append(command)
        return result()

    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )
    controller = PowerModeController(config, runner=runner)
    state = controller.enter_cooling()

    assert state["transition_state"] == "ready"
    assert state["reboot_required"] is False
    assert not any("nvpmodel" in " ".join(command) for command in commands)


def test_full_charge_stops_charger_and_restores_normal(tmp_path):
    config = PowerModeConfig(
        state_path=str(tmp_path / "power-mode.json"),
        cooling_marker_path=str(tmp_path / "cooling"),
    )
    controller = PowerModeController(config, runner=lambda *_args: result())
    controller.set_auto_charge_enabled(True)
    adapter = ChargeControlAdapter(
        ChargeControlConfig(full_battery_percent=95, full_confirmation_samples=3),
        controller,
    )
    calls = []
    adapter._stop_remote = lambda: calls.append("stop")
    controller.restore_normal = lambda: calls.append("normal")
    adapter.start_motion_control = lambda: calls.append("motion") or {"motion_control": "running"}

    sample = {"available": True, "percent": 95, "charger_controller_active": True}
    for _ in range(3):
        adapter.observe_power(sample)
    for _ in range(50):
        if calls == ["stop", "normal", "motion"]:
            break
        time.sleep(0.01)

    assert calls == ["stop", "normal", "motion"]


def test_thermal_recovery_retries_charge_once_after_delay(monkeypatch):
    class FakePowerMode:
        def snapshot(self):
            return {"auto_charge_enabled": True}

    adapter = ChargeControlAdapter(
        ChargeControlConfig(
            thermal_recovery_delay_seconds=10,
            thermal_retry_cooldown_seconds=60,
        ),
        FakePowerMode(),
    )
    calls = []
    adapter._start_remote = lambda: calls.append("retry") or {"stdout": "active"}
    clock = iter([100.0, 100.0, 109.9, 110.1, 111.0, 170.0])
    monkeypatch.setattr("roamerx_edge.charge_control_adapter.time.monotonic", lambda: next(clock))

    thermal = {"available": True, "charge_state": "thermal_protection"}
    waiting = {"available": True, "charge_state": "waiting"}
    adapter.observe_power(thermal)
    adapter.observe_power(waiting)
    adapter.observe_power(waiting)
    adapter.observe_power(waiting)
    for _ in range(50):
        if calls:
            break
        time.sleep(0.01)

    assert calls == ["retry"]


def test_low_battery_starts_charge_after_confirmation():
    class FakePowerMode:
        def snapshot(self):
            return {"auto_charge_enabled": False}

    adapter = ChargeControlAdapter(
        ChargeControlConfig(low_battery_start_percent=20, low_battery_confirmation_samples=2),
        FakePowerMode(),
    )
    calls = []
    adapter.start = lambda: calls.append("start") or {"charge": {}}
    sample = {
        "available": True,
        "percent": 20,
        "charger_controller_active": True,
        "bluetooth_connected": True,
        "charge_pin": 1,
        "negative_contact": 1,
        "positive_contact": 1,
    }

    adapter.observe_power(sample)
    assert calls == []
    adapter.observe_power(sample)
    for _ in range(50):
        if calls:
            break
        time.sleep(0.01)

    assert calls == ["start"]


def test_charge_waits_for_dock_before_stopping_motion():
    class FakePowerMode:
        def __init__(self):
            self.stages = []

        def snapshot(self):
            return {"auto_charge_enabled": False, "charge_stage": "idle"}

        def set_charge_stage(self, stage, detail=""):
            self.stages.append((stage, detail))

        def enter_cooling(self):
            raise AssertionError("must not enter cooling before dock verification")

    power_mode = FakePowerMode()
    adapter = ChargeControlAdapter(ChargeControlConfig(), power_mode)
    adapter._set_charge_pile_state = lambda state: {"state": state}

    result = adapter.start()

    assert result["charge_stage"] == "waiting_for_dock"
    assert "蓝牙未连接" in result["missing"]
    assert power_mode.stages[-1][0] == "waiting_for_dock"


def test_charge_commands_stop_and_restore_3588_runtime_eggs():
    class FakePowerMode:
        def __init__(self):
            self.stage = "idle"

        def enter_cooling(self):
            return {"mode": "cooling_standby"}

        def set_auto_charge_enabled(self, _enabled):
            return None

        def restore_normal(self):
            return {"mode": "normal"}

        def snapshot(self):
            return {"auto_charge_enabled": False, "charge_stage": self.stage}

        def set_charge_stage(self, stage, _detail=""):
            self.stage = stage

    power_mode = FakePowerMode()
    adapter = ChargeControlAdapter(ChargeControlConfig(), power_mode)
    commands = []
    adapter._run = lambda action, command: commands.append((action, command)) or {"action": action}
    adapter._latest_power = {
        "charger_controller_active": True,
        "bluetooth_connected": True,
        "charge_pin": 1,
        "negative_contact": 1,
        "positive_contact": 1,
    }

    adapter.start()
    adapter.stop()

    assert commands[0][0] == "pile_lying"
    assert "robot-launch stop 3 4" in commands[1][1]
    assert "robot-launch stop push_image spline_daemon motion_control dog_task" in commands[2][1]
    assert "printf 'lying" in commands[2][1]
    assert "systemctl restart roamerx-charge-pile.service" in commands[2][1]
    assert "roamerx-leg-power" not in commands[2][1]
    assert "systemctl stop rkaiq_3A.service rknn_server.service lightdm.service" in commands[2][1]
    restore = commands[3][1]
    assert "dog_returning" in restore
    assert "sudo timeout --signal=TERM --kill-after=2 6" in restore
    assert "pkill -KILL -x dog_returning" in restore
    assert "printf 'unknown" in restore
    assert "systemctl start roamerx-charge-pile.service" in restore
    assert "roamerx-leg-power" not in commands[1][1]
    assert "systemctl start rkaiq_3A.service rknn_server.service lightdm.service" in restore
    assert "systemctl restart robot-launch.service" in restore
    assert "robot-launch start 3 4" in restore
    assert "systemctl is-active --quiet robot-launch.service" in restore
    assert "/arc/mc_state" in restore
    assert "ros2 topic echo /arc/mc_state --once" in restore
    assert "|| true" in restore
    assert commands[4][0] == "motion_start"
