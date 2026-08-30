import time
from types import SimpleNamespace

from roamerx_edge.charge_control_adapter import ChargeControlAdapter
from roamerx_edge.config import ChargeControlConfig, PowerModeConfig
from roamerx_edge.local_store import LocalStore
from roamerx_edge.power_mode_controller import PowerModeController


def result(returncode=0, stdout="", stderr=""):
    return SimpleNamespace(returncode=returncode, stdout=stdout, stderr=stderr)


def test_start_motion_control_is_idempotent_when_dog_task_is_running():
    class FakePowerMode:
        def snapshot(self):
            return {"auto_charge_enabled": False, "charge_stage": "idle"}

    adapter = ChargeControlAdapter(ChargeControlConfig(), FakePowerMode())
    calls = []
    adapter._run = lambda action, command: calls.append((action, command)) or {"action": action}

    assert adapter.start_motion_control()["motion_control"] == "running"
    assert calls[0][0] == "motion_start"
    command = calls[0][1]
    assert "robot-launch start 3 4" not in command
    assert 'if ! robot-launch egg "$egg" 2>/dev/null | grep -qi running; then' in command
    assert 'robot-launch start "$egg"' in command
    assert 'robot-launch egg "$egg" 2>/dev/null | grep -qi running' in command


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
    assert ["sudo", "systemctl", "stop", config.teleop_bridge_service] not in commands
    assert ["sudo", "systemctl", "disable", "--now", config.monitoring_service] in commands
    assert [config.navigation_script, "full-stop"] in commands
    assert ["sudo", "pkill", "-TERM", "-f", "[r]tk_ntrip_bridge.py|[s]ixents_gps_driver"] in commands
    remote_stop = next(command for command in commands if command[0] == "ssh")
    assert all(egg in remote_stop[-1] for egg in config.controller_runtime_eggs)
    assert all(service in remote_stop[-1] for service in config.controller_runtime_services)
    assert "power_daemon" not in remote_stop[-1]
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
    assert "controller_service_roamerx_charge_pile_service" not in services
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

    assert len(services) == 26
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
    adapter._return_legacy_and_restore_arc = lambda: calls.append("stop")
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
    adapter._set_legacy_pile_state = lambda _state: calls.append("retry") or {"stdout": "active"}
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


def test_low_battery_requests_one_return_task_only_below_threshold():
    class FakePowerMode:
        def snapshot(self):
            return {"auto_charge_enabled": False}

    adapter = ChargeControlAdapter(
        ChargeControlConfig(low_battery_start_percent=20, low_battery_confirmation_samples=2),
        FakePowerMode(),
    )
    calls = []
    adapter.set_low_battery_handler(lambda episode_id, percent: calls.append((episode_id, percent)))
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
    assert calls == []
    sample["percent"] = 19
    adapter.observe_power(sample)
    adapter.observe_power(sample)
    for _ in range(50):
        if calls:
            break
        time.sleep(0.01)

    assert len(calls) == 1
    assert calls[0][1] == 19
    adapter.observe_power(sample)
    adapter.observe_power(sample)
    time.sleep(0.05)
    assert len(calls) == 1


def test_low_battery_episode_survives_restart_and_rearms_at_25_percent(tmp_path):
    class FakePowerMode:
        def snapshot(self):
            return {"auto_charge_enabled": False}

    store = LocalStore(str(tmp_path / "edge.db"))
    config = ChargeControlConfig(low_battery_confirmation_samples=2)
    first = ChargeControlAdapter(config, FakePowerMode(), store)
    first_calls = []
    first.set_low_battery_handler(lambda episode_id, percent: first_calls.append((episode_id, percent)))
    first.observe_power({"available": True, "percent": 19})
    first.observe_power({"available": True, "percent": 19})
    for _ in range(50):
        if first_calls:
            break
        time.sleep(0.01)
    assert len(first_calls) == 1

    restarted = ChargeControlAdapter(config, FakePowerMode(), store)
    restarted_calls = []
    restarted.set_low_battery_handler(
        lambda episode_id, percent: restarted_calls.append((episode_id, percent))
    )
    restarted.observe_power({"available": True, "percent": 19})
    restarted.observe_power({"available": True, "percent": 19})
    time.sleep(0.05)
    assert restarted_calls == []

    restarted.observe_power({"available": True, "percent": 25})
    restarted.observe_power({"available": True, "percent": 19})
    restarted.observe_power({"available": True, "percent": 19})
    for _ in range(50):
        if restarted_calls:
            break
        time.sleep(0.01)
    assert len(restarted_calls) == 1
    assert restarted_calls[0][0] != first_calls[0][0]
    store.close()


def test_manual_disconnect_pauses_low_battery_auto_charge():
    class FakePowerMode:
        def __init__(self):
            self.stage = "charging"

        def snapshot(self):
            return {"auto_charge_enabled": False, "charge_stage": self.stage}

        def set_auto_charge_enabled(self, _enabled):
            return None

        def set_charge_stage(self, stage, _detail=""):
            self.stage = stage

        def restore_normal(self):
            return {"mode": "normal"}

    adapter = ChargeControlAdapter(
        ChargeControlConfig(
            low_battery_start_percent=20,
            low_battery_confirmation_samples=2,
            manual_disconnect_auto_charge_pause_seconds=300,
        ),
        FakePowerMode(),
    )
    adapter._stop_remote = lambda: {"disconnect_requested": True}
    adapter.start_motion_control = lambda: {"motion_control": "running"}
    adapter.stop()

    calls = []
    adapter.set_low_battery_handler(lambda episode_id, percent: calls.append((episode_id, percent)))
    sample = {"available": True, "percent": 19}
    adapter.observe_power(sample)
    adapter.observe_power(sample)
    assert calls == []

    adapter._manual_disconnect_inhibit_until = time.monotonic() - 1
    adapter.observe_power(sample)
    adapter.observe_power(sample)
    for _ in range(50):
        if calls:
            break
        time.sleep(0.01)

    assert len(calls) == 1
    assert calls[0][1] == 19


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
    adapter._set_legacy_pile_state = lambda _state: {"action": "legacy_lying"}

    result = adapter.start()

    assert result["charge_stage"] == "waiting_for_dock"
    assert result["missing"] == ["充电诊断状态刷新中"]
    assert result["diagnostics_refreshed"] is False
    assert power_mode.stages[-1][0] == "waiting_for_dock"


def test_charge_start_refreshes_legacy_status_before_reporting_missing_conditions():
    class FakePowerMode:
        def __init__(self):
            self.stages = []

        def snapshot(self):
            return {"auto_charge_enabled": False, "charge_stage": "waiting_for_dock"}

        def set_charge_stage(self, stage, detail=""):
            self.stages.append((stage, detail))

    refreshed = {
        "charger_controller_mode": "legacy",
        "bluetooth_connected": True,
        "charge_pin": 0,
        "negative_contact": 0,
        "positive_contact": 0,
    }
    adapter = ChargeControlAdapter(
        ChargeControlConfig(), FakePowerMode(), power_refresh=lambda: refreshed
    )
    adapter._set_legacy_pile_state = lambda _state: {"action": "legacy_lying"}

    result = adapter.start()
    with adapter._lock:
        adapter._pending_charge = False
    adapter._dock_monitor_thread.join(timeout=1)

    assert result["diagnostics_refreshed"] is True
    assert "旧版充电诊断未运行" not in result["missing"]
    assert "充电极片未接触" in result["missing"]


def test_pending_dock_monitor_starts_charge_once_after_contact():
    class FakePowerMode:
        def snapshot(self):
            return {"auto_charge_enabled": False, "charge_stage": "waiting_for_dock"}

        def set_charge_stage(self, _stage, _detail=""):
            return None

    samples = [
        {
            "charger_controller_mode": "legacy",
            "bluetooth_connected": True,
            "charge_pin": 0,
            "negative_contact": 0,
            "positive_contact": 0,
        },
        {
            "charger_controller_mode": "legacy",
            "bluetooth_connected": True,
            "charge_pin": 1,
            "negative_contact": 1,
            "positive_contact": 1,
        },
    ]

    def refresh():
        return samples.pop(0) if samples else {
            "charger_controller_mode": "legacy",
            "bluetooth_connected": True,
            "charge_pin": 1,
            "negative_contact": 1,
            "positive_contact": 1,
        }

    adapter = ChargeControlAdapter(
        ChargeControlConfig(
            dock_status_poll_interval_seconds=0.01,
            dock_contact_wait_timeout_seconds=1.0,
        ),
        FakePowerMode(),
        power_refresh=refresh,
    )
    adapter._set_legacy_pile_state = lambda _state: {"action": "legacy_lying"}
    calls = []

    def begin_once():
        with adapter._lock:
            if not adapter._pending_charge:
                return {"charge_stage": "idle"}
            adapter._pending_charge = False
        calls.append(True)
        return {"charge_stage": "waiting_current", "dock_ready": True}

    adapter._begin_charge = begin_once
    result = adapter.start()
    adapter._dock_monitor_thread.join(timeout=1)
    if adapter._charge_begin_thread:
        adapter._charge_begin_thread.join(timeout=1)

    assert result["charge_stage"] == "waiting_for_dock"
    assert calls == [True]


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
        "charger_controller_mode": "legacy",
        "bluetooth_connected": True,
        "charge_pin": 1,
        "negative_contact": 1,
        "positive_contact": 1,
    }

    adapter.start()
    adapter.stop()

    assert commands[0][0] == "pile_lying"
    assert "roamerx-charge-pile-arbiter legacy-lying" in commands[0][1]
    assert "robot-launch stop 3 4" in commands[1][1]
    assert commands[2][0] == "legacy_charge_confirm"
    assert "pgrep -f" in commands[2][1]
    restore = commands[3][1]
    assert commands[3][0] == "legacy_return"
    assert "roamerx-charge-pile-arbiter legacy-return" in restore
    assert "arc_platform" in restore
    assert commands[4][0] == "motion_start"
