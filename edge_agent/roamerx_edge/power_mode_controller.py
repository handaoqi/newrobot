from __future__ import annotations

import json
import logging
import os
import subprocess
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from .config import PowerModeConfig
from .protocol import ProtocolError


LOGGER = logging.getLogger(__name__)


def _run(command: list[str], timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run(command, capture_output=True, text=True, timeout=timeout, check=False)


class PowerModeController:
    def __init__(self, config: PowerModeConfig, runner: Callable = _run) -> None:
        self.config = config
        self.runner = runner
        self._lock = threading.RLock()
        self._state_path = Path(config.state_path)
        self._cooling_marker = Path(config.cooling_marker_path)
        self._state = self._load_state()
        self._services: dict = {}

    def snapshot(self) -> dict:
        with self._lock:
            return {**self._state, "services": dict(self._services)}

    def refresh_service_status(self, power: dict | None = None) -> dict:
        power = power or {}
        mode = self.snapshot().get("mode", "normal")
        cooling = mode == "cooling_standby"
        probes = (
            ("edge_agent", "Edge Agent", ["systemctl", "is-active", "roamerx-edge-agent.service"], True),
            ("detection_video", "AI识别与视频", ["systemctl", "is-active", self.config.monitoring_service], not cooling),
            ("lidar_imu", "Livox LiDAR/IMU", ["pgrep", "-f", "[l]ivox_driver_node"], not cooling),
            ("rtk", "RTK差分定位", ["pgrep", "-f", "[r]tk_ntrip_bridge.py"], not cooling),
            ("localization", "激光定位", ["pgrep", "-f", "[l]ocalization_node"], not cooling),
            ("navigation", "Nav2导航", ["pgrep", "-f", "[n]avigo_container"], not cooling),
        )
        services = {}
        for key, name, command, expected in probes:
            try:
                result = self.runner(command, 4)
                active = result.returncode == 0
            except Exception:
                active = False
            services[key] = {
                "name": name,
                "active": active,
                "expected_active": expected,
                "matches_mode": active == expected,
            }
        charger_active = bool(power.get("charger_controller_active"))
        services["charge_controller"] = {
            "name": "充电控制",
            "active": charger_active,
            "expected_active": cooling,
            "matches_mode": charger_active == cooling,
        }
        profile_name = self._power_profile()
        services["power_profile"] = {
            "name": "NX功耗档位",
            "active": profile_name != "unknown",
            "value": profile_name,
            "expected_value": "10W" if cooling else "MAXN",
            "matches_mode": profile_name == ("10W" if cooling else "MAXN"),
        }
        with self._lock:
            self._services = services
        return self.snapshot()

    def reconcile_startup(self) -> dict:
        state = self.snapshot()
        profile = self._power_profile()
        if state.get("mode") == "cooling_standby":
            self._set_cooling_marker(True)
            self._run_optional(
                ["sudo", "systemctl", "disable", "--now", self.config.monitoring_service],
                20,
                allowed_returncodes={0, 1},
            )
            self._stop_sensor_processes()
            if profile == "10W":
                with self._lock:
                    self._update(transition_state="ready", reboot_required=False, last_error="", last_warning="")
            return self.snapshot()
        if state.get("transition_state") == "rebooting" and profile == "MAXN":
            return self._start_normal_services()
        return state

    def enter_cooling(self) -> dict:
        with self._lock:
            self._update(mode="cooling_standby", transition_state="entering", last_error="", last_warning="")
        try:
            self._set_cooling_marker(True)
            self._run_required(["sudo", "systemctl", "disable", "--now", self.config.monitoring_service], 20)
            self._run_required([self.config.navigation_script, "full-stop"], 30)
            self._stop_sensor_processes()
            with self._lock:
                self._update(
                    mode="cooling_standby",
                    transition_state="ready",
                    reboot_required=False,
                    last_error="",
                    last_warning="",
                )
        except Exception as exc:
            with self._lock:
                self._update(mode="cooling_standby", transition_state="error", last_error=str(exc))
            raise ProtocolError("COOLING_MODE_FAILED", str(exc)) from exc
        return self.snapshot()

    def ensure_cooling_power_profile(self) -> dict:
        if self._power_profile() == "10W":
            return self.snapshot()
        with self._lock:
            self._update(
                mode="cooling_standby",
                transition_state="rebooting",
                reboot_required=True,
                last_error="",
                last_warning="NX正在重启并切换到10W功耗档位",
            )
        self._schedule_power_mode_reboot(self.config.cooling_power_mode)
        return self.snapshot()

    def restore_normal(self) -> dict:
        with self._lock:
            self._update(mode="normal", transition_state="restoring", last_error="", last_warning="")
        self._set_cooling_marker(False)
        self._run_required(["sudo", "systemctl", "enable", self.config.monitoring_service], 20)
        if self._power_profile() != "MAXN":
            with self._lock:
                self._update(
                    mode="normal",
                    transition_state="rebooting",
                    reboot_required=True,
                    auto_charge_enabled=False,
                    last_warning="NX正在重启并恢复MAXN功耗档位",
                )
            self._schedule_power_mode_reboot(self.config.normal_power_mode)
            return self.snapshot()
        return self._start_normal_services()

    def _start_normal_services(self) -> dict:
        failures = []
        for command, timeout in (
            ([self.config.sensor_start_script], 45),
            (["sudo", "systemctl", "enable", "--now", self.config.monitoring_service], 20),
            ([self.config.navigation_script, "start"], self.config.normal_start_timeout_seconds),
        ):
            try:
                self._run_required(command, timeout)
            except Exception as exc:
                failures.append(str(exc))
        if failures:
            message = "; ".join(failures)
            with self._lock:
                self._update(mode="normal", transition_state="error", last_error=message)
            raise ProtocolError("NORMAL_MODE_RESTORE_FAILED", message)
        with self._lock:
            self._update(
                mode="normal",
                transition_state="ready",
                reboot_required=False,
                auto_charge_enabled=False,
                last_error="",
                last_warning="",
            )
        return self.snapshot()

    def set_auto_charge_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._update(auto_charge_enabled=bool(enabled))

    def _run_required(self, command: list[str], timeout: float) -> None:
        result = self.runner(command, timeout)
        if result.returncode != 0:
            output = (result.stderr or result.stdout or "command failed").strip()[-1200:]
            raise RuntimeError(f"{' '.join(command)}: {output}")

    def _stop_sensor_processes(self) -> None:
        for pattern in (
            "[r]os2 launch livox_driver lidar.launch.py|[l]ivox_driver_node|"
            "[p]ointcloud_to_laserscan_node|[s]tatic_transform_publisher .*base_link livox_frame",
            "[r]tk_ntrip_bridge.py|[s]ixents_gps_driver",
        ):
            self._run_optional(
                ["sudo", "pkill", "-TERM", "-f", pattern],
                10,
                allowed_returncodes={0, 1},
            )

    def _run_optional(self, command: list[str], timeout: float, allowed_returncodes: set[int]) -> None:
        result = self.runner(command, timeout)
        if result.returncode not in allowed_returncodes:
            output = (result.stderr or result.stdout or "command failed").strip()[-1200:]
            raise RuntimeError(f"{' '.join(command)}: {output}")

    def _power_profile(self) -> str:
        try:
            result = self.runner(["sudo", "nvpmodel", "-q"], 5)
            return next(
                (line.strip().removeprefix("NV Power Mode:").strip() for line in result.stdout.splitlines() if "NV Power Mode:" in line),
                "unknown",
            )
        except Exception:
            return "unknown"

    def _schedule_power_mode_reboot(self, mode: int) -> None:
        unit = f"roamerx-power-mode-{int(datetime.now(timezone.utc).timestamp())}"
        try:
            self._run_required(
                [
                    "sudo", "systemd-run", f"--unit={unit}", "--collect",
                    f"--on-active={self.config.reboot_delay_seconds}s",
                    "/usr/sbin/nvpmodel", "--force", "-m", str(mode),
                ],
                15,
            )
        except Exception as exc:
            with self._lock:
                self._update(transition_state="error", reboot_required=False, last_error=str(exc))
            raise ProtocolError("POWER_MODE_REBOOT_FAILED", str(exc)) from exc

    def _set_cooling_marker(self, enabled: bool) -> None:
        if enabled:
            self._cooling_marker.parent.mkdir(parents=True, exist_ok=True)
            self._cooling_marker.touch()
            return
        self._cooling_marker.unlink(missing_ok=True)

    def _load_state(self) -> dict:
        try:
            value = json.loads(self._state_path.read_text(encoding="utf-8"))
            if isinstance(value, dict) and value.get("mode") in {"normal", "cooling_standby"}:
                return value
        except (OSError, ValueError):
            pass
        return {
            "mode": "normal",
            "transition_state": "ready",
            "auto_charge_enabled": False,
            "reboot_required": False,
            "last_error": "",
            "last_warning": "",
            "updated_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        }

    def _update(self, **values) -> None:
        self._state.update(values)
        self._state["updated_at"] = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        self._state_path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self._state_path.with_suffix(self._state_path.suffix + ".tmp")
        temp_path.write_text(json.dumps(self._state, ensure_ascii=False), encoding="utf-8")
        os.replace(temp_path, self._state_path)
