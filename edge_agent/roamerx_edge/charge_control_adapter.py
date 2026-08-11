from __future__ import annotations

import logging
import shlex
import subprocess
import threading

from .config import ChargeControlConfig
from .power_mode_controller import PowerModeController
from .protocol import ProtocolError


LOGGER = logging.getLogger(__name__)


class ChargeControlAdapter:
    def __init__(self, config: ChargeControlConfig, power_mode: PowerModeController) -> None:
        self.config = config
        self.power_mode = power_mode
        self._lock = threading.Lock()
        self._full_samples = 0
        self._completion_thread: threading.Thread | None = None

    def start(self) -> dict:
        mode = self.power_mode.enter_cooling()
        try:
            charge = self._start_remote()
        except Exception:
            self.power_mode.set_auto_charge_enabled(False)
            raise
        self.power_mode.set_auto_charge_enabled(True)
        mode = self.power_mode.ensure_cooling_power_profile()
        return {"power_mode": mode, "charge": charge, "auto_restore_on_full": True}

    def _start_remote(self) -> dict:
        unit = shlex.quote(self.config.service_name)
        working_directory = shlex.quote(self.config.working_directory)
        executable = shlex.quote(self.config.executable)
        cooling_eggs = " ".join(shlex.quote(egg) for egg in self.config.cooling_stop_eggs)
        command = (
            f"sudo systemctl stop {unit}.service 2>/dev/null || true; "
            "robot-launch stop arc_platform >/dev/null 2>&1 || true; "
            f"robot-launch stop {cooling_eggs} >/dev/null 2>&1 || true; sleep 4; "
            f"sudo systemd-run --unit={unit} --collect --property=User=root "
            f"--working-directory={working_directory} {executable}; "
            "sleep 3; "
            f"systemctl is-active {unit}.service"
        )
        return self._run("start", command)

    def stop(self) -> dict:
        self.power_mode.set_auto_charge_enabled(False)
        charge = self._stop_remote()
        mode = self.power_mode.restore_normal()
        return {"charge": charge, "power_mode": mode, "auto_restore_on_full": False}

    def _stop_remote(self) -> dict:
        unit = shlex.quote(self.config.service_name)
        normal_eggs = " ".join(shlex.quote(egg) for egg in self.config.normal_start_eggs)
        command = (
            f"sudo systemctl stop {unit}.service; "
            f"robot-launch start {normal_eggs} >/dev/null 2>&1; sleep 5; "
            "robot-launch start arc_platform >/dev/null 2>&1; sleep 3; "
            "robot-launch egg arc_platform"
        )
        return self._run("stop", command)

    def observe_power(self, power: dict | None) -> None:
        power = power or {}
        snapshot = self.power_mode.snapshot()
        if not snapshot.get("auto_charge_enabled"):
            self._full_samples = 0
            return
        full = bool(
            power.get("available")
            and int(power.get("percent") or 0) >= self.config.full_battery_percent
            and power.get("charger_controller_active")
        )
        self._full_samples = self._full_samples + 1 if full else 0
        if self._full_samples < self.config.full_confirmation_samples:
            return
        with self._lock:
            if self._completion_thread and self._completion_thread.is_alive():
                return
            self._full_samples = 0
            self.power_mode.set_auto_charge_enabled(False)
            self._completion_thread = threading.Thread(
                target=self._finish_full_charge,
                daemon=True,
                name="full-charge-restore",
            )
            self._completion_thread.start()

    def _finish_full_charge(self) -> None:
        try:
            LOGGER.info("battery full confirmed; stopping charge and restoring normal mode")
            self._stop_remote()
            self.power_mode.restore_normal()
        except Exception:
            LOGGER.exception("automatic full-charge restore failed")

    def _run(self, action: str, remote_command: str) -> dict:
        completed = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", self.config.remote_host, remote_command],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=self.config.command_timeout_seconds,
        )
        result = {
            "action": action,
            "returncode": completed.returncode,
            "stdout": completed.stdout[-3000:],
            "stderr": completed.stderr[-3000:],
        }
        if completed.returncode != 0:
            raise ProtocolError("CHARGE_CONTROL_FAILED", result["stderr"] or result["stdout"])
        return result
