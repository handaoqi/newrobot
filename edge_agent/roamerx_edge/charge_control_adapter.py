from __future__ import annotations

import logging
import shlex
import subprocess
import threading
import time

from .config import ChargeControlConfig
from .power_mode_controller import PowerModeController
from .protocol import ProtocolError


LOGGER = logging.getLogger(__name__)


class ChargeControlAdapter:
    def __init__(self, config: ChargeControlConfig, power_mode: PowerModeController) -> None:
        self.config = config
        self.power_mode = power_mode
        self._lock = threading.RLock()
        self._full_samples = 0
        self._completion_thread: threading.Thread | None = None
        self._thermal_recovery_started_at: float | None = None
        self._last_thermal_retry_at: float | None = None
        self._previous_charge_state: str | None = None
        self._thermal_retry_thread: threading.Thread | None = None
        self._low_battery_samples = 0
        self._last_low_battery_start_at: float | None = None
        self._low_battery_start_thread: threading.Thread | None = None
        self._charge_begin_thread: threading.Thread | None = None
        self._latest_power: dict = {}
        self._pending_charge = False
        self._low_battery_handler = None
        self._full_charge_handler = None

    def set_low_battery_handler(self, handler) -> None:
        """Register the application-level safe-stop callback."""
        self._low_battery_handler = handler

    def set_full_charge_handler(self, handler) -> None:
        self._full_charge_handler = handler

    def start(self) -> dict:
        with self._lock:
            self._previous_charge_state = None
            self._thermal_recovery_started_at = None
            self._pending_charge = True
        # This only announces charging readiness to the pile.  It must not
        # stop the 3588 runtime until the dock contacts are confirmed.
        pile = self._set_charge_pile_state("lying")
        self._set_charge_stage("waiting_for_dock", self._dock_detail(self._latest_power))
        if self._charger_ready(self._latest_power):
            return self._begin_charge()
        return {
            "charge_stage": "waiting_for_dock",
            "charge_pile": pile,
            "dock_ready": False,
            "missing": self._missing_dock_conditions(self._latest_power),
        }

    def start_motion_control(self) -> dict:
        snapshot = self.power_mode.snapshot()
        if snapshot.get("auto_charge_enabled") or snapshot.get("charge_stage") not in {None, "idle"}:
            raise ProtocolError("CHARGING_ACTIVE", "cannot start motion control while charging is requested")
        return self._set_motion_control("start")

    def stop_motion_control(self) -> dict:
        return self._set_motion_control("stop")

    def _set_motion_control(self, action: str) -> dict:
        expected = "running" if action == "start" else "stopped"
        command = (
            "set -e; "
            f"robot-launch {action} 3 4; sleep 2; "
            "for egg in 3 4; do "
            f"robot-launch egg \"$egg\" 2>/dev/null | grep -qi {expected}; "
            "done"
        )
        result = self._run(f"motion_{action}", command)
        return {**result, "motion_control": expected}

    def _set_charge_pile_state(self, state: str) -> dict:
        unit = shlex.quote(self.config.service_name)
        command = (
            f"printf '{state}\\n' | sudo tee /var/lib/roamerx-charge-pile/state >/dev/null; "
            f"sudo systemctl restart {unit}.service; sleep 2; "
            f"systemctl is-active {unit}.service"
        )
        return self._run(f"pile_{state}", command)

    def _begin_charge(self) -> dict:
        with self._lock:
            if not self._pending_charge:
                return {"charge_stage": "idle"}
            self._pending_charge = False
        self._set_charge_stage("starting_charge", "充电桩接触已确认，正在停止运控")
        mode = self.power_mode.enter_cooling()
        motion = self.stop_motion_control()
        try:
            charge = self._start_remote()
        except Exception:
            self.power_mode.set_auto_charge_enabled(False)
            self._set_charge_stage("error", "充电服务启动失败")
            raise
        self.power_mode.set_auto_charge_enabled(True)
        self._set_charge_stage("waiting_current", "等待 BMS 上报充电电流")
        return {"power_mode": mode, "motion": motion, "charge": charge, "auto_restore_on_full": True}

    def _start_remote(self) -> dict:
        unit = shlex.quote(self.config.service_name)
        cooling_eggs = " ".join(shlex.quote(egg) for egg in self.config.cooling_stop_eggs)
        cooling_services = " ".join(shlex.quote(service) for service in self.config.cooling_stop_services)
        command = (
            f"sudo systemctl stop {cooling_services}; "
            "robot-launch stop arc_platform >/dev/null 2>&1 || true; "
            f"robot-launch stop {cooling_eggs} >/dev/null 2>&1 || true; sleep 4; "
            "printf 'lying\n' | sudo tee /var/lib/roamerx-charge-pile/state >/dev/null; "
            f"sudo systemctl restart {unit}.service; "
            "sleep 3; "
            f"systemctl is-active {unit}.service"
        )
        result = self._run("start", command)
        return result

    def stop(self) -> dict:
        with self._lock:
            self._pending_charge = False
        self.power_mode.set_auto_charge_enabled(False)
        self._set_charge_stage("stopping_charge", "正在断开充电并恢复运控")
        # The return command has already made the pile leave its charging
        # state before its optional service/egg checks run.  Do not leave the
        # whole robot in cooling_standby when one of those checks is late.
        try:
            charge = self._stop_remote()
        except ProtocolError as exc:
            LOGGER.warning("charge disconnect completed with diagnostics: %s", exc)
            charge = {"warning": str(exc), "disconnect_requested": True}
        mode = self.power_mode.restore_normal()
        self._set_charge_stage("idle", "")
        motion = self.start_motion_control()
        return {"charge": charge, "power_mode": mode, "motion": motion, "auto_restore_on_full": False}

    def _stop_remote(self) -> dict:
        unit = shlex.quote(self.config.service_name)
        return_executable = shlex.quote(self.config.return_executable)
        controller_service = shlex.quote(self.config.controller_service_name)
        normal_services = " ".join(shlex.quote(service) for service in self.config.normal_start_services)
        required_eggs = (
            "arc_platform", "spline_daemon", "motion_control",
            "zenoh_route", "dog_task", "ecal2ros", "imu_daemon",
        )
        verify_eggs = " ".join(shlex.quote(egg) for egg in required_eggs)
        command = (
            f"sudo systemctl stop {unit}.service 2>/dev/null || true; "
            f"sudo chmod +x {return_executable}; "
            f"sudo timeout --signal=TERM --kill-after=2 6 {return_executable} "
            ">/tmp/roamerx-charge-return.log 2>&1 || true; "
            "sudo pkill -KILL -x dog_returning 2>/dev/null || true; "
            "printf 'unknown\n' | sudo tee /var/lib/roamerx-charge-pile/state >/dev/null; "
            f"sudo systemctl start {unit}.service || true; sleep 2; "
            "robot-launch stop push_image dog_task motion_control spline_daemon "
            "ecal2ros imu_daemon monitor zenoh_route arc_platform >/dev/null 2>&1 || true; sleep 2; "
            f"sudo systemctl start {normal_services} || true; "
            f"sudo systemctl restart {controller_service} || true; sleep 15; "
            "robot-launch start 3 4 >/dev/null 2>&1; sleep 2; "
            "for egg in 3 4; do robot-launch egg \"$egg\" 2>/dev/null | grep -qi running || true; done; "
            f"systemctl is-active --quiet {controller_service} || true; "
            f"for egg in {verify_eggs}; do robot-launch egg \"$egg\" 2>/dev/null | grep -qi running || true; done; "
            "mkdir -p /tmp/roamerx_ros_logs; "
            ". /opt/ros/humble/setup.bash; "
            "export ROS_LOG_DIR=/tmp/roamerx_ros_logs ROS_DOMAIN_ID=24 RMW_IMPLEMENTATION=rmw_zenoh_cpp; "
            "mc_state=\"$(timeout 12 ros2 topic echo /arc/mc_state --once 2>/dev/null || true)\"; "
            "printf '%s\\n' \"$mc_state\"; "
            # Some controller builds do not publish this ROS topic. The dock
            # return, pile reset, and egg checks above are authoritative; keep
            # this output for diagnosis but do not leave charge state stuck.
            "true"
        )
        result = self._run("stop", command)
        return result

    def observe_power(self, power: dict | None) -> None:
        power = power or {}
        with self._lock:
            self._latest_power = dict(power)
        snapshot = self.power_mode.snapshot()
        if power.get("charging"):
            self._set_charge_stage("charging", "BMS 正在充电")
        elif snapshot.get("auto_charge_enabled") and power.get("thermal_protection"):
            self._set_charge_stage("thermal_protection", "BMS 温度保护")

        if self._pending_charge and self._charger_ready(power):
            self._begin_charge_async()

        if not snapshot.get("auto_charge_enabled"):
            self._full_samples = 0
            with self._lock:
                self._previous_charge_state = None
                self._thermal_recovery_started_at = None
                pending_charge = self._pending_charge
            if (
                not pending_charge
                and not power.get("charging")
                and snapshot.get("charge_stage") not in {None, "idle"}
            ):
                # A prior disconnect can complete on the dock while a final
                # optional diagnostic fails. Reconcile the UI with the BMS
                # instead of permanently showing "stopping charge".
                self._set_charge_stage("idle", "")
            self._observe_low_battery(power)
            return

        self._observe_thermal_recovery(power)
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

    def _observe_low_battery(self, power: dict) -> None:
        if not power.get("available", False) or power.get("percent") is None:
            with self._lock:
                self._low_battery_samples = 0
            return
        percent = int(power["percent"])
        now = time.monotonic()
        with self._lock:
            if percent > self.config.low_battery_start_percent:
                self._low_battery_samples = 0
                return
            if (
                self._pending_charge
                or self.power_mode.snapshot().get("auto_charge_enabled")
                or (self._charge_begin_thread and self._charge_begin_thread.is_alive())
            ):
                return
            self._low_battery_samples += 1
            cooldown_elapsed = (
                self._last_low_battery_start_at is None
                or now - self._last_low_battery_start_at >= self.config.low_battery_start_cooldown_seconds
            )
            start_running = bool(
                self._low_battery_start_thread and self._low_battery_start_thread.is_alive()
            )
            if self._low_battery_samples < self.config.low_battery_confirmation_samples:
                return
            if not cooldown_elapsed or start_running:
                return
            self._low_battery_samples = 0
            self._last_low_battery_start_at = now
            self._low_battery_start_thread = threading.Thread(
                target=self._start_for_low_battery,
                daemon=True,
                name="low-battery-charge-start",
            )
            self._low_battery_start_thread.start()

    def _start_for_low_battery(self) -> None:
        try:
            LOGGER.warning(
                "battery is at or below %d%%; starting automatic charge",
                self.config.low_battery_start_percent,
            )
            if callable(self._low_battery_handler):
                self._low_battery_handler()
            result = self.start()
            LOGGER.info("automatic low-battery charge prepared: %s", result.get("charge_stage", "starting"))
        except Exception:
            LOGGER.exception("automatic low-battery charge start failed")

    def _observe_thermal_recovery(self, power: dict) -> None:
        state = power.get("charge_state")
        if not state or not power.get("available", False):
            return
        now = time.monotonic()
        with self._lock:
            previous = self._previous_charge_state
            self._previous_charge_state = str(state)
            if previous == "thermal_protection" and state == "waiting":
                self._thermal_recovery_started_at = now
                LOGGER.info("thermal protection cleared; waiting for charger recovery")
            elif state != "waiting":
                self._thermal_recovery_started_at = None

            recovery_started = self._thermal_recovery_started_at
            cooldown_elapsed = (
                self._last_thermal_retry_at is None
                or now - self._last_thermal_retry_at >= self.config.thermal_retry_cooldown_seconds
            )
            retry_running = bool(
                self._thermal_retry_thread and self._thermal_retry_thread.is_alive()
            )
            should_retry = bool(
                state == "waiting"
                and recovery_started is not None
                and now - recovery_started >= self.config.thermal_recovery_delay_seconds
                and cooldown_elapsed
                and not retry_running
            )
            if should_retry:
                self._last_thermal_retry_at = now
                self._thermal_recovery_started_at = None
                self._thermal_retry_thread = threading.Thread(
                    target=self._retry_after_thermal_protection,
                    daemon=True,
                    name="thermal-charge-retry",
                )
                self._thermal_retry_thread.start()

    def _retry_after_thermal_protection(self) -> None:
        try:
            LOGGER.warning("retrying charge start after thermal protection recovery")
            result = self._start_remote()
            LOGGER.info("charge retry completed: %s", result.get("stdout", "").strip()[-200:])
        except Exception:
            LOGGER.exception("automatic charge retry after thermal protection failed")

    def _finish_full_charge(self) -> None:
        try:
            LOGGER.info("battery full confirmed; stopping charge and restoring normal mode")
            self.stop()
            if callable(self._full_charge_handler):
                self._full_charge_handler()
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

    @staticmethod
    def _charger_ready(power: dict) -> bool:
        return all((
            power.get("charger_controller_active") is True,
            power.get("bluetooth_connected") is True,
            power.get("charge_pin") == 1,
            power.get("negative_contact") == 1,
            power.get("positive_contact") == 1,
        ))

    @staticmethod
    def _missing_dock_conditions(power: dict) -> list[str]:
        checks = (
            ("charger_controller_active", "充电桩控制器离线"),
            ("bluetooth_connected", "蓝牙未连接"),
            ("charge_pin", "充电极片未接触"),
            ("negative_contact", "负极异常"),
            ("positive_contact", "正极异常"),
        )
        return [label for key, label in checks if power.get(key) is not True and power.get(key) != 1]

    def _dock_detail(self, power: dict) -> str:
        missing = self._missing_dock_conditions(power)
        return "、".join(missing) if missing else "充电桩接触已确认"

    def _begin_charge_async(self) -> None:
        with self._lock:
            if self._charge_begin_thread and self._charge_begin_thread.is_alive():
                return
            self._charge_begin_thread = threading.Thread(
                target=self._begin_charge,
                daemon=True,
                name="verified-dock-charge-start",
            )
            self._charge_begin_thread.start()

    def _set_charge_stage(self, stage: str, detail: str = "") -> None:
        setter = getattr(self.power_mode, "set_charge_stage", None)
        if callable(setter):
            setter(stage, detail)
