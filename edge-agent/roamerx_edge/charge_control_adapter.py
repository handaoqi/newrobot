from __future__ import annotations

import logging
import shlex
import subprocess
import threading
import time
import uuid
from typing import Callable

from .config import ChargeControlConfig
from .power_mode_controller import PowerModeController
from .protocol import ProtocolError


LOGGER = logging.getLogger(__name__)


class ChargeControlAdapter:
    def __init__(
        self,
        config: ChargeControlConfig,
        power_mode: PowerModeController,
        store=None,
        power_refresh: Callable[[], dict] | None = None,
    ) -> None:
        self.config = config
        self.power_mode = power_mode
        self.store = store
        self._power_refresh = power_refresh
        self._lock = threading.RLock()
        self._full_samples = 0
        self._completion_thread: threading.Thread | None = None
        self._thermal_recovery_started_at: float | None = None
        self._last_thermal_retry_at: float | None = None
        self._previous_charge_state: str | None = None
        self._thermal_retry_thread: threading.Thread | None = None
        self._low_battery_samples = 0
        self._last_low_battery_start_at: float | None = None
        self._manual_disconnect_inhibit_until = 0.0
        self._low_battery_start_thread: threading.Thread | None = None
        self._charge_begin_thread: threading.Thread | None = None
        self._dock_monitor_thread: threading.Thread | None = None
        self._pending_charge_started_at: float | None = None
        self._state_recovery_thread: threading.Thread | None = None
        self._latest_power: dict = {}
        self._pending_charge = False
        self._low_battery_handler = None
        self._full_charge_handler = None
        self._charge_started_handler = None
        persisted = store.get_metadata("low_battery_episode") if store else None
        self._low_battery_episode = persisted if isinstance(persisted, dict) else {}

    def set_low_battery_handler(self, handler) -> None:
        """Register the application-level safe-stop callback."""
        self._low_battery_handler = handler

    def set_full_charge_handler(self, handler) -> None:
        self._full_charge_handler = handler

    def set_charge_started_handler(self, handler) -> None:
        self._charge_started_handler = handler

    def start(self) -> dict:
        with self._lock:
            self._previous_charge_state = None
            self._thermal_recovery_started_at = None
            self._pending_charge = True
            self._pending_charge_started_at = time.monotonic()
        # The arbiter stops ARC before enabling the vendor helper. The helper
        # then provides per-pole feedback while the robot waits for contact.
        pile = self._set_legacy_pile_state("lying")
        diagnostics_refreshed = False
        refresh_error = ""
        if self._power_refresh:
            try:
                refreshed = self._power_refresh() or {}
                with self._lock:
                    self._latest_power = dict(refreshed)
                diagnostics_refreshed = True
            except Exception as exc:
                refresh_error = str(exc)
                LOGGER.warning("failed to refresh dock status after starting charge service", exc_info=True)
        detail = (
            self._dock_detail(self._latest_power)
            if diagnostics_refreshed or self._latest_power.get("charger_controller_mode") == "legacy"
            else "充电诊断状态刷新中"
        )
        self._set_charge_stage("waiting_for_dock", detail)
        if self._charger_ready(self._latest_power):
            return self._begin_charge()
        self._start_dock_monitor()
        response = {
            "charge_stage": "waiting_for_dock",
            "charge_pile": pile,
            "dock_ready": False,
            "diagnostics_refreshed": diagnostics_refreshed,
            "pending_monitor": True,
            "missing": (
                self._missing_dock_conditions(self._latest_power)
                if diagnostics_refreshed else ["充电诊断状态刷新中"]
            ),
        }
        if refresh_error:
            response["diagnostics_refresh_error"] = refresh_error
        return response

    def start_motion_control(self) -> dict:
        snapshot = self.power_mode.snapshot()
        if snapshot.get("auto_charge_enabled") or snapshot.get("charge_stage") not in {None, "idle"}:
            raise ProtocolError("CHARGING_ACTIVE", "cannot start motion control while charging is requested")
        return self._set_motion_control("start")

    def stop_motion_control(self) -> dict:
        return self._set_motion_control("stop")

    def _set_motion_control(self, action: str) -> dict:
        expected = "running" if action == "start" else "stopped"
        if action == "start":
            # `robot-launch start 3 4` exits non-zero when either component
            # is already running.  Starting motion control must be idempotent:
            # retain a healthy dog_task and start only a missing component.
            command = (
                "set -e; "
                "for egg in 3 4; do "
                "if ! robot-launch egg \"$egg\" 2>/dev/null | grep -qi running; then "
                "robot-launch start \"$egg\"; fi; "
                "done; sleep 2; "
                "for egg in 3 4; do "
                "robot-launch egg \"$egg\" 2>/dev/null | grep -qi running; "
                "done"
            )
        else:
            command = (
                "set -e; "
                "robot-launch stop 3 4; sleep 2; "
                "for egg in 3 4; do "
                "robot-launch egg \"$egg\" 2>/dev/null | grep -qi stopped; "
                "done"
            )
        result = self._run(f"motion_{action}", command)
        return {**result, "motion_control": expected}

    def _begin_charge(self) -> dict:
        with self._lock:
            if not self._pending_charge:
                return {"charge_stage": "idle"}
            self._pending_charge = False
            self._pending_charge_started_at = None
        self._set_charge_stage("starting_charge", "充电桩接触已确认，正在停止运控")
        mode = self.power_mode.enter_cooling()
        motion = self.stop_motion_control()
        try:
            charge = self._legacy_lying_confirmed()
        except Exception:
            self.power_mode.set_auto_charge_enabled(False)
            self._set_charge_stage("error", "充电服务启动失败")
            raise
        self.power_mode.set_auto_charge_enabled(True)
        self._set_charge_stage("waiting_current", "等待 BMS 上报充电电流")
        result = {
            "charge_stage": "waiting_current",
            "dock_ready": True,
            "diagnostics_refreshed": True,
            "pending_monitor": False,
            "power_mode": mode,
            "motion": motion,
            "charge": charge,
            "auto_restore_on_full": True,
        }
        if self._charge_started_handler:
            try:
                self._charge_started_handler(dict(result))
            except Exception:
                LOGGER.exception("charge-started callback failed")
        return result

    def stop(self, *, manual: bool = True) -> dict:
        with self._lock:
            self._pending_charge = False
            self._pending_charge_started_at = None
            if manual:
                self._low_battery_samples = 0
                self._manual_disconnect_inhibit_until = (
                    time.monotonic() + self.config.manual_disconnect_auto_charge_pause_seconds
                )
                LOGGER.info(
                    "manual charge disconnect; automatic low-battery charge paused for %.0f seconds",
                    self.config.manual_disconnect_auto_charge_pause_seconds,
                )
        self.power_mode.set_auto_charge_enabled(False)
        self._set_charge_stage("stopping_charge", "正在断开充电并恢复运控")
        try:
            charge = self._return_legacy_and_restore_arc()
        except ProtocolError as exc:
            self._set_charge_stage("error", "充电桩未确认断开")
            raise exc
        mode = self.power_mode.restore_normal()
        self._set_charge_stage("idle", "")
        motion = self.start_motion_control()
        if manual:
            with self._lock:
                self._manual_disconnect_inhibit_until = (
                    time.monotonic() + self.config.manual_disconnect_auto_charge_pause_seconds
                )
        return {"charge": charge, "power_mode": mode, "motion": motion, "auto_restore_on_full": False}

    def _set_legacy_pile_state(self, state: str) -> dict:
        if state not in {"lying", "unknown"}:
            raise ProtocolError("CHARGE_STATE_INVALID", state)
        action = "legacy-lying" if state == "lying" else "legacy-status"
        return self._run(f"pile_{state}", f"sudo {shlex.quote(self.config.arbiter_path)} {action}")

    def _legacy_lying_confirmed(self) -> dict:
        unit = shlex.quote(self.config.service_name)
        command = (
            f"systemctl is-active --quiet {unit}.service; "
            "pgrep -f '[d]og_lying_down' >/dev/null"
        )
        return self._run("legacy_charge_confirm", command)

    def _return_legacy_and_restore_arc(self) -> dict:
        command = (
            f"sudo {shlex.quote(self.config.arbiter_path)} legacy-return; "
            f"robot-launch egg {shlex.quote(self.config.arc_platform_egg)} 2>/dev/null | grep -qi running"
        )
        return self._run("legacy_return", command)

    def observe_power(self, power: dict | None) -> None:
        power = power or {}
        with self._lock:
            self._latest_power = dict(power)
        snapshot = self.power_mode.snapshot()
        if power.get("charging"):
            self._set_charge_stage("charging", "BMS 正在充电")
        elif snapshot.get("auto_charge_enabled") and power.get("thermal_protection"):
            self._set_charge_stage("thermal_protection", "BMS 温度保护")

        # A persisted charging state must not keep the robot in cooling mode
        # after a restart if ARC is back in normal mode and BMS is discharging.
        if (
            snapshot.get("auto_charge_enabled")
            and not power.get("charging")
            and power.get("charger_controller_mode") == "arc_platform"
            and power.get("arc_dock_state") in {None, 0, 5}
        ):
            self.power_mode.set_auto_charge_enabled(False)
            self._set_charge_stage("idle", "未检测到充电接触，已恢复正常工作模式")
            if snapshot.get("mode") == "cooling_standby":
                self._restore_after_stale_charge_async()
            return

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
            if now < self._manual_disconnect_inhibit_until:
                self._low_battery_samples = 0
                return
            if percent >= self.config.low_battery_rearm_percent:
                self._low_battery_samples = 0
                if self._low_battery_episode.get("active"):
                    self._low_battery_episode = {}
                    self._persist_low_battery_episode()
                    LOGGER.info(
                        "battery recovered to %d%%; low-battery return latch re-armed",
                        percent,
                    )
                return
            if percent >= self.config.low_battery_start_percent:
                self._low_battery_samples = 0
                return
            if self._low_battery_episode.get("active"):
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
            episode_id = str(uuid.uuid4())
            self._low_battery_episode = {
                "active": True,
                "episode_id": episode_id,
                "battery_percent": percent,
                "triggered_at_epoch": time.time(),
            }
            self._persist_low_battery_episode()
            self._low_battery_start_thread = threading.Thread(
                target=self._start_for_low_battery,
                args=(episode_id, percent),
                daemon=True,
                name="low-battery-charge-start",
            )
            self._low_battery_start_thread.start()

    def _persist_low_battery_episode(self) -> None:
        if self.store:
            self.store.set_metadata("low_battery_episode", self._low_battery_episode)

    def _start_for_low_battery(self, episode_id: str, percent: int) -> None:
        try:
            LOGGER.warning(
                "battery is %d%% (below %d%%); requesting one automatic return-to-charge task episode=%s",
                percent,
                self.config.low_battery_start_percent,
                episode_id,
            )
            if callable(self._low_battery_handler):
                self._low_battery_handler(episode_id, percent)
        except Exception:
            LOGGER.exception("automatic low-battery return request failed episode=%s", episode_id)

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
            result = self._set_legacy_pile_state("lying")
            LOGGER.info("charge retry completed: %s", result.get("stdout", "").strip()[-200:])
        except Exception:
            LOGGER.exception("automatic charge retry after thermal protection failed")

    def _finish_full_charge(self) -> None:
        try:
            LOGGER.info("battery full confirmed; stopping charge and restoring normal mode")
            self.stop(manual=False)
            if callable(self._full_charge_handler):
                self._full_charge_handler()
        except Exception:
            LOGGER.exception("automatic full-charge restore failed")

    def _restore_after_stale_charge_async(self) -> None:
        with self._lock:
            if self._state_recovery_thread and self._state_recovery_thread.is_alive():
                return
            self._state_recovery_thread = threading.Thread(
                target=self._restore_after_stale_charge,
                daemon=True,
                name="stale-charge-state-restore",
            )
            self._state_recovery_thread.start()

    def _restore_after_stale_charge(self) -> None:
        try:
            self.power_mode.restore_normal()
        except Exception:
            LOGGER.exception("failed to restore normal mode after stale charge state")

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
            power.get("charger_controller_mode") == "legacy",
            power.get("bluetooth_connected") is True,
            power.get("charge_pin") == 1,
            power.get("negative_contact") == 1,
            power.get("positive_contact") == 1,
        ))

    @staticmethod
    def _missing_dock_conditions(power: dict) -> list[str]:
        checks = (
            ("charger_controller_mode", "旧版充电诊断未运行"),
            ("bluetooth_connected", "蓝牙未连接"),
            ("charge_pin", "充电极片未接触"),
            ("negative_contact", "负极异常"),
            ("positive_contact", "正极异常"),
        )
        return [
            label for key, label in checks
            if (
                power.get(key) != "legacy"
                if key == "charger_controller_mode"
                else power.get(key) is not True and power.get(key) != 1
            )
        ]

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

    def _start_dock_monitor(self) -> None:
        if not self._power_refresh:
            return
        with self._lock:
            if self._dock_monitor_thread and self._dock_monitor_thread.is_alive():
                return
            self._dock_monitor_thread = threading.Thread(
                target=self._monitor_pending_dock,
                daemon=True,
                name="pending-dock-status-monitor",
            )
            self._dock_monitor_thread.start()

    def _monitor_pending_dock(self) -> None:
        interval = max(0.1, float(self.config.dock_status_poll_interval_seconds))
        timeout = max(interval, float(self.config.dock_contact_wait_timeout_seconds))
        while True:
            with self._lock:
                if not self._pending_charge:
                    return
                started_at = self._pending_charge_started_at or time.monotonic()
                elapsed = time.monotonic() - started_at
                if elapsed >= timeout:
                    self._pending_charge = False
                    self._pending_charge_started_at = None
                    self._set_charge_stage("error", "等待充电极片接触超时")
                    LOGGER.error("dock contact was not confirmed within %.1f seconds", timeout)
                    return
            time.sleep(interval)
            try:
                power = self._power_refresh() or {}
            except Exception:
                LOGGER.warning("pending dock status refresh failed", exc_info=True)
                continue
            self.observe_power(power)

    def _set_charge_stage(self, stage: str, detail: str = "") -> None:
        setter = getattr(self.power_mode, "set_charge_stage", None)
        if callable(setter):
            setter(stage, detail)
