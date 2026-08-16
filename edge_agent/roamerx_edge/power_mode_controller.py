from __future__ import annotations

import json
import logging
import os
import re
import shlex
import subprocess
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from .config import PowerModeConfig
from .protocol import ProtocolError


LOGGER = logging.getLogger(__name__)

LOCAL_SERVICE_NAMES = {
    "roamerx-dev-agent.service": ("dev_agent", "NX · 远程开发 Agent"),
    "roamerx-robot-mcp.service": ("robot_mcp", "NX · 机器人 MCP"),
    "roamerx-zenoh.service": ("zenoh", "NX · ROS Zenoh 路由"),
    "roamerx-5g-share.service": ("network_share", "NX · 5G 网络共享"),
}
CONTROLLER_EGG_NAMES = {
    "arc_platform": "3588 · 本体基础平台",
    "monitor": "3588 · 设备监控",
    "time_sync": "3588 · 硬件时间同步",
    "power_daemon": "3588 · 电源管理",
    "spline_daemon": "3588 · 轨迹插值",
    "motion_control": "3588 · 运动控制",
    "zenoh_route": "3588 · ROS Zenoh 路由",
    "dog_task": "3588 · 机器狗任务",
    "push_image": "3588 · 视频推流",
    "ecal2ros": "3588 · eCAL/ROS 转发",
    "imu_daemon": "3588 · IMU 转发",
}
CONTROLLER_SERVICE_NAMES = {
    "robot-launch.service": "3588 · Robot Launch 管理器",
    "roamerx-charge-pile.service": "3588 · 充电控制服务",
    "rkaiq_3A.service": "3588 · 相机 3A",
    "rknn_server.service": "3588 · RKNN 推理服务",
    "lightdm.service": "3588 · 图形会话",
}


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
        probes = [
            ("edge_agent", "NX · Edge Agent", ["systemctl", "is-active", "roamerx-edge-agent.service"], True),
        ]
        for unit in self.config.always_on_services:
            key, name = LOCAL_SERVICE_NAMES.get(
                unit,
                (f"nx_service_{self._safe_key(unit)}", f"NX · {unit}"),
            )
            probes.append((key, name, ["systemctl", "is-active", unit], True))
        probes.extend((
            ("detection_video", "NX · AI识别与视频", ["systemctl", "is-active", self.config.monitoring_service], not cooling),
            ("teleop_bridge", "NX · 遥控运动桥", ["systemctl", "is-active", self.config.teleop_bridge_service], True),
            ("lidar_imu", "NX · Livox LiDAR/IMU", ["pgrep", "-f", "[l]ivox_driver_node"], not cooling),
            ("rtk", "NX · RTK差分定位", ["pgrep", "-f", "[r]tk_ntrip_bridge.py"], not cooling),
            ("localization", "NX · 激光定位", ["pgrep", "-f", "[l]ocalization_node"], not cooling),
            ("navigation", "NX · Nav2导航", ["pgrep", "-f", "[n]avigo_container"], not cooling),
        ))
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
        remote = self._remote_runtime_status()
        legacy_charge_session = self._legacy_charge_session_active()
        for egg in (*self.config.controller_always_eggs, *self.config.controller_runtime_eggs):
            key = f"controller_egg_{self._safe_key(egg)}"
            available = remote is not None and egg in remote["eggs"]
            active = bool(available and remote["eggs"][egg])
            expected = True if egg in self.config.controller_always_eggs else not cooling
            if egg == "arc_platform" and legacy_charge_session:
                expected = False
            services[key] = {
                "name": CONTROLLER_EGG_NAMES.get(egg, f"3588 · {egg}"),
                "active": active,
                "available": available,
                "expected_active": expected,
                "matches_mode": available and active == expected,
            }
        for unit in (*self.config.controller_always_services, *self.config.controller_runtime_services):
            key = f"controller_service_{self._safe_key(unit)}"
            available = remote is not None and unit in remote["services"]
            active = bool(available and remote["services"][unit])
            expected = True if unit in self.config.controller_always_services else not cooling
            services[key] = {
                "name": CONTROLLER_SERVICE_NAMES.get(unit, f"3588 · {unit}"),
                "active": active,
                "available": available,
                "expected_active": expected,
                "matches_mode": available and active == expected,
            }
        with self._lock:
            self._services = services
        return self.snapshot()

    def reconcile_startup(self) -> dict:
        state = self.snapshot()
        if state.get("mode") == "cooling_standby":
            self._set_cooling_marker(True)
            self._run_optional(
                ["sudo", "systemctl", "disable", "--now", self.config.monitoring_service],
                20,
                allowed_returncodes={0, 1},
            )
            self._stop_sensor_processes()
            self._stop_remote_cooling_services()
            with self._lock:
                self._update(transition_state="ready", reboot_required=False, last_error="", last_warning="")
            return self.snapshot()
        self._set_cooling_marker(False)
        try:
            return self._start_normal_services(skip_arc=self._legacy_charge_session_active())
        except ProtocolError:
            # Keep Edge online so the platform can display the failure and retry.
            LOGGER.exception("failed to reconcile normal-mode services during startup")
            return self.snapshot()

    def enter_cooling(self) -> dict:
        with self._lock:
            self._update(mode="cooling_standby", transition_state="entering", last_error="", last_warning="")
        try:
            self._set_cooling_marker(True)
            passive_confirmed = self._request_motor_passive()
            self._run_required(["sudo", "systemctl", "disable", "--now", self.config.monitoring_service], 20)
            self._run_required([self.config.navigation_script, "full-stop"], 30)
            self._stop_sensor_processes()
            self._stop_remote_cooling_services()
            with self._lock:
                self._update(
                    mode="cooling_standby",
                    transition_state="ready",
                    reboot_required=False,
                    last_error="",
                    last_warning="" if passive_confirmed else "未确认关节自由态，请检查本体控制模式",
                )
        except Exception as exc:
            with self._lock:
                self._update(mode="cooling_standby", transition_state="error", last_error=str(exc))
            raise ProtocolError("COOLING_MODE_FAILED", str(exc)) from exc
        return self.snapshot()

    def restore_normal(self) -> dict:
        with self._lock:
            self._update(mode="normal", transition_state="restoring", last_error="", last_warning="")
        self._set_cooling_marker(False)
        self._run_required(["sudo", "systemctl", "enable", self.config.monitoring_service], 20)
        return self._start_normal_services()

    def _start_normal_services(self, *, skip_arc: bool = False) -> dict:
        # Motion recovery after leaving the dock must not be held hostage by
        # localization readiness. Navigation exposes its own readiness state.
        try:
            local_units = [
                *self.config.always_on_services,
                self.config.monitoring_service,
                self.config.teleop_bridge_service,
            ]
            self._run_required(
                ["sudo", "systemctl", "enable", "--now", *local_units], 30
            )
            self._start_remote_normal_services(skip_arc=skip_arc)
        except Exception as exc:
            message = str(exc)
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
        # The navigation script may wait for NDT convergence. It must not hold
        # up Edge's MQTT status reporting after the robot has left the dock.
        navigation_log = "/tmp/roamerx-normal-navigation-start.log"
        command = [
            "bash", "-lc",
            f"nohup {shlex.quote(self.config.navigation_script)} start "
            f">{shlex.quote(navigation_log)} 2>&1 </dev/null &",
        ]
        try:
            self._run_required(command, 5)
        except Exception as exc:
            warning = f"导航栈启动请求失败：{exc}"
            LOGGER.warning("normal mode restored without navigation launch: %s", exc)
            with self._lock:
                self._update(last_warning=warning)
        return self.snapshot()

    def _start_remote_normal_services(self, *, skip_arc: bool = False) -> None:
        services = (*self.config.controller_always_services, *self.config.controller_runtime_services)
        eggs = (*self.config.controller_always_eggs, *self.config.controller_runtime_eggs)
        if skip_arc:
            eggs = tuple(egg for egg in eggs if egg != "arc_platform")
        quoted_services = " ".join(shlex.quote(service) for service in services)
        quoted_eggs = " ".join(shlex.quote(egg) for egg in eggs)
        command = (
            "set -e; "
            f"sudo systemctl start {quoted_services}; "
            f"for egg in {quoted_eggs}; do "
            "if ! robot-launch egg \"$egg\" 2>/dev/null | grep -q running; then "
            "robot-launch stop \"$egg\" >/dev/null 2>&1 || true; sleep 1; "
            "robot-launch start \"$egg\" >/dev/null; fi; done; "
            f"for service in {quoted_services}; do systemctl is-active --quiet \"$service\"; done; "
            "deadline=$((SECONDS + 30)); "
            f"for egg in {quoted_eggs}; do "
            "until robot-launch egg \"$egg\" 2>/dev/null | grep -q running; do "
            "if [ \"$SECONDS\" -ge \"$deadline\" ]; then robot-launch egg \"$egg\" 2>/dev/null || true; exit 1; fi; "
            "sleep 1; done; done"
        )
        self._run_required(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", self.config.controller_host, command],
            self.config.normal_start_timeout_seconds,
        )

    def _stop_remote_cooling_services(self) -> None:
        """Stop all non-charging 3588 workloads without touching BMS control."""
        eggs = self.config.controller_runtime_eggs
        services = self.config.controller_runtime_services
        quoted_eggs = " ".join(shlex.quote(egg) for egg in eggs)
        quoted_services = " ".join(shlex.quote(service) for service in services)
        command = (
            "set -e; "
            f"for egg in {quoted_eggs}; do robot-launch stop \"$egg\" >/dev/null 2>&1 || true; done; "
            f"sudo systemctl stop {quoted_services}; "
            f"for egg in {quoted_eggs}; do "
            "robot-launch egg \"$egg\" 2>/dev/null | grep -q stopped || exit 1; "
            "done; "
            f"for service in {quoted_services}; do "
            "! systemctl is-active --quiet \"$service\"; "
            "done"
        )
        self._run_required(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", self.config.controller_host, command],
            90,
        )

    def _legacy_charge_session_active(self) -> bool:
        state = self.snapshot()
        return state.get("charge_stage") in {
            "waiting_for_dock", "starting_charge", "waiting_current", "charging", "thermal_protection",
        }

    def set_auto_charge_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._update(auto_charge_enabled=bool(enabled))

    def set_charge_stage(self, stage: str, detail: str = "") -> None:
        with self._lock:
            self._update(charge_stage=stage, charge_stage_detail=detail)

    def _run_required(self, command: list[str], timeout: float) -> None:
        result = self.runner(command, timeout)
        if result.returncode != 0:
            output = (result.stderr or result.stdout or "command failed").strip()
            if len(output) > 1200:
                output = f"{output[:600]}\n... output truncated ...\n{output[-600:]}"
            raise RuntimeError(f"{' '.join(command)}: {output}")

    def _request_motor_passive(self) -> bool:
        command = [
            "bash",
            "-lc",
            "source /opt/ros/humble/setup.bash && "
            "export ROS_DOMAIN_ID=24 RMW_IMPLEMENTATION=rmw_zenoh_cpp && "
            "timeout 8 ros2 topic pub --once /teleop_action std_msgs/msg/String '{data: passive}'",
        ]
        try:
            result = self.runner(command, 10)
            if result.returncode == 0:
                # The SDK enters motor-free mode shortly after passive() succeeds.
                self.runner(["sleep", "2"], 3)
                return True
            LOGGER.warning("motor passive request failed: %s", (result.stderr or result.stdout).strip()[-800:])
        except Exception:
            LOGGER.exception("motor passive request failed")
        return False

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

    def _remote_runtime_status(self) -> dict[str, dict[str, bool]] | None:
        checks = []
        eggs = (*self.config.controller_always_eggs, *self.config.controller_runtime_eggs)
        services = (*self.config.controller_always_services, *self.config.controller_runtime_services)
        for index, egg in enumerate(eggs):
            quoted = shlex.quote(egg)
            checks.append(
                f"if robot-launch egg {quoted} 2>/dev/null | grep -q running; "
                f"then echo egg_{index}=1; else echo egg_{index}=0; fi"
            )
        for index, service in enumerate(services):
            quoted = shlex.quote(service)
            checks.append(
                f"if systemctl is-active --quiet {quoted}; "
                f"then echo service_{index}=1; else echo service_{index}=0; fi"
            )
        try:
            result = self.runner(
                [
                    "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
                    self.config.controller_host, "; ".join(checks),
                ],
                10,
            )
            if result.returncode != 0:
                return None
            values = {
                key: value == "1"
                for key, value in re.findall(r"^([a-z0-9_]+)=([01])$", result.stdout, flags=re.MULTILINE)
            }
            return {
                "eggs": {egg: values.get(f"egg_{index}", False) for index, egg in enumerate(eggs)},
                "services": {
                    service: values.get(f"service_{index}", False)
                    for index, service in enumerate(services)
                },
            }
        except Exception:
            return None

    @staticmethod
    def _safe_key(value: str) -> str:
        return re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_")

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
            "charge_stage": "idle",
            "charge_stage_detail": "",
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
