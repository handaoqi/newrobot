from __future__ import annotations

import glob
import json
import logging
import re
import shlex
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .config import ChargeControlConfig, TelemetryConfig
from .telemetry_collector import TelemetryCollector


LOGGER = logging.getLogger(__name__)

_GIB = 1024 ** 3


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str


def _run(command: list[str], timeout: float) -> CommandResult:
    completed = subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
    return CommandResult(completed.returncode, completed.stdout)


class SystemTelemetryProbe:
    def __init__(
        self,
        config: TelemetryConfig,
        telemetry: TelemetryCollector,
        charge_config: ChargeControlConfig | None = None,
        runner: Callable[[list[str], float], CommandResult] = _run,
    ) -> None:
        self.config = config
        self.telemetry = telemetry
        self.charge_config = charge_config or ChargeControlConfig()
        self.runner = runner
        self._last_legacy_status: dict[str, int | bool | None] = {}
        self._last_legacy_status_at = 0.0
        self._power_poll_lock = threading.Lock()

    def poll(self) -> None:
        try:
            self.poll_power()
        except Exception:
            LOGGER.warning("failed to read battery telemetry", exc_info=True)
        try:
            self._poll_network()
        except Exception:
            LOGGER.warning("failed to read cellular telemetry", exc_info=True)
        try:
            self._poll_audio()
        except Exception:
            LOGGER.warning("failed to read audio telemetry", exc_info=True)
        try:
            self._poll_storage()
        except Exception:
            LOGGER.warning("failed to read storage telemetry", exc_info=True)

    def poll_power(self) -> dict:
        """Refresh only BMS/dock telemetry and return the new snapshot."""
        with self._power_poll_lock:
            self._poll_power()
            return self.telemetry.latest_power() or {}

    def _poll_storage(self) -> None:
        """Report free space, plus whatever retention last deleted.

        This deliberately does not walk the bag and map trees - that costs
        hundreds of stat calls on every probe interval. Occupancy comes from
        statvfs, and the per-session detail comes from the report
        mapping_rosbag.sh already wrote when it pruned.
        """
        probe = Path(self.config.storage_probe_path).expanduser()
        payload: dict = {"path": str(probe), "available": False}
        try:
            usage = shutil.disk_usage(probe)
        except OSError as exc:
            payload["error"] = str(exc)
            self.telemetry.on_storage(payload)
            return
        payload.update(
            available=True,
            total_bytes=usage.total,
            used_bytes=usage.used,
            free_bytes=usage.free,
            free_gib=round(usage.free / _GIB, 2),
            used_percent=round(usage.used / usage.total * 100, 1) if usage.total else None,
        )
        retention = self._latest_retention_report()
        if retention:
            payload["last_retention"] = retention
        self.telemetry.on_storage(payload)

    def _latest_retention_report(self) -> dict | None:
        """Summarise the most recent retention pass across all bag roots."""
        newest: dict | None = None
        newest_at = 0.0
        for path in sorted(glob.glob(self.config.storage_retention_report_glob)):
            try:
                report = json.loads(Path(path).read_text(encoding="utf-8"))
            except (OSError, ValueError):
                # A half-written or corrupt report is not worth an alarm; the
                # occupancy numbers above are the part that matters.
                continue
            generated_at = float(report.get("generated_at_unix") or 0.0)
            if generated_at < newest_at:
                continue
            deleted = [
                {
                    "path": entry.get("path"),
                    "size_bytes": entry.get("size_bytes"),
                    "reason": entry.get("reason"),
                }
                for root in report.get("roots", [])
                for entry in root.get("deleted", [])
            ]
            newest_at = generated_at
            newest = {
                "report_path": path,
                "generated_at_unix": generated_at,
                "applied": bool(report.get("applied")),
                "reclaimed_bytes": report.get("reclaimed_bytes", 0),
                "failed_count": report.get("failed_count", 0),
                # Bounded: a pass that removes dozens of sessions must not turn
                # every status message into a manifest of them.
                "deleted_count": len(deleted),
                "deleted": deleted[:10],
            }
        return newest

    def _poll_power(self) -> None:
        result = self.runner(
            [
                "ssh",
                "-o", "BatchMode=yes",
                "-o", "ConnectTimeout=3",
                self.config.battery_ssh_host,
                "timeout 4 ecal_mon_cli --proto power_mcu/bms_info -c 1 2>/dev/null; "
                "echo __ARC_PLATFORM__; if robot-launch egg arc_platform 2>/dev/null | grep -qi running; then echo running; else echo inactive; fi; "
                "echo __ARC_DOCK_STATE__; "
                "mkdir -p /tmp/roamerx_ros_logs; . /opt/ros/humble/setup.bash; "
                "export ROS_LOG_DIR=/tmp/roamerx_ros_logs ROS_DOMAIN_ID=24 RMW_IMPLEMENTATION=rmw_zenoh_cpp; "
                "timeout 2 ros2 topic echo /arc/dock_state --once 2>/dev/null || true; "
                "echo __LEGACY_SERVICE__; if systemctl is-active --quiet roamerx-charge-pile.service; then echo active; else echo inactive; fi; "
                "echo __LEGACY_MODE__; cat /var/lib/roamerx-charge-pile/state 2>/dev/null || echo unknown; "
                "echo __LEGACY_MODULE__; cat /run/roamerx-charge-pile/module 2>/dev/null || echo unknown; "
                "echo __LEGACY_STATUS__; sudo journalctl -u roamerx-charge-pile.service -n 40 --no-pager 2>/dev/null | grep -E 'connected=|charge pin=' | tail -n 4; "
                "echo __ARC_SERIAL_OWNER__; sudo lsof -n -F c /dev/ttyUSB0 2>/dev/null | sed -n 's/^c//p' | head -n 1",
            ],
            12,
        )
        values = {
            key: int(value)
            for key, value in re.findall(
                r"^(power|volt|current|temp|error):\s*(-?\d+)\s*$",
                result.stdout,
                flags=re.MULTILINE,
            )
        }
        if "power" not in values:
            raise RuntimeError(f"battery output unavailable (rc={result.returncode})")
        current_ma = values.get("current")
        if current_ma is not None and current_ma >= 2**31:
            current_ma -= 2**32
        arc_platform_active = "__ARC_PLATFORM__\nrunning" in result.stdout
        dock_match = re.search(
            r"__ARC_DOCK_STATE__\n(.*?)(?:__ARC_SERIAL_OWNER__|\Z)",
            result.stdout,
            flags=re.DOTALL,
        )
        dock_raw = dock_match.group(1) if dock_match else ""
        dock_state_match = re.search(r"^state:\s*(\d+)", dock_raw, flags=re.MULTILINE)
        dock_error_match = re.search(
            r"^error_msg:\s*['\"]?(.*?)['\"]?\s*$", dock_raw, flags=re.MULTILINE
        )
        arc_dock_state = int(dock_state_match.group(1)) if dock_state_match else None
        arc_dock_error = dock_error_match.group(1).strip() if dock_error_match else ""
        legacy_active = "__LEGACY_SERVICE__\nactive" in result.stdout
        legacy_mode_match = re.search(r"__LEGACY_MODE__\n(lying|unknown|return)", result.stdout)
        legacy_mode = legacy_mode_match.group(1) if legacy_mode_match else "unknown"
        legacy_module_match = re.search(r"__LEGACY_MODULE__\n(lying|unknown|return)", result.stdout)
        legacy_module = legacy_module_match.group(1) if legacy_module_match else "unknown"
        legacy_match = re.search(
            r"__LEGACY_STATUS__\n(.*?)(?:__ARC_SERIAL_OWNER__|\Z)",
            result.stdout,
            flags=re.DOTALL,
        )
        legacy_status = self._parse_legacy_status(legacy_match.group(1) if legacy_match else "")
        serial_match = re.search(r"__ARC_SERIAL_OWNER__\n([^\n]*)", result.stdout)
        serial_owner = serial_match.group(1).strip() if serial_match else ""
        charging = bool(
            current_ma is not None
            and current_ma >= self.config.charging_current_threshold_ma
        )
        if legacy_active:
            self._remember_legacy_status(legacy_status)
        elif (
            # The legacy probe must never preempt a healthy ARC controller.
            # Both implementations use /dev/ttyUSB0; legacy-status stops
            # arc_platform while it samples the vendor helper. Running that
            # probe from the normal telemetry loop interrupts motion/teleop.
            not arc_platform_active
            and not charging
            and values["power"] > self.charge_config.low_battery_start_percent
            and arc_dock_state in {None, 0, 5}
            and time.monotonic() - self._last_legacy_status_at
            >= self.config.legacy_charge_status_interval_seconds
        ):
            self._probe_legacy_status()
        if not legacy_status.get("available"):
            legacy_status = dict(self._last_legacy_status)

        dock_contact = arc_dock_state in {1, 2}
        if legacy_active:
            bluetooth_connected = legacy_status.get("bluetooth_connected")
            charge_pin = legacy_status.get("charge_pin")
            negative_contact = legacy_status.get("negative_contact")
            positive_contact = legacy_status.get("positive_contact")
            charger_confirmed = bool(
                bluetooth_connected is True
                and charge_pin == 1
                and negative_contact == 1
                and positive_contact == 1
            )
        else:
            # ARC exposes only a combined contact state. The cached legacy
            # probe is diagnostic data, not an ARC contact assertion.
            bluetooth_connected = legacy_status.get("bluetooth_connected")
            charge_pin = legacy_status.get("charge_pin")
            negative_contact = legacy_status.get("negative_contact")
            positive_contact = legacy_status.get("positive_contact")
            charger_confirmed = bool(arc_platform_active and dock_contact and not arc_dock_error)
        controller_active = legacy_active or arc_platform_active
        controller_mode = "legacy" if legacy_active else "arc_platform"
        charging_requested = (legacy_active and legacy_mode == "lying") or arc_dock_state in {1, 2}
        temperature_c = round(values["temp"] / 1000, 1) if "temp" in values else None
        bms_error_code = values.get("error")
        thermal_protection_by_bms = bms_error_code == 1034
        thermal_protection_by_temperature = bool(
            temperature_c is not None
            and temperature_c >= self.config.charging_overheat_threshold_c
        )
        thermal_protection = bool(
            charging_requested
            and charger_confirmed
            and not charging
            and (thermal_protection_by_bms or thermal_protection_by_temperature)
        )
        if charging:
            charge_state = "charging"
        elif thermal_protection:
            charge_state = "thermal_protection"
        elif charging_requested and charger_confirmed:
            charge_state = "waiting"
        elif controller_active:
            charge_state = "ready"
        else:
            charge_state = "disconnected"
        self.telemetry.on_battery(
            values["power"],
            charging,
            voltage_v=round(values["volt"] / 1000, 2) if "volt" in values else None,
            current_a=round(current_ma / 1000, 2) if current_ma is not None else None,
            temperature_c=temperature_c,
            error_code=bms_error_code,
            source="power_mcu/bms_info",
            bluetooth_connected=bluetooth_connected,
            charge_pin=charge_pin,
            negative_contact=negative_contact,
            positive_contact=positive_contact,
            charger_controller_active=controller_active,
            charger_controller_mode=controller_mode,
            charger_backend="legacy_helper" if legacy_active else "arc_platform",
            legacy_charge_module=legacy_module,
            arc_platform_active=arc_platform_active,
            arc_dock_state=arc_dock_state,
            arc_dock_error=arc_dock_error,
            arc_dock_contact=dock_contact,
            serial_owner=serial_owner,
            charging_requested=charging_requested,
            charge_state=charge_state,
            thermal_protection=thermal_protection,
            thermal_protection_source=(
                "bms_error_1034" if thermal_protection_by_bms
                else "temperature_threshold" if thermal_protection_by_temperature
                else ""
            ),
            charging_overheat_threshold_c=self.config.charging_overheat_threshold_c,
            full_battery_percent=self.charge_config.full_battery_percent,
            low_battery_start_percent=self.charge_config.low_battery_start_percent,
            low_battery_confirmation_samples=self.charge_config.low_battery_confirmation_samples,
            rated_capacity_wh=self.config.battery_rated_capacity_wh,
            remaining_energy_wh=round(
                self.config.battery_rated_capacity_wh * max(0, min(100, values["power"])) / 100, 1
            ),
            remaining_energy_estimated=True,
        )

    def _probe_legacy_status(self) -> None:
        try:
            self.runner(
                [
                    "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
                    self.config.battery_ssh_host,
                    f"sudo {shlex.quote(self.charge_config.arbiter_path)} legacy-status >/dev/null 2>&1 || true",
                ],
                22,
            )
            result = self.runner(
                [
                    "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
                    self.config.battery_ssh_host,
                    "sudo cat /run/roamerx-charge-pile/legacy-status.txt 2>/dev/null || true",
                ],
                5,
            )
            self._remember_legacy_status(self._parse_legacy_status(result.stdout))
        except Exception:
            LOGGER.warning("legacy charge-pile diagnostic probe failed", exc_info=True)

    def _remember_legacy_status(self, status: dict[str, int | bool | None]) -> None:
        self._last_legacy_status_at = time.monotonic()
        if status.get("available"):
            self._last_legacy_status = dict(status)

    @staticmethod
    def _parse_legacy_status(raw: str) -> dict[str, int | bool | None]:
        connected_match = re.findall(r"connected=(yes|no)", raw)
        state_matches = re.findall(
            r"charge pin=(\d+).*c-status=(\d+),c\+status=(\d+)", raw,
        )
        connected = bool(connected_match and connected_match[-1] == "yes")
        if not connected:
            return {"available": bool(connected_match), "bluetooth_connected": False,
                    "charge_pin": None, "negative_contact": None, "positive_contact": None}
        if not state_matches:
            return {"available": True, "bluetooth_connected": True,
                    "charge_pin": None, "negative_contact": None, "positive_contact": None}
        charge_pin, negative, positive = state_matches[-1]
        return {"available": True, "bluetooth_connected": True,
                "charge_pin": int(charge_pin), "negative_contact": int(negative),
                "positive_contact": int(positive)}

    def _poll_network(self) -> None:
        serial_script = (
            'stty -F "$1" 115200 raw -echo; '
            'timeout 2 cat "$1" & reader=$!; sleep 0.2; '
            'printf "AT+CSQ\\rAT+QNWINFO\\rAT+QENG=\\\"servingcell\\\"\\r" > "$1"; '
            'wait "$reader"'
        )
        result = self.runner(
            ["timeout", "3", "bash", "-c", serial_script, "_", self.config.modem_at_device],
            4,
        )
        csq_match = re.search(r"\+CSQ:\s*(\d+),", result.stdout)
        network_match = re.search(r'\+QNWINFO:\s*"([^"]+)"', result.stdout)
        if not csq_match and not network_match:
            raise RuntimeError(f"cellular output unavailable (rc={result.returncode})")
        csq = int(csq_match.group(1)) if csq_match else None
        signal_percent = None if csq is None or csq == 99 else round(min(csq, 31) * 100 / 31)
        raw_type = network_match.group(1) if network_match else ""
        network_type = {
            "NR5G-SA": "5G SA",
            "NR5G-NSA": "5G NSA",
            "FDD LTE": "4G LTE",
            "TDD LTE": "4G LTE",
        }.get(raw_type, raw_type or "蜂窝网络")
        serving = re.search(r'\+QENG:\s*"servingcell"[^\r\n]*', result.stdout)
        self.telemetry.on_network(
            network_type,
            signal_percent,
            csq=csq,
            modem="Quectel RG255AA-CN",
            serving_cell=serving.group(0) if serving else None,
            source=self.config.modem_at_device,
        )

    def _poll_audio(self) -> None:
        configured_sink = shlex.quote(self.config.charger_speaker_sink)
        remote_probe = (
            f"configured={configured_sink}; "
            "if pactl get-sink-volume \"$configured\" >/dev/null 2>&1; then sink=$configured; "
            "else sink=$(pactl list short sinks | awk '$2 ~ /usb-/ {print $2; exit}'); fi; "
            "test -n \"$sink\"; "
            "pactl get-sink-volume \"$sink\"; pactl get-sink-mute \"$sink\"; echo __SINK__:$sink"
        )
        remote = self.runner(
            [
                "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
                self.config.battery_ssh_host,
                remote_probe,
            ],
            5,
        )
        local_probe = (
            "card=$(awk '/USB-Audio/{print $1; exit}' /proc/asound/cards); "
            "test -n \"$card\"; "
            "controls=$(amixer -c \"$card\" scontrols); "
            "control=$(printf '%s\\n' \"$controls\" | sed -n \"s/^Simple mixer control '\\([^']*\\)'.*/\\1/p\" "
            "| grep -E '^(PCM|Playback Feature Unit)$' | head -n1); "
            "test -n \"$control\"; amixer -c \"$card\" sget \"$control\"; "
            "echo __CARD__:$card; echo __CONTROL__:$control"
        )
        local = self.runner(["bash", "-lc", local_probe], 4)
        remote_volume = re.search(r"/\s*(\d+)%\s*/", remote.stdout)
        local_volumes = re.findall(r"\[(\d+)%\]", local.stdout)
        remote_mute = re.search(r"Mute:\s*(yes|no)", remote.stdout)
        remote_sink = re.search(r"__SINK__:(\S+)", remote.stdout)
        local_card = re.search(r"__CARD__:(\S+)", local.stdout)
        local_control = re.search(r"__CONTROL__:(.+)", local.stdout)
        self.telemetry.on_audio(
            speaker_3588={
                "online": bool(remote_volume),
                "volume_percent": int(remote_volume.group(1)) if remote_volume else None,
                "muted": bool(remote_mute and remote_mute.group(1) == "yes"),
                "device": "3588 Type-C USB Audio",
                "sink": remote_sink.group(1) if remote_sink else None,
            },
            speaker_nx={
                "online": bool(local_volumes),
                "volume_percent": int(local_volumes[0]) if local_volumes else None,
                "muted": "[off]" in local.stdout,
                "device": "NX USB Audio",
                "card": local_card.group(1) if local_card else None,
                "control": local_control.group(1).strip() if local_control else None,
            },
        )
