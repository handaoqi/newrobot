from __future__ import annotations

import logging
import re
import shlex
import subprocess
from dataclasses import dataclass
from typing import Callable

from .config import ChargeControlConfig, TelemetryConfig
from .telemetry_collector import TelemetryCollector


LOGGER = logging.getLogger(__name__)


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

    def poll(self) -> None:
        try:
            self._poll_power()
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

    def _poll_power(self) -> None:
        result = self.runner(
            [
                "ssh",
                "-o", "BatchMode=yes",
                "-o", "ConnectTimeout=3",
                self.config.battery_ssh_host,
                "timeout 4 ecal_mon_cli --proto power_mcu/bms_info -c 1 2>/dev/null; "
                "echo __CHARGE_SERVICE__; systemctl is-active roamerx-charge-pile.service 2>/dev/null || true; "
                "echo __CHARGE_MODE__; cat /var/lib/roamerx-charge-pile/state 2>/dev/null || echo unknown; "
                "echo __CHARGE_STATE__; sudo journalctl -u roamerx-charge-pile.service -n 80 --no-pager 2>/dev/null "
                "| grep -E 'connected=|charge pin=' | tail -n 2",
            ],
            7,
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
        connected_match = re.findall(r"connected=(yes|no)", result.stdout)
        state_matches = re.findall(
            r"charge pin=(\d+).*c-status=(\d+),c\+status=(\d+)",
            result.stdout,
        )
        bluetooth_connected = bool(connected_match and connected_match[-1] == "yes")
        charge_pin = int(state_matches[-1][0]) if state_matches else None
        negative_contact = int(state_matches[-1][1]) if state_matches else None
        positive_contact = int(state_matches[-1][2]) if state_matches else None
        if not bluetooth_connected:
            charge_pin = None
            negative_contact = None
            positive_contact = None
        controller_active = "__CHARGE_SERVICE__\nactive" in result.stdout
        mode_match = re.search(r"__CHARGE_MODE__\n(lying|unknown)", result.stdout)
        controller_mode = mode_match.group(1) if mode_match else "unknown"
        charging_requested = controller_mode == "lying"
        charger_confirmed = bool(
            controller_active
            and bluetooth_connected
            and charge_pin == 1
            and negative_contact == 1
            and positive_contact == 1
        )
        charging = bool(
            current_ma is not None
            and current_ma >= self.config.charging_current_threshold_ma
        )
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
        elif controller_active and bluetooth_connected:
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
            "card=$(awk '/USB-Audio/{gsub(/[\\[\\]]/,\"\",$2); print $2; exit}' /proc/asound/cards); "
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
