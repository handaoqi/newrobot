from types import SimpleNamespace

from roamerx_edge.config import TelemetryConfig
from roamerx_edge.safety_policy import RuntimeSafetyState
from roamerx_edge.system_telemetry import CommandResult, SystemTelemetryProbe
from roamerx_edge.telemetry_collector import TelemetryCollector


def make_probe(outputs):
    collector = TelemetryCollector(
        SimpleNamespace(current_map_id="1", current_map_version="v1"),
        RuntimeSafetyState(),
    )

    def runner(_command, _timeout):
        return CommandResult(0, outputs.pop(0))

    return collector, SystemTelemetryProbe(TelemetryConfig(), collector, runner=runner)


def test_probe_parses_bms_and_5g_status():
    collector, probe = make_probe(
        [
            "power: 48\nvolt: 45300\ncurrent: 3300\ntemp: 43000\nerror: 271\n",
            '\\r\\n+CSQ: 21,99\\r\\n+QNWINFO: "NR5G-SA","46001","NR N78",627264\\r\\n',
            "Volume: front-left: 65536 / 100% / 0.00 dB\nMute: no\n",
            "Front Left: Playback 6400 [56%] [on]\n",
        ]
    )

    probe.poll()
    status = collector.build_status_snapshot()

    assert status["power"]["percent"] == 48
    assert status["power"]["charging"] is True
    assert status["power"]["voltage_v"] == 45.3
    assert status["power"]["current_a"] == 3.3
    assert status["power"]["remaining_energy_wh"] == 103.7
    assert status["power"]["rated_capacity_wh"] == 216.0
    assert status["power"]["remaining_energy_estimated"] is True
    assert status["network"]["type"] == "5G SA"
    assert status["network"]["signal_percent"] == 68


def test_charger_state_reports_thermal_protection_when_hot_and_current_is_missing():
    collector, probe = make_probe(
        [
            "power: 61\nvolt: 46358\ntemp: 49000\nerror: 1034\n"
            "__ARC_PLATFORM__\nrunning\n__ARC_DOCK_STATE__\nstate: 2\nerror_msg: ''\n"
            "__ARC_SERIAL_OWNER__\narc_platform\n",
            '+CSQ: 21,99\n+QNWINFO: "NR5G-SA"\n',
            "Volume: front-left: 65536 / 100% / 0.00 dB\nMute: no\n",
            "Front Left: Playback 6400 [56%] [on]\n",
        ]
    )

    probe.poll()
    power = collector.build_status_snapshot()["power"]

    assert power["charging"] is False
    assert power["charge_state"] == "thermal_protection"
    assert power["thermal_protection"] is True
    assert power["charging_overheat_threshold_c"] == 49.0
    assert power["current_a"] is None
    assert power["bluetooth_connected"] is None
    assert power["charge_pin"] is None


def test_bms_error_1034_keeps_thermal_protection_latched_below_ui_threshold():
    collector, probe = make_probe(
        [
            "power: 89\nvolt: 48313\ntemp: 47000\nerror: 1034\n"
            "__ARC_PLATFORM__\nrunning\n__ARC_DOCK_STATE__\nstate: 2\nerror_msg: ''\n"
            "__ARC_SERIAL_OWNER__\narc_platform\n",
            '+CSQ: 21,99\n+QNWINFO: "NR5G-SA"\n',
            "Volume: front-left: 65536 / 100% / 0.00 dB\nMute: no\n",
            "Front Left: Playback 6400 [56%] [on]\n",
        ]
    )

    probe.poll()
    power = collector.build_status_snapshot()["power"]

    assert power["temperature_c"] == 47.0
    assert power["thermal_protection"] is True
    assert power["charge_state"] == "thermal_protection"
    assert power["thermal_protection_source"] == "bms_error_1034"


def test_persistent_charge_controller_is_ready_without_requesting_charge():
    collector, probe = make_probe(
        [
            "power: 61\nvolt: 46358\ncurrent: -1700\ntemp: 45000\nerror: 0\n"
            "__ARC_PLATFORM__\nrunning\n__ARC_DOCK_STATE__\nstate: 0\nerror_msg: ''\n"
            "__ARC_SERIAL_OWNER__\narc_platform\n",
            '+CSQ: 21,99\n+QNWINFO: "NR5G-SA"\n',
            "Volume: front-left: 65536 / 100% / 0.00 dB\nMute: no\n",
            "Front Left: Playback 6400 [56%] [on]\n",
        ]
    )

    probe.poll()
    power = collector.build_status_snapshot()["power"]

    assert power["charger_controller_active"] is True
    assert power["charging_requested"] is False
    assert power["charge_state"] == "ready"
    assert power["charging"] is False


def test_probe_decodes_unsigned_bms_current_as_signed_discharge():
    collector, probe = make_probe(
        [
            "power: 51\nvolt: 44780\ncurrent: 4294965416\ntemp: 48000\nerror: 1039\n"
            "__CHARGE_SERVICE__\ninactive\n__CHARGE_STATE__\n"
            "charge pin=1,c-adc=3.29,c+adc=1.70,c-status=1,c+status=1,tagid=65535\n"
            "connected=yes\n",
            '+CSQ: 21,99\n+QNWINFO: "NR5G-SA"\n',
            "Volume: front-left: 65536 / 100% / 0.00 dB\nMute: no\n",
            "Front Left: Playback 6400 [56%] [on]\n",
        ]
    )

    probe.poll()
    power = collector.build_status_snapshot()["power"]

    assert power["current_a"] == -1.88
    assert power["charging"] is False
