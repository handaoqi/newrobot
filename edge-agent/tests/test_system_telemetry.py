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
    assert status["network"]["type"] == "5G SA"
    assert status["network"]["signal_percent"] == 68


def test_charger_state_reports_thermal_protection_when_hot_and_current_is_missing():
    collector, probe = make_probe(
        [
            "power: 61\nvolt: 46358\ntemp: 46000\nerror: 1034\n"
            "__CHARGE_SERVICE__\nactive\n__CHARGE_STATE__\n"
            "charge pin=1,c-adc=3.29,c+adc=1.70,c-status=1,c+status=1,tagid=65535\n"
            "connected=yes\n",
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
    assert power["current_a"] is None
    assert power["bluetooth_connected"] is True
    assert power["charge_pin"] == 1


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
