import json
import subprocess

import pytest

from roamerx_edge.config import NavigationStackConfig
from roamerx_edge.navigation_stack_adapter import NavigationStackAdapter
from roamerx_edge.protocol import ProtocolError


def test_reload_map_is_deferred_when_mapping_stopped_consumers(tmp_path, monkeypatch):
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(tmp_path / "nav.sh")))
    monkeypatch.setattr(adapter, "status", lambda: {
        "action": "status",
        "returncode": 0,
        "stdout": "Processes:\nvel_cmd_udp_pub\nLocalization:\nstatus: unavailable\n",
        "stderr": "",
    })
    monkeypatch.setattr(
        adapter,
        "reload_map",
        lambda *_args: (_ for _ in ()).throw(AssertionError("reload must be deferred")),
    )

    result = adapter.reload_map_if_running("/maps/map.pcd", "/maps/map.yaml")

    assert result["deferred"] is True
    assert result["reason"] == "map_consumers_inactive"


def test_reload_map_runs_when_both_consumers_are_active(tmp_path, monkeypatch):
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(tmp_path / "nav.sh")))
    monkeypatch.setattr(adapter, "status", lambda: {
        "action": "status",
        "returncode": 0,
        "stdout": (
            "ros2 launch localization localization.launch.py\n"
            "ros2 launch robot_navigo navigation_bringup.launch.py\n"
        ),
        "stderr": "",
    })
    calls = []
    monkeypatch.setattr(adapter, "reload_map", lambda pcd, yaml: calls.append((pcd, yaml)) or {"returncode": 0})

    result = adapter.reload_map_if_running("/maps/map.pcd", "/maps/map.yaml")

    assert result["returncode"] == 0
    assert calls == [("/maps/map.pcd", "/maps/map.yaml")]


def test_reload_map_reloads_localization_when_nav2_is_inactive(tmp_path, monkeypatch):
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(tmp_path / "nav.sh")))
    monkeypatch.setattr(adapter, "status", lambda: {
        "action": "status",
        "returncode": 0,
        "stdout": "ros2 launch localization localization.launch.py\nstatus: 0\n",
        "stderr": "",
    })
    calls = []
    monkeypatch.setattr(
        adapter,
        "reload_localization_map",
        lambda pcd: calls.append(pcd) or {"returncode": 0},
    )
    monkeypatch.setattr(
        adapter,
        "reload_navigation_map",
        lambda *_args: (_ for _ in ()).throw(AssertionError("inactive Nav2 must be deferred")),
    )

    result = adapter.reload_map_if_running("/maps/map.pcd", "/maps/map.yaml")

    assert calls == ["/maps/map.pcd"]
    assert result["localization_reloaded"] is True
    assert result["navigation_reloaded"] is False
    assert result["deferred_consumers"] == ["navigation"]


def test_reload_map_reloads_nav2_when_localization_is_inactive(tmp_path, monkeypatch):
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(tmp_path / "nav.sh")))
    monkeypatch.setattr(adapter, "status", lambda: {
        "action": "status",
        "returncode": 0,
        "stdout": "ros2 launch robot_navigo navigation_bringup.launch.py\n",
        "stderr": "",
    })
    calls = []
    monkeypatch.setattr(
        adapter,
        "reload_navigation_map",
        lambda yaml: calls.append(yaml) or {"returncode": 0},
    )
    monkeypatch.setattr(
        adapter,
        "reload_localization_map",
        lambda *_args: (_ for _ in ()).throw(AssertionError("inactive localization must be deferred")),
    )

    result = adapter.reload_map_if_running("/maps/map.pcd", "/maps/map.yaml")

    assert calls == ["/maps/map.yaml"]
    assert result["localization_reloaded"] is False
    assert result["navigation_reloaded"] is True
    assert result["deferred_consumers"] == ["localization"]


def test_navigation_map_reload_rejects_unsuccessful_service_response(tmp_path, monkeypatch):
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(tmp_path / "nav.sh")))
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *_args, **_kwargs: subprocess.CompletedProcess(
            args=["ros2", "service", "call"],
            returncode=0,
            stdout="nav2_msgs.srv.LoadMap_Response(result=1)",
            stderr="",
        ),
    )

    with pytest.raises(ProtocolError) as captured:
        adapter.reload_navigation_map("/maps/map.yaml")

    assert captured.value.code == "NAVIGATION_MAP_RELOAD_FAILED"
    assert captured.value.details["yaml_path"] == "/maps/map.yaml"


def test_navigation_map_reload_prefers_legacy_traversable_sidecar(tmp_path, monkeypatch):
    raw_yaml = tmp_path / "map.yaml"
    raw_yaml.write_text("image: map.pgm\n", encoding="utf-8")
    traversable_yaml = tmp_path / "map_traversable.yaml"
    traversable_yaml.write_text("image: map_traversable.pgm\n", encoding="utf-8")
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(tmp_path / "nav.sh")))
    calls = []

    def successful_run(args, **_kwargs):
        calls.append(args)
        return subprocess.CompletedProcess(args=args, returncode=0, stdout="result: 0", stderr="")

    monkeypatch.setattr(subprocess, "run", successful_run)

    result = adapter.reload_navigation_map(str(raw_yaml))

    assert result["traversable_grid"] is True
    assert result["requested_yaml_path"] == str(raw_yaml)
    assert result["yaml_path"] == str(traversable_yaml)
    assert str(traversable_yaml) in calls[0][-1]



def _ready_status_stdout(*, localization_status: str = "3") -> str:
    return (
        "ros2 launch localization localization.launch.py\n"
        "ros2 launch robot_navigo navigation_bringup.launch.py\n"
        "/planner_server\n"
        "/controller_server\n"
        "/bt_navigator\n"
        "active [3]\n"
        "/follow_waypoints\n"
        "/cmd_vel\n"
        f"status: {localization_status}\n"
    )


def test_start_is_already_ready_only_when_localization_is_valid(tmp_path, monkeypatch):
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(tmp_path / "nav.sh")))
    runs = []
    monkeypatch.setattr(adapter, "_run", lambda action, timeout_seconds=0: runs.append((action, timeout_seconds)) or {"action": action, "returncode": 0})

    monkeypatch.setattr(adapter, "status", lambda: {"returncode": 0, "stdout": _ready_status_stdout(localization_status="0")})
    adapter.start()
    assert runs == [("start", 180)]

    runs.clear()
    monkeypatch.setattr(adapter, "status", lambda: {"returncode": 0, "stdout": _ready_status_stdout(localization_status="3")})
    result = adapter.start()
    assert result["recovery"] == "already_ready"
    assert runs == []


def test_looks_ready_reads_tokens_after_verbose_cmd_vel_dump():
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path="/tmp/nav.sh"))
    verbose = "cmd_vel:\n" + ("Node name: ecal2ros2\n" * 200)
    stdout = verbose + _ready_status_stdout()
    assert adapter._looks_ready(stdout[-6000:]) is True


def test_restart_localization_allows_process_and_map_service_startup(tmp_path, monkeypatch):
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(tmp_path / "nav.sh")))
    runs = []
    monkeypatch.setattr(
        adapter,
        "_run",
        lambda action, timeout_seconds=0: runs.append((action, timeout_seconds))
        or {"action": action, "returncode": 0},
    )

    adapter.restart_localization()

    assert runs == [("restart-localization", 90)]


def test_timeout_output_is_decoded_before_building_protocol_error(tmp_path, monkeypatch):
    script = tmp_path / "nav.sh"
    script.write_text("#!/bin/sh\n", encoding="utf-8")
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(script)))

    def raise_timeout(*_args, **_kwargs):
        raise subprocess.TimeoutExpired(
            cmd=[str(script), "start"],
            timeout=180,
            output=b"partial stdout \xff",
            stderr=b"sensor lock timed out",
        )

    monkeypatch.setattr(subprocess, "run", raise_timeout)

    with pytest.raises(ProtocolError) as captured:
        adapter._run("start", timeout_seconds=180)

    assert captured.value.code == "NAV_COMMAND_FAILED"
    assert captured.value.message == "sensor lock timed out"
    json.dumps({"error_message": captured.value.message})


def test_execution_activation_failure_rolls_back_to_configured_inactive(tmp_path, monkeypatch):
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(tmp_path / "nav.sh")))
    monkeypatch.setattr(adapter, "status", lambda: {"returncode": 0, "stdout": ""})
    calls = []

    def run(action, *, timeout_seconds):
        calls.append(action)
        if action == "activate-execution":
            raise ProtocolError("NAV_COMMAND_FAILED", "controller lifecycle resume failed")
        assert action == "deactivate-execution"
        return {"action": action, "returncode": 0, "stdout": ""}

    monkeypatch.setattr(adapter, "_run", run)

    with pytest.raises(ProtocolError) as captured:
        adapter.activate_execution()

    assert captured.value.code == "NAVIGATION_EXECUTION_ACTIVATION_FAILED"
    assert calls == ["activate-execution", "deactivate-execution"]
    assert captured.value.details["rollback"]["action"] == "deactivate-execution"
    assert adapter.lifecycle_snapshot()["execution_state"] == "configured_inactive"
    assert adapter.lifecycle_snapshot()["execution_active"] is False


def test_ten_no_goal_lifecycle_cycles_leave_execution_inactive(tmp_path, monkeypatch):
    adapter = NavigationStackAdapter(NavigationStackConfig(script_path=str(tmp_path / "nav.sh")))
    monkeypatch.setattr(adapter, "status", lambda: {"returncode": 0, "stdout": ""})
    actions = []
    monkeypatch.setattr(
        adapter,
        "_run",
        lambda action, *, timeout_seconds: actions.append(action)
        or {"action": action, "returncode": 0, "stdout": ""},
    )

    for _ in range(10):
        adapter.prepare()
        adapter.activate_execution()
        adapter.deactivate_execution()
        snapshot = adapter.lifecycle_snapshot()
        assert snapshot["stack_prepared"] is True
        assert snapshot["execution_active"] is False
        assert snapshot["execution_state"] == "configured_inactive"

    assert actions.count("prepare") == 10
    assert actions.count("activate-execution") == 10
    assert actions.count("deactivate-execution") == 10
