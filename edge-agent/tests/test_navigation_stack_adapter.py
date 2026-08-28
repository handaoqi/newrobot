from roamerx_edge.config import NavigationStackConfig
from roamerx_edge.navigation_stack_adapter import NavigationStackAdapter


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
