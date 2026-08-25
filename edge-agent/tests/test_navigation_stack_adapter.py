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
