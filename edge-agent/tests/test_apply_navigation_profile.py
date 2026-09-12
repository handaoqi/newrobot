from types import SimpleNamespace

from roamerx_edge.navigation_controllers import (
    SUPPORTED_NAVIGATION_COMBOS,
    navigation_capabilities,
)
from roamerx_edge.protocol import ProtocolError
from roamerx_edge.ros_adapter import RosAdapter


def test_navigation_capabilities_lists_all_registered_combos():
    caps = navigation_capabilities()
    assert caps["global_controllers"] == ["navfn", "smac_hybrid", "theta_star"]
    assert caps["local_controllers"] == ["ilqr", "mppi", "rpp"]
    assert len(SUPPORTED_NAVIGATION_COMBOS) == 9
    assert {"global_controller": "navfn", "local_controller": "rpp"} in caps["supported_combos"]
    assert {
        "global_controller": "smac_hybrid", "local_controller": "ilqr"
    } in caps["supported_combos"]


def test_apply_navigation_profile_rolls_back_on_readback_failure(monkeypatch):
    adapter = object.__new__(RosAdapter)
    adapter._last_good_navigation_profile = {
        "generation": 1,
        "global_controller": "theta_star",
        "global_plugin_id": "ThetaStar",
        "local_controller": "rpp",
        "local_plugin_id": "RPP",
        "detour_enabled": True,
        "collision_slowdown_enabled": True,
        "collision_stop_enabled": True,
        "require_yaw": False,
        "final_approach": False,
        "outdoor": False,
        "readback": {},
    }
    calls = []

    adapter.set_global_controller = lambda mode: calls.append(("global", mode))
    adapter.set_safety_profile = lambda **kwargs: calls.append(("safety", kwargs))
    adapter.set_waypoint_profile = lambda **kwargs: calls.append(("waypoint", kwargs))
    adapter.set_local_controller = lambda mode: calls.append(("local", mode))
    adapter.set_smoother = lambda mode: calls.append(("smoother", mode))
    adapter.apply_outdoor_gps_profile = lambda **kwargs: calls.append(("outdoor", kwargs))
    adapter._rtk_is_navigation_pose_source = lambda: False

    def fake_get(node, names, **kwargs):
        if node == "/planner_server":
            # Wrong readback triggers rollback.
            return {names[0]: True}
        return {name: True for name in names}

    adapter._get_remote_parameters = fake_get
    restored = []

    def fake_restore(snapshot):
        restored.append(snapshot)

    adapter._restore_navigation_profile = fake_restore

    try:
        adapter.apply_navigation_profile(
            generation=2,
            global_controller="theta_star",
            local_controller="mppi",
        )
        raise AssertionError("expected readback failure")
    except ProtocolError as exc:
        assert exc.code == "NAV_PROFILE_READBACK_FAILED"
    assert restored and restored[0]["generation"] == 1


def test_apply_navigation_profile_success_stores_last_good(monkeypatch):
    adapter = object.__new__(RosAdapter)
    adapter._last_good_navigation_profile = None
    adapter.set_global_controller = lambda mode: None
    adapter.set_safety_profile = lambda **kwargs: None
    adapter.set_waypoint_profile = lambda **kwargs: None
    adapter.set_local_controller = lambda mode: None
    adapter.set_smoother = lambda mode: None
    adapter.apply_outdoor_gps_profile = lambda **kwargs: None
    adapter._rtk_is_navigation_pose_source = lambda: False

    def fake_get(node, names, **kwargs):
        result = {}
        for name in names:
            if name.endswith("use_astar"):
                result[name] = False
            elif name == "PolygonStop.enabled":
                result[name] = True
            else:
                result[name] = True
        return result

    adapter._get_remote_parameters = fake_get
    applied = adapter.apply_navigation_profile(
        generation=3,
        global_controller="theta_star",
        local_controller="rpp",
        detour_enabled=True,
    )
    assert applied["generation"] == 3
    assert applied["global_plugin_id"] == "ThetaStar"
    assert applied["local_plugin_id"] == "RPP"
    assert adapter._last_good_navigation_profile["generation"] == 3
