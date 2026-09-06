from roamerx_edge.navigation_controllers import (
    global_controller_plugin_id,
    local_controller_plugin_id,
    normalize_global_controller,
    normalize_local_controller,
)


def test_normalize_local_controller_defaults_to_registered_mppi():
    assert normalize_local_controller(None) == "mppi"
    assert normalize_local_controller("mppi") == "mppi"
    assert normalize_local_controller("rpp") == "rpp"
    assert normalize_local_controller("unknown") == "mppi"


def test_normalize_global_controller_defaults_to_theta_star():
    assert normalize_global_controller(None) == "theta_star"
    assert normalize_global_controller("navfn") == "navfn"
    assert normalize_global_controller("invalid") == "theta_star"


def test_controller_plugin_ids():
    assert local_controller_plugin_id("rpp") == "RPP"
    assert local_controller_plugin_id("mppi") == "FollowPath"
    assert global_controller_plugin_id("theta_star") == "ThetaStar"
    assert global_controller_plugin_id("navfn") == "NavFn"
