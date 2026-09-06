from __future__ import annotations

# Keep selector ids aligned with the plugin lists in navigo_params.yaml.
DEFAULT_LOCAL_CONTROLLER = "mppi"
DEFAULT_GLOBAL_CONTROLLER = "theta_star"

LOCAL_CONTROLLERS = frozenset({"mppi", "rpp"})
GLOBAL_CONTROLLERS = frozenset({"theta_star", "navfn"})

LOCAL_CONTROLLER_PLUGIN_IDS = {
    "mppi": "FollowPath",
    "rpp": "RPP",
}

GLOBAL_CONTROLLER_PLUGIN_IDS = {
    "theta_star": "ThetaStar",
    "navfn": "NavFn",
}


def normalize_local_controller(value: object | None) -> str:
    normalized = str(value or DEFAULT_LOCAL_CONTROLLER).strip().lower()
    return normalized if normalized in LOCAL_CONTROLLERS else DEFAULT_LOCAL_CONTROLLER


def normalize_global_controller(value: object | None) -> str:
    normalized = str(value or DEFAULT_GLOBAL_CONTROLLER).strip().lower()
    return normalized if normalized in GLOBAL_CONTROLLERS else DEFAULT_GLOBAL_CONTROLLER


def local_controller_plugin_id(value: object | None) -> str:
    return LOCAL_CONTROLLER_PLUGIN_IDS[normalize_local_controller(value)]


def global_controller_plugin_id(value: object | None) -> str:
    return GLOBAL_CONTROLLER_PLUGIN_IDS[normalize_global_controller(value)]
