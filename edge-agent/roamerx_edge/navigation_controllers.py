from __future__ import annotations

# Keep selector ids aligned with the plugin lists in navigo_params.yaml.
DEFAULT_LOCAL_CONTROLLER = "mppi"
DEFAULT_GLOBAL_CONTROLLER = "theta_star"

LOCAL_CONTROLLERS = frozenset({"mppi", "rpp", "ilqr"})
GLOBAL_CONTROLLERS = frozenset({"theta_star", "navfn", "smac_hybrid"})

LOCAL_CONTROLLER_PLUGIN_IDS = {
    "mppi": "FollowPath",
    "rpp": "RPP",
    "ilqr": "ILQR",
}

GLOBAL_CONTROLLER_PLUGIN_IDS = {
    "theta_star": "ThetaStar",
    "navfn": "NavFn",
    "smac_hybrid": "SmacHybrid",
}

# Page/API combinations that this robot build statically registers.
SUPPORTED_NAVIGATION_COMBOS = tuple(
    (global_name, local_name)
    for global_name in sorted(GLOBAL_CONTROLLERS)
    for local_name in sorted(LOCAL_CONTROLLERS)
)


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


def navigation_capabilities() -> dict:
    """Static capability report for the plugins registered in navigo_params."""
    return {
        "global_controllers": sorted(GLOBAL_CONTROLLERS),
        "local_controllers": sorted(LOCAL_CONTROLLERS),
        "global_plugin_ids": dict(GLOBAL_CONTROLLER_PLUGIN_IDS),
        "local_plugin_ids": dict(LOCAL_CONTROLLER_PLUGIN_IDS),
        "supported_combos": [
            {"global_controller": global_name, "local_controller": local_name}
            for global_name, local_name in SUPPORTED_NAVIGATION_COMBOS
        ],
    }
