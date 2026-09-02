from __future__ import annotations

# The deployed navigo controller server currently loads only FollowPath, whose
# implementation is MPPI. Keep this registry aligned with navigo_params.yaml;
# advertising an unregistered plugin makes the task fail after dispatch.
DEFAULT_LOCAL_CONTROLLER = "mppi"
DEFAULT_GLOBAL_CONTROLLER = "theta_star"

LOCAL_CONTROLLERS = frozenset({"mppi"})
GLOBAL_CONTROLLERS = frozenset({"theta_star", "navfn"})

LOCAL_CONTROLLER_PLUGIN_IDS = {
    "mppi": "FollowPath",
}

GLOBAL_CONTROLLER_PLUGIN_IDS = {
    "theta_star": "ThetaStar",
    "navfn": "GridBased",
}


def normalize_local_controller(value: object | None) -> str:
    normalized = str(value or DEFAULT_LOCAL_CONTROLLER).strip().lower()
    # RPP was accepted by older route payloads but is not a plugin in the
    # custom navigo controller server. Treat it as a legacy alias so old
    # routes remain executable with the registered MPPI controller.
    if normalized == "rpp":
        return DEFAULT_LOCAL_CONTROLLER
    return normalized if normalized in LOCAL_CONTROLLERS else DEFAULT_LOCAL_CONTROLLER


def normalize_global_controller(value: object | None) -> str:
    normalized = str(value or DEFAULT_GLOBAL_CONTROLLER).strip().lower()
    return normalized if normalized in GLOBAL_CONTROLLERS else DEFAULT_GLOBAL_CONTROLLER


def local_controller_plugin_id(value: object | None) -> str:
    return LOCAL_CONTROLLER_PLUGIN_IDS[normalize_local_controller(value)]


def global_controller_plugin_id(value: object | None) -> str:
    return GLOBAL_CONTROLLER_PLUGIN_IDS[normalize_global_controller(value)]
