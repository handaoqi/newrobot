#!/bin/bash
set -eo pipefail

EDGE_SOURCE_DIR="${EDGE_SOURCE_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
PROJECT_DIR="${PROJECT_DIR:-$(cd "${EDGE_SOURCE_DIR}/../robot" && pwd)}"
SCRIPT_DIR="${SCRIPT_DIR:-${PROJECT_DIR}/script/robot}"
EDGE_RUNTIME_DIR="${EDGE_RUNTIME_DIR:-/home/dogrobot/runtime/nx-edge}"
EDGE_DATA_DIR="${EDGE_DATA_DIR:-${EDGE_RUNTIME_DIR}/data/edge-agent}"
EDGE_CONFIG="${EDGE_CONFIG:-${EDGE_RUNTIME_DIR}/conf/edge-agent.yaml}"

bash "${SCRIPT_DIR}/wait_for_valid_time.sh"

source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

for _ in $(seq 1 20); do
    if pgrep -x rmw_zenohd > /dev/null 2>&1; then
        break
    fi
    sleep 0.25
done
if ! pgrep -x rmw_zenohd > /dev/null 2>&1; then
    echo "[start_edge] rmw_zenohd is unavailable" >&2
    exit 1
fi

COOLING_MARKER="${EDGE_DATA_DIR}/cooling_standby"
if [[ -f "$COOLING_MARKER" ]]; then
    echo "[start_edge] cooling standby active; sensor startup skipped"
else
    if ! bash "${SCRIPT_DIR}/ensure_navigation_sensors.sh"; then
        echo "[start_edge] sensor startup incomplete; Edge will stay online for remote recovery" >&2
    fi
fi

export PYTHONPATH="${EDGE_SOURCE_DIR}:${PYTHONPATH:-}"
cd "${EDGE_SOURCE_DIR}"
exec python3 "${EDGE_SOURCE_DIR}/run_edge_agent.py" --config "${EDGE_CONFIG}"
