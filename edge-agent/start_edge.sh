#!/bin/bash
set -eo pipefail

EDGE_DIR="${ROAMERX_EDGE_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
PROJECT_DIR="${ROAMERX_ROBOT_PROJECT_DIR:-/home/robot/genisom_roamerx_open}"
EDGE_CONFIG="${ROAMERX_EDGE_CONFIG:-/home/robot/edge_agent/config.yaml}"

"$PROJECT_DIR/script/robot/wait_for_valid_time.sh"

source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"
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

COOLING_MARKER=/home/robot/edge_agent/data/cooling_standby
if [[ -f "$COOLING_MARKER" ]]; then
    echo "[start_edge] cooling standby active; sensor startup skipped"
else
    if ! "$PROJECT_DIR/script/robot/ensure_navigation_sensors.sh"; then
        echo "[start_edge] sensor startup incomplete; Edge will stay online for remote recovery" >&2
    fi
fi

export PYTHONPATH="$EDGE_DIR${PYTHONPATH:+:$PYTHONPATH}"
cd "$EDGE_DIR"
exec python3 run_edge_agent.py --config "$EDGE_CONFIG"
