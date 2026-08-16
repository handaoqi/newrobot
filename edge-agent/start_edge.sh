#!/bin/bash
set -eo pipefail

/home/robot/genisom_roamerx_open/script/robot/wait_for_valid_time.sh

source /opt/ros/humble/setup.bash
source /home/robot/genisom_roamerx_open/install/setup.bash
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
    if ! /home/robot/genisom_roamerx_open/script/robot/ensure_navigation_sensors.sh; then
        echo "[start_edge] sensor startup incomplete; Edge will stay online for remote recovery" >&2
    fi
fi

export PYTHONPATH=/home/robot/genisom_roamerx_open/edge_agent:$PYTHONPATH
cd /home/robot/genisom_roamerx_open/edge_agent
exec python3 run_edge_agent.py --config /home/robot/edge_agent/config.yaml
