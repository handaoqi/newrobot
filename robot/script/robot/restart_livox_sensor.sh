#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/robot/genisom_roamerx_open}"

kill_matching() {
  local pattern="$1"
  local pids
  pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
  if [[ -z "${pids}" ]]; then
    return
  fi
  kill ${pids} 2>/dev/null || true
  sleep 1
  pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
  if [[ -n "${pids}" ]]; then
    kill -9 ${pids} 2>/dev/null || true
  fi
}

kill_matching "/opt/ros/humble/bin/ros2 launch livox_driver lidar.launch.py"
kill_matching "/livox_driver/livox_driver_node"
sleep 1

"${PROJECT_DIR}/script/robot/ensure_navigation_sensors.sh"
echo "Livox LiDAR/IMU restarted and data verified."
