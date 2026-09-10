#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/dogrobot/robot}"
SCRIPT_DIR="${SCRIPT_DIR:-${PROJECT_DIR}/script/robot}"

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

# Stop the child first so its ROS launch parent can reap it.  Killing the
# launch process first can leave livox_driver_node as a transient zombie;
# ensure_mapping_sensors.sh would then mistake that zombie for a live driver.
# Anchor both expressions to argv[0] so `pgrep -f` cannot match the caller just
# because its command line contains one of these commands.
kill_matching '^/opt/robot-driver/install/livox_driver/lib/livox_driver/livox_driver_node($| )'
kill_matching '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch livox_driver lidar\.launch\.py($| )'
sleep 1

"${SCRIPT_DIR}/ensure_navigation_sensors.sh"
echo "Livox LiDAR/IMU restarted and data verified."
