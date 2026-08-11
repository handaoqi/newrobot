#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/robot/genisom_roamerx_open}"
WAIT_SECONDS="${WAIT_SECONDS:-12}"

set +u
source /opt/ros/humble/setup.bash
source /opt/robot-driver/install/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

if ! pgrep -x rmw_zenohd >/dev/null 2>&1; then
  echo "ERROR: rmw_zenohd is not running" >&2
  exit 1
fi

if ! pgrep -f 'livox_driver_node' >/dev/null 2>&1; then
  echo "Starting Livox LiDAR/IMU driver..."
  setsid ros2 launch livox_driver lidar.launch.py >/tmp/livox.log 2>&1 < /dev/null &
fi

if ! pgrep -f '/static_transform_publisher .*base_link livox_frame' >/dev/null 2>&1; then
  setsid ros2 run tf2_ros static_transform_publisher \
    0.382765605 -0.046855740 0.513445457 \
    0.007172121 -0.043589169 -0.009509268 0.998978536 \
    base_link livox_frame >/tmp/lidar_tf.log 2>&1 < /dev/null &
fi

wait_for_message() {
  local topic="$1"
  local label="$2"
  local deadline=$((SECONDS + WAIT_SECONDS))
  while (( SECONDS < deadline )); do
    if timeout 2 ros2 topic echo "${topic}" --once \
        --qos-reliability best_effort >/dev/null 2>&1; then
      echo "${label} data OK: ${topic}"
      return 0
    fi
    sleep 0.5
  done
  echo "ERROR: ${label} has no fresh data on ${topic} within ${WAIT_SECONDS}s" >&2
  tail -n 30 /tmp/livox.log >&2 2>/dev/null || true
  return 1
}

wait_for_message /front_lidar LiDAR
wait_for_message /front_lidar/imu IMU

# Mapping can continue on LiDAR+IMU when GNSS quality is poor, but the RTK
# bridge must be running so every keyframe can record coordinates and quality.
if ! pgrep -f 'rtk_ntrip_bridge.py' >/dev/null 2>&1; then
  echo "Starting optional RTK/GNSS recording chain..."
  if ! "${PROJECT_DIR}/script/robot/start_rtk_ntrip.sh" >/tmp/roamerx_rtk_start.log 2>&1; then
    echo "WARNING: RTK/GNSS startup failed; mapping will use LiDAR+IMU only" >&2
    tail -n 20 /tmp/roamerx_rtk_start.log >&2 2>/dev/null || true
  fi
fi
