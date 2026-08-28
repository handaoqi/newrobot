#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/dogrobot/robot}"
SCRIPT_DIR="${SCRIPT_DIR:-${PROJECT_DIR}/script/robot}"
# A MID-360 can need noticeably longer to resume UDP point output after a
# charging-standby cycle. Avoid treating a healthy cold start as a failure.
WAIT_SECONDS="${WAIT_SECONDS:-90}"

SENSOR_LOCK_FILE="${MAPPING_SENSOR_LOCK_FILE:-/tmp/roamerx-mapping-sensors.lock}"
if [ "${MAPPING_SENSOR_LOCK_HELD:-0}" != "1" ]; then
  # Sensor launchers run in the background.  Keep the lock in flock itself
  # and close its fd in this script so those long-lived children cannot retain
  # the lock after readiness checks finish.
  exec flock --exclusive --close "${SENSOR_LOCK_FILE}" \
    env MAPPING_SENSOR_LOCK_HELD=1 "$0" "$@"
fi

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

"${SCRIPT_DIR}/ensure_lidar_static_tf.sh"

wait_for_message() {
  local topic="$1"
  local label="$2"
  local deadline=$((SECONDS + WAIT_SECONDS))
  while (( SECONDS < deadline )); do
    # A fresh Zenoh ROS 2 subscriber needs several seconds for discovery;
    # a two-second probe repeatedly timed out before it could receive data.
    if timeout 6 ros2 topic echo "${topic}" --once \
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
  if ! "${SCRIPT_DIR}/start_rtk_ntrip.sh" >/tmp/roamerx_rtk_start.log 2>&1; then
    echo "WARNING: RTK/GNSS startup failed; mapping will use LiDAR+IMU only" >&2
    tail -n 20 /tmp/roamerx_rtk_start.log >&2 2>/dev/null || true
  fi
fi
