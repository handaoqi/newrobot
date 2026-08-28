#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/dogrobot/robot}"
LOCK_FILE="${LIDAR_TF_LOCK_FILE:-/tmp/roamerx-lidar-static-tf.lock}"
LOG_FILE="${LIDAR_TF_LOG_FILE:-/tmp/lidar_tf.log}"
PROCESS_PATTERN='/static_transform_publisher .*base_link livox_frame'

# Several sensor entrypoints may run concurrently during login, mapping, or
# navigation startup. Serialize the check-and-start sequence so they cannot
# each create a publisher before the other process becomes visible to pgrep.
# The publisher is long-lived, so it must not inherit the lock descriptor.
if [ "${LIDAR_TF_LOCK_HELD:-0}" != "1" ]; then
  exec flock --exclusive --close "${LOCK_FILE}" \
    env LIDAR_TF_LOCK_HELD=1 "$0" "$@"
fi

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

if pgrep -f "${PROCESS_PATTERN}" >/dev/null 2>&1; then
  exit 0
fi

setsid ros2 run tf2_ros static_transform_publisher \
  0.382765605 -0.046855740 0.513445457 \
  0.007172121 -0.043589169 -0.009509268 0.998978536 \
  base_link livox_frame --ros-args -r __node:=lidar_extrinsics_tf \
  >"${LOG_FILE}" 2>&1 < /dev/null &

# Keep the lock until the executable is visible. Without this wait, a second
# caller can acquire the lock while `ros2 run` is still resolving the binary.
for _ in $(seq 1 50); do
  if pgrep -f "${PROCESS_PATTERN}" >/dev/null 2>&1; then
    exit 0
  fi
  sleep 0.1
done

echo "ERROR: LiDAR static TF publisher failed to start" >&2
tail -n 30 "${LOG_FILE}" >&2 2>/dev/null || true
exit 1
