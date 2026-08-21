#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/dogrobot/robot}"
SCRIPT_DIR="${SCRIPT_DIR:-${PROJECT_DIR}/script/robot}"
CONFIG="${SCRIPT_DIR}/official_ukf_shadow.yaml"
LOG_DIR="/tmp/roamerx_official_ukf"
mkdir -p "${LOG_DIR}"
set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
PUBLISH_TF="${PUBLISH_TF:-false}"

case "${1:-start}" in
  start)
    if ! pgrep -f "[p]ython3 ${SCRIPT_DIR}/odom_stamp_sanitizer.py" >/dev/null; then
      setsid python3 "${SCRIPT_DIR}/odom_stamp_sanitizer.py" >"${LOG_DIR}/sanitizer.log" 2>&1 < /dev/null &
    fi
    if ! pgrep -f "[p]ython3 ${SCRIPT_DIR}/ndt_odom_sanitizer.py" >/dev/null; then
      setsid python3 "${SCRIPT_DIR}/ndt_odom_sanitizer.py" >"${LOG_DIR}/ndt_sanitizer.log" 2>&1 < /dev/null &
    fi
    if ! pgrep -f "[p]ython3 ${SCRIPT_DIR}/imu_covariance_sanitizer.py" >/dev/null; then
      setsid python3 "${SCRIPT_DIR}/imu_covariance_sanitizer.py" >"${LOG_DIR}/imu_sanitizer.log" 2>&1 < /dev/null &
    fi
    if ! pgrep -f '[u]kf_node --ros-args.*__node:=local_ukf' >/dev/null; then
      setsid ros2 run robot_localization ukf_node --ros-args -r __node:=local_ukf -r odometry/filtered:=/odometry/official_ukf_local -r diagnostics:=/diagnostics/official_ukf_local --params-file "${CONFIG}" -p publish_tf:="${PUBLISH_TF}" >"${LOG_DIR}/local_ukf.log" 2>&1 < /dev/null &
    fi
    if ! pgrep -f '[u]kf_node --ros-args.*__node:=global_ukf' >/dev/null; then
      setsid ros2 run robot_localization ukf_node --ros-args -r __node:=global_ukf -r odometry/filtered:=/odometry/official_ukf_global -r diagnostics:=/diagnostics/official_ukf_global --params-file "${CONFIG}" -p publish_tf:="${PUBLISH_TF}" >"${LOG_DIR}/global_ukf.log" 2>&1 < /dev/null &
    fi
    ;;
  stop)
    pkill -f "[p]ython3 ${SCRIPT_DIR}/odom_stamp_sanitizer.py" || true
    pkill -f "[p]ython3 ${SCRIPT_DIR}/ndt_odom_sanitizer.py" || true
    pkill -f "[p]ython3 ${SCRIPT_DIR}/imu_covariance_sanitizer.py" || true
    pkill -f '[u]kf_node --ros-args.*__node:=local_ukf' || true
    pkill -f '[u]kf_node --ros-args.*__node:=global_ukf' || true
    ;;
  status)
    pgrep -af 'odom_stamp_sanitizer.py|ukf_node.*(local_ukf|global_ukf)' || true
    ros2 topic hz /odometry/official_ukf_local 2>/dev/null || true
    ;;
  *) echo "Usage: $0 {start|stop|status}" >&2; exit 2 ;;
esac
