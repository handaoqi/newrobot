#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/robot/genisom_roamerx_open}"
LOG_DIR="${LOG_DIR:-/tmp/roamerx_nav_logs}"
PLATFORM="${PLATFORM:-NX_XG3588}"
if [ "${PLATFORM}" = "linux" ]; then
  PLATFORM="NX_XG3588"
fi

mkdir -p "${LOG_DIR}"

/home/robot/genisom_roamerx_open/script/robot/wait_for_valid_time.sh

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

usage() {
  echo "Usage: $0 {start|stop|restart|status}"
}

is_bridge_running() {
  pgrep -f "vel_cmd_udp_pub" >/dev/null 2>&1
}

is_bridge_ros_ready() {
  ros2 topic info /teleop_action -v 2>/dev/null | grep -q "Node name: vel_cmd_udp_publisher" && \
    ros2 topic info /cmd_vel -v 2>/dev/null | grep -q "Node name: vel_cmd_udp_publisher"
}

kill_pattern() {
  local pattern="$1"
  local pids
  pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
  if [ -z "${pids}" ]; then
    return 0
  fi
  echo "[stop] ${pattern}: ${pids}"
  kill ${pids} 2>/dev/null || true
  sleep 1
  pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
  if [ -n "${pids}" ]; then
    kill -9 ${pids} 2>/dev/null || true
  fi
}

start_bridge() {
  if is_bridge_running; then
    if is_bridge_ros_ready; then
      echo "Teleop control bridge already running."
      status_bridge
      return 0
    fi
    echo "Teleop control bridge process exists but ROS graph is not ready; restarting it..."
    kill_pattern "vel_cmd_udp_pub"
  fi

  echo "Starting teleop control bridge without localization/Nav2..."
  setsid bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' && exec ros2 run robot_navigo vel_cmd_udp_pub --ros-args -p platform:='${PLATFORM}'" \
    >"${LOG_DIR}/teleop_control.log" 2>&1 < /dev/null &

  for _ in $(seq 1 12); do
    if is_bridge_running && is_bridge_ros_ready; then
      echo "Teleop control bridge started."
      status_bridge
      return 0
    fi
    sleep 1
  done

  echo "ERROR: teleop control bridge did not start." >&2
  tail -n 80 "${LOG_DIR}/teleop_control.log" >&2 || true
  return 1
}

stop_bridge() {
  echo "Stopping teleop control bridge..."
  kill_pattern "vel_cmd_udp_pub"
  echo "Stopped."
}

status_bridge() {
  echo "Processes:"
  pgrep -af "vel_cmd_udp_pub" || true
  echo
  echo "Topics:"
  ros2 topic info /cmd_vel -v 2>/dev/null | sed -n '1,80p' || true
  ros2 topic info /teleop_action -v 2>/dev/null | sed -n '1,80p' || true
}

MODE="${1:-start}"
case "${MODE}" in
  start)
    start_bridge
    ;;
  stop)
    stop_bridge
    ;;
  restart)
    stop_bridge
    start_bridge
    ;;
  status)
    status_bridge
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage
    exit 1
    ;;
esac
