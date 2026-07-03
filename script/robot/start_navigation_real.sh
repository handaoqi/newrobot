#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/robot/genisom_roamerx_open}"
MAP_YAML="${MAP_YAML:-${PROJECT_DIR}/map/map.yaml}"
PCD_MAP="${PCD_MAP:-/home/robot/.jszr/map/map.pcd}"
LOG_DIR="${LOG_DIR:-/tmp/roamerx_nav_logs}"
PLATFORM="${PLATFORM:-NX_XG3588}"
MC_CONTROLLER_TYPE="${MC_CONTROLLER_TYPE:-RL_TRACK_VELOCITY}"
COMMUNICATION_TYPE="${COMMUNICATION_TYPE:-UDP}"

mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u

usage() {
  echo "Usage: $0 {start|stop|restart|status|load-map}"
  echo
  echo "Env:"
  echo "  PROJECT_DIR=${PROJECT_DIR}"
  echo "  MAP_YAML=${MAP_YAML}"
  echo "  PCD_MAP=${PCD_MAP}"
  echo "  LOG_DIR=${LOG_DIR}"
  echo "  PLATFORM=${PLATFORM}"
  echo "  COMMUNICATION_TYPE=${COMMUNICATION_TYPE}"
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

is_running() {
  pgrep -f "ros2 launch localization localization.launch.py" >/dev/null 2>&1 || \
    pgrep -f "ros2 launch robot_navigo navigation_bringup.launch.py" >/dev/null 2>&1
}

stop_stack() {
  echo "Stopping RoamerX navigation stack..."
  kill_pattern "ros2 launch robot_navigo navigation_bringup.launch.py"
  kill_pattern "component_container_isolated.*navigo_container"
  kill_pattern "vel_cmd_udp_pub"
  kill_pattern "vel_cmd_lcm_pub"
  kill_pattern "mode_status_pub"
  kill_pattern "odom_to_tf_broadcaster"
  kill_pattern "ros2 launch localization localization.launch.py"
  kill_pattern "localization_node"
  kill_pattern "static_transform_publisher.*base_link livox_frame"
  echo "Stopped."
}

wait_for_node() {
  local node="$1"
  local timeout_s="$2"
  for _ in $(seq 1 "${timeout_s}"); do
    if ros2 node list 2>/dev/null | grep -qx "${node}"; then
      return 0
    fi
    sleep 1
  done
  echo "ERROR: timed out waiting for node ${node}" >&2
  return 1
}

load_pcd_map() {
  echo "Loading localization PCD map: ${PCD_MAP}"
  wait_for_node "/localization" 20
  ros2 service call /load_map_service robots_dog_msgs/srv/LoadMap \
    "{pcd_path: '${PCD_MAP}'}" | tee "${LOG_DIR}/load_map.last.log"
}

wait_for_localization() {
  echo "Waiting for localization status=3..."
  for _ in $(seq 1 30); do
    local status
    status="$(timeout 3 ros2 topic echo /localization_info --once 2>/dev/null | awk '/status:/{print $2; exit}' || true)"
    if [ "${status}" = "3" ]; then
      echo "Localization OK."
      return 0
    fi
    sleep 1
  done
  echo "WARN: localization did not report status=3 within timeout." >&2
  return 0
}

start_stack() {
  if is_running; then
    echo "Navigation/localization already appears to be running."
    echo "Use '$0 restart' to stop and start again."
    return 0
  fi

  if [ ! -f "${MAP_YAML}" ]; then
    echo "ERROR: map yaml not found: ${MAP_YAML}" >&2
    exit 1
  fi
  if [ ! -f "${PCD_MAP}" ]; then
    echo "ERROR: PCD map not found: ${PCD_MAP}" >&2
    exit 1
  fi

  echo "Starting localization..."
  nohup bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && exec ros2 launch localization localization.launch.py" \
    >"${LOG_DIR}/localization.log" 2>&1 &

  sleep 3
  load_pcd_map
  wait_for_localization

  echo "Starting Nav2/Navigo..."
  nohup bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && exec ros2 launch robot_navigo navigation_bringup.launch.py platform:='${PLATFORM}' mc_controller_type:='${MC_CONTROLLER_TYPE}' communication_type:='${COMMUNICATION_TYPE}' map:='${MAP_YAML}'" \
    >"${LOG_DIR}/navigation.log" 2>&1 &

  echo
  echo "Navigation stack started."
  echo "Logs:"
  echo "  ${LOG_DIR}/localization.log"
  echo "  ${LOG_DIR}/navigation.log"
}

status_stack() {
  echo "Processes:"
  pgrep -af "ros2 launch localization localization.launch.py|ros2 launch robot_navigo navigation_bringup.launch.py|localization_node|navigo_container|vel_cmd_udp_pub|mode_status_pub" || true
  echo
  echo "Lifecycle:"
  ros2 lifecycle get /planner_server 2>/dev/null || true
  ros2 lifecycle get /controller_server 2>/dev/null || true
  ros2 lifecycle get /bt_navigator 2>/dev/null || true
  echo
  echo "Localization:"
  timeout 3 ros2 topic echo /localization_info --once 2>/dev/null | awk '/status:|  x:|  y:|  z:|speed:/{print}' || true
  echo
  echo "cmd_vel:"
  ros2 topic info /cmd_vel -v 2>/dev/null | sed -n '1,80p' || true
}

MODE="${1:-start}"
case "${MODE}" in
  start)
    start_stack
    ;;
  stop)
    stop_stack
    ;;
  restart)
    stop_stack
    start_stack
    ;;
  status)
    status_stack
    ;;
  load-map)
    load_pcd_map
    wait_for_localization
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage
    exit 1
    ;;
esac
