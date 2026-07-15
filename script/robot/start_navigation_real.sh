#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/robot/genisom_roamerx_open}"
MAP_YAML="${MAP_YAML:-/home/robot/.jszr/map/map.yaml}"
PCD_MAP="${PCD_MAP:-}"
LOG_DIR="${LOG_DIR:-/tmp/roamerx_nav_logs}"
PLATFORM="${PLATFORM:-NX_XG3588}"
if [ "${PLATFORM}" = "linux" ]; then
  PLATFORM="NX_XG3588"
fi
MC_CONTROLLER_TYPE="${MC_CONTROLLER_TYPE:-RL_TRACK_VELOCITY}"
COMMUNICATION_TYPE="${COMMUNICATION_TYPE:-UDP}"
LOCALIZATION_WAIT_SECONDS="${LOCALIZATION_WAIT_SECONDS:-60}"
REQUIRE_RTK="${REQUIRE_RTK:-0}"
RTK_WAIT_SECONDS="${RTK_WAIT_SECONDS:-45}"

mkdir -p "${LOG_DIR}"

# The filtered PCD is used to generate the navigation occupancy map, but may
# remove sparse structural features that NDT needs.  Prefer the raw mapping PCD
# for localization when it belongs to the same active map; allow PCD_MAP to
# override this behavior for explicit operator experiments.
if [ -z "${PCD_MAP}" ]; then
  ACTIVE_PCD="$(readlink -f /home/robot/.jszr/map/map.pcd 2>/dev/null || true)"
  RAW_PCD="$(dirname "$(dirname "${ACTIVE_PCD}")")/map.raw_dynamic_unfiltered.pcd"
  if [ -f "${RAW_PCD}" ]; then
    PCD_MAP="${RAW_PCD}"
  else
    PCD_MAP="/home/robot/.jszr/map/map.pcd"
  fi
fi

/home/robot/genisom_roamerx_open/script/robot/wait_for_valid_time.sh

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

usage() {
  echo "Usage: $0 {start|stop|restart|status|load-map|full-stop}"
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

is_localization_running() {
  pgrep -f "ros2 launch localization localization.launch.py" >/dev/null 2>&1 || \
    pgrep -f "localization_node" >/dev/null 2>&1
}

is_navigation_running() {
  pgrep -f "ros2 launch robot_navigo navigation_bringup.launch.py" >/dev/null 2>&1 || \
    pgrep -f "component_container_isolated.*navigo_container" >/dev/null 2>&1
}

is_running() {
  is_localization_running && is_navigation_running
}

stop_navigation() {
  echo "Stopping Nav2/Navigo while preserving localization..."
  kill_pattern "ros2 launch robot_navigo navigation_bringup.launch.py"
  kill_pattern "component_container_isolated.*navigo_container"
  kill_pattern "vel_cmd_udp_pub"
  kill_pattern "vel_cmd_lcm_pub"
  kill_pattern "mode_status_pub"
  kill_pattern "odom_to_tf_broadcaster"
  kill_pattern "pointcloud_to_laserscan_node"
  echo "Navigation stopped. Localization is still running."
}

stop_stack() {
  stop_navigation
  echo "Stopping localization..."
  kill_pattern "ros2 launch localization localization.launch.py"
  kill_pattern "localization_node"
  kill_pattern "static_transform_publisher.*base_link livox_frame"
  echo "Full navigation stack stopped."
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

ensure_rtk() {
  echo "Starting RTK/NTRIP..."
  "${PROJECT_DIR}/script/robot/start_rtk_ntrip.sh" >/tmp/roamerx_rtk_start.log 2>&1 || {
    cat /tmp/roamerx_rtk_start.log >&2
    return 1
  }
  if [ "${REQUIRE_RTK}" != "1" ]; then
    return 0
  fi
  echo "Waiting for RTK fix on /fix..."
  for _ in $(seq 1 "${RTK_WAIT_SECONDS}"); do
    local status
    status="$(timeout 3 ros2 topic echo /fix --once 2>/dev/null | awk '/status:/{getline; if ($1=="status:") print $2; exit}' || true)"
    if [ "${status}" = "0" ] || [ "${status}" = "1" ] || [ "${status}" = "2" ]; then
      echo "RTK/GNSS fix OK."
      return 0
    fi
    sleep 1
  done
  echo "ERROR: RTK/GNSS did not report a valid fix within ${RTK_WAIT_SECONDS}s." >&2
  return 1
}

load_pcd_map() {
  echo "Loading localization PCD map: ${PCD_MAP}"
  timeout 25 ros2 service call /load_map_service robots_dog_msgs/srv/LoadMap \
    "{pcd_path: '${PCD_MAP}'}" | tee "${LOG_DIR}/load_map.last.log"
}

wait_for_localization() {
  echo "Waiting for localization status=3..."
  for _ in $(seq 1 "${LOCALIZATION_WAIT_SECONDS}"); do
    if ! is_localization_running; then
      echo "ERROR: localization process exited before reporting status=3." >&2
      return 1
    fi
    local status
    status="$("${PROJECT_DIR}/script/robot/read_localization_status.py" --timeout 3 2>/dev/null || true)"
    if [ "${status}" = "3" ]; then
      echo "Localization OK."
      return 0
    fi
    sleep 1
  done
  echo "ERROR: localization did not report status=3 within ${LOCALIZATION_WAIT_SECONDS}s." >&2
  return 1
}

localization_is_valid() {
  local status
  status="$("${PROJECT_DIR}/script/robot/read_localization_status.py" --timeout 3 2>/dev/null || true)"
  [ "${status}" = "3" ]
}

start_stack() {
  if is_running; then
    echo "Navigation and localization already appear to be running."
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

  ensure_rtk

  local localization_started=false
  if ! is_localization_running; then
    echo "Starting localization..."
    setsid bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' && exec ros2 launch localization localization.launch.py" \
      >"${LOG_DIR}/localization.log" 2>&1 < /dev/null &
    sleep 3
    localization_started=true
  fi

  if [ "${localization_started}" = "true" ]; then
    load_pcd_map
    if ! wait_for_localization; then
      echo "ERROR: refusing to start Nav2/Navigo without valid localization." >&2
      return 1
    fi
  elif localization_is_valid; then
    echo "Localization is already valid; preserving its current map and pose."
  else
    echo "ERROR: localization is running but not valid. Initialize or relocalize first; refusing to reset its pose." >&2
    return 1
  fi

  if is_navigation_running; then
    echo "Nav2/Navigo already appears to be running."
  else
    echo "Starting Nav2/Navigo..."
    setsid bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' && exec ros2 launch robot_navigo navigation_bringup.launch.py platform:='${PLATFORM}' mc_controller_type:='${MC_CONTROLLER_TYPE}' communication_type:='${COMMUNICATION_TYPE}' map:='${MAP_YAML}'" \
      >"${LOG_DIR}/navigation.log" 2>&1 < /dev/null &
  fi

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
  local status
  status="$("${PROJECT_DIR}/script/robot/read_localization_status.py" --timeout 3 2>/dev/null || true)"
  echo "status: ${status:-unavailable}"
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
    stop_navigation
    ;;
  restart)
    stop_navigation
    start_stack
    ;;
  full-stop)
    stop_stack
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
