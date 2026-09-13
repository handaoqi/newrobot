#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/dogrobot/robot}"
SCRIPT_DIR="${SCRIPT_DIR:-${PROJECT_DIR}/script/robot}"
RUNTIME_DATA_ROOT="${ROAMERX_DATA_ROOT:-/home/dogrobot/runtime/nx-edge/data/jszr}"
MAP_YAML="${MAP_YAML:-}"
if [ -z "${MAP_YAML}" ]; then
  # New maps already write map.yaml with traversable-terrain semantics.  For
  # legacy maps, use an explicitly generated sidecar grid when present so the
  # original map remains available for immediate rollback.
  if [ -f "${RUNTIME_DATA_ROOT}/map/map_traversable.yaml" ]; then
    MAP_YAML="${RUNTIME_DATA_ROOT}/map/map_traversable.yaml"
  else
    MAP_YAML="${RUNTIME_DATA_ROOT}/map/map.yaml"
  fi
fi
PCD_MAP="${PCD_MAP:-}"
LOG_DIR="${LOG_DIR:-/tmp/roamerx_nav_logs}"
PLATFORM="${PLATFORM:-NX_XG3588}"
if [ "${PLATFORM}" = "linux" ]; then
  PLATFORM="NX_XG3588"
fi
MC_CONTROLLER_TYPE="${MC_CONTROLLER_TYPE:-RL_TRACK_VELOCITY}"
COMMUNICATION_TYPE="${COMMUNICATION_TYPE:-UDP}"
USE_OFFICIAL_UKF="${USE_OFFICIAL_UKF:-false}"
LOCALIZATION_WAIT_SECONDS="${LOCALIZATION_WAIT_SECONDS:-60}"
LOCALIZATION_START_WAIT_SECONDS="${LOCALIZATION_START_WAIT_SECONDS:-30}"
MAP_LOAD_TIMEOUT_SECONDS="${MAP_LOAD_TIMEOUT_SECONDS:-25}"
REQUIRE_RTK="${REQUIRE_RTK:-0}"
RTK_WAIT_SECONDS="${RTK_WAIT_SECONDS:-45}"

mkdir -p "${LOG_DIR}"
# Child Nav2 components inherit this limit.  The systemd unit also sets
# LimitCORE=infinity; keep the explicit shell setting for manual launches.
ulimit -c unlimited 2>/dev/null || true
CORE_DIR="${ROAMERX_CORE_DIR:-/tmp/roamerx-core}"
mkdir -p "${CORE_DIR}"
chmod 700 "${CORE_DIR}" 2>/dev/null || true

# The filtered PCD is used to generate the navigation occupancy map, but may
# remove sparse structural features that NDT needs.  Prefer the raw mapping PCD
# for localization when it belongs to the same active map; allow PCD_MAP to
# override this behavior for explicit operator experiments.
if [ -z "${PCD_MAP}" ]; then
  ACTIVE_PCD="$(readlink -f "${RUNTIME_DATA_ROOT}/map/map.pcd" 2>/dev/null || true)"
  RAW_PCD="$(dirname "$(dirname "${ACTIVE_PCD}")")/map.raw_dynamic_unfiltered.pcd"
  if [ -f "${RAW_PCD}" ]; then
    PCD_MAP="${RAW_PCD}"
  else
    PCD_MAP="${RUNTIME_DATA_ROOT}/map/map.pcd"
  fi
fi

# Select the FAST-LIO range profile from the activated map metadata. Legacy
# maps without a manifest use the conservative indoor profile.
NAVIGATION_SCENE_SCOPE="${ROAMERX_NAVIGATION_SCENE_SCOPE:-indoor}"
ACTIVE_MAP_DIR="$(dirname "$(readlink -f "${PCD_MAP}" 2>/dev/null || printf '%s' "${PCD_MAP}")")"
ACTIVE_MAP_MANIFEST="${ACTIVE_MAP_DIR}/map_manifest.json"
if [ -f "${ACTIVE_MAP_MANIFEST}" ]; then
  NAVIGATION_SCENE_SCOPE="$(python3 -c 'import json,sys; print((json.load(open(sys.argv[1])).get("scene_scope") or "indoor").lower())' "${ACTIVE_MAP_MANIFEST}" 2>/dev/null || printf 'indoor')"
fi
case "${NAVIGATION_SCENE_SCOPE}" in
  indoor|transition|outdoor) ;;
  *) NAVIGATION_SCENE_SCOPE="indoor" ;;
esac
export ROAMERX_NAVIGATION_SCENE_SCOPE="${NAVIGATION_SCENE_SCOPE}"

bash "${SCRIPT_DIR}/wait_for_valid_time.sh"

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

usage() {
  echo "Usage: $0 {start|stop|restart|restart-localization|ensure-localization-odom|stop-localization|status|load-map|full-stop}"
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

is_localization_node_alive() {
  pgrep -f "/localization/lib/localization/localization_node" >/dev/null 2>&1
}

is_localization_running() {
  is_localization_node_alive
}

is_navigation_running() {
  pgrep -f "ros2 launch robot_navigo navigation_bringup.launch.py" >/dev/null 2>&1 || \
    pgrep -f "component_container_isolated.*navigo_container" >/dev/null 2>&1
}

is_rtk_running() {
  pgrep -f "rtk_ntrip_bridge.py" >/dev/null 2>&1 && \
    pgrep -f "sixents_gps_driver" >/dev/null 2>&1
}

is_running() {
  is_localization_running && is_navigation_running
}

stop_navigation() {
  echo "Stopping Nav2/Navigo while preserving localization..."
  kill_pattern "ros2 bag record.*roamerx_nav_logs/diagnostics"
  kill_pattern "ros2 launch robot_navigo navigation_bringup.launch.py"
  kill_pattern "component_container_isolated.*navigo_container"
  kill_pattern "vel_cmd_lcm_pub"
  kill_pattern "mode_status_pub"
  kill_pattern "odom_to_tf_broadcaster"
  echo "Navigation stopped. Localization is still running."
}

stop_stack() {
  stop_navigation
  stop_localization
  kill_pattern "static_transform_publisher.*base_link livox_frame"
  kill_pattern "pointcloud_to_laserscan_node"
  kill_pattern "self_filter_scan.py"
  kill_pattern "sensor_health_monitor"
  echo "Full navigation stack stopped."
}

stop_localization() {
  echo "Stopping localization..."
  kill_pattern "ros2 launch localization localization.launch.py"
  kill_pattern "ros2 launch robot_slam lio_odometry.launch.py"
  kill_pattern "localization_node"
  kill_pattern "__node:=lio_odometry"
}

wait_for_localization_process() {
  local deadline=$((SECONDS + LOCALIZATION_START_WAIT_SECONDS))
  while (( SECONDS < deadline )); do
    # Check the real executable instead of ros2cli's cached graph. A stale
    # /localization graph entry can outlive its process and race map loading.
    if is_localization_node_alive; then
      echo "Localization process is running."
      return 0
    fi
    sleep 1
  done
  echo "ERROR: localization process did not start within ${LOCALIZATION_START_WAIT_SECONDS}s." >&2
  return 1
}

ensure_rtk() {
  echo "Starting RTK/NTRIP..."
  "${SCRIPT_DIR}/start_rtk_ntrip.sh" >/tmp/roamerx_rtk_start.log 2>&1 || {
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
  "${SCRIPT_DIR}/load_localization_map.py" \
    "${PCD_MAP}" --timeout "${MAP_LOAD_TIMEOUT_SECONDS}" 2>&1 \
    | tee "${LOG_DIR}/load_map.last.log"
}

wait_for_localization() {
  echo "Waiting for localization status=3..."
  local deadline=$((SECONDS + LOCALIZATION_WAIT_SECONDS))
  while (( SECONDS < deadline )); do
    if ! is_localization_running; then
      echo "ERROR: localization process exited before reporting status=3." >&2
      return 1
    fi
    local status
    status="$("${SCRIPT_DIR}/read_localization_status.py" --timeout 1 2>/dev/null || true)"
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
  status="$("${SCRIPT_DIR}/read_localization_status.py" --timeout 3 2>/dev/null || true)"
  [ "${status}" = "3" ]
}

localization_has_fresh_lio() {
  # A FAST-LIO process may remain alive after entering SAFE_HOLD. Require a
  # real odometry sample; a publisher entry in the ROS graph is insufficient.
  timeout 3 ros2 topic echo /odom/lio_odom --once >/dev/null 2>&1
}

restart_localization_only() {
  # Used by the edge task-recovery path. Nav2 remains alive but has no active
  # goal (the task executor cancelled it before invoking this action). Do not
  # wait for status=3 here: the caller sends the last trusted initial pose only
  # after this node and its map service are ready.
  if [ ! -f "${PCD_MAP}" ]; then
    echo "ERROR: PCD map not found: ${PCD_MAP}" >&2
    return 1
  fi
  stop_localization
  echo "Restarting localization only..."
  setsid bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' && exec ros2 launch localization localization.launch.py" \
    >"${LOG_DIR}/localization.log" 2>&1 < /dev/null &
  if ! wait_for_localization_process; then
    echo "ERROR: localization node did not start." >&2
    return 1
  fi
  load_pcd_map
  echo "Localization restarted; waiting for trusted-pose recovery."
}

recover_invalid_localization() {
  if localization_has_fresh_lio; then
    echo "FAST-LIO is publishing; reloading the active PCD map."
    load_pcd_map
  else
    echo "FAST-LIO has no fresh odometry; restarting localization and LIO."
    restart_localization_only
  fi
  wait_for_localization
}

ensure_localization_odom() {
  if ! is_localization_running; then
    echo "Starting localization odometry for mapping preparation..."
    setsid bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' && exec ros2 launch localization localization.launch.py" \
      >"${LOG_DIR}/localization.log" 2>&1 < /dev/null &
    if ! wait_for_localization_process; then
      echo "ERROR: localization node did not start for mapping preparation." >&2
      return 1
    fi
  fi

  echo "Waiting for /odom/localization_odom..."
  for _ in $(seq 1 15); do
    if timeout 3 ros2 topic echo /odom/localization_odom --once >/dev/null 2>&1; then
      echo "Localization odometry OK: /odom/localization_odom"
      return 0
    fi
    sleep 1
  done
  echo "ERROR: localization is running but /odom/localization_odom has no data." >&2
  return 1
}

start_stack() {
  WAIT_SECONDS="${NAV_SENSOR_WAIT_SECONDS:-25}" "${SCRIPT_DIR}/ensure_navigation_sensors.sh"
  if ! is_rtk_running; then
    ensure_rtk
  fi
  if is_running; then
    if localization_is_valid; then
      echo "RTK, navigation sensors, localization, and Nav2 already appear to be running."
      echo "Use '$0 restart' to stop and start again."
      return 0
    fi
    echo "Nav2 is running but localization is not status=3; starting localization recovery."
    if recover_invalid_localization; then
      return 0
    fi
    echo "ERROR: localization is running but not valid. Initialize or relocalize first." >&2
    return 1
  fi

  if [ ! -f "${MAP_YAML}" ]; then
    echo "ERROR: map yaml not found: ${MAP_YAML}" >&2
    exit 1
  fi
  if [ ! -f "${PCD_MAP}" ]; then
    echo "ERROR: PCD map not found: ${PCD_MAP}" >&2
    exit 1
  fi

  local localization_started=false
  if ! is_localization_node_alive; then
    if pgrep -f "ros2 launch localization localization.launch.py" >/dev/null 2>&1; then
      echo "Localization node is dead; clearing leftover launch and restarting."
      stop_localization
    fi
    echo "Starting localization..."
    setsid bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' && exec ros2 launch localization localization.launch.py" \
      >"${LOG_DIR}/localization.log" 2>&1 < /dev/null &
    if ! wait_for_localization_process; then
      echo "ERROR: localization node did not start." >&2
      return 1
    fi
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
    echo "Localization is running but not yet status=3; starting localization recovery."
    if ! recover_invalid_localization; then
      echo "ERROR: localization is running but not valid. Initialize or relocalize first; refusing to reset its pose." >&2
      return 1
    fi
  fi

  if is_navigation_running; then
    echo "Nav2/Navigo already appears to be running."
  else
    # The systemd-managed remote bridge is persistent. navigation_bringup
    # intentionally does not create a second UDP velocity bridge.
    echo "Starting Nav2/Navigo..."
    setsid bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' && exec ros2 launch robot_navigo navigation_bringup.launch.py platform:='${PLATFORM}' mc_controller_type:='${MC_CONTROLLER_TYPE}' communication_type:='${COMMUNICATION_TYPE}' use_official_ukf:='${USE_OFFICIAL_UKF}' map:='${MAP_YAML}'" \
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
  local planner controller bt waypoint
  planner="$(ros2 lifecycle get /planner_server 2>/dev/null || true)"
  controller="$(ros2 lifecycle get /controller_server 2>/dev/null || true)"
  bt="$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
  waypoint="$(ros2 lifecycle get /waypoint_follower 2>/dev/null || true)"
  printf '%s\n' "${planner}" "${controller}" "${bt}" "${waypoint}"
  echo
  echo "Localization:"
  local status
  status="$("${SCRIPT_DIR}/read_localization_status.py" --timeout 3 2>/dev/null || true)"
  echo "status: ${status:-unavailable}"
  echo
  echo "cmd_vel:"
  ros2 topic info /cmd_vel -v 2>/dev/null | sed -n '1,20p' || true
  echo
  # Repeat compact tokens at the end. Edge truncates status stdout to the last
  # 6000 characters, and the verbose /cmd_vel dump would otherwise hide them.
  echo "Ready-check:"
  if echo "${planner}" | grep -q 'active \[3\]'; then
    echo "/planner_server"
    echo "active [3]"
  fi
  if echo "${controller}" | grep -q 'active \[3\]'; then
    echo "/controller_server"
    echo "active [3]"
  fi
  if echo "${bt}" | grep -q 'active \[3\]'; then
    echo "/bt_navigator"
    echo "active [3]"
  fi
  if echo "${waypoint}" | grep -q 'active \[3\]'; then
    echo "/follow_waypoints"
  fi
  echo "/cmd_vel"
  echo "status: ${status:-unavailable}"
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
  restart-localization)
    restart_localization_only
    ;;
  ensure-localization-odom)
    ensure_localization_odom
    ;;
  stop-localization)
    stop_localization
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
