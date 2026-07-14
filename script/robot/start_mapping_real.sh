#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/robot/genisom_roamerx_open}"
MAP_DIR="${MAP_DIR:-/home/robot/.jszr/map}"
LOG_DIR="${LOG_DIR:-/tmp/roamerx_mapping_logs}"
SLAM_CONFIG="${SLAM_CONFIG:-${PROJECT_DIR}/install/robot_slam/share/robot_slam/config/config.yaml}"
REQUIRE_RTK="${REQUIRE_RTK:-1}"
RTK_WAIT_SECONDS="${RTK_WAIT_SECONDS:-45}"

mkdir -p "${LOG_DIR}" "${MAP_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

usage() {
  echo "Usage: $0 {start|save|stop|restart|status}"
  echo
  echo "start   Start SLAM and switch to mapping state."
  echo "save    Ask SLAM to save the current map."
  echo "stop    Save map, then stop SLAM."
  echo "restart Stop existing SLAM, then start again."
  echo "status  Show SLAM-related processes and topics."
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

wait_for_service() {
  local service="$1"
  local timeout_s="$2"
  for _ in $(seq 1 "${timeout_s}"); do
    if ros2 service list 2>/dev/null | grep -qx "${service}"; then
      return 0
    fi
    sleep 1
  done
  echo "ERROR: timed out waiting for service ${service}" >&2
  return 1
}

ensure_rtk() {
  if [ "${REQUIRE_RTK}" != "1" ]; then
    return 0
  fi
  echo "Starting RTK/NTRIP..."
  "${PROJECT_DIR}/script/robot/start_rtk_ntrip.sh" >/tmp/roamerx_rtk_start.log 2>&1 || {
    cat /tmp/roamerx_rtk_start.log >&2
    return 1
  }
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
  echo "Set REQUIRE_RTK=0 to start mapping without RTK." >&2
  return 1
}

start_mapping() {
  if pgrep -f "robot_slam.*mapping|/robot_slam/mapping|lib/robot_slam/mapping" >/dev/null 2>&1; then
    echo "Mapping node already appears to be running."
    return 0
  fi
  if [ ! -f "${SLAM_CONFIG}" ]; then
    echo "ERROR: SLAM config not found: ${SLAM_CONFIG}" >&2
    exit 1
  fi

  ensure_rtk

  echo "Starting SLAM mapping node..."
  setsid bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' && exec '${PROJECT_DIR}/install/robot_slam/lib/robot_slam/mapping' --ros-args --params-file '${SLAM_CONFIG}'" \
    >"${LOG_DIR}/mapping.log" 2>&1 < /dev/null &

  wait_for_service "/slam_state_service" 20
  echo "Switching SLAM to mapping state..."
  ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 3}" | tee "${LOG_DIR}/start_mapping.last.log"
  sleep 2
  if ! pgrep -f "robot_slam.*mapping|/robot_slam/mapping|lib/robot_slam/mapping" >/dev/null 2>&1; then
    echo "ERROR: mapping process exited after switching ACTIVE. Recent log:" >&2
    tail -n 80 "${LOG_DIR}/mapping.log" >&2 || true
    return 1
  fi
  echo "Mapping started. Logs: ${LOG_DIR}/mapping.log"
}

save_map() {
  wait_for_service "/slam_state_service" 5
  echo "Saving map..."
  ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 5}" | tee "${LOG_DIR}/save_map.last.log"
  sleep 3
  echo "Recent map files:"
  ls -lt "${MAP_DIR}" | head -10 || true
}

stop_mapping() {
  if ros2 service list 2>/dev/null | grep -qx "/slam_state_service"; then
    save_map || true
  fi
  kill_pattern "robot_slam.*mapping"
  kill_pattern "/robot_slam/mapping"
  kill_pattern "ros2 launch robot_slam"
  echo "Mapping stopped."
}

status_mapping() {
  echo "Processes:"
  pgrep -af "robot_slam.*mapping|ros2 launch robot_slam" || true
  echo
  echo "Services:"
  ros2 service list 2>/dev/null | grep -E "slam_state|mapping|map" || true
  echo
  echo "RTK:"
  ros2 topic info /fix 2>/dev/null || true
  timeout 3 ros2 topic echo /rtk/ntrip_status --once 2>/dev/null | sed -n '1,40p' || true
  echo
  echo "Recent map files:"
  ls -lt "${MAP_DIR}" | head -10 || true
}

MODE="${1:-start}"
case "${MODE}" in
  start)
    start_mapping
    ;;
  save)
    save_map
    ;;
  stop)
    stop_mapping
    ;;
  restart)
    stop_mapping
    start_mapping
    ;;
  status)
    status_mapping
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage
    exit 1
    ;;
esac
