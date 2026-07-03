#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/robot/genisom_roamerx_open}"
MAP_DIR="${MAP_DIR:-/home/robot/.jszr/map}"
LOG_DIR="${LOG_DIR:-/tmp/roamerx_mapping_logs}"
SLAM_CONFIG="${SLAM_CONFIG:-${PROJECT_DIR}/install/robot_slam/share/robot_slam/config/config.yaml}"

mkdir -p "${LOG_DIR}" "${MAP_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u

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

start_mapping() {
  if pgrep -f "robot_slam.*mapping|/robot_slam/mapping|lib/robot_slam/mapping" >/dev/null 2>&1; then
    echo "Mapping node already appears to be running."
    return 0
  fi
  if [ ! -f "${SLAM_CONFIG}" ]; then
    echo "ERROR: SLAM config not found: ${SLAM_CONFIG}" >&2
    exit 1
  fi

  echo "Starting SLAM mapping node..."
  nohup bash -lc "source /opt/ros/humble/setup.bash && source '${PROJECT_DIR}/install/setup.bash' && exec ros2 run robot_slam mapping --ros-args --params-file '${SLAM_CONFIG}'" \
    >"${LOG_DIR}/mapping.log" 2>&1 &

  wait_for_service "/slam_state_service" 20
  echo "Switching SLAM to mapping state..."
  ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 3}" | tee "${LOG_DIR}/start_mapping.last.log"
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
