#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/dogrobot/robot}"
BAG_ROOT="${BAG_ROOT:-/home/robot/rosbags/mapping}"
STATE_DIR="${STATE_DIR:-/tmp/roamerx_mapping_rosbag}"
MIN_FREE_GB="${MIN_FREE_GB:-10}"
PID_FILE="${STATE_DIR}/recorder.pid"
SESSION_FILE="${STATE_DIR}/session.env"
LOG_FILE="${STATE_DIR}/recorder.log"
ROSBAG_EXTRA_TOPICS="${ROSBAG_EXTRA_TOPICS:-}"

mkdir -p "${BAG_ROOT}" "${STATE_DIR}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

is_running() {
  [ -f "${PID_FILE}" ] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null
}

load_session() {
  BAG_DIR=""
  STARTED_AT_UNIX="0"
  STOPPED_AT_UNIX="0"
  if [ -f "${SESSION_FILE}" ]; then
    # Values are generated locally and shell-escaped before being written.
    source "${SESSION_FILE}"
  fi
}

print_status() {
  load_session
  local running=false pid=0 size=0 duration=0
  if is_running; then
    running=true
    pid="$(cat "${PID_FILE}")"
  fi
  if [ -n "${BAG_DIR}" ] && [ -d "${BAG_DIR}" ]; then
    size="$(du -sb "${BAG_DIR}" 2>/dev/null | awk '{print $1}')"
  fi
  if [ "${STARTED_AT_UNIX:-0}" -gt 0 ] 2>/dev/null; then
    local end_time
    end_time="${STOPPED_AT_UNIX:-0}"
    [ "${end_time}" -gt 0 ] 2>/dev/null || end_time="$(date +%s)"
    duration=$(( end_time - STARTED_AT_UNIX ))
  fi
  python3 - "${running}" "${pid}" "${BAG_DIR}" "${STARTED_AT_UNIX:-0}" "${duration}" "${size:-0}" "${LOG_FILE}" <<'PY'
import json
import sys

print(json.dumps({
    "running": sys.argv[1] == "true",
    "pid": int(sys.argv[2]),
    "bag_dir": sys.argv[3] or None,
    "started_at_unix": int(sys.argv[4]),
    "duration_seconds": max(0, int(sys.argv[5])),
    "size_bytes": int(sys.argv[6]),
    "log_path": sys.argv[7],
}, ensure_ascii=False))
PY
}

start_recording() {
  if is_running; then
    print_status
    return 0
  fi
  rm -f "${PID_FILE}"
  local available_kb required_kb label stamp bag_dir
  available_kb="$(df -Pk "${BAG_ROOT}" | awk 'NR==2 {print $4}')"
  required_kb=$((MIN_FREE_GB * 1024 * 1024))
  if [ "${available_kb}" -lt "${required_kb}" ]; then
    echo "ERROR: less than ${MIN_FREE_GB}GB free under ${BAG_ROOT}" >&2
    exit 2
  fi
  label="${1:-mapping}"
  label="$(printf '%s' "${label}" | tr -cd '[:alnum:]_.-' | cut -c1-48)"
  [ -n "${label}" ] || label="mapping"
  stamp="$(date +%Y%m%d_%H%M%S)"
  bag_dir="${BAG_ROOT}/${stamp}_${label}"

  set +u
  source /opt/ros/humble/setup.bash
  source "${PROJECT_DIR}/install/setup.bash"
  set -u

  local -a topics=(
    /front_lidar
    /front_lidar/imu
    /odom/mc_odom
    /fix
    /tf
    /tf_static
    /rosout
  )
  if [ -n "${ROSBAG_EXTRA_TOPICS}" ]; then
    local -a extra_topics
    read -r -a extra_topics <<<"${ROSBAG_EXTRA_TOPICS}"
    topics+=("${extra_topics[@]}")
  fi

  setsid ros2 bag record \
    -o "${bag_dir}" \
    "${topics[@]}" \
    >"${LOG_FILE}" 2>&1 < /dev/null &
  local pid=$!
  printf '%s\n' "${pid}" >"${PID_FILE}"
  {
    printf 'BAG_DIR=%q\n' "${bag_dir}"
    printf 'STARTED_AT_UNIX=%q\n' "$(date +%s)"
    printf 'STOPPED_AT_UNIX=0\n'
  } >"${SESSION_FILE}"
  sleep 2
  if ! is_running; then
    echo "ERROR: rosbag recorder exited; see ${LOG_FILE}" >&2
    tail -n 40 "${LOG_FILE}" >&2 || true
    exit 1
  fi
  print_status
}

stop_recording() {
  load_session
  if is_running; then
    local pid
    pid="$(cat "${PID_FILE}")"
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
  fi
  rm -f "${PID_FILE}"
  if [ "${STARTED_AT_UNIX:-0}" -gt 0 ] 2>/dev/null; then
    {
      printf 'BAG_DIR=%q\n' "${BAG_DIR}"
      printf 'STARTED_AT_UNIX=%q\n' "${STARTED_AT_UNIX}"
      printf 'STOPPED_AT_UNIX=%q\n' "$(date +%s)"
    } >"${SESSION_FILE}"
  fi
  print_status
}

case "${1:-status}" in
  start) start_recording "${2:-mapping}" ;;
  stop) stop_recording ;;
  status) print_status ;;
  *) echo "Usage: $0 {start [label]|stop|status}" >&2; exit 2 ;;
esac
