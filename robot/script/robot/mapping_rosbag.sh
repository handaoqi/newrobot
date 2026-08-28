#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/dogrobot/robot}"
BAG_ROOT="${BAG_ROOT:-/home/dogrobot/runtime/nx-edge/data/rosbags/mapping}"
STATE_DIR="${STATE_DIR:-/tmp/roamerx_mapping_rosbag-${UID:-$(id -u)}}"
MIN_FREE_GB="${MIN_FREE_GB:-10}"
# Retention runs only when the disk is close to the refusal threshold below.
# Steady-state recording never deletes anything; this exists so that running
# out of space degrades into "the oldest unreferenced bags go" instead of
# "mapping stops working".
PRUNE_TOOL="${PRUNE_TOOL:-$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/prune_runtime_storage.py}"
PRUNE_ENABLED="${PRUNE_ENABLED:-1}"
PRUNE_TRIGGER_GB="${PRUNE_TRIGGER_GB:-$((MIN_FREE_GB * 2))}"
PID_FILE="${STATE_DIR}/recorder.pid"
SESSION_FILE="${STATE_DIR}/session.env"
LOG_FILE="${STATE_DIR}/recorder.log"
# Kept beside the bags rather than in STATE_DIR: the state dir is under /tmp and
# is per-uid, and the Edge Agent has to be able to find this report to report
# deletions upstream. The leading dot keeps it out of the session scan.
RETENTION_REPORT="${RETENTION_REPORT:-${BAG_ROOT}/.retention.json}"
ROSBAG_EXTRA_TOPICS="${ROSBAG_EXTRA_TOPICS:-}"

mkdir -p "${BAG_ROOT}" "${STATE_DIR}"
if [ ! -w "${STATE_DIR}" ]; then
  echo "ERROR: rosbag state directory is not writable by $(id -un): ${STATE_DIR}" >&2
  echo "Set STATE_DIR to a writable per-user directory and retry." >&2
  exit 1
fi

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

run_prune() {
  # $1: "apply" to actually delete, anything else for a dry run.
  # $2+: extra flags for the retention tool.
  # The JSON report goes to stdout; callers that own stdout must redirect it.
  local mode="$1"
  shift
  if [ ! -f "${PRUNE_TOOL}" ]; then
    echo "ERROR: retention tool not found: ${PRUNE_TOOL}" >&2
    return 1
  fi
  local -a args=("${PRUNE_TOOL}" --bag-root "${BAG_ROOT}")
  local in_flight
  # The bag being written right now is the one deletion would hurt most, and
  # it is too new to be referenced by any map manifest yet.
  in_flight="$( { load_session; is_running && printf '%s' "${BAG_DIR}"; } )" || true
  [ -n "${in_flight}" ] && args+=(--exclude "${in_flight}")
  [ "${mode}" = "apply" ] && args+=(--apply)
  args+=("$@")
  python3 "${args[@]}"
}

reclaim_space_if_needed() {
  local available_gb trigger_kb available_kb
  available_kb="$(df -Pk "${BAG_ROOT}" | awk 'NR==2 {print $4}')"
  trigger_kb=$((PRUNE_TRIGGER_GB * 1024 * 1024))
  if [ "${PRUNE_ENABLED}" != "1" ] || [ "${available_kb}" -ge "${trigger_kb}" ]; then
    return 0
  fi
  available_gb=$((available_kb / 1024 / 1024))
  echo "NOTICE: ${available_gb}GB free is under the ${PRUNE_TRIGGER_GB}GB retention trigger; pruning old recordings" >&2
  # Bags only. Map sessions are addressable by name from the cloud platform
  # (map_activation_adapter._resolve_source_dir resolves map_dir/<version>),
  # so removing one is a decision for an operator with the platform's map list
  # in front of them - `mapping_rosbag.sh prune --apply` does that on request.
  if ! run_prune apply --skip-maps >"${RETENTION_REPORT}" 2>>"${LOG_FILE}"; then
    # Never block a recording on cleanup: the free-space gate below is still
    # the authority on whether there is room.
    echo "WARNING: retention pass failed; see ${LOG_FILE}" >&2
    return 0
  fi
  python3 - "${RETENTION_REPORT}" >&2 <<'PY'
import json
import sys

try:
    with open(sys.argv[1], encoding="utf-8") as handle:
        report = json.load(handle)
except (OSError, ValueError) as exc:
    print(f"WARNING: unreadable retention report: {exc}")
    raise SystemExit(0)

freed = report.get("reclaimed_bytes", 0) / 1024 ** 3
for root in report.get("roots", []):
    for entry in root.get("deleted", []):
        print(f"RECLAIMED {entry['path']} "
              f"({entry['size_bytes'] / 1024 ** 3:.2f}GiB, {entry['reason']})")
print(f"RECLAIMED total {freed:.2f}GiB across {report.get('failed_count', 0)} failures")
PY
}

start_recording() {
  if is_running; then
    print_status
    return 0
  fi
  rm -f "${PID_FILE}"
  local available_kb required_kb label stamp bag_dir
  reclaim_space_if_needed
  available_kb="$(df -Pk "${BAG_ROOT}" | awk 'NR==2 {print $4}')"
  required_kb=$((MIN_FREE_GB * 1024 * 1024))
  if [ "${available_kb}" -lt "${required_kb}" ]; then
    echo "ERROR: less than ${MIN_FREE_GB}GB free under ${BAG_ROOT} after retention" >&2
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

  # Both phases run robot_slam, so the odometry pair below is always available:
  # /odom/lio_odom is the FAST-LIO2 frontend output that feeds the localization
  # UKF (config.yaml lio_primary.topic), /slam_odom is the backend-corrected
  # pose. Recording only the latter makes frontend drift indistinguishable from
  # a bad backend correction.
  local -a topics=(
    /front_lidar
    /front_lidar/imu
    /fix
    /rtk_pvh
    /rtk/ntrip_status
    /odom/lio_odom
    /odom/localization_odom
    /slam_odom
    /tf
    /tf_static
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
  # Dry run unless --apply is passed, so this is safe to poll for reporting.
  # Remaining arguments go straight to the retention tool (--skip-maps,
  # --bag-max-total-gib, ...).
  prune)
    shift
    if [ "${1:-}" = "--apply" ]; then
      shift
      run_prune apply "$@"
    else
      run_prune dry "$@"
    fi
    ;;
  *) echo "Usage: $0 {start [label]|stop|status|prune [--apply] [tool args...]}" >&2; exit 2 ;;
esac
