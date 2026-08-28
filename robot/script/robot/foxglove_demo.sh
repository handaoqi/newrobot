#!/usr/bin/env bash
#
# One-key entry point for the Foxglove debugging MVP.
#
#   foxglove_demo.sh preflight            check the environment, change nothing
#   foxglove_demo.sh live                 fake dog + read-only bridge, live view
#   foxglove_demo.sh web [duration]       live + self-hosted Lichtblick in a browser
#   foxglove_demo.sh verify               browser-check the layouts in docs/yuwang
#   foxglove_demo.sh record [--duration N] fake dog -> MCAP bag, then exit
#   foxglove_demo.sh convert <bag_dir>    sqlite3 bag -> MCAP (source untouched)
#   foxglove_demo.sh play <bag_dir>       replay a bag with --clock + bridge
#   foxglove_demo.sh info <bag_dir>       ros2 bag info, no side effects
#   foxglove_demo.sh stop                 stop anything this script started
#
# The demo and every replay run on ROS_DOMAIN_ID 77 over rmw_fastrtps_cpp.
# Production runs on domain 24 over rmw_zenoh_cpp; two different RMW
# implementations do not discover each other, so demo traffic cannot reach the
# robot even if the domain id were wrong.
set -euo pipefail
# Monitor mode puts every background job in its own process group, so $! is
# also the group id. Without it a `ros2 launch` that gets SIGKILLed leaves its
# node orphaned and still holding the bridge port; with it the whole group goes.
set -m

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
WORKSPACE="${WORKSPACE:-/home/dogrobot/robot}"
BAG_ROOT="${BAG_ROOT:-/home/dogrobot/runtime/nx-edge/data/rosbags/foxglove-demo}"
STATE_DIR="${STATE_DIR:-/tmp/roamerx_foxglove_demo-${UID:-$(id -u)}}"
STORAGE_CONF="${STORAGE_CONF:-${SCRIPT_DIR}/rosbag_storage_mcap.yaml}"
MIN_FREE_GB="${MIN_FREE_GB:-5}"

DEMO_DOMAIN_ID="${DEMO_DOMAIN_ID:-77}"
DEMO_RMW="${DEMO_RMW:-rmw_fastrtps_cpp}"
BRIDGE_ADDRESS="${BRIDGE_ADDRESS:-127.0.0.1}"
BRIDGE_PORT="${BRIDGE_PORT:-8765}"

# Self-hosted Lichtblick. WEB_ADDRESS is what operators change to reach the page
# from another machine; the bridge itself can stay on 127.0.0.1 because the web
# server proxies /ws to it, so only one port is ever exposed.
WEB_DIST="${WEB_DIST:-/home/dogrobot/runtime/nx-edge/install/lichtblick-web/dist}"
WEB_ADDRESS="${WEB_ADDRESS:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-8080}"
WEB_LAYOUT="${WEB_LAYOUT:-/home/dogrobot/docs/yuwang/demo_patrol_layout.json}"

PID_FILE="${STATE_DIR}/children.pid"

# Recorded topics. Kept in sync with REQUIRED_TOPICS in
# roamerx_patrol_demo/bag_contract.py; `record` verifies the bag against that
# same list afterwards, so a divergence fails loudly instead of producing a bag
# that silently misses a topic.
DEMO_TOPICS=(
  /clock /map /tf /tf_static /odom /cmd_vel /scan
  /camera/front/image/compressed /imu/data /battery_state /diagnostics
  /plan /goal_pose /patrol/trajectory /patrol/status
)

log()  { printf '[foxglove-demo] %s\n' "$*" >&2; }
die()  { printf '[foxglove-demo] ERROR: %s\n' "$*" >&2; exit 1; }

setup_env() {
  [ -f "${WORKSPACE}/install/setup.bash" ] || \
    die "workspace overlay not found: ${WORKSPACE}/install/setup.bash (run: cd ${WORKSPACE} && colcon build --packages-select roamerx_patrol_demo)"
  # The ament setup scripts read unset variables, so -u has to come off for the
  # duration of the sourcing and go straight back on afterwards.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${WORKSPACE}/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID="${DEMO_DOMAIN_ID}"
  export RMW_IMPLEMENTATION="${DEMO_RMW}"
  # rmw_fastrtps needs no router; make sure a zenoh session config left in the
  # environment by the production shell cannot influence this one.
  unset ZENOH_SESSION_CONFIG_URI ZENOH_ROUTER_CONFIG_URI ZENOH_CONFIG_OVERRIDE || true
  mkdir -p "${STATE_DIR}"
}

# --- child process bookkeeping ------------------------------------------------
# Only PIDs written here are ever signalled. Nothing is matched by name, so a
# production node that happens to share an executable name is never touched.

track() { echo "$1" >>"${PID_FILE}"; }

stop_children() {
  # Read the file exactly once. It used to be redirected from three times, which
  # raced against a second instance's EXIT trap: the -f guard passed, then the
  # other trap removed the file, and the next redirection died with "No such file
  # or directory" mid-teardown -- leaving the remaining children running.
  local pids=()
  # 2>/dev/null precedes the input redirection deliberately: bash applies
  # redirections left to right, so putting it last would let the "No such file"
  # message escape before stderr was silenced.
  mapfile -t pids 2>/dev/null <"${PID_FILE}" || return 0
  rm -f "${PID_FILE}"

  local pid
  for pid in "${pids[@]}"; do
    [ -n "${pid}" ] || continue
    # Signal the group, then the bare pid: the group covers the nodes that
    # `ros2 launch` spawned, the bare pid covers the case where the job never
    # became a group leader.
    kill -INT "-${pid}" 2>/dev/null || true
    kill -INT "${pid}" 2>/dev/null || true
  done
  # rosbag needs a moment to close the last mcap chunk cleanly.
  local waited=0
  while [ "${waited}" -lt 150 ]; do
    local alive=0
    for pid in "${pids[@]}"; do
      [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null && alive=1
    done
    [ "${alive}" -eq 0 ] && break
    sleep 0.1
    waited=$((waited + 1))
  done
  for pid in "${pids[@]}"; do
    [ -n "${pid}" ] || continue
    kill -KILL "-${pid}" 2>/dev/null || true
    kill -KILL "${pid}" 2>/dev/null || true
  done
}

trap stop_children EXIT INT TERM

# --- checks -------------------------------------------------------------------

check_port() {
  local port="${1:-${BRIDGE_PORT}}" hint="${2:-BRIDGE_PORT}"
  if command -v ss >/dev/null 2>&1 && ss -ltn "sport = :${port}" 2>/dev/null | grep -q ":${port}"; then
    die "port ${port} is already in use; set ${hint} to a free port"
  fi
}

check_disk() {
  local target="$1" free_gb
  mkdir -p "${target}"
  free_gb=$(df -BG --output=avail "${target}" | tail -1 | tr -dc '0-9')
  [ -n "${free_gb}" ] || die "cannot determine free space on ${target}"
  [ "${free_gb}" -ge "${MIN_FREE_GB}" ] || die "only ${free_gb} GiB free on ${target}, need ${MIN_FREE_GB} GiB"
}

resolve_bag() {
  local raw="$1" resolved
  [ -n "${raw}" ] || die "a bag directory is required"
  resolved="$(readlink -f "${raw}")" || die "cannot resolve path: ${raw}"
  [ -d "${resolved}" ] || die "not a directory: ${resolved}"
  [ -f "${resolved}/metadata.yaml" ] || die "not a complete rosbag2 directory (no metadata.yaml): ${resolved}
An orphan .db3/.mcap file is not a valid input. Copy the directory and run 'ros2 bag reindex' on the copy if the metadata was lost."
  printf '%s' "${resolved}"
}

# --- subcommands --------------------------------------------------------------

cmd_preflight() {
  setup_env
  log "ROS_DISTRO=${ROS_DISTRO:-unset} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  ros2 pkg prefix roamerx_patrol_demo >/dev/null || die "roamerx_patrol_demo is not on the overlay"
  ros2 pkg prefix foxglove_bridge >/dev/null || die "foxglove_bridge is not installed"
  ros2 pkg prefix rosbag2_storage_mcap >/dev/null || die "rosbag2_storage_mcap is not installed"
  [ -f "${STORAGE_CONF}" ] || die "mcap storage config not found: ${STORAGE_CONF}"
  check_port
  check_disk "${BAG_ROOT}"
  log "preflight OK: bridge will bind ${BRIDGE_ADDRESS}:${BRIDGE_PORT}, bags go to ${BAG_ROOT}"
}

start_bridge() {
  local use_sim_time="${1:-false}"
  ros2 launch roamerx_patrol_demo foxglove_readonly.launch.py \
    domain_id:="${DEMO_DOMAIN_ID}" rmw:="${DEMO_RMW}" \
    address:="${BRIDGE_ADDRESS}" port:="${BRIDGE_PORT}" \
    use_sim_time:="${use_sim_time}" &
  track "$!"
}

cmd_live() {
  local duration="${1:-120.0}"
  cmd_preflight
  log "starting fake dog (${duration}s) + read-only bridge"
  start_bridge false
  ros2 launch roamerx_patrol_demo indoor_patrol_demo.launch.py \
    duration_sec:="${duration}" &
  local publisher=$!
  track "${publisher}"
  log "connect Foxglove to ws://${BRIDGE_ADDRESS}:${BRIDGE_PORT} and import docs/yuwang/demo_patrol_layout.json"
  wait "${publisher}" || true
  log "patrol finished; stopping bridge"
}

cmd_web() {
  local duration="${1:-300.0}"
  [ -d "${WEB_DIST}" ] || \
    die "Lichtblick bundle not found at ${WEB_DIST}
Run: ${SCRIPT_DIR}/fetch_lichtblick_web.sh"
  [ -f "${WEB_LAYOUT}" ] || die "layout not found: ${WEB_LAYOUT}"
  check_port "${WEB_PORT}" WEB_PORT
  cmd_preflight

  log "starting fake dog (${duration}s) + read-only bridge + Lichtblick on :${WEB_PORT}"
  start_bridge false
  python3 "${SCRIPT_DIR}/foxglove_web_serve.py" \
    --dist "${WEB_DIST}" --address "${WEB_ADDRESS}" --port "${WEB_PORT}" \
    --bridge-host "${BRIDGE_ADDRESS}" --bridge-port "${BRIDGE_PORT}" \
    --default-layout "${WEB_LAYOUT}" &
  track "$!"

  ros2 launch roamerx_patrol_demo indoor_patrol_demo.launch.py \
    duration_sec:="${duration}" &
  local publisher=$!
  track "${publisher}"

  local shown="${WEB_ADDRESS}"
  [ "${shown}" = "0.0.0.0" ] && shown="$(hostname -I 2>/dev/null | awk '{print $1}')"
  log "open http://${shown:-127.0.0.1}:${WEB_PORT}/ -- the layout is already loaded"
  log "the page connects to the bridge through ws://${shown:-127.0.0.1}:${WEB_PORT}/ws"
  wait "${publisher}" || true
  log "patrol finished; stopping bridge and web server"
}

cmd_verify() {
  local frontend="/home/dogrobot/platform/frontend"
  [ -d "${WEB_DIST}" ] || die "Lichtblick bundle not found at ${WEB_DIST}; run ${SCRIPT_DIR}/fetch_lichtblick_web.sh"
  [ -d "${frontend}/node_modules/@playwright" ] || \
    die "playwright is not installed in ${frontend}; run 'npm install' there first"
  log "checking docs/yuwang layouts in a real browser (this starts its own web servers)"
  # Handed over entirely: playwright manages its own servers on 8091-8093 and
  # cleans them up, so they must not go into this script's PID file.
  ( cd "${frontend}" && npm run --silent test:foxglove )
}

cmd_record() {
  local duration="120.0" name=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --duration) duration="${2:?--duration needs a value}"; shift 2 ;;
      --name)     name="${2:?--name needs a value}"; shift 2 ;;
      *) die "unknown option for record: $1" ;;
    esac
  done
  cmd_preflight
  # Sanitised so a name from an operator can never escape BAG_ROOT.
  name="$(printf '%s' "${name}" | tr -c 'A-Za-z0-9_-' '_')"
  local stamp bag_name bag staging
  stamp="$(date +%Y%m%d_%H%M%S)"
  bag_name="${stamp}${name:+_${name}}"
  bag="${BAG_ROOT}/${bag_name}"
  [ -e "${bag}" ] && die "target already exists: ${bag}"
  # Staged inside BAG_ROOT rather than /tmp so the final move is a rename on the
  # same filesystem, and staged one level down so rosbag2 names the mcap file
  # after the bag rather than after a temporary directory.
  staging="${BAG_ROOT}/.partial-$$-${stamp}"
  mkdir -p "${staging}"

  log "recording ${duration}s of demo data to ${bag}"
  ros2 bag record --storage mcap --storage-config-file "${STORAGE_CONF}" \
    --output "${staging}/${bag_name}" "${DEMO_TOPICS[@]}" &
  track "$!"
  # The recorder must have its subscriptions up before the publisher starts, or
  # the transient-local /map and /tf_static latch into nothing.
  sleep 3

  ros2 launch roamerx_patrol_demo indoor_patrol_demo.launch.py \
    duration_sec:="${duration}" &
  local publisher=$! status=0
  track "${publisher}"
  wait "${publisher}" || status=$?
  log "patrol finished; flushing bag"
  stop_children
  [ "${status}" -eq 0 ] || die "the patrol publisher exited with status ${status}; partial data left at ${staging}"

  verify_bag "${staging}/${bag_name}" "${staging}"
  mv "${staging}/${bag_name}" "${bag}"
  rmdir "${staging}"
  log "wrote ${bag}"
  ros2 bag info "${bag}"
}

# A bag that exists but holds nothing is the failure mode worth catching here:
# the recorder starting late, the publisher dying, or a topic name typo all
# produce a well-formed empty bag that every later step happily accepts.
verify_bag() {
  local candidate="$1" keep="$2" count
  [ -f "${candidate}/metadata.yaml" ] || die "recording produced no metadata.yaml; left at ${keep}"
  count="$(python3 - "${candidate}/metadata.yaml" <<'PY'
import re, sys
text = open(sys.argv[1], encoding="utf-8").read()
match = re.search(r"^\s*message_count:\s*(\d+)\s*$", text, re.MULTILINE)
print(match.group(1) if match else 0)
PY
)"
  [ "${count}" -gt 0 ] || die "the bag recorded 0 messages; left at ${keep}
Check that the publisher ran on ROS_DOMAIN_ID=${DEMO_DOMAIN_ID} with RMW_IMPLEMENTATION=${DEMO_RMW}."
  local missing=() topic
  for topic in "${DEMO_TOPICS[@]}"; do
    grep -qF "name: ${topic}" "${candidate}/metadata.yaml" || missing+=("${topic}")
  done
  [ "${#missing[@]}" -eq 0 ] || die "the bag is missing expected topics: ${missing[*]}
Left at ${keep}."
  log "bag contract OK: ${count} messages, all ${#DEMO_TOPICS[@]} expected topics present"
}

cmd_convert() {
  local source target conf
  source="$(resolve_bag "${1:-}")"
  target="${2:-${source}-mcap}"
  [ -e "${target}" ] && die "target already exists: ${target}"
  check_disk "$(dirname "${target}")"
  setup_env
  conf="$(mktemp "${STATE_DIR}/convert.XXXXXX.yaml")"
  cat >"${conf}" <<EOF
output_bags:
  - uri: ${target}
    storage_id: mcap
    all: true
EOF
  log "converting ${source} -> ${target} (source is opened read-only and not modified)"
  ros2 bag convert -i "${source}" -o "${conf}"
  rm -f "${conf}"
  ros2 bag info "${target}"
  log "converted; open ${target}/*.mcap in Foxglove"
}

cmd_play() {
  local bag rate
  bag="$(resolve_bag "${1:-}")"
  rate="${2:-0.5}"
  cmd_preflight
  log "replaying ${bag} at ${rate}x on domain ${DEMO_DOMAIN_ID}"
  start_bridge true
  ros2 bag play "${bag}" --clock 100 --rate "${rate}" --start-paused &
  local player=$!
  track "${player}"
  log "player is paused; press SPACE in this terminal to start, connect Foxglove to ws://${BRIDGE_ADDRESS}:${BRIDGE_PORT}"
  wait "${player}" || true
}

cmd_info() {
  local bag
  bag="$(resolve_bag "${1:-}")"
  setup_env
  ros2 bag info "${bag}"
}

cmd_stop() {
  stop_children
  log "stopped"
}

main() {
  local command="${1:-}"
  shift || true
  case "${command}" in
    preflight) cmd_preflight ;;
    live)      cmd_live "$@" ;;
    web)       cmd_web "$@" ;;
    verify)    cmd_verify ;;
    record)    cmd_record "$@" ;;
    convert)   cmd_convert "$@" ;;
    play)      cmd_play "$@" ;;
    info)      cmd_info "$@" ;;
    stop)      cmd_stop ;;
    ""|-h|--help|help)
      sed -n '3,14p' "$(readlink -f "${BASH_SOURCE[0]}")" | sed 's/^# \{0,1\}//'
      ;;
    *) die "unknown command: ${command} (try --help)" ;;
  esac
}

main "$@"
