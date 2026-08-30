#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 BAG_DIR [--rate N] [--start-offset SEC] [--duration SEC] [-- SHADOW_COMMAND ...]" >&2
}

[ "$#" -ge 1 ] || { usage; exit 2; }
BAG_DIR="$1"
shift
[ -f "${BAG_DIR}/metadata.yaml" ] || { echo "ERROR: not a rosbag: ${BAG_DIR}" >&2; exit 2; }

rate="1.0"
start_offset="0"
duration=""
declare -a shadow_command=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --rate) rate="$2"; shift 2 ;;
    --start-offset) start_offset="$2"; shift 2 ;;
    --duration) duration="$2"; shift 2 ;;
    --) shift; shadow_command=("$@"); break ;;
    *) usage; exit 2 ;;
  esac
done

export ROS_DOMAIN_ID="${SHADOW_ROS_DOMAIN_ID:-77}"
export RMW_IMPLEMENTATION="${SHADOW_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [ "${ROS_DOMAIN_ID}" = "24" ]; then
  echo "ERROR: shadow replay refuses the production ROS domain 24" >&2
  exit 3
fi

set +u
source /opt/ros/humble/setup.bash
source /home/dogrobot/robot/install/setup.bash
set -u

output_root="${SHADOW_OUTPUT_ROOT:-/home/dogrobot/runtime/nx-edge/data/rosbags/avoidance-shadow}"
session="${output_root}/$(date +%Y%m%d_%H%M%S)_$(basename "${BAG_DIR}")"
mkdir -p "${session}"
shadow_pid=""
cleanup() {
  if [ -n "${shadow_pid}" ] && kill -0 "${shadow_pid}" 2>/dev/null; then
    kill -INT -- "-${shadow_pid}" 2>/dev/null || kill -INT "${shadow_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ "${#shadow_command[@]}" -gt 0 ]; then
  setsid "${shadow_command[@]}" >"${session}/shadow-stack.log" 2>&1 < /dev/null &
  shadow_pid="$!"
  sleep 2
fi

# Domain isolation is the first guard; absence of the SDK bridge in that graph
# is the second. Recorded and shadow-produced velocity topics cannot reach the
# production robot even if a bag contains non-zero commands.
if ros2 node list 2>/dev/null | grep -Eq 'vel_cmd_(udp|lcm)_pub'; then
  echo "ERROR: SDK velocity bridge detected in shadow domain ${ROS_DOMAIN_ID}" >&2
  exit 3
fi

declare -a play_args=(
  "${BAG_DIR}" --clock --disable-keyboard-controls --rate "${rate}" --start-offset "${start_offset}"
  --remap
  /cmd_vel:=/shadow/recorded/cmd_vel
  /cmd_vel_raw:=/shadow/recorded/cmd_vel_raw
  /cmd_vel_nav:=/shadow/recorded/cmd_vel_nav
  /teleop_cmd_vel:=/shadow/recorded/teleop_cmd_vel
  /mode_switch_cmd:=/shadow/recorded/mode_switch_cmd
)

python3 - "${session}/session.json" "${BAG_DIR}" "${ROS_DOMAIN_ID}" "${rate}" <<'PY'
import json, os, sys, time
path, bag, domain, rate = sys.argv[1:]
with open(path, "w", encoding="utf-8") as handle:
    json.dump({
        "schema": "roamerx.avoidance-shadow.v1",
        "bag_path": os.path.realpath(bag),
        "ros_domain_id": int(domain),
        "rate": float(rate),
        "sdk_bridge_allowed": False,
        "started_at_unix": round(time.time(), 3),
    }, handle, ensure_ascii=False, indent=2)
    handle.write("\n")
PY

set +e
if [ -n "${duration}" ]; then
  timeout --signal=INT --kill-after=5 "${duration}" \
    ros2 bag play "${play_args[@]}" 2>&1 | tee "${session}/replay.log"
  replay_status="${PIPESTATUS[0]}"
else
  ros2 bag play "${play_args[@]}" 2>&1 | tee "${session}/replay.log"
  replay_status="${PIPESTATUS[0]}"
fi
set -e
# timeout returns 124 after delivering the intentional SIGINT cutoff.
if [ "${replay_status}" -ne 0 ] &&
   { [ -z "${duration}" ] || [ "${replay_status}" -ne 124 ]; }; then
  echo "ERROR: shadow replay failed with status ${replay_status}" >&2
  exit "${replay_status}"
fi
python3 "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/summarize_avoidance_bag.py" \
  "${BAG_DIR}" --output "${session}/avoidance_summary.json" >/dev/null
echo "Shadow replay complete: ${session}"
