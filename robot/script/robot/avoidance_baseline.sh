#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
RECORDER="${NAVIGATION_ROSBAG_SCRIPT:-${SCRIPT_DIR}/navigation_rosbag.sh}"
SUMMARY_TOOL="${AVOIDANCE_SUMMARY_TOOL:-${SCRIPT_DIR}/summarize_avoidance_bag.py}"
STATE_DIR="${STATE_DIR:-/tmp/roamerx_navigation_rosbag}"

usage() {
  echo "Usage: $0 start {straight|turn|narrow_passage|static_box|wall_corner|person_crossing} [label]" >&2
  echo "       $0 stop {pass|fail|aborted|unreviewed} [operator notes...]" >&2
  echo "       $0 status" >&2
}

write_manifest() {
  local bag_dir="$1" scenario="$2" outcome="$3" notes="$4"
  python3 - "${bag_dir}/avoidance_scenario.json" "${scenario}" "${outcome}" "${notes}" <<'PY'
import json
import os
import sys
import time

path, scenario, outcome, notes = sys.argv[1:]
payload = {}
if os.path.isfile(path):
    with open(path, encoding="utf-8") as handle:
        payload = json.load(handle)
payload.update({
    "schema": "roamerx.avoidance-scenario.v1",
    "scenario": scenario or payload.get("scenario"),
    "outcome": outcome or payload.get("outcome", "recording"),
    "operator_notes": notes or payload.get("operator_notes", ""),
    "updated_at_unix": round(time.time(), 3),
    "motion_started_by_recorder": False,
    "required_review": [
        "path_was_confirmed_clear_before_motion",
        "no_contact_or_collision",
        "goal_or_route_result",
        "minimum_clearance_video_cross_check",
    ],
})
temporary = path + ".tmp"
with open(temporary, "w", encoding="utf-8") as handle:
    json.dump(payload, handle, ensure_ascii=False, indent=2)
    handle.write("\n")
os.replace(temporary, path)
PY
}

load_session() {
  BAG_DIR=""
  if [ -f "${STATE_DIR}/session.env" ]; then
    source "${STATE_DIR}/session.env"
  fi
}

case "${1:-}" in
  start)
    scenario="${2:-}"
    case "${scenario}" in
      straight|turn|narrow_passage|static_box|wall_corner|person_crossing) ;;
      *) usage; exit 2 ;;
    esac
    label="${3:-${scenario}}"
    output="$("${RECORDER}" start "baseline_${scenario}_${label}")"
    load_session
    [ -n "${BAG_DIR}" ] || { echo "ERROR: recorder did not report a bag directory" >&2; exit 1; }
    write_manifest "${BAG_DIR}" "${scenario}" "recording" ""
    printf '%s\n' "${output}"
    ;;
  stop)
    shift
    outcome="${1:-unreviewed}"
    case "${outcome}" in pass|fail|aborted|unreviewed) ;; *) usage; exit 2 ;; esac
    [ "$#" -eq 0 ] || shift
    notes="$*"
    load_session
    bag_dir="${BAG_DIR:-}"
    scenario=""
    if [ -n "${bag_dir}" ] && [ -f "${bag_dir}/avoidance_scenario.json" ]; then
      scenario="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("scenario", ""))' "${bag_dir}/avoidance_scenario.json")"
    fi
    # Write the reviewed scenario manifest before producing the final summary.
    # Disable navigation_rosbag's background pass to avoid two writers racing.
    output="$(AVOIDANCE_SUMMARY_ENABLED=0 "${RECORDER}" stop)"
    if [ -n "${bag_dir}" ] && [ -d "${bag_dir}" ]; then
      write_manifest "${bag_dir}" "${scenario}" "${outcome}" "${notes}"
      if [ -f "${bag_dir}/metadata.yaml" ]; then
        python3 "${SUMMARY_TOOL}" "${bag_dir}" \
          --output "${bag_dir}/avoidance_summary.json" >/dev/null
      fi
    fi
    printf '%s\n' "${output}"
    ;;
  status)
    exec "${RECORDER}" status
    ;;
  *) usage; exit 2 ;;
esac
