#!/usr/bin/env bash
set -euo pipefail

export BAG_ROOT="${BAG_ROOT:-/home/robot/rosbags/navigation}"
export STATE_DIR="${STATE_DIR:-/tmp/roamerx_navigation_rosbag}"
export ROSBAG_EXTRA_TOPICS="${ROSBAG_EXTRA_TOPICS:-/localization_info /status /laser_scan /cmd_vel /cmd_vel_raw}"

exec "$(dirname "$0")/mapping_rosbag.sh" "$@"
