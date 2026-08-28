#!/usr/bin/env bash
set -euo pipefail

export BAG_ROOT="${BAG_ROOT:-/home/dogrobot/runtime/nx-edge/data/rosbags/navigation}"
export STATE_DIR="${STATE_DIR:-/tmp/roamerx_navigation_rosbag}"
# /odom/nav2 is not a relay of /odom/localization_odom: tf_publisher flattens
# roll/pitch (yawOnly) before republishing, and navigo_params.yaml points both
# planner_server and controller_server at it. It is the pose Nav2 actually
# consumed, so a replay that only has /odom/localization_odom cannot explain
# Nav2 behaviour.
# /plan is the global path from planner_server; /transformed_global_plan is the
# pruned segment MPPI was tracking (see navigo_mppi_controller/controller.cpp).
# /cmd_vel_raw -> /cmd_vel spans collision_monitor, so its interventions show up
# as the difference between the two.
export ROSBAG_EXTRA_TOPICS="${ROSBAG_EXTRA_TOPICS:-/localization_info /status /laser_scan /odom/nav2 /plan /transformed_global_plan /cmd_vel /cmd_vel_raw}"

exec "$(dirname "$0")/mapping_rosbag.sh" "$@"
