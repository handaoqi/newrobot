#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/dogrobot/runtime/nx-edge"
for dir in bin data conf scripts docs install; do test -d "$ROOT/$dir"; done
for path in \
  "$ROOT/data/jszr/map" \
  "$ROOT/data/rosbags" \
  "$ROOT/data/edge-agent" \
  "$ROOT/conf/edge-agent.yaml" \
  "$ROOT/install/genisom_l1_sdk"; do
  [[ -e "$path" ]] || { echo "Missing runtime asset: $path" >&2; exit 1; }
done
set +u
source /opt/ros/humble/setup.bash
source /home/dogrobot/robot/install/setup.bash
set -u
ros2 pkg prefix localization
ros2 pkg prefix robot_slam
ros2 pkg prefix robot_navigo
systemctl is-active roamerx-edge-agent roamerx-dev-agent roamerx-bike-bot roamerx-robot-mcp
echo "NX runtime is healthy."
