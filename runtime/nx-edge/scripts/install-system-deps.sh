#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $EUID -ne 0 ]]; then
  exec sudo "$0" "$@"
fi

mapfile -t APT_PACKAGES < <(grep -Ev '^[[:space:]]*(#|$)' "$ROOT/install/apt-packages.txt")
mapfile -t ROS_PACKAGES < <(grep -Ev '^[[:space:]]*(#|$)' "$ROOT/install/ros-packages.txt")
apt-get update
apt-get install -y "${APT_PACKAGES[@]}" "${ROS_PACKAGES[@]}"
rosdep init 2>/dev/null || true
sudo -u dogrobot rosdep update
sudo -u dogrobot rosdep install --from-paths /home/dogrobot/robot/src --ignore-src -r -y --rosdistro humble
sudo -u dogrobot -H python3 -m pip install --user -r "$ROOT/install/python-requirements.txt"

