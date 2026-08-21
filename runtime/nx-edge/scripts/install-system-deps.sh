#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $EUID -ne 0 ]]; then
  exec sudo "$0" "$@"
fi

mapfile -t APT_PACKAGES < <(grep -Ev '^[[:space:]]*(#|$)' "$ROOT/install/apt-packages.txt")
mapfile -t ROS_PACKAGES < <(grep -Ev '^[[:space:]]*(#|$)' "$ROOT/install/ros-packages.txt")

# Jetson images commonly provide an NVIDIA-patched OpenCV. A blind apt install
# can replace it with Ubuntu OpenCV and remove ROS/OpenCV development packages.
# Refuse that transaction unless the operator explicitly opts in.
if [[ -f /etc/nv_tegra_release ]] && [[ "${ROAMERX_ALLOW_JETSON_OPENCV_REPLACEMENT:-0}" != "1" ]]; then
  simulation="$(apt-get -s install "${APT_PACKAGES[@]}" "${ROS_PACKAGES[@]}" 2>&1 || true)"
  if grep -Eq '^Remv (libopencv|ros-humble-(cv-bridge|image-transport))' <<<"$simulation"; then
    echo "Refusing dependency install: apt would replace Jetson OpenCV/ROS packages." >&2
    echo "Use the JetPack-matched package set or set ROAMERX_ALLOW_JETSON_OPENCV_REPLACEMENT=1 after review." >&2
    exit 1
  fi
fi

apt-get update
apt-get install -y "${APT_PACKAGES[@]}" "${ROS_PACKAGES[@]}"
rosdep init 2>/dev/null || true
sudo -u dogrobot rosdep update
sudo -u dogrobot rosdep install --from-paths /home/dogrobot/robot/src --ignore-src -r -y --rosdistro humble
sudo -u dogrobot -H python3 -m pip install --user -r "$ROOT/install/python-requirements.txt"
