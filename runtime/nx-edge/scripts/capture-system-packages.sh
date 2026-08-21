#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/install/installed-system-packages.lock"
PATTERN='^(ros-humble-|nvidia-|cuda-|ecal$|tensorrt|libnvinfer|libprotobuf|protobuf-|alsa-utils$|bluez$|build-essential$|chrony$|cmake$|curl$|ffmpeg$|g\+\+$|gcc$|git$|graphicsmagick$|jq$|libboost|libeigen3-dev$|libgraphicsmagick|libomp|libompl|libopencv|libpcl|libsdl2|libusb|libyaml-cpp|make$|ninja-build$|openssh-client$|pkg-config$|pulseaudio-utils$|python3($|-)|rsync$|sqlite3$)'
dpkg-query -W -f='${Package}\t${Version}\t${Architecture}\n' | grep -E "$PATTERN" | sort > "$OUT"
echo "$OUT"
