#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
protoc --cpp_out=. power_mcu.proto
g++ -std=c++17 -O2 leg_power.cpp power_mcu.pb.cc -o roamerx-leg-power \
  -lecal_core -lprotobuf -lpthread
sudo install -m 0755 roamerx-leg-power /usr/local/bin/roamerx-leg-power
