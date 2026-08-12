#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export BIKE_BOT_DEVICE_ID="ZSL-1A-07-audio-test"
exec /usr/bin/python3 run_edge.py --config config.test.yaml
