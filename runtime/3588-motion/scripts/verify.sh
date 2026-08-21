#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROAMERX_3588_RUNTIME:-/home/firefly/dogrobot-runtime}"
for dir in bin data conf scripts docs install; do test -d "$ROOT/$dir"; done
test -x "$ROOT/bin/motionctl"
test -x "$ROOT/install/charge-pile/dog_send_three_states/dog_status_unknown"
robot-launch list

