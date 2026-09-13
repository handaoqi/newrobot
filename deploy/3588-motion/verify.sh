#!/usr/bin/env bash
set -euo pipefail

HOST="${ROAMERX_3588_HOST:-3588}"
REMOTE_ROOT="${ROAMERX_3588_RUNTIME:-/home/firefly/dogrobot-runtime}"
ssh "$HOST" "'$REMOTE_ROOT/scripts/verify.sh'"
