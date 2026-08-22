#!/usr/bin/env bash
set -euo pipefail

HOST="${ROAMERX_CLOUD_HOST:-cloud}"
REMOTE_RUNTIME="${ROAMERX_CLOUD_RUNTIME:-/opt/roamerx/runtime/platform}"

ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' preflight"
ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' status"
ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' verify"
