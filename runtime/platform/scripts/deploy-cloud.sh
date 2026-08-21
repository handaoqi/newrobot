#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HOST="${ROAMERX_CLOUD_HOST:-cloud}"
REMOTE_SOURCE="${ROAMERX_CLOUD_SOURCE:-/opt/roamerx/source}"
REMOTE_RUNTIME="${ROAMERX_CLOUD_RUNTIME:-/opt/roamerx/runtime/platform}"
START=false

if [[ "${1:-}" == "--start" ]]; then
  START=true
elif [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--start]" >&2
  exit 2
fi

ssh "$HOST" "mkdir -p '$REMOTE_SOURCE/platform' '$REMOTE_RUNTIME'"
rsync -a --delete \
  --exclude='node_modules/' --exclude='dist/' --exclude='__pycache__/' \
  "$REPO_ROOT/platform/backend/" "$HOST:$REMOTE_SOURCE/platform/backend/"
rsync -a --delete \
  --exclude='node_modules/' --exclude='dist/' \
  "$REPO_ROOT/platform/frontend/" "$HOST:$REMOTE_SOURCE/platform/frontend/"
rsync -a --delete \
  --exclude='data/' --exclude='conf/platform.env' --exclude='conf/mosquitto.conf' \
  --exclude='conf/mosquitto.passwd' \
  "$REPO_ROOT/runtime/platform/" "$HOST:$REMOTE_RUNTIME/"

ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' init"
ssh "$HOST" "sed -i 's#^ROAMERX_SOURCE_ROOT=.*#ROAMERX_SOURCE_ROOT=$REMOTE_SOURCE#' '$REMOTE_RUNTIME/conf/platform.env'"
if "$START"; then
  ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' up; '$REMOTE_RUNTIME/bin/platformctl' verify"
else
  echo "Synced cloud runtime. Review $HOST:$REMOTE_RUNTIME/conf/platform.env, prepare the MQTT password, then run with --start."
fi
