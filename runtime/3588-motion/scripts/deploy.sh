#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "$ROOT/../.." && pwd)"
HOST="${ROAMERX_3588_HOST:-3588}"
REMOTE_ROOT="${ROAMERX_3588_RUNTIME:-/home/firefly/dogrobot-runtime}"

ssh "$HOST" "mkdir -p '$REMOTE_ROOT'/{bin,data,conf,scripts,docs,install}"
rsync -a --delete "$ROOT/bin/" "$HOST:$REMOTE_ROOT/bin/"
rsync -a "$ROOT/scripts/" "$HOST:$REMOTE_ROOT/scripts/"
rsync -a "$ROOT/docs/" "$HOST:$REMOTE_ROOT/docs/"
rsync -a "$ROOT/install/charge-pile/" "$HOST:$REMOTE_ROOT/install/charge-pile/"
rsync -a "$REPO_ROOT/edge-agent/tools/leg_power/" "$HOST:$REMOTE_ROOT/install/leg-power/"
ssh "$HOST" "chmod +x '$REMOTE_ROOT/bin/motionctl'; test -x '$REMOTE_ROOT/install/charge-pile/dog_send_three_states/dog_status_unknown'"
echo "Deployed managed 3588 runtime to $HOST:$REMOTE_ROOT"
