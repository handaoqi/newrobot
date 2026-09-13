#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="${ROAMERX_3588_HOST:-3588}"
REMOTE_ROOT="${ROAMERX_3588_RUNTIME:-/home/firefly/dogrobot-runtime}"
DRY_RUN=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=true ;;
    --help) echo "Usage: deploy/3588-motion/deploy.sh [--dry-run]"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

if "$DRY_RUN"; then
  echo "3588 deployment dry-run: host=$HOST runtime=$REMOTE_ROOT"
  exit 0
fi

ssh "$HOST" "mkdir -p '$REMOTE_ROOT'/{bin,data,conf,scripts,docs,install}"
rsync -a --delete "$REPO_ROOT/runtime/3588-motion/bin/" "$HOST:$REMOTE_ROOT/bin/"
rsync -a "$REPO_ROOT/runtime/3588-motion/scripts/" "$HOST:$REMOTE_ROOT/scripts/"
rsync -a "$REPO_ROOT/runtime/3588-motion/docs/" "$HOST:$REMOTE_ROOT/docs/"
rsync -a "$REPO_ROOT/runtime/3588-motion/install/charge-pile/" "$HOST:$REMOTE_ROOT/install/charge-pile/"
rsync -a "$REPO_ROOT/edge-agent/tools/leg_power/" "$HOST:$REMOTE_ROOT/install/leg-power/"
ssh "$HOST" "chmod +x '$REMOTE_ROOT/bin/motionctl'; test -x '$REMOTE_ROOT/install/charge-pile/dog_send_three_states/dog_status_unknown'"
echo "Deployed managed 3588 runtime to $HOST:$REMOTE_ROOT"
