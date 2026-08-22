#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="${ROAMERX_CLOUD_HOST:-cloud}"
REMOTE_SOURCE="${ROAMERX_CLOUD_SOURCE:-/opt/roamerx/source}"
REMOTE_RUNTIME="${ROAMERX_CLOUD_RUNTIME:-/opt/roamerx/runtime/platform}"
START=false
DRY_RUN=false

usage() {
  cat <<'EOF'
Usage: deploy/platform/deploy.sh [--start] [--dry-run]

Synchronizes platform source and runtime templates to the cloud host. It never
overwrites remote runtime data or credentials.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start) START=true ;;
    --dry-run) DRY_RUN=true ;;
    --help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

run() {
  printf '+ %s\n' "$*"
  if ! "$DRY_RUN"; then
    "$@"
  fi
}

if "$DRY_RUN"; then
  echo "Platform deployment dry-run: host=$HOST source=$REMOTE_SOURCE runtime=$REMOTE_RUNTIME start=$START"
  exit 0
fi

run ssh "$HOST" "mkdir -p '$REMOTE_SOURCE/platform' '$REMOTE_RUNTIME'"
run rsync -a --delete \
  --exclude='node_modules/' --exclude='dist/' --exclude='__pycache__/' \
  "$REPO_ROOT/platform/backend/" "$HOST:$REMOTE_SOURCE/platform/backend/"
run rsync -a --delete \
  --exclude='node_modules/' --exclude='dist/' \
  "$REPO_ROOT/platform/frontend/" "$HOST:$REMOTE_SOURCE/platform/frontend/"
run rsync -a --delete \
  --exclude='data/' --exclude='conf/platform.env' --exclude='conf/mosquitto.conf' \
  --exclude='conf/mosquitto.passwd' \
  "$REPO_ROOT/runtime/platform/" "$HOST:$REMOTE_RUNTIME/"
run ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' init"
run ssh "$HOST" "sed -i 's#^ROAMERX_SOURCE_ROOT=.*#ROAMERX_SOURCE_ROOT=$REMOTE_SOURCE#' '$REMOTE_RUNTIME/conf/platform.env'"

if "$START"; then
  run ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' up"
  run ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' verify"
else
  echo "Synced cloud runtime. Review $HOST:$REMOTE_RUNTIME/conf/platform.env, prepare the MQTT password, then rerun with --start."
fi
