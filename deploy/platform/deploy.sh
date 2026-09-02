#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="${ROAMERX_CLOUD_HOST:-cloud}"
REMOTE_SOURCE="${ROAMERX_CLOUD_SOURCE:-/opt/roamerx/source}"
REMOTE_RUNTIME="${ROAMERX_CLOUD_RUNTIME:-/opt/roamerx/runtime/platform}"
LOCAL_LICHTBLICK_DIST="${ROAMERX_LICHTBLICK_DIST:-${REPO_ROOT}/runtime/platform/install/lichtblick-web/dist}"
LOCAL_LICHTBLICK_LAYOUT="${ROAMERX_LICHTBLICK_LAYOUT:-${REPO_ROOT}/docs/yuwang/embedded_scene_layout.json}"
PREPARE_LICHTBLICK_INDEX="${REPO_ROOT}/robot/script/robot/inject_lichtblick_layout.py"
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
if [[ ! -d "$LOCAL_LICHTBLICK_DIST" && -d "$REPO_ROOT/runtime/nx-edge/install/lichtblick-web/dist" ]]; then
  LOCAL_LICHTBLICK_DIST="$REPO_ROOT/runtime/nx-edge/install/lichtblick-web/dist"
fi
if [[ -d "$LOCAL_LICHTBLICK_DIST" ]]; then
  prepared_lichtblick_index=""
  if [[ -f "$LOCAL_LICHTBLICK_LAYOUT" ]]; then
    prepared_lichtblick_index="$(mktemp)"
    python3 "$PREPARE_LICHTBLICK_INDEX" \
      --dist "$LOCAL_LICHTBLICK_DIST" \
      --layout "$LOCAL_LICHTBLICK_LAYOUT" \
      --output "$prepared_lichtblick_index"
  fi
  run rsync -a --delete "$LOCAL_LICHTBLICK_DIST/" \
    "$HOST:$REMOTE_RUNTIME/install/lichtblick-web/dist/"
  if [[ -n "$prepared_lichtblick_index" ]]; then
    run rsync -a "$prepared_lichtblick_index" \
      "$HOST:$REMOTE_RUNTIME/install/lichtblick-web/dist/index.html"
    rm -f "$prepared_lichtblick_index"
  else
    echo "Warning: default layout not found at $LOCAL_LICHTBLICK_LAYOUT; /foxglove/ will open without the embedded scene layout." >&2
  fi
else
  echo "Warning: Lichtblick bundle not found at $LOCAL_LICHTBLICK_DIST; /foxglove/ will remain unavailable." >&2
fi
run ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' init"
run ssh "$HOST" "sed -i 's#^ROAMERX_SOURCE_ROOT=.*#ROAMERX_SOURCE_ROOT=$REMOTE_SOURCE#' '$REMOTE_RUNTIME/conf/platform.env'"

if "$START"; then
  run ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' up"
  run ssh "$HOST" "'$REMOTE_RUNTIME/bin/platformctl' verify"
else
  echo "Synced cloud runtime. Review $HOST:$REMOTE_RUNTIME/conf/platform.env, prepare the MQTT password, then rerun with --start."
fi
