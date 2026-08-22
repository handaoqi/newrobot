#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROBOT_HOST="${ROAMERX_ROBOT_HOST:-}"
ROBOT_PROJECT_DIR="${ROAMERX_ROBOT_PROJECT_DIR:-/home/dogrobot/robot}"
EDGE_SOURCE_DIR="${ROAMERX_EDGE_SOURCE_DIR:-/home/dogrobot/edge-agent}"
EDGE_RUNTIME_DIR="${ROAMERX_EDGE_RUNTIME_DIR:-/home/dogrobot/runtime/nx-edge}"
BUILD=false
INSTALL_SERVICE=false
DRY_RUN=false

usage() {
  cat <<'EOF'
Usage: deploy/nx-edge/deploy.sh [--host user@robot] [--build] [--install-service] [--dry-run]

Runtime configuration, state, maps, build outputs and logs are preserved.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host) ROBOT_HOST="${2:?missing host}"; shift ;;
    --build) BUILD=true ;;
    --install-service) INSTALL_SERVICE=true ;;
    --dry-run) DRY_RUN=true ;;
    --help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

remote_exec() {
  if [[ -n "$ROBOT_HOST" ]]; then
    if "$DRY_RUN"; then printf '+ ssh %s %s\n' "$ROBOT_HOST" "$*"; else ssh "$ROBOT_HOST" "$@"; fi
  elif "$DRY_RUN"; then
    printf '+ bash -lc %s\n' "$*"
  else
    bash -lc "$*"
  fi
}

target_path() {
  if [[ -n "$ROBOT_HOST" ]]; then printf '%s:%s' "$ROBOT_HOST" "$1"; else printf '%s' "$1"; fi
}

if "$DRY_RUN"; then
  echo "NX deployment dry-run: host=${ROBOT_HOST:-local} project=$ROBOT_PROJECT_DIR edge=$EDGE_SOURCE_DIR runtime=$EDGE_RUNTIME_DIR build=$BUILD install_service=$INSTALL_SERVICE"
  exit 0
fi

remote_exec "mkdir -p '$ROBOT_PROJECT_DIR' '$EDGE_SOURCE_DIR' '$EDGE_RUNTIME_DIR/data/edge-agent' '$EDGE_RUNTIME_DIR/conf'"
if [[ -n "$ROBOT_HOST" || "$(readlink -f "$REPO_ROOT/robot")" != "$(readlink -f "$ROBOT_PROJECT_DIR")" ]]; then
  rsync -a --exclude='build/' --exclude='install/' --exclude='log/' \
    "$REPO_ROOT/robot/" "$(target_path "$ROBOT_PROJECT_DIR/")"
fi
if [[ -n "$ROBOT_HOST" || "$(readlink -f "$REPO_ROOT/edge-agent")" != "$(readlink -f "$EDGE_SOURCE_DIR")" ]]; then
  rsync -a --exclude='config.yaml' --exclude='data/' --exclude='*.log' \
    "$REPO_ROOT/edge-agent/" "$(target_path "$EDGE_SOURCE_DIR/")"
fi

if "$BUILD"; then remote_exec "cd '$ROBOT_PROJECT_DIR' && ./build.sh all"; fi

if "$INSTALL_SERVICE"; then
  systemd_sources=(
    edge-agent/systemd/roamerx-edge-agent.service
    edge-agent/systemd/roamerx-teleop-bridge.service
    edge-agent/systemd/roamerx-mapping.service
    edge-agent/systemd/roamerx-zenoh.service
    edge-agent/systemd/roamerx-5g-share.service
    dev-agent/systemd/roamerx-dev-agent.service
    dev-agent/systemd/roamerx-local-asr.service
    platform/bot-version-test/systemd/roamerx-bike-bot.service
    robot/systemd/roamerx-robot-mcp.service
  )
  for source in "${systemd_sources[@]}"; do
    filename="$(basename "$source")"
    rsync -a "$REPO_ROOT/$source" "$(target_path "/tmp/$filename")"
    remote_exec "sudo install -m 0644 '/tmp/$filename' '/etc/systemd/system/$filename'"
  done
  rsync -a "$REPO_ROOT/edge-agent/tools/roamerx-5g-share" "$(target_path /tmp/roamerx-5g-share)"
  remote_exec "sudo install -m 0755 /tmp/roamerx-5g-share /usr/local/sbin/roamerx-5g-share && sudo systemctl daemon-reload && sudo systemctl enable roamerx-edge-agent roamerx-teleop-bridge roamerx-dev-agent roamerx-local-asr roamerx-bike-bot roamerx-robot-mcp roamerx-zenoh roamerx-5g-share"
fi

echo "NX code deployed to ${ROBOT_HOST:+$ROBOT_HOST:}$ROBOT_PROJECT_DIR"
