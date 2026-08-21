#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROBOT_HOST="${ROAMERX_ROBOT_HOST:-}"
ROBOT_PROJECT_DIR="${ROAMERX_ROBOT_PROJECT_DIR:-/home/dogrobot/robot}"
EDGE_SOURCE_DIR="${ROAMERX_EDGE_SOURCE_DIR:-/home/dogrobot/edge-agent}"
EDGE_RUNTIME_DIR="${ROAMERX_EDGE_RUNTIME_DIR:-/home/robot/edge_agent}"
BUILD=false
INSTALL_SERVICE=false

usage() {
  cat <<'EOF'
Usage: deploy/robot/deploy.sh [--host user@robot] [--build] [--install-service]

Without --host, files are installed on the current machine. Runtime config,
state, maps, build outputs, and logs are preserved.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host) ROBOT_HOST="${2:?missing host}"; shift ;;
    --build) BUILD=true ;;
    --install-service) INSTALL_SERVICE=true ;;
    --help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

remote_exec() {
  if [[ -n "$ROBOT_HOST" ]]; then
    ssh "$ROBOT_HOST" "$@"
  else
    bash -lc "$*"
  fi
}

target_path() {
  if [[ -n "$ROBOT_HOST" ]]; then
    printf '%s:%s' "$ROBOT_HOST" "$1"
  else
    printf '%s' "$1"
  fi
}

remote_exec "mkdir -p '$ROBOT_PROJECT_DIR' '$EDGE_SOURCE_DIR' '$EDGE_RUNTIME_DIR'"
if [[ -n "$ROBOT_HOST" || "$(readlink -f "$REPO_ROOT/robot")" != "$(readlink -f "$ROBOT_PROJECT_DIR")" ]]; then
  rsync -a \
    --exclude='build/' --exclude='install/' --exclude='log/' \
    "$REPO_ROOT/robot/" "$(target_path "$ROBOT_PROJECT_DIR/")"
fi
if [[ -n "$ROBOT_HOST" || "$(readlink -f "$REPO_ROOT/edge-agent")" != "$(readlink -f "$EDGE_SOURCE_DIR")" ]]; then
  rsync -a \
    --exclude='config.yaml' --exclude='data/' --exclude='*.log' \
    "$REPO_ROOT/edge-agent/" "$(target_path "$EDGE_SOURCE_DIR/")"
fi

if "$BUILD"; then
  remote_exec "cd '$ROBOT_PROJECT_DIR' && ./build.sh all"
fi

if "$INSTALL_SERVICE"; then
  remote_exec "sudo install -m 0644 '$EDGE_SOURCE_DIR/systemd/roamerx-edge-agent.service' /etc/systemd/system/roamerx-edge-agent.service && sudo systemctl daemon-reload && sudo systemctl enable --now roamerx-edge-agent.service"
fi

echo "Robot code deployed to ${ROBOT_HOST:+$ROBOT_HOST:}$ROBOT_PROJECT_DIR"
