#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROBOT_HOST="${ROAMERX_ROBOT_HOST:-}"
ROBOT_PROJECT_DIR="${ROAMERX_ROBOT_PROJECT_DIR:-/home/dogrobot/robot}"
EDGE_RUNTIME_DIR="${ROAMERX_EDGE_RUNTIME_DIR:-/home/dogrobot/runtime/nx-edge}"
EDGE_RELEASES_DIR="${ROAMERX_EDGE_RELEASES_DIR:-$EDGE_RUNTIME_DIR/releases}"
EDGE_CURRENT_LINK="${ROAMERX_EDGE_CURRENT_LINK:-$EDGE_RUNTIME_DIR/current-edge}"
TARGET_REPO_ROOT="$(dirname "$ROBOT_PROJECT_DIR")"
BUILD=false
INSTALL_SERVICE=false
INSTALL_EDGE_SERVICE=false
INIT_SYSTEM_DEPS=false
RESTART_EDGE=false
DRY_RUN=false

usage() {
  cat <<'EOF'
Usage: deploy/nx-edge/deploy.sh [--host user@robot] [--init-system-deps] [--build] [--install-edge-service] [--install-service] [--restart-edge] [--dry-run]

Runtime configuration, state, maps, build outputs and logs are preserved.
--init-system-deps installs and verifies required NX runtime packages.
--install-service includes --init-system-deps automatically.
--install-edge-service updates only the Edge Agent systemd unit.
--restart-edge restarts the Edge service only when no local task is active.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host) ROBOT_HOST="${2:?missing host}"; shift ;;
    --init-system-deps) INIT_SYSTEM_DEPS=true ;;
    --build) BUILD=true ;;
    --install-edge-service) INSTALL_EDGE_SERVICE=true ;;
    --install-service) INSTALL_SERVICE=true; INSTALL_EDGE_SERVICE=true; INIT_SYSTEM_DEPS=true ;;
    --restart-edge) RESTART_EDGE=true ;;
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
  echo "NX deployment dry-run: host=${ROBOT_HOST:-local} project=$ROBOT_PROJECT_DIR edge_current=$EDGE_CURRENT_LINK runtime=$EDGE_RUNTIME_DIR init_system_deps=$INIT_SYSTEM_DEPS build=$BUILD install_edge_service=$INSTALL_EDGE_SERVICE install_service=$INSTALL_SERVICE restart_edge=$RESTART_EDGE"
  exit 0
fi

if [[ -n "$(git -C "$REPO_ROOT" status --porcelain --untracked-files=all -- edge-agent)" ]]; then
  echo "Refusing Edge deployment from a dirty edge-agent tree; deploy a committed worktree" >&2
  exit 73
fi
EDGE_RELEASE_ID="${ROAMERX_EDGE_RELEASE_ID:-$(git -C "$REPO_ROOT" rev-parse --short=12 HEAD)}"
EDGE_RELEASE_DIR="$EDGE_RELEASES_DIR/$EDGE_RELEASE_ID"

remote_exec "mkdir -p '$ROBOT_PROJECT_DIR' '$EDGE_RELEASE_DIR/edge-agent' '$EDGE_RUNTIME_DIR/data/edge-agent' '$EDGE_RUNTIME_DIR/conf'"

if "$INIT_SYSTEM_DEPS"; then
  rsync -a "$REPO_ROOT/deploy/nx-edge/init-system-deps.sh" "$(target_path /tmp/roamerx-init-nx-system-deps.sh)"
  remote_exec "chmod 0755 /tmp/roamerx-init-nx-system-deps.sh && /tmp/roamerx-init-nx-system-deps.sh"
fi

if [[ -n "$ROBOT_HOST" || "$(readlink -f "$REPO_ROOT/robot")" != "$(readlink -f "$ROBOT_PROJECT_DIR")" ]]; then
  rsync -a --exclude='build/' --exclude='install/' --exclude='log/' \
    "$REPO_ROOT/robot/" "$(target_path "$ROBOT_PROJECT_DIR/")"
fi
rsync -a --delete \
  --exclude='__pycache__/' --exclude='*.pyc' \
  --exclude='config.yaml' --exclude='data/' --exclude='*.log' \
  "$REPO_ROOT/edge-agent/" "$(target_path "$EDGE_RELEASE_DIR/edge-agent/")"

PREVIOUS_EDGE_RELEASE="$(remote_exec "readlink -f '$EDGE_CURRENT_LINK' 2>/dev/null || true")"
remote_exec "candidate='$EDGE_RUNTIME_DIR/.current-edge-$EDGE_RELEASE_ID'; ln -sfn '$EDGE_RELEASE_DIR' \"\$candidate\"; mv -Tf \"\$candidate\" '$EDGE_CURRENT_LINK'"

if "$BUILD"; then remote_exec "cd '$ROBOT_PROJECT_DIR' && ./build.sh all"; fi

if "$INSTALL_EDGE_SERVICE"; then
  rsync -a "$REPO_ROOT/edge-agent/systemd/roamerx-edge-agent.service" \
    "$(target_path /tmp/roamerx-edge-agent.service)"
  remote_exec "sudo install -m 0644 /tmp/roamerx-edge-agent.service /etc/systemd/system/roamerx-edge-agent.service && sudo rm -f /run/systemd/system/roamerx-edge-agent.service.d/recovery-source.conf && sudo systemctl daemon-reload && sudo systemctl enable roamerx-edge-agent.service"
fi

if "$INSTALL_SERVICE"; then
  rsync -a "$REPO_ROOT/platform/deploy/install_jetson_onnxruntime.sh" "$(target_path /tmp/roamerx-install-vision-runtime.sh)"
  remote_exec "chmod 0755 /tmp/roamerx-install-vision-runtime.sh && /tmp/roamerx-install-vision-runtime.sh"
  systemd_sources=(
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

remote_exec "python3 '$EDGE_CURRENT_LINK/edge-agent/tools/write_mapping_deployment_manifest.py' --repo '$TARGET_REPO_ROOT' --output '$EDGE_RUNTIME_DIR/conf/mapping-deployment.json'"

if "$RESTART_EDGE"; then
  if ! remote_exec "EDGE_DB_PATH='$EDGE_RUNTIME_DIR/data/edge-agent/edge.db' bash '$EDGE_CURRENT_LINK/edge-agent/tools/restart-edge-agent-safely' && sleep 3 && systemctl is-active --quiet roamerx-edge-agent.service && main_pid=\$(systemctl show roamerx-edge-agent.service -p MainPID --value) && test \"\$(readlink -f /proc/\$main_pid/cwd)\" = \"\$(readlink -f '$EDGE_CURRENT_LINK/edge-agent')\""; then
    if [[ -n "$PREVIOUS_EDGE_RELEASE" ]]; then
      echo "Edge release health check failed; rolling back to $PREVIOUS_EDGE_RELEASE" >&2
      remote_exec "candidate='$EDGE_RUNTIME_DIR/.current-edge-rollback'; ln -sfn '$PREVIOUS_EDGE_RELEASE' \"\$candidate\"; mv -Tf \"\$candidate\" '$EDGE_CURRENT_LINK'; sudo systemctl restart roamerx-edge-agent.service"
    fi
    exit 76
  fi
fi

echo "NX code deployed to ${ROBOT_HOST:+$ROBOT_HOST:}$ROBOT_PROJECT_DIR (edge release $EDGE_RELEASE_ID)"
