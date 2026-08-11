#!/usr/bin/env bash
# Build locally and deploy the current platform to the configured cloud host.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLOUD_HOST="${ROAMERX_CLOUD_HOST:-cloud}"
REMOTE_ROOT="${ROAMERX_CLOUD_ROOT:-/opt/roamerx/current}"
BASE_URL="${ROAMERX_CLOUD_URL:-http://39.107.250.69:8088}"
DEPLOY_FRONTEND=true
DEPLOY_BACKEND=true

usage() {
  cat <<'EOF'
Usage: scripts/deploy_cloud_platform.sh [options]

Build locally, sync the platform to the cloud, then verify health.

Options:
  --frontend-only  Build and publish frontend/dist only.
  --backend-only   Publish backend only and restart API/worker.
  --help           Show this help.

Environment overrides:
  ROAMERX_CLOUD_HOST, ROAMERX_CLOUD_ROOT, ROAMERX_CLOUD_URL
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --frontend-only) DEPLOY_BACKEND=false ;;
    --backend-only) DEPLOY_FRONTEND=false ;;
    --help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if ! ssh -o BatchMode=yes -o ConnectTimeout=10 "$CLOUD_HOST" "test -d '$REMOTE_ROOT'"; then
  echo "Cloud host or runtime path is unavailable: $CLOUD_HOST:$REMOTE_ROOT" >&2
  exit 1
fi

if "$DEPLOY_FRONTEND"; then
  echo "[deploy] Building frontend locally..."
  (cd "$PROJECT_DIR/frontend" && npm run build)
  echo "[deploy] Syncing frontend dist..."
  rsync -a "$PROJECT_DIR/frontend/dist/" "$CLOUD_HOST:$REMOTE_ROOT/frontend/dist/"
fi

if "$DEPLOY_BACKEND"; then
  echo "[deploy] Syncing backend source..."
  rsync -a \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    --exclude='data/' \
    "$PROJECT_DIR/backend/" "$CLOUD_HOST:$REMOTE_ROOT/backend/"
  echo "[deploy] Applying database migrations..."
  ssh "$CLOUD_HOST" "set -a; source /opt/roamerx/shared/center.env; set +a; cd '$REMOTE_ROOT' && /root/miniconda/envs/py310/bin/python backend/manage.py migrate --noinput"
  echo "[deploy] Restarting cloud API and device worker..."
  ssh "$CLOUD_HOST" "systemctl restart roamerx-center-api.service roamerx-device-worker.service && sleep 3 && systemctl is-active roamerx-center-api.service roamerx-device-worker.service"
fi

echo "[deploy] Checking cloud API..."
curl --noproxy '*' --fail --silent --show-error --max-time 15 "$BASE_URL/api/maps/" >/dev/null
echo "[deploy] Complete: $BASE_URL"
