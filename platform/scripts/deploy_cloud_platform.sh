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
  # Publish the build as a mirror so stale content-hashed bundles cannot be
  # selected by an old cached index or left behind after a build changes.
  rsync -a --delete "$PROJECT_DIR/frontend/dist/" "$CLOUD_HOST:$REMOTE_ROOT/frontend/dist/"
  # The build workspace may use restrictive file permissions. Nginx must be
  # able to traverse the directory and read every static asset after syncing.
  ssh "$CLOUD_HOST" "chmod -R a+rX '$REMOTE_ROOT/frontend/dist'"
  local_index_sha="$(sha256sum "$PROJECT_DIR/frontend/dist/index.html" | awk '{print $1}')"
  remote_index_sha="$(ssh "$CLOUD_HOST" "sha256sum '$REMOTE_ROOT/frontend/dist/index.html'" | awk '{print $1}')"
  if [[ -z "$local_index_sha" || "$local_index_sha" != "$remote_index_sha" ]]; then
    echo "[deploy] Frontend index verification failed (local=$local_index_sha remote=$remote_index_sha)" >&2
    exit 1
  fi
  echo "[deploy] Frontend index verified: $local_index_sha"
  public_index_sha="$(curl --noproxy '*' --fail --silent --show-error --max-time 15 "$BASE_URL/" | sha256sum | awk '{print $1}')"
  if [[ -z "$public_index_sha" || "$local_index_sha" != "$public_index_sha" ]]; then
    echo "[deploy] Public frontend verification failed (local=$local_index_sha public=$public_index_sha url=$BASE_URL/)" >&2
    exit 1
  fi
  echo "[deploy] Public frontend verified: $public_index_sha"
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
