#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set -a
# shellcheck disable=SC1090
source "$ROOT/conf/platform.env"
set +a
docker compose --env-file "$ROOT/conf/platform.env" -f "$ROOT/compose.yaml" ps
curl --fail --silent --show-error --max-time 15 \
  "http://127.0.0.1:${PLATFORM_HTTP_PORT:-8088}/api/maps/" >/dev/null
echo "Platform runtime is healthy."

