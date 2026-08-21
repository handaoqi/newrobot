#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
DEST="${1:-$ROOT/data/backups/platform-$STAMP.tar.gz}"
mkdir -p "$(dirname "$DEST")"
tar -C "$ROOT" -czf "$DEST" data/backend conf/platform.env conf/mosquitto.passwd
echo "$DEST"

