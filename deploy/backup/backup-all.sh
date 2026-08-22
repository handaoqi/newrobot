#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BACKUP_ROOT="${ROAMERX_BACKUP_ROOT:-$REPO_ROOT/backup}"
STAMP="$(date +%Y%m%d_%H%M%S)"
DEST="$BACKUP_ROOT/$STAMP"
THREE_EIGHTY_EIGHT_HOST="${ROAMERX_3588_HOST:-3588}"
THREE_EIGHTY_EIGHT_RUNTIME="${ROAMERX_3588_RUNTIME:-/home/firefly/dogrobot-runtime}"
DRY_RUN=false

usage() {
  cat <<'EOF'
Usage: deploy/backup/backup-all.sh [DESTINATION] [--dry-run]

Creates a recoverable configuration backup for platform, NX Edge and the
managed 3588 overlay. Large logs, rosbag/MCAP, caches, models and media are
excluded and recorded in the manifest.
EOF
}

POSITIONAL=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=true ;;
    --help) usage; exit 0 ;;
    -*) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    *) POSITIONAL+=("$1") ;;
  esac
  shift
done
if [[ ${#POSITIONAL[@]} -gt 1 ]]; then
  echo "Only one destination may be specified." >&2
  exit 2
fi
if [[ ${#POSITIONAL[@]} -eq 1 ]]; then DEST="${POSITIONAL[0]}"; fi

PLATFORM_RUNTIME="${ROAMERX_PLATFORM_RUNTIME:-$REPO_ROOT/runtime/platform}"
NX_RUNTIME="${ROAMERX_NX_RUNTIME:-$REPO_ROOT/runtime/nx-edge}"

if "$DRY_RUN"; then
  cat <<EOF
Backup dry-run
  destination: $DEST
  platform:    $PLATFORM_RUNTIME
  nx-edge:     $NX_RUNTIME
  3588:        $THREE_EIGHTY_EIGHT_HOST:$THREE_EIGHTY_EIGHT_RUNTIME
  excludes:    build/install/cache/log/rosbag/mcap/media/model archives
EOF
  exit 0
fi

case "$DEST" in
  "$BACKUP_ROOT"/*) ;;
  *) echo "Backup destination must be under $BACKUP_ROOT" >&2; exit 2 ;;
esac
if [[ -e "$DEST" ]]; then
  echo "Backup destination already exists: $DEST" >&2
  exit 1
fi
install -d -m 0700 "$DEST" "$DEST/platform" "$DEST/nx-edge" "$DEST/3588"

copy_path() {
  local source="$1" target="$2"
  if [[ -e "$source" || -L "$source" ]]; then
    install -d "$(dirname "$target")"
    if [[ -d "$source" && ! -L "$source" ]]; then
      rsync -a --exclude='*.log' --exclude='*.mcap' --exclude='*.bag' --exclude='*.db3' "$source/" "$target/"
    else
      rsync -a "$source" "$target"
    fi
  else
    printf 'missing\t%s\n' "$source" >> "$DEST/manifest-missing.tsv"
  fi
}

copy_path "$PLATFORM_RUNTIME/data/backend/db" "$DEST/platform/data/backend/db"
copy_path "$PLATFORM_RUNTIME/conf/platform.env" "$DEST/platform/conf/platform.env"
copy_path "$PLATFORM_RUNTIME/conf/mosquitto.conf" "$DEST/platform/conf/mosquitto.conf"
copy_path "$PLATFORM_RUNTIME/conf/mosquitto.passwd" "$DEST/platform/conf/mosquitto.passwd"
copy_path "$PLATFORM_RUNTIME/compose.yaml" "$DEST/platform/compose.yaml"

copy_path "$NX_RUNTIME/conf" "$DEST/nx-edge/conf"
copy_path "$NX_RUNTIME/data/edge-agent" "$DEST/nx-edge/data/edge-agent"
copy_path "$NX_RUNTIME/data/jszr" "$DEST/nx-edge/data/jszr"
copy_path "$NX_RUNTIME/data/robot-state" "$DEST/nx-edge/data/robot-state"
copy_path "$NX_RUNTIME/data/ros-home/.config" "$DEST/nx-edge/data/ros-home/.config"

ssh "$THREE_EIGHTY_EIGHT_HOST" "test -d '$THREE_EIGHTY_EIGHT_RUNTIME'" \
  && rsync -a --exclude='*.log' --exclude='*.mcap' --exclude='*.bag' \
    "$THREE_EIGHTY_EIGHT_HOST:$THREE_EIGHTY_EIGHT_RUNTIME/conf/" "$DEST/3588/runtime/conf/" \
  && rsync -a "$THREE_EIGHTY_EIGHT_HOST:$THREE_EIGHTY_EIGHT_RUNTIME/data/" "$DEST/3588/runtime/data/" \
  && ssh "$THREE_EIGHTY_EIGHT_HOST" "robot-launch list; if [[ -f /var/lib/roamerx-charge-pile/state ]]; then cat /var/lib/roamerx-charge-pile/state; fi" > "$DEST/3588/runtime-state.txt"

{
  printf 'created_at\t%s\n' "$(date --iso-8601=seconds)"
  printf 'scope\trecoverable configuration, databases, maps and state\n'
  printf 'excluded\tlogs, rosbag/MCAP, caches, models, media and build artifacts\n'
  printf '3588_host\t%s\n' "$THREE_EIGHTY_EIGHT_HOST"
  printf '3588_runtime\t%s\n' "$THREE_EIGHTY_EIGHT_RUNTIME"
} > "$DEST/manifest.tsv"

find "$DEST" -type f -not -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > "$DEST/SHA256SUMS"
chmod 0600 "$DEST/SHA256SUMS" "$DEST/manifest.tsv" "$DEST/3588/runtime-state.txt" 2>/dev/null || true
cp "$REPO_ROOT/deploy/backup/restore-check.sh" "$DEST/restore-check.sh"
chmod 0700 "$DEST/restore-check.sh"
echo "Created three-endpoint backup: $DEST"
