#!/usr/bin/env bash
set -euo pipefail

BACKUP="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

cp -a "$BACKUP/platform" "$TMP/platform"
cp -a "$BACKUP/nx-edge" "$TMP/nx-edge"
cp -a "$BACKUP/3588" "$TMP/3588"

if command -v sqlite3 >/dev/null; then
  while IFS= read -r db; do
    [[ "$(sqlite3 "$db" 'PRAGMA integrity_check;')" == "ok" ]] || {
      echo "Restored SQLite check failed: $db" >&2
      exit 1
    }
  done < <(find "$TMP" -type f \( -name '*.sqlite3' -o -name '*.db' \))
fi
echo "Restore check passed in temporary directory: $TMP"
