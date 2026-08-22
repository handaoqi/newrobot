#!/usr/bin/env bash
set -euo pipefail

BACKUP="${1:?Usage: deploy/backup/verify-backup.sh BACKUP_DIR}"
[[ -d "$BACKUP" ]] || { echo "Backup directory not found: $BACKUP" >&2; exit 2; }
[[ -s "$BACKUP/SHA256SUMS" ]] || { echo "Missing SHA256SUMS" >&2; exit 1; }
(cd "$BACKUP" && sha256sum -c SHA256SUMS)
for required in manifest.tsv platform nx-edge 3588 restore-check.sh; do
  [[ -e "$BACKUP/$required" ]] || { echo "Missing backup item: $required" >&2; exit 1; }
done
if command -v sqlite3 >/dev/null; then
  while IFS= read -r db; do
    [[ "$(sqlite3 "$db" 'PRAGMA integrity_check;')" == "ok" ]] || {
      echo "SQLite integrity check failed: $db" >&2
      exit 1
    }
  done < <(find "$BACKUP" -type f \( -name '*.sqlite3' -o -name '*.db' \))
fi
echo "Backup verified: $BACKUP"
