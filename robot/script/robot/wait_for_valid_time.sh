#!/usr/bin/env bash
set -euo pipefail

MIN_YEAR="${ROAMERX_MIN_VALID_YEAR:-2026}"
TIME_SYNC_TIMEOUT_SECONDS="${ROAMERX_TIME_SYNC_TIMEOUT_SECONDS:-90}"

log() {
  echo "[time-check] $*"
}

system_year() {
  date +%Y 2>/dev/null || echo 1970
}

chrony_synced() {
  if ! command -v chronyc >/dev/null 2>&1; then
    timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -qx "yes"
    return
  fi

  chronyc tracking 2>/dev/null | awk -F: '
    /Leap status/ {
      gsub(/^[ \t]+|[ \t]+$/, "", $2)
      if ($2 == "Normal") found=1
    }
    END { exit found ? 0 : 1 }
  '
}

deadline=$((SECONDS + TIME_SYNC_TIMEOUT_SECONDS))
while true; do
  year="$(system_year)"
  if [[ "${year}" =~ ^[0-9]+$ ]] && [ "${year}" -ge "${MIN_YEAR}" ] && chrony_synced; then
    log "system time is valid: $(date '+%F %T %Z %z')"
    exit 0
  fi

  if [ "${SECONDS}" -ge "${deadline}" ]; then
    log "ERROR: time is not valid after ${TIME_SYNC_TIMEOUT_SECONDS}s: $(date '+%F %T %Z %z')" >&2
    if command -v chronyc >/dev/null 2>&1; then
      chronyc tracking >&2 || true
      chronyc sources -v >&2 || true
    else
      timedatectl status >&2 || true
    fi
    exit 1
  fi

  log "waiting for valid synchronized time, current=$(date '+%F %T %Z %z')"
  sleep 2
done
