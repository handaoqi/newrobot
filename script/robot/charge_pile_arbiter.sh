#!/usr/bin/env bash
# Own the hand-off between the vendor pile helper and arc_platform. Both use
# /dev/ttyUSB0, so this script is the only supported way to switch modes.
set -euo pipefail

ACTION="${1:-}"
SERVICE="${CHARGE_PILE_SERVICE:-roamerx-charge-pile.service}"
STATE_FILE="${CHARGE_PILE_STATE_FILE:-/var/lib/roamerx-charge-pile/state}"
BIN_DIR="${CHARGE_PILE_BIN_DIR:-/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/dog_send_three_states}"
LOCK_FILE="${CHARGE_PILE_LOCK_FILE:-/run/roamerx-charge-pile.lock}"
UART="${CHARGE_PILE_UART:-/dev/ttyUSB0}"
STATUS_FILE="${CHARGE_PILE_STATUS_FILE:-/run/roamerx-charge-pile/legacy-status.txt}"
MODULE_FILE="${CHARGE_PILE_MODULE_FILE:-/run/roamerx-charge-pile/module}"

exec 9>"${LOCK_FILE}"
flock -x 9

legacy_pattern='[d]og_(lying_down|returning|status_unknown)'

wait_uart_free() {
  local deadline=$((SECONDS + 8))
  while sudo lsof -t "${UART}" >/dev/null 2>&1; do
    if (( SECONDS >= deadline )); then
      echo "charge UART is still busy: $(sudo lsof -n "${UART}" 2>/dev/null | tail -n +2)" >&2
      return 1
    fi
    sleep 0.2
  done
}

wait_uart_owner() {
  local pattern="$1"
  local deadline=$((SECONDS + 12))
  while ! uart_owner_commands | grep -Eq "${pattern}"; do
    if (( SECONDS >= deadline )); then
      echo "expected UART owner ${pattern}, found: $(uart_owner_commands)" >&2
      return 1
    fi
    sleep 0.25
  done
}

uart_owner_commands() {
  local pid
  while read -r pid; do
    [ -n "${pid}" ] && ps -p "${pid}" -o args= 2>/dev/null || true
  done < <(sudo lsof -t "${UART}" 2>/dev/null || true)
}

stop_all_owners() {
  sudo systemctl stop "${SERVICE}" 2>/dev/null || true
  sudo pkill -TERM -f "${legacy_pattern}" 2>/dev/null || true
  robot-launch stop arc_platform >/dev/null 2>&1 || true
  sleep 0.5
  sudo pkill -KILL -f "${legacy_pattern}" 2>/dev/null || true
  wait_uart_free
}

start_arc() {
  stop_all_owners
  robot-launch start arc_platform >/dev/null
  local deadline=$((SECONDS + 12))
  while ! robot-launch egg arc_platform 2>/dev/null | grep -qi 'running'; do
    if (( SECONDS >= deadline )); then
      robot-launch egg arc_platform 2>/dev/null || true
      echo "arc_platform did not start" >&2
      return 1
    fi
    sleep 0.5
  done
  wait_uart_owner 'arc_platform'
}

start_legacy() {
  local state="$1"
  stop_all_owners
  printf '%s\n' "${state}" | sudo tee "${STATE_FILE}" >/dev/null
  sudo install -d -m 0755 "$(dirname "${MODULE_FILE}")"
  printf '%s\n' "${state}" | sudo tee "${MODULE_FILE}" >/dev/null
  sudo systemctl start "${SERVICE}"
  local deadline=$((SECONDS + 8))
  while ! pgrep -f "${legacy_pattern}" >/dev/null; do
    if (( SECONDS >= deadline )); then
      sudo journalctl -u "${SERVICE}" -n 30 --no-pager >&2 || true
      echo "legacy charge helper did not start" >&2
      return 1
    fi
    sleep 0.25
  done
  wait_uart_owner 'dog_(lying_down|status_unknown)'
}

case "${ACTION}" in
  arc)
    start_arc
    ;;
  legacy-lying)
    start_legacy lying
    ;;
  legacy-status)
    trap 'start_arc || true' EXIT
    start_legacy unknown
    sleep 3
    sudo install -d -m 0755 "$(dirname "${STATUS_FILE}")"
    sudo journalctl -u "${SERVICE}" -n 40 --no-pager | grep -E 'connected=|charge pin=' | tail -n 4 \
      | sudo tee "${STATUS_FILE}" >/dev/null || true
    stop_all_owners
    start_arc
    sudo cat "${STATUS_FILE}" 2>/dev/null || true
    trap - EXIT
    ;;
  legacy-return)
    trap 'start_arc || true' EXIT
    stop_all_owners
    sudo install -d -m 0755 "$(dirname "${MODULE_FILE}")"
    printf 'return\n' | sudo tee "${MODULE_FILE}" >/dev/null
    sudo timeout --signal=TERM --kill-after=2 6 "${BIN_DIR}/dog_returning" \
      >/tmp/roamerx-charge-return.log 2>&1 || true
    sudo pkill -KILL -f '[d]og_returning' 2>/dev/null || true
    wait_uart_free
    start_arc
    trap - EXIT
    ;;
  *)
    echo "usage: $0 {arc|legacy-lying|legacy-status|legacy-return}" >&2
    exit 2
    ;;
esac
