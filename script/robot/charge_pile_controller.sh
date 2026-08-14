#!/usr/bin/env bash
set -euo pipefail

STATE_FILE="${CHARGE_PILE_STATE_FILE:-/var/lib/roamerx-charge-pile/state}"
BIN_DIR="${CHARGE_PILE_BIN_DIR:-/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/dog_send_three_states}"

state="unknown"
if [ -r "${STATE_FILE}" ]; then
  read -r state < "${STATE_FILE}" || true
fi

case "${state}" in
  lying)
    executable="${BIN_DIR}/dog_lying_down"
    ;;
  unknown|*)
    executable="${BIN_DIR}/dog_status_unknown"
    ;;
esac

exec "${executable}"
