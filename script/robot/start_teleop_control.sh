#!/usr/bin/env bash
set -euo pipefail

SERVICE="roamerx-teleop-bridge.service"

usage() {
  echo "Usage: $0 {start|status}"
  echo "The remote-control bridge is persistent and cannot be stopped or restarted by this helper."
}

case "${1:-start}" in
  start)
    sudo systemctl enable --now "${SERVICE}"
    systemctl --no-pager --full status "${SERVICE}"
    ;;
  status)
    systemctl --no-pager --full status "${SERVICE}"
    ;;
  stop|restart)
    echo "ERROR: ${SERVICE} is persistent; ${1} is not supported." >&2
    exit 2
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
