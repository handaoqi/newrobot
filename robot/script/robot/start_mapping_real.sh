#!/usr/bin/env bash
set -euo pipefail

# Thin wrapper around the same MappingAdapter used by cloud mapping.start.
# Do not start robot_slam/slam.launch.py on the robot; that launch file starts RViz.

EDGE_SOURCE_DIR="${EDGE_SOURCE_DIR:-/home/dogrobot/edge-agent}"
EDGE_CONFIG="${EDGE_CONFIG:-/home/dogrobot/runtime/nx-edge/conf/edge-agent.yaml}"
export PYTHONPATH="${EDGE_SOURCE_DIR}${PYTHONPATH:+:$PYTHONPATH}"

usage() {
  echo "Usage: $0 {start|save|stop|restart|status}"
  echo
  echo "start   Start SLAM via roamerx-mapping.service and wait until motion is allowed."
  echo "save    Ask SLAM to save the current map without uploading."
  echo "stop    Save if possible, then stop the mapping unit."
  echo "restart Stop existing mapping, then start again."
  echo "status  Show MappingAdapter status JSON."
}

MODE="${1:-start}"
case "${MODE}" in
  start|save|stop|restart|status)
    exec python3 -m roamerx_edge.mapping_cli --config "${EDGE_CONFIG}" "${MODE}"
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage
    exit 1
    ;;
esac
