#!/usr/bin/env bash
# Generate a rollback-safe 2D navigation grid for legacy maps.  It never
# overwrites map.pgm/map.yaml; start_navigation_real.sh prefers this sidecar.
set -euo pipefail

map_dir="${1:-/home/dogrobot/runtime/nx-edge/data/jszr/map}"
converter="${PCD2GRID_BINARY:-/home/dogrobot/robot/install/robot_slam/lib/robot_slam/pcd2grid_streaming}"
prefix="${map_dir}/map_traversable"

if [[ ! -f "${map_dir}/map.pcd" ]]; then
  echo "ERROR: map.pcd is missing in ${map_dir}" >&2
  exit 2
fi
if [[ ! -x "${converter}" ]]; then
  echo "ERROR: pcd2grid_streaming is missing: ${converter}" >&2
  exit 2
fi
if [[ -e "${prefix}.pgm" || -e "${prefix}.yaml" ]]; then
  echo "ERROR: refusing to overwrite ${prefix}.{pgm,yaml}; remove or archive it after rollback review" >&2
  exit 3
fi

"${converter}" "${map_dir}/map.pcd" "${prefix}" \
  0.05 0.50 0.75 200000000 3 1
echo "Generated ${prefix}.pgm and ${prefix}.yaml (5–50 cm terrain traversable)."
