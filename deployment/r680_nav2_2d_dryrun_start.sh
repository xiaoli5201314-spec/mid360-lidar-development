#!/usr/bin/env bash
set -euo pipefail
MAP_ARG="${1:-}"

# 2D mode must not continue while an old 3D display/costmap chain is alive.
/root/r680_nav/scripts/r680_mid360_3d_mapping_stop.sh

if [[ -n "${MAP_ARG}" ]]; then
  exec /root/r680_nav/scripts/r680_nav2_dryrun_start.sh "${MAP_ARG}"
else
  exec /root/r680_nav/scripts/r680_nav2_dryrun_start.sh
fi
