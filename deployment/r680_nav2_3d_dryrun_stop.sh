#!/usr/bin/env bash
set -euo pipefail

result=0
/root/r680_nav/scripts/r680_nav2_dryrun_stop.sh "$@" || result=1
/root/r680_nav/scripts/r680_mid360_3d_mapping_stop.sh || result=1
if (( result == 0 )); then
  echo "Stopped 3D-assisted Nav2 dry-run and 3D display chain."
else
  echo "3D-assisted Nav2 dry-run stop failed; inspect process records and logs." >&2
fi
exit "${result}"
