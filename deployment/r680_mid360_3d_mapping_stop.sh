#!/usr/bin/env bash
set -euo pipefail

result=0
/root/r680_nav/scripts/r680_octomap3d_stop.sh || result=1
/root/r680_nav/scripts/r680_display_accumulated_cloud_stop.sh || result=1
if (( result == 0 )); then
  echo "3D mapping display chain stopped."
else
  echo "3D mapping display chain stop failed; inspect process records and logs." >&2
fi
exit "${result}"
