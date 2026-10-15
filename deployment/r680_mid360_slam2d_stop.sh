#!/usr/bin/env bash
set -eo pipefail

result=0
/root/r680_nav/scripts/r680_mid360_slam_toolbox_stop.sh || result=1
/root/r680_nav/scripts/r680_mid360_pointcloud_to_scan_stop.sh || result=1
if (( result == 0 )); then
  echo "Requested legacy 2D mapping stop (pointcloud_to_laserscan + slam_toolbox)."
else
  echo "Legacy 2D mapping stop failed; inspect slam_toolbox and pointcloud_to_laserscan." >&2
fi
exit "${result}"
