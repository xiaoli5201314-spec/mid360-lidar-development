#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STOP_HELPER="${SCRIPT_DIR}/r680_process_stop.sh"
STATIC_TF_PID_FILE="/root/r680_nav/run/r680_nav_map_to_camera_init.pid"
NAV2_PID_FILE="/root/r680_nav/run/r680_nav2_navigation.pid"
MAP_PID_FILE="/root/r680_nav/run/r680_nav_map_server.pid"

result=0
"${STOP_HELPER}" --pid-file "${NAV2_PID_FILE}" --match "nav2_bringup navigation_launch.py" --label "Nav2 launch" >/dev/null 2>&1 || result=1
"${STOP_HELPER}" --pid-file "${MAP_PID_FILE}" --match "nav2_map_server map_server" --label "map_server" >/dev/null 2>&1 || result=1
"${STOP_HELPER}" --pid-file "${STATIC_TF_PID_FILE}" --match "static_transform_publisher" --label "map_to_camera_init static transform" >/dev/null 2>&1 || result=1
if (( result == 0 )); then
  echo "Requested Nav2 stack stop."
else
  echo "Nav2 stack stop failed; owned PID records were retained for inspection." >&2
fi
exit "${result}"
