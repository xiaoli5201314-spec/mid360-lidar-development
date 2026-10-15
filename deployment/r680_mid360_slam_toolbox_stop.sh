#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STOP_HELPER="${SCRIPT_DIR}/r680_process_stop.sh"

result=0
"${STOP_HELPER}" --token "slam_toolbox" --label "slam_toolbox launch" >/dev/null 2>&1 || result=1
"${STOP_HELPER}" --token "async_slam_toolbox_node" --label "slam_toolbox node" >/dev/null 2>&1 || result=1
if (( result == 0 )); then
  echo "Requested slam_toolbox stop."
else
  echo "slam_toolbox stop failed; inspect the remaining process." >&2
fi
exit "${result}"
