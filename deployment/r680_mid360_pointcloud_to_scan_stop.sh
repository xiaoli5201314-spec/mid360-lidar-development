#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STOP_HELPER="${SCRIPT_DIR}/r680_process_stop.sh"

result=0
"${STOP_HELPER}" --token "pointcloud_to_laserscan_node" --label "pointcloud_to_laserscan" >/dev/null 2>&1 || result=1
"${STOP_HELPER}" --token "r680_scan_qos_relay.py" --label "scan QoS relay" >/dev/null 2>&1 || result=1
if (( result == 0 )); then
  echo "Requested pointcloud_to_laserscan and scan QoS relay stop."
else
  echo "pointcloud_to_laserscan or scan QoS relay stop failed; inspect remaining processes." >&2
fi
exit "${result}"
