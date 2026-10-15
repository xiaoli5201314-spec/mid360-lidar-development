#!/usr/bin/env bash
set -euo pipefail

reject_active_chassis() {
  local pid cmd
  if [[ -r /root/r680_nav/run/r680_nav2_chassis_bridge.pid ]]; then
    pid="$(tr -d '\r\n' </root/r680_nav/run/r680_nav2_chassis_bridge.pid)"
    if [[ "${pid}" =~ ^[1-9][0-9]*$ ]] && kill -0 "${pid}" 2>/dev/null; then
      echo "Refusing to start a non-chassis workflow while the real chassis stack is active; run r680_nav2_chassis_stop.sh first." >&2
      return 1
    fi
  fi
  for proc in /proc/[0-9]*/cmdline; do
    [[ -r "${proc}" ]] || continue
    cmd="$(tr '\0' ' ' <"${proc}" 2>/dev/null || true)"
    if [[ " ${cmd} " == *" /root/r680_nav/scripts/r680_ackermann_bridge.py "* &&
          " ${cmd} " == *" --write "* ]]; then
      echo "Refusing to start a non-chassis workflow while the real chassis stack is active; run r680_nav2_chassis_stop.sh first." >&2
      return 1
    fi
  done
}

reject_active_chassis

startup_success=0
display_started=0
octomap_started=0

cleanup_failed_start() {
  local rc=$?
  trap - EXIT INT TERM
  if (( startup_success == 0 )); then
    if (( octomap_started == 1 )); then
      /root/r680_nav/scripts/r680_octomap3d_stop.sh || rc=1
    fi
    if (( display_started == 1 )); then
      /root/r680_nav/scripts/r680_display_accumulated_cloud_stop.sh || rc=1
    fi
  fi
  exit "${rc}"
}
trap cleanup_failed_start EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

require_exec() {
  local path="$1"
  [[ -x "${path}" ]] || {
    echo "Required 3D helper is missing: ${path}" >&2
    return 1
  }
}

for helper in \
  /root/r680_nav/scripts/r680_nav2_dryrun_stop.sh \
  /root/r680_nav/scripts/r680_mid360_slam2d_stop.sh \
  /root/r680_nav/scripts/r680_mid360_3d_mapping_stop.sh \
  /root/r680_nav/scripts/r680_display_accumulated_cloud_start.sh \
  /root/r680_nav/scripts/r680_display_accumulated_cloud_stop.sh \
  /root/r680_nav/scripts/r680_octomap3d_start.sh \
  /root/r680_nav/scripts/r680_octomap3d_stop.sh; do
  require_exec "${helper}"
done

echo "Stopping 2D mapping and Nav2 before starting 3D-only mode..."
/root/r680_nav/scripts/r680_nav2_dryrun_stop.sh --quiet
/root/r680_nav/scripts/r680_mid360_slam2d_stop.sh
/root/r680_nav/scripts/r680_mid360_3d_mapping_stop.sh

echo "Starting bounded 3D display cloud..."
/root/r680_nav/scripts/r680_display_accumulated_cloud_start.sh
display_started=1

echo "Starting OctoMap occupancy mapping..."
/root/r680_nav/scripts/r680_octomap3d_start.sh
octomap_started=1

startup_success=1
echo "3D mapping display chain is ready."
echo "  accumulated cloud: /r680/display/cloud_accumulated"
echo "  OctoMap: /occupied_cells_vis_array"
