#!/usr/bin/env bash
set -euo pipefail
[[ "$#" -eq 0 ]] || exit 2
# BEGIN transaction protocol
process_birth() {
  [[ -r "/proc/$1/stat" ]] || return 1
  sed 's/^.*) //' "/proc/$1/stat" | awk '{print $20}'
}
process_is_start() {
  local pid="$1" birth="$2" arg found=0
  [[ "${pid}" =~ ^[1-9][0-9]*$ && "${pid}" -gt 1 ]] || return 1
  [[ "$(process_birth "${pid}" 2>/dev/null || true)" == "${birth}" ]] || return 1
  while IFS= read -r -d '' arg; do
    [[ "${arg##*/}" == "${TX_SCRIPT##*/}" ]] && found=1
  done <"/proc/${pid}/cmdline"
  (( found == 1 ))
}
start_transaction() {
  mkdir -p "${TX_ROOT}" || return 1
  exec 7>"${TX_ROOT}/${TX_NAME}.gate" || return 1
  flock -n 7 || { echo "A stop/start transaction is in progress." >&2; return 1; }
  exec 9>"${TX_ROOT}/${TX_NAME}.lock" || return 1
  flock -n 9 || { echo "A start transaction is already running." >&2; return 1; }
  TX_PID="${BASHPID}"
  TX_BIRTH="$(process_birth "${TX_PID}")"
  rm -f -- "${TX_ROOT}/${TX_NAME}.cancel"
  printf '%s %s\n' "${TX_PID}" "${TX_BIRTH}" >"${TX_ROOT}/${TX_NAME}.starter"
  TX_OWNED=1
  flock -u 7
  exec 7>&-
}
check_cancelled() {
  [[ ! -e "${TX_ROOT}/${TX_NAME}.cancel" ]] ||
    { echo "Startup cancelled by stop." >&2; return 1; }
}
finish_transaction() {
  if [[ "${TX_OWNED:-0}" == 1 ]]; then
    rm -f -- "${TX_ROOT}/${TX_NAME}.starter" || true
    flock -u 9
    exec 9>&-
    TX_OWNED=0
  fi
}
cancel_start_transaction() {
  local starter= birth= state
  mkdir -p "${TX_ROOT}" || return 1
  # Close admission until this stop exits; no new start can erase cancellation.
  exec 7>"${TX_ROOT}/${TX_NAME}.gate" || return 1
  flock -x 7 || return 1
  printf 'cancel\n' >"${TX_ROOT}/${TX_NAME}.cancel" || return 1
  if [[ -s "${TX_ROOT}/${TX_NAME}.starter" ]]; then
    read -r starter birth <"${TX_ROOT}/${TX_NAME}.starter" || return 1
    if kill -0 "${starter}" 2>/dev/null; then
      process_is_start "${starter}" "${birth}" ||
        { echo "Starter identity mismatch; use physical emergency stop." >&2; return 1; }
      kill -TERM "${starter}" 2>/dev/null || true
    fi
  fi
  exec 9>"${TX_ROOT}/${TX_NAME}.lock" || return 1
  flock -w 180 9 ||
    { echo "Cancelled starter has not released its transaction; use physical emergency stop." >&2; return 1; }
  # Wait for the verified shell itself as well as its transaction/EXIT cleanup.
  if [[ -n "${starter}" ]]; then
    for _ in {1..100}; do
      process_is_start "${starter}" "${birth}" 2>/dev/null || break
      state="$(ps -o stat= -p "${starter}" 2>/dev/null || true)"
      [[ "${state}" == Z* ]] && break
      sleep 0.05 7>&- 9>&-
    done
    if process_is_start "${starter}" "${birth}" 2>/dev/null; then
      state="$(ps -o stat= -p "${starter}" 2>/dev/null || true)"
      [[ "${state}" == Z* ]] ||
        { echo "Starter has not exited; use physical emergency stop." >&2; return 1; }
    fi
  fi
}
# END transaction protocol
MODE_FILE=/root/r680_nav/run/r680_nav2_chassis.mode
result=0
TX_ROOT=/root/r680_nav/run
TX_NAME=r680_nav2_chassis
TX_SCRIPT=r680_nav2_chassis_start.sh
cancel_start_transaction || result=1
deactivate_failed=0
nav_stopped=0
# Source failure must not skip the serial bridge's zero-speed shutdown.
set +u
if ! source /opt/ros/humble/setup.bash; then result=1; fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
if [[ -e /root/r680_nav/run/r680_rviz_multi_goal.pid ]]; then
  /root/r680_nav/scripts/r680_rviz_multi_goal_stop.sh 7>&- 9>&- || result=1
fi
for node in /controller_server /velocity_smoother; do
  if ! timeout 5 ros2 lifecycle set "${node}" deactivate >/dev/null 2>&1; then
    deactivate_failed=1
    break
  else
    state="$(timeout 3 ros2 lifecycle get "${node}" 2>/dev/null || true)"
    if [[ "${state}" != "inactive [2]" ]]; then
      deactivate_failed=1
      break
    fi
  fi
done
if (( deactivate_failed == 1 )); then
  if /root/r680_nav/scripts/r680_nav2_stop.sh --quiet 7>&- 9>&-; then
    nav_stopped=1
  else
    result=1
  fi
fi
# The bridge watchdog is 0.5 s; leave enough time to send repeated neutral frames.
sleep 0.7 || result=1
if ! /root/r680_nav/scripts/r680_nav2_chassis_bridge_stop.sh 7>&- 9>&-; then
  echo "Bridge may still own the serial port. Press physical emergency stop NOW." >&2
  result=1
fi
if (( nav_stopped == 0 )); then
  /root/r680_nav/scripts/r680_nav2_stop.sh --quiet 7>&- 9>&- || result=1
fi
mode="$(tr -d '\r\n' <"${MODE_FILE}" 2>/dev/null || true)"
if [[ "${mode}" == 3d ]]; then
  /root/r680_nav/scripts/r680_mid360_3d_mapping_stop.sh 7>&- 9>&- || result=1
fi
/root/r680_nav/scripts/r680_mid360_pointcloud_to_scan_stop.sh 7>&- 9>&- || result=1
if (( result == 0 )); then
  rm -f -- "${MODE_FILE}"
fi
echo "Stop sequence finished; Livox/TF/FAST-LIO remain available for diagnosis."
exit "${result}"
