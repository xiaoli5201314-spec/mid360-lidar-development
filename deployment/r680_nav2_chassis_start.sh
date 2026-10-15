#!/usr/bin/env bash
set -euo pipefail
[[ "${1:-}" == --mode && ( "${2:-}" == 2d || "${2:-}" == 3d ) && "${3:-}" == --arm ]] &&
  [[ "$#" -eq 3 || ( "$#" -eq 4 && "${4:-}" == --multi ) ||
     ( "$#" -eq 5 && "${4:-}" == --multi && "${5:-}" == --dynamic-obstacle ) ||
     ( "$#" -eq 6 && "${4:-}" == --multi && "${5:-}" == --dynamic-obstacle && "${6:-}" == --reverse ) ]] ||
  { echo "Usage: $0 --mode 2d|3d --arm [--multi [--dynamic-obstacle [--reverse]]] (use a fixed-mode wrapper)" >&2; exit 2; }
mode="$2"
multi_mode=0
[[ "${4:-}" == --multi ]] && multi_mode=1
dynamic_obstacle_mode=0
[[ "${5:-}" == --dynamic-obstacle ]] && dynamic_obstacle_mode=1
reverse_mode=0
[[ "${6:-}" == --reverse ]] && reverse_mode=1
if (( dynamic_obstacle_mode == 1 )) &&
   [[ "${mode}" != 2d || "${multi_mode}" != 1 ]]; then
  echo "Dynamic obstacle mode is restricted to the 2D multi-point wrapper." >&2
  exit 2
fi
if (( reverse_mode == 1 )) &&
   [[ "${mode}" != 2d || "${multi_mode}" != 1 || "${dynamic_obstacle_mode}" != 1 ]]; then
  echo "Reverse mode is restricted to the dedicated 2D dynamic multi-point wrapper." >&2
  exit 2
fi

# Ordinary profiles keep their verified scan projection.  The isolated
# dynamic profiles widen the vertical/near-field window for low chair legs
# and close obstacles, and restamp delayed scans before Nav2 consumes them.
scan_min_height=-0.20
scan_max_height=0.40
scan_range_min=0.30
scan_restamp_now=0
if (( dynamic_obstacle_mode == 1 )); then
  scan_min_height=-0.35
  scan_max_height=1.20
  scan_range_min=0.10
  scan_restamp_now=1
fi

RUN_DIR=/root/r680_nav/run
MODE_FILE="${RUN_DIR}/r680_nav2_chassis.mode"
SCAN_PID_FILE="${RUN_DIR}/r680_nav2_chassis_scan.pid"
TX_ROOT="${RUN_DIR}"
TX_NAME=r680_nav2_chassis
TX_SCRIPT=r680_nav2_chassis_start.sh
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
# BEGIN helper control
process_birth() {
  [[ -r "/proc/$1/stat" ]] || return 1
  sed 's/^.*) //' "/proc/$1/stat" | awk '{print $20}'
}
helper_pid=
helper_script=
helper_birth=
helper_previous_pid=
helper_attempt=0
run_helper() {
  check_cancelled || return 1
  helper_script="$1"
  helper_previous_pid="${!:-}"
  helper_attempt=1
  "$@" 7>&- 9>&- &
  helper_pid=$!
  helper_birth="$(process_birth "${helper_pid}" 2>/dev/null || true)"
  local rc=0
  wait "${helper_pid}" || rc=$?
  helper_pid=
  helper_birth=
  helper_attempt=0
  (( rc == 0 )) || return "${rc}"
  check_cancelled
}
stop_active_helper() {
  local arg found=0 identity_ok=1 parent candidate="${helper_pid:-${!:-}}"
  # Capture in this shell, outside command substitution: $$ may still name
  # an outer shell when this function is executing in a Bash subshell.
  local owner_pid="${BASHPID}"
  if [[ -z "${helper_pid:-}" ]]; then
    [[ "${helper_attempt:-0}" == 1 && -n "${candidate}" &&
       "${candidate}" != "${helper_previous_pid:-}" ]] || return 0
    helper_pid="${candidate}"
  fi
  if kill -0 "${helper_pid}" 2>/dev/null; then
    # A forked helper can still have its parent's argv until exec completes.
    # Retry briefly, retaining the PPid/birth check on every observation.
    for _ in {1..40}; do
      kill -0 "${helper_pid}" 2>/dev/null || break
      if [[ -n "${helper_birth}" ]]; then
        [[ "$(process_birth "${helper_pid}" 2>/dev/null || true)" == "${helper_birth}" ]] || identity_ok=0
      else
        parent="$(awk '/^PPid:/ {print $2}' "/proc/${helper_pid}/status" 2>/dev/null || true)"
        [[ "${parent}" == "${owner_pid}" ]] || identity_ok=0
      fi
      (( identity_ok == 1 )) || break
      found=0
      if [[ -r "/proc/${helper_pid}/cmdline" ]]; then
        while IFS= read -r -d '' arg; do
          [[ "${arg##*/}" == "${helper_script##*/}" ]] && found=1
        done <"/proc/${helper_pid}/cmdline"
      fi
      (( found == 1 )) && break
      [[ "$(ps -o stat= -p "${helper_pid}" 2>/dev/null || true)" == Z* ]] && break
      sleep 0.05 7>&- 9>&-
    done
    if (( found == 1 && identity_ok == 1 )); then
      kill -TERM "${helper_pid}" 2>/dev/null || true
    elif [[ "$(ps -o stat= -p "${helper_pid}" 2>/dev/null || true)" != Z* ]]; then
      echo "Helper identity changed; waiting without a blind signal." >&2
      identity_ok=0
    fi
  fi
  # Even a failed identity check must not release the transaction while our
  # direct child could still arm a bridge. The outer stop reports a timeout.
  wait "${helper_pid}" 2>/dev/null || true
  (( identity_ok == 1 ))
}
# END helper control
reject_existing_bridge() {
  local proc pid
  if [[ -r "${RUN_DIR}/r680_nav2_chassis_bridge.pid" ]]; then
    pid="$(tr -d '\r\n' <"${RUN_DIR}/r680_nav2_chassis_bridge.pid")"
    [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid bridge PID record." >&2; return 1; }
    if kill -0 "${pid}" 2>/dev/null; then
      echo "Real chassis bridge already running; stop it explicitly first." >&2
      return 1
    fi
  fi
  for proc in /proc/[0-9]*/cmdline; do
    [[ -r "${proc}" ]] || continue
    if tr '\0' '\n' <"${proc}" 2>/dev/null | grep -Eq '(^|/)r680_.*ackermann_bridge\.(py|sh)$'; then
      echo "Existing bridge found; refusing to replace it." >&2
      return 1
    fi
  done
}
reject_manual_controller() {
  local lock=/root/r680_nav/run/r680_chassis_control.lock
  mkdir -p "${RUN_DIR}"
  exec 6>"${lock}"
  if ! flock -n 6; then
    echo "Manual web chassis control is active; stop it with Ctrl+C before starting Nav2." >&2
    exec 6>&-
    return 1
  fi
  flock -u 6
  exec 6>&-
}
require_topic() {
  timeout 15 ros2 topic echo "$1" --once --field header >/dev/null 2>&1 ||
    { echo "Required live topic unavailable: $1" >&2; return 1; }
}
scan_count() {
  local info
  # A nonexistent /scan normally makes topic info return nonzero; distinguish
  # that from a failed graph query before deciding it has zero publishers.
  if info="$(timeout 5 ros2 topic info /scan 2>/dev/null)"; then
    awk '/^Publisher count:/ {print $3}' <<<"${info}"
  else
    local topics
    topics="$(timeout 5 ros2 topic list 2>/dev/null)" || return 1
    if grep -Fxq /scan <<<"${topics}"; then return 1; fi
    printf '0\n'
  fi
}
nav_attempt=0
display_attempt=0
scan_attempt=0
scan_pid=
scan_birth=
bridge_attempt=0
multi_goal_attempt=0
success=0
stop_owned_scan_start() {
  local member_count
  [[ -n "${scan_pid:-}" ]] || return 0
  if kill -0 "${scan_pid}" 2>/dev/null; then
    [[ -n "${scan_birth:-}" &&
       "$(process_birth "${scan_pid}" 2>/dev/null || true)" == "${scan_birth}" ]] || {
      echo "Scan starter identity changed; refusing a blind process-group signal." >&2
      return 1
    }
  fi
  # ensure_single_scan verifies that this freshly forked PID is also the
  # dedicated setsid PGID.  The group may outlive its leader, so query the
  # group itself instead of treating a missing leader as a completed stop.
  kill -TERM -- "-${scan_pid}" 2>/dev/null || true
  for _ in $(seq 1 50); do
    member_count="$(ps -eo pgid=,stat= 2>/dev/null | awk -v group="${scan_pid}" '
      $1 == group && $2 !~ /^Z/ { count++ }
      END { print count+0 }
    ')" || return 1
    if [[ "${member_count}" == "0" ]]; then
      wait "${scan_pid}" 2>/dev/null || true
      rm -f -- "${SCAN_PID_FILE}"
      return 0
    fi
    sleep 0.1 7>&- 9>&-
  done
  echo "Owned scan process group did not stop (PGID ${scan_pid})." >&2
  return 1
}
cleanup() {
  local rc=$?
  trap - EXIT
  trap '' INT TERM
  if (( success == 0 )); then
    # A helper may have started its child before our wait returned. Cancel and
    # join it first; bridge_attempt was set BEFORE entering this signal window.
    stop_active_helper || rc=1
    if (( bridge_attempt == 1 )); then
      for node in /controller_server /velocity_smoother; do
        timeout 5 ros2 lifecycle set "${node}" deactivate >/dev/null 2>&1 7>&- 9>&- || true
      done
      /root/r680_nav/scripts/r680_nav2_stop.sh --quiet 7>&- 9>&- || rc=1
      sleep 0.7 7>&- 9>&- || rc=1
      /root/r680_nav/scripts/r680_nav2_chassis_bridge_stop.sh 7>&- 9>&- || rc=1
    elif (( nav_attempt == 1 )); then
      /root/r680_nav/scripts/r680_nav2_stop.sh --quiet 7>&- 9>&- || rc=1
    fi
    if (( ${multi_goal_attempt:-0} == 1 )); then
      /root/r680_nav/scripts/r680_rviz_multi_goal_stop.sh 7>&- 9>&- || rc=1
    fi
    if (( display_attempt == 1 )); then
      /root/r680_nav/scripts/r680_mid360_3d_mapping_stop.sh 7>&- 9>&- || rc=1
    fi
    if (( scan_attempt == 1 )); then
      stop_owned_scan_start || rc=1
      /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan_stop.sh 7>&- 9>&- || rc=1
    fi
  fi
  finish_transaction || rc=1
  exit "${rc}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# BEGIN realtime safety gate
run_realtime_safety_gate() {
  run_helper python3 -u /root/r680_nav/scripts/r680_nav_realtime_preflight.py \
    --duration 4.0 \
    --min-odometry-hz 5.0 \
    --max-odometry-age 0.30 \
    --min-scan-hz 5.0 \
    --max-scan-age 0.40 \
    --max-receive-silence 0.50 \
    --max-odometry-position 1000.0 \
    --min-quaternion-norm 0.50 \
    --max-quaternion-norm 1.50
}
# END realtime safety gate
ensure_single_scan() {
  local count scan_pgid
  count="$(scan_count)" || { echo "Cannot determine /scan publisher ownership." >&2; return 1; }
  [[ "${count}" =~ ^[0-9]+$ ]] || return 1
  (( count <= 1 )) || { echo "Multiple /scan publishers; refusing start." >&2; return 1; }
  if (( count == 0 )); then
    scan_attempt=1
    CLOUD_IN=/cloud_registered_body SCAN_OUT=/scan SCAN_FRAME=body \
      SCAN_MIN_HEIGHT="${scan_min_height}" SCAN_MAX_HEIGHT="${scan_max_height}" \
      SCAN_RANGE_MIN="${scan_range_min}" RESTAMP_SCAN_NOW="${scan_restamp_now}" \
      setsid /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan.sh \
      >/root/r680_nav/logs/r680_nav2_chassis_scan.log 2>&1 7>&- 9>&- &
    scan_pid=$!
    scan_birth="$(process_birth "${scan_pid}" 2>/dev/null || true)"
    [[ -n "${scan_birth}" ]] || { echo "Cannot record scan starter identity." >&2; return 1; }
    printf '%s\n' "${scan_pid}" >"${SCAN_PID_FILE}"
    scan_pgid=
    for _ in $(seq 1 40); do
      kill -0 "${scan_pid}" 2>/dev/null || { echo "Scan converter exited." >&2; return 1; }
      [[ "$(process_birth "${scan_pid}" 2>/dev/null || true)" == "${scan_birth}" ]] || {
        echo "Scan starter identity changed during startup." >&2
        return 1
      }
      scan_pgid="$(ps -o pgid= -p "${scan_pid}" 2>/dev/null | tr -d ' ')"
      [[ "${scan_pgid}" == "${scan_pid}" ]] && break
      sleep 0.05
    done
    [[ "${scan_pgid}" == "${scan_pid}" ]] || {
      echo "Scan converter did not establish its dedicated process group." >&2
      return 1
    }
  fi
  count=0
  for _ in $(seq 1 30); do
    if [[ -n "${scan_pid:-}" ]]; then
      kill -0 "${scan_pid}" 2>/dev/null || { echo "Scan converter exited." >&2; return 1; }
      [[ "$(process_birth "${scan_pid}" 2>/dev/null || true)" == "${scan_birth}" ]] || {
        echo "Scan starter identity changed while waiting for /scan." >&2
        return 1
      }
    fi
    count="$(scan_count)" || { sleep 1; continue; }
    (( count <= 1 )) || { echo "Multiple /scan publishers; refusing start." >&2; return 1; }
    (( count == 1 )) && break
    sleep 1
  done
  [[ "${count}" == "1" ]] || { echo "Timed out waiting for the owned /scan publisher." >&2; return 1; }
  require_topic /scan || return 1
  count="$(scan_count)" || return 1
  [[ "${count}" == 1 ]] || { echo "/scan must have exactly one live publisher." >&2; return 1; }
}

start_transaction
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
mkdir -p /root/r680_nav/logs

# Main workflow
reject_existing_bridge
reject_manual_controller
[[ -s /root/r680_nav/time/last_good_epoch ]] ||
  { echo "Missing last_good_epoch; synchronize the board clock first." >&2; exit 1; }
run_helper /root/r680_nav/scripts/r680_check_3588_time_sane.sh
[[ "$(tr -d '\r\n' </sys/class/net/eth0/carrier)" == 1 ]] ||
  { echo "eth0 carrier is not ready." >&2; exit 1; }
ip -4 -o addr show dev eth0 | awk '$4=="192.0.2.10/24" {found=1} END {exit !found}' ||
  { echo "eth0 must have IPv4 192.0.2.10/24." >&2; exit 1; }
for service in mid360-livox-driver.service mid360-static-tf.service mid360-vehicle-tf.service fastlio-mapping.service; do
  check_cancelled
  systemctl start "${service}" 7>&- 9>&-
  systemctl is-active --quiet "${service}" ||
    { echo "Required service is not active: ${service}" >&2; exit 1; }
done
require_topic /Odometry
require_topic /cloud_registered_body
tf_output="$(timeout 10 ros2 run tf2_ros tf2_echo camera_init base_link 2>&1 || true)"
grep -q "Translation:" <<<"${tf_output}" ||
  { echo "Required TF camera_init -> base_link missing." >&2; exit 1; }
run_helper /root/r680_nav/scripts/r680_nav2_stop.sh --quiet
run_helper /root/r680_nav/scripts/r680_nav2_dryrun_stop.sh --quiet
run_helper /root/r680_nav/scripts/r680_mid360_slam_toolbox_stop.sh
if [[ "${mode}" == 2d ]]; then
  run_helper /root/r680_nav/scripts/r680_mid360_3d_mapping_stop.sh
  if (( reverse_mode == 1 )); then
    params=/root/r680_nav/config/r680_nav2_reverse_dynamic_params.yaml
  elif (( dynamic_obstacle_mode == 1 )); then
    params=/root/r680_nav/config/r680_nav2_dynamic_obstacle_params.yaml
  else
    params=/root/r680_nav/config/r680_nav2_params.yaml
  fi
else
  display_attempt=1
  run_helper /root/r680_nav/scripts/r680_mid360_3d_mapping_start.sh
  params=/root/r680_nav/config/r680_nav2_3d_params.yaml
fi
# The 3D display start stops slam2d and its scan converter: ensure /scan AFTER it.
ensure_single_scan
nav_attempt=1
run_helper /root/r680_nav/scripts/r680_nav2_start.sh "${params}" /root/r680_nav/maps/mid360_slam2d_latest.yaml
check_cancelled
echo "Checking live /Odometry and /scan timing before chassis arming..."
run_realtime_safety_gate || {
  echo "Realtime navigation preflight failed; chassis remains disarmed." >&2
  exit 1
}
check_cancelled
bridge_attempt=1
if (( reverse_mode == 1 )); then
  run_helper /root/r680_nav/scripts/r680_nav2_chassis_bridge_start.sh --arm --reverse
else
  run_helper /root/r680_nav/scripts/r680_nav2_chassis_bridge_start.sh --arm
fi
if (( multi_mode == 1 )); then
  multi_goal_attempt=1
  run_helper /root/r680_nav/scripts/r680_rviz_multi_goal_start.sh
fi
if (( reverse_mode == 1 )); then
  printf '%s\n' '2d-reverse-dynamic-multi' >"${MODE_FILE}"
elif (( dynamic_obstacle_mode == 1 )); then
  printf '%s\n' '2d-dynamic-multi' >"${MODE_FILE}"
else
  printf '%s\n' "${mode}" >"${MODE_FILE}"
fi
success=1
if (( reverse_mode == 1 )); then
  echo "Real chassis 2d reverse/dynamic multi-point ready: 先 2D Pose Estimate，再连续点击多个 2D Goal Pose。"
  echo "Reeds-Shepp forward/reverse replanning uses the 1.40 m minimum turning radius; blocked paths stop and retry."
  echo "RViz on VM: ~/r680_nav2_dynamic_multi_view_rviz.sh"
elif (( dynamic_obstacle_mode == 1 )); then
  echo "Real chassis 2d dynamic-obstacle multi-point ready: 先 2D Pose Estimate，再连续点击多个 2D Goal Pose。"
  echo "Blocked path policy: stop, retain the current waypoint, replan at 1 Hz, and resume only with a valid path."
  echo "RViz on VM: ~/r680_nav2_dynamic_multi_view_rviz.sh"
elif (( multi_mode == 1 )); then
  echo "Real chassis ${mode} multi-point ready: 先 2D Pose Estimate，再连续点击多个 2D Goal Pose。"
  echo "RViz multi-goal queue: first click starts; later clicks queue in order; final point returns to first."
  if [[ "${mode}" == 3d ]]; then
    echo "RViz on VM: ~/r680_nav2_3d_multi_view_rviz.sh"
  else
    echo "RViz on VM: ~/r680_nav2_multi_view_rviz.sh"
  fi
elif [[ "${mode}" == 3d ]]; then
  echo "Real chassis 3d single-goal ready: 先 2D Pose Estimate，再点击一个 2D Goal Pose。"
  echo "RViz on VM: ~/r680_nav2_3d_view_rviz.sh"
else
  echo "Real chassis 2d single-goal ready: 先 2D Pose Estimate，再点击一个 2D Goal Pose。"
  echo "RViz on VM: ~/r680_nav2_dryrun_view_rviz.sh"
fi
echo "Watch: /root/r680_nav/scripts/r680_nav2_cmd_vel_watch.sh"
echo "Status: /root/r680_nav/scripts/r680_nav2_chassis_status.sh"
echo "Stop: /root/r680_nav/scripts/r680_nav2_chassis_stop.sh"
