#!/usr/bin/env bash
set -euo pipefail
[[ "$#" -le 2 ]] || { echo "Usage: $0 [PARAMS_YAML [MAP_YAML]]" >&2; exit 2; }
RUN_DIR=/root/r680_nav/run
LOG_DIR=/root/r680_nav/logs
PID_FILE="${RUN_DIR}/r680_nav2_localized.pid"
PGID_FILE="${RUN_DIR}/r680_nav2_localized.pgid"
LOG_FILE="${LOG_DIR}/r680_nav2_localized.log"
PARAMS_FILE="${1:-/root/r680_nav/config/r680_nav2_params.yaml}"
MAP_YAML="${2:-/root/r680_nav/maps/mid360_slam2d_latest.yaml}"
TX_ROOT="${RUN_DIR}"
TX_NAME=r680_nav2_localized
TX_SCRIPT=r680_nav2_start.sh
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
# BEGIN launch ownership
stop_owned_launch() {
  local pid="${owned_pid:-${!:-}}" cmd= pgid= member state members leader_gone=0
  (( launch_attempt == 1 )) || return 0
  [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || return 0
  [[ "${pid}" -gt 1 ]] || return 1
  # A signal can arrive before the setsid child execs; wait for argv to settle.
  for _ in {1..40}; do
    if ! kill -0 "${pid}" 2>/dev/null; then leader_gone=1; break; fi
    state="$(ps -o stat= -p "${pid}" 2>/dev/null || true)"
    if [[ "${state}" == Z* ]]; then leader_gone=1; break; fi
    cmd="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
    [[ "${cmd}" == *"nav2_bringup bringup_launch.py"* && "${pgid}" == "${pid}" ]] && break
    sleep 0.05 7>&- 9>&-
  done
  if (( leader_gone == 0 )); then
    [[ "${cmd}" == *"nav2_bringup bringup_launch.py"* && "${pgid}" == "${pid}" ]] ||
      { echo "Cannot verify in-memory Nav2 PID ${pid}; refusing blind signal." >&2; return 1; }
  fi
  # The in-memory setsid PGID survives its leader. Never equate a reaped or
  # zombie launcher with an empty group; validate every remaining live member.
  members="$(ps -eo pid=,pgid=,stat= | awk -v wanted="${pid}" '$2==wanted && $3 !~ /^Z/ {print $1}')" || return 1
  [[ -n "${members}" ]] || return 0
  while read -r member; do
    [[ "${member}" =~ ^[1-9][0-9]*$ ]] || return 1
    state="$(ps -o stat= -p "${member}" 2>/dev/null || true)"
    [[ -n "${state}" && "${state}" != Z* ]] || continue
    cmd="$(tr '\0' ' ' <"/proc/${member}/cmdline" 2>/dev/null || true)"
    case "${cmd}" in
      *"nav2_bringup bringup_launch.py"*|*"/opt/ros/humble/lib/nav2_"*|*"/opt/ros/humble/lib/rclcpp_components/component_container"*) ;;
      *) echo "Unowned Nav2 group member ${member}." >&2; return 1 ;;
    esac
  done <<<"${members}"
  kill -TERM -- "-${pid}" 2>/dev/null || true
  for _ in {1..80}; do
    members="$(ps -eo pid=,pgid=,stat= | awk -v wanted="${pid}" '$2==wanted && $3 !~ /^Z/ {print $1}')" || return 1
    if [[ -z "${members}" ]]; then
      wait "${pid}" 2>/dev/null || true
      return 0
    fi
    sleep 0.1 7>&- 9>&-
  done
  echo "Nav2 rollback timed out for in-memory PID ${pid}; retain records." >&2
  return 1
}
# END launch ownership
launch_attempt=0
owned_pid=
ready=0
cleanup() {
  local rc=$?
  trap - EXIT
  trap '' INT TERM
  if (( launch_attempt == 1 && ready == 0 )); then
    if stop_owned_launch; then
      rm -f -- "${PID_FILE}" "${PGID_FILE}" || rc=1
    else
      rc=1
    fi
  fi
  finish_transaction || rc=1
  exit "${rc}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start_transaction
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
mkdir -p "${LOG_DIR}"
[[ -s "${PARAMS_FILE}" && -s "${MAP_YAML}" ]] || { echo "Missing params or map YAML." >&2; exit 1; }
require_topic() {
  timeout 8 ros2 topic echo "$1" --once --field header >/dev/null 2>&1 ||
    { echo "Required topic unavailable: $1" >&2; exit 1; }
}
require_topic /Odometry
require_topic /scan
tf_output="$(timeout 8 ros2 run tf2_ros tf2_echo camera_init base_link 2>&1 || true)"
grep -q "Translation:" <<<"${tf_output}" ||
  { echo "Required TF camera_init -> base_link unavailable." >&2; exit 1; }

OWNED_RUNNING=0
if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(tr -d '\r\n' <"${PID_FILE}")"
  [[ "${old_pid}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid localized PID." >&2; exit 1; }
  if kill -0 "${old_pid}" 2>/dev/null; then
    old_cmdline="$(tr '\0' ' ' <"/proc/${old_pid}/cmdline")"
    old_pgid="$(ps -o pgid= -p "${old_pid}" | tr -d ' ')"
    if [[ "${old_cmdline}" == *"nav2_bringup bringup_launch.py"* && "${old_pgid}" == "${old_pid}" ]]; then
      OWNED_RUNNING=1
    else
      echo "Recorded PID does not own localized bringup." >&2
      exit 1
    fi
  fi
fi
reject_unowned_duplicate() {
  local node="$1" nodes count
  nodes="$(timeout 6 ros2 node list 2>/dev/null)" || return 1
  count="$(awk -v wanted="${node}" '$0 == wanted {count++} END {print count+0}' <<<"${nodes}")"
  if (( count > 1 )) || [[ "${count}" == 1 && "${OWNED_RUNNING}" != 1 ]]; then
    echo "Refusing ${node}: duplicate or not owned by the recorded localized PID." >&2
    return 1
  fi
}
reject_unowned_duplicate /amcl
reject_unowned_duplicate /planner_server
if [[ "${OWNED_RUNNING}" == 1 ]]; then
  echo "Localized Nav2 already running (pid ${old_pid})."
  exit 0
fi

# BEGIN launch process
check_cancelled
launch_attempt=1
setsid ros2 launch nav2_bringup bringup_launch.py \
  slam:=False map:="${MAP_YAML}" params_file:="${PARAMS_FILE}" autostart:=True \
  >"${LOG_FILE}" 2>&1 7>&- 9>&- &
owned_pid=$!
NAV2_PID="${owned_pid}"
printf '%s\n' "${owned_pid}" >"${PID_FILE}"
printf '%s\n' "${owned_pid}" >"${PGID_FILE}"
# END launch process

# BEGIN localization readiness
localization_ready=0
deadline=$((SECONDS + 60))
while (( SECONDS < deadline )); do
  check_cancelled
  kill -0 "${NAV2_PID}" 2>/dev/null || exit 1
  map_state="$(timeout 3 ros2 lifecycle get /map_server 2>/dev/null || true)"
  amcl_state="$(timeout 3 ros2 lifecycle get /amcl 2>/dev/null || true)"
  if [[ "${map_state}" == "active [3]" && "${amcl_state}" == "active [3]" ]]; then
    localization_ready=1
    break
  fi
  sleep 1 7>&- 9>&-
done
(( localization_ready == 1 )) || { echo "map_server/AMCL not ready." >&2; exit 1; }
echo "AMCL is ready. NOW execute RViz 2D Pose Estimate; do not send a goal yet."
if [[ "${PARAMS_FILE}" == /root/r680_nav/config/r680_nav2_3d_params.yaml ]]; then
  echo "RViz on VM: ~/r680_nav2_3d_view_rviz.sh"
else
  echo "RViz on VM: ~/r680_nav2_dryrun_view_rviz.sh"
fi
echo "Waiting up to 180 seconds for AMCL map -> base_link localization TF before arming the chassis bridge."
initial_pose_ready=0
deadline=$((SECONDS + 180))
while (( SECONDS < deadline )); do
  check_cancelled
  kill -0 "${NAV2_PID}" 2>/dev/null || exit 1
  tf_output="$(timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
  if grep -q "Translation:" <<<"${tf_output}"; then
    initial_pose_ready=1
    break
  fi
done
(( initial_pose_ready == 1 )) || { echo "No AMCL map -> base_link localization TF received; startup cancelled." >&2; exit 1; }
# END localization readiness

endpoint_present() {
  local info
  info="$(timeout 5 ros2 topic info "$1" -v 2>/dev/null)" || return 1
  awk -v wanted="$2" -v role="$3" '
    /^Node name:/ {node=$3}
    /^Endpoint type:/ {if (node==wanted && $3==role) found=1}
    END {exit !found}' <<<"${info}"
}
lifecycle_active_with_retry() {
  local node="$1" state=""
  for _ in {1..3}; do
    if state="$(timeout 3 ros2 lifecycle get "${node}" 2>/dev/null)"; then
      if [[ "${state}" == "active [3]" ]]; then
        return 0
      fi
      last_readiness_failure="${node} lifecycle state: ${state:-<none>}"
    else
      last_readiness_failure="${node} lifecycle query failed"
    fi
    sleep 0.25
  done
  return 1
}
lifecycle_nodes=(/map_server /amcl /controller_server /planner_server /bt_navigator
  /global_costmap/global_costmap /local_costmap/local_costmap /velocity_smoother)
last_readiness_failure="readiness probe did not complete"
deadline=$((SECONDS + 120))
while (( SECONDS < deadline )); do
  check_cancelled
  kill -0 "${NAV2_PID}" 2>/dev/null || { echo "Nav2 launch exited; inspect ${LOG_FILE}." >&2; exit 1; }
  active=1
  for node in "${lifecycle_nodes[@]}"; do
    if ! lifecycle_active_with_retry "${node}"; then active=0; break; fi
  done
  if (( active == 1 )) && endpoint_present /goal_pose bt_navigator SUBSCRIPTION &&
     endpoint_present /cmd_vel velocity_smoother PUBLISHER; then
    ready=1
    break
  fi
  sleep 1
done
(( ready == 1 )) || {
  echo "Nav2 lifecycle/endpoint readiness failed (${last_readiness_failure}); inspect ${LOG_FILE}." >&2
  exit 1
}
echo "Localized Nav2 ready. First use RViz 2D Pose Estimate, then 2D Goal Pose."
echo "AMCL map -> base_link TF is expected after the initial pose estimate."
