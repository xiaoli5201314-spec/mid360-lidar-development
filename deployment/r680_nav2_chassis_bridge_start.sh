#!/usr/bin/env bash
set -euo pipefail
[[ "$#" -ge 1 && "$#" -le 2 && "${1:-}" == "--arm" ]] ||
  { echo "Usage: $0 --arm [--reverse]" >&2; exit 2; }
allow_reverse=0
if [[ "$#" -eq 2 ]]; then
  [[ "${2:-}" == "--reverse" ]] ||
    { echo "Usage: $0 --arm [--reverse]" >&2; exit 2; }
  allow_reverse=1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
DEVICE=/dev/serial/by-id/YOUR_CHASSIS_SERIAL
PID_FILE=/root/r680_nav/run/r680_nav2_chassis_bridge.pid
PGID_FILE=/root/r680_nav/run/r680_nav2_chassis_bridge.pgid
LOG_FILE=/root/r680_nav/logs/r680_nav2_chassis_bridge.log
TX_ROOT=/root/r680_nav/run
TX_NAME=r680_nav2_chassis_bridge
TX_SCRIPT=r680_nav2_chassis_bridge_start.sh
mkdir -p /root/r680_nav/run /root/r680_nav/logs
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
    rm -- "${TX_ROOT}/${TX_NAME}.starter" || true
    flock -u 9
    exec 9>&-
    TX_OWNED=0
  fi
}
# END transaction protocol
start_transaction || exit 1
preflight_cleanup() {
  local rc=$?
  trap - EXIT
  finish_transaction || rc=1
  exit "${rc}"
}
trap preflight_cleanup EXIT
exec 8>/root/r680_nav/run/r680_nav2_chassis_bridge.instance.lock
flock -n 8 || { echo "Another bridge start/stop is in progress." >&2; exit 1; }
[[ -c "${DEVICE}" ]] || { echo "Fixed chassis serial device is missing." >&2; exit 1; }
command -v fuser >/dev/null || { echo "fuser is required to check serial ownership." >&2; exit 1; }
if fuser "${DEVICE}" >/dev/null 2>&1; then
  echo "Serial device is already occupied; refusing to arm." >&2
  exit 1
fi
if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(tr -d '\r\n' <"${PID_FILE}")"
  [[ "${old_pid}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid bridge PID record." >&2; exit 1; }
  if kill -0 "${old_pid}" 2>/dev/null; then
    echo "Recorded bridge PID still exists; inspect before arming." >&2
    exit 1
  fi
fi
# Inspect argv tokens; do not match the command string of the caller.
for proc in /proc/[0-9]*/cmdline; do
  [[ -r "${proc}" ]] || continue
  if tr '\0' '\n' <"${proc}" 2>/dev/null | grep -Eq '(^|/)r680_.*ackermann_bridge\.(py|sh)$'; then
    echo "An existing chassis bridge was found; refusing to arm." >&2
    exit 1
  fi
done
cmd_info="$(timeout 6 ros2 topic info /cmd_vel -v 2>/dev/null)" ||
  { echo "/cmd_vel discovery failed." >&2; exit 1; }
topic_type="$(awk '/^Type:/ {print $2; exit}' <<<"${cmd_info}")"
[[ "${topic_type}" == "geometry_msgs/msg/Twist" ]] ||
  { echo "/cmd_vel has an unexpected message type." >&2; exit 1; }
publisher_count="$(awk '/^Publisher count:/ {print $3}' <<<"${cmd_info}")"
[[ "${publisher_count}" =~ ^[0-9]+$ ]] ||
  { echo "/cmd_vel publisher count is malformed." >&2; exit 1; }
publisher_endpoints="$(awk '
  /^Node name:/ {node=$3}
  /^Node namespace:/ {namespace=$3}
  /^Endpoint type:/ {
    if ($3=="PUBLISHER") {
      if (node=="" || namespace=="") exit 2
      print namespace ":" node
    }
    node=""; namespace=""
  }
' <<<"${cmd_info}")" ||
  { echo "/cmd_vel publisher endpoint data is malformed." >&2; exit 1; }
mapfile -t cmd_publishers <<<"${publisher_endpoints}"
[[ "${#cmd_publishers[@]}" -eq "${publisher_count}" ]] ||
  { echo "/cmd_vel publisher count does not match its endpoint records." >&2; exit 1; }
behavior_count=0
smoother_count=0
for endpoint in "${cmd_publishers[@]}"; do
  case "${endpoint}" in
    /:behavior_server) behavior_count=$((behavior_count + 1)) ;;
    /:velocity_smoother) smoother_count=$((smoother_count + 1)) ;;
    *) echo "Unexpected /cmd_vel publisher: ${endpoint}" >&2; exit 1 ;;
  esac
done
[[ "${publisher_count}" -eq 2 && "${behavior_count}" -eq 1 && "${smoother_count}" -eq 1 ]] ||
  { echo "/cmd_vel must have exactly one behavior_server and one velocity_smoother publisher." >&2; exit 1; }

require_active_lifecycle_node() {
  local node="$1" state="" query_succeeded=0
  for _ in {1..5}; do
    if state="$(timeout 4 ros2 lifecycle get "${node}" 2>/dev/null)"; then
      query_succeeded=1
      [[ "${state}" == "active [3]" ]] && return 0
    fi
    sleep 0.25
  done
  if (( query_succeeded == 0 )); then
    echo "Lifecycle query failed for ${node} after bounded retries." >&2
  else
    echo "${node} must be active before arming the chassis bridge; last state: ${state:-<none>}." >&2
  fi
  return 1
}
require_active_lifecycle_node /behavior_server
require_active_lifecycle_node /velocity_smoother

get_param_exact_with_retry() {
  local node="$1" parameter="$2" expected="$3" label="$4"
  local output=""
  # Composable Nav2 nodes can briefly reject parameter services while the
  # lifecycle manager has just finished activating them. Retry discovery,
  # but keep the exact value check so this never weakens the arm gate.
  for _ in {1..5}; do
    output="$(timeout 8 ros2 param get "${node}" "${parameter}" 2>/dev/null || true)"
    if grep -Fqx "${expected}" <<<"${output}"; then
      return 0
    fi
    sleep 0.25
  done
  echo "Could not verify ${label}; last response: ${output:-<none>}" >&2
  return 1
}

get_param_exact_with_retry /behavior_server behavior_plugins "String values are: ['wait']" \
  "behavior_server plugins" ||
  { echo "behavior_server must load only wait before arming the chassis bridge." >&2; exit 1; }
get_param_exact_with_retry /behavior_server wait.plugin "String value is: nav2_behaviors/Wait" \
  "behavior_server wait plugin" ||
  { echo "behavior_server wait.plugin must be nav2_behaviors/Wait." >&2; exit 1; }

# BEGIN launch ownership
stop_owned_launch() {
  local pid="${owned_pid:-${!:-}}" cmd pgid
  (( launch_attempt == 1 )) || return 0
  [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || return 0
  for _ in {1..40}; do
    kill -0 "${pid}" 2>/dev/null || return 0
    cmd="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
    if [[ " ${cmd} " == *" /root/r680_nav/scripts/r680_ackermann_bridge.py "* &&
          " ${cmd} " == *" --write "* && "${pgid}" == "${pid}" ]]; then break; fi
    [[ "$(ps -o stat= -p "${pid}" 2>/dev/null || true)" == Z* ]] && return 0
    sleep 0.05 7>&- 8>&- 9>&-
  done
  [[ " ${cmd} " == *" /root/r680_nav/scripts/r680_ackermann_bridge.py "* &&
     " ${cmd} " == *" --write "* && "${pgid}" == "${pid}" ]] ||
    { echo "Cannot verify in-memory bridge PID ${pid}; use physical emergency stop." >&2; return 1; }
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in {1..120}; do
    if ! kill -0 "${pid}" 2>/dev/null ||
       [[ "$(ps -o stat= -p "${pid}" 2>/dev/null || true)" == Z* ]]; then
      wait "${pid}" 2>/dev/null || true
      return 0
    fi
    sleep 0.1 7>&- 8>&- 9>&-
  done
  echo "Bridge rollback timed out for in-memory PID ${pid}; use physical emergency stop." >&2
  return 1
}
# END launch ownership
launch_attempt=0
owned_pid=
ready=0
bridge_reverse_args=()
if (( allow_reverse == 1 )); then
  bridge_reverse_args+=(--allow-reverse)
fi
cleanup() {
  local rc=$?
  trap - EXIT
  trap '' INT TERM
  if (( launch_attempt == 1 && ready == 0 )); then
    # No PID file is needed: this shell still owns the child in memory.
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
# BEGIN launch process
check_cancelled
launch_attempt=1
setsid python3 -u /root/r680_nav/scripts/r680_ackermann_bridge.py \
  --device "${DEVICE}" --baudrate 115200 --cmd-topic /cmd_vel \
  --rate 20 --cmd-timeout 0.5 --max-linear-x 0.26 --max-angular-z 0.19 \
  --minimum-turning-radius 1.40 --startup-delay 6.0 --write --allow-nonzero \
  "${bridge_reverse_args[@]}" --control-lock /root/r680_nav/run/r680_chassis_control.lock \
  >"${LOG_FILE}" 2>&1 7>&- 8>&- 9>&- &
owned_pid=$!
pid="${owned_pid}"
check_cancelled
printf '%s\n' "${owned_pid}" >"${PID_FILE}"
printf '%s\n' "${owned_pid}" >"${PGID_FILE}"
# END launch process
sleep 1 7>&- 8>&- 9>&-
kill -0 "${pid}" 2>/dev/null ||
  { echo "Bridge exited during startup; inspect ${LOG_FILE}." >&2; exit 1; }
ready=1
echo "C50C V4 bridge armed (PID/PGID ${pid}); startup neutral delay is 6 seconds."
