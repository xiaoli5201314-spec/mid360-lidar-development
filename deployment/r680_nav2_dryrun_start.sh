#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

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

LOG_DIR="/root/r680_nav/logs"
RUN_DIR="/root/r680_nav/run"
SCAN_PID_FILE="${RUN_DIR}/r680_nav2_dryrun_scan.pid"
SCAN_OWNED_FILE="${RUN_DIR}/r680_nav2_dryrun_scan.owned"
mkdir -p "${LOG_DIR}" "${RUN_DIR}"
startup_attempted=0
startup_success=0

# BEGIN startup rollback
cleanup_failed_start() {
  local rc=$?
  trap - EXIT INT TERM
  if (( startup_success == 0 && startup_attempted == 1 )); then
    echo "Nav2 startup failed; rolling back owned scan, map_server and Nav2 processes." >&2
    /root/r680_nav/scripts/r680_nav2_dryrun_stop.sh --quiet || rc=1
  fi
  exit "${rc}"
}
# END startup rollback
trap cleanup_failed_start EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

reset_ros2_daemon() {
  echo "Resetting ROS2 daemon..."
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 2
}

# BEGIN preflight
require_exec() {
  local path="$1"
  [[ -x "${path}" ]] || {
    echo "Required executable helper is missing: ${path}" >&2
    return 1
  }
}

preflight_required_helpers() {
  local helper
  for helper in \
    /root/r680_nav/scripts/r680_process_stop.sh \
    /root/r680_nav/scripts/r680_nav2_dryrun_stop.sh \
    /root/r680_nav/scripts/r680_mid360_slam2d_stop.sh \
    /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan.sh \
    /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan_stop.sh \
    /root/r680_nav/scripts/r680_nav_map_server.sh \
    /root/r680_nav/scripts/r680_nav_nav2_stack.sh; do
    require_exec "${helper}"
  done
}
# END preflight

# BEGIN topic ownership
topic_publisher_count() {
  local topic="$1" info count topics
  if info="$(timeout 5 ros2 topic info "${topic}" 2>/dev/null)"; then
    count="$(awk -F: '/^Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${info}")"
    [[ "${count}" =~ ^[0-9]+$ ]] || return 1
    printf '%s\n' "${count}"
    return 0
  fi
  topics="$(timeout 5 ros2 topic list 2>/dev/null)" || return 1
  if grep -Fxq "${topic}" <<<"${topics}"; then
    return 1
  fi
  printf '0\n'
}

require_zero_publishers() {
  local topic="$1" count
  count="$(topic_publisher_count "${topic}")" || {
    echo "Cannot determine publisher count for ${topic}." >&2
    return 1
  }
  if (( count != 0 )); then
    echo "${topic} must have zero publishers before this workflow starts; found ${count}." >&2
    return 1
  fi
}

ensure_owned_scan() {
  local count scan_pid
  timeout 6 ros2 topic echo /cloud_registered_body --once --field header >/dev/null 2>&1 || {
    echo "Required live /cloud_registered_body is unavailable; refusing to start /scan." >&2
    return 1
  }
  count="$(topic_publisher_count /scan)" || {
    echo "Cannot determine /scan publisher ownership." >&2
    return 1
  }
  if (( count != 0 )); then
    echo "/scan must have zero publishers before Nav2 owns it; found ${count}." >&2
    return 1
  fi
  # Ownership marker survives until dry-run stop confirms scan is gone.
  printf 'starting\n' >"${SCAN_OWNED_FILE}"
  CLOUD_IN=/cloud_registered_body SCAN_OUT=/scan SCAN_FRAME=body \
    setsid /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan.sh \
    >"${LOG_DIR}/r680_nav2_dryrun_scan.log" 2>&1 &
  scan_pid=$!
  printf '%s\n' "${scan_pid}" >"${SCAN_PID_FILE}"
  printf '%s\n' "${scan_pid}" >"${SCAN_OWNED_FILE}"
  for _ in $(seq 1 20); do
    kill -0 "${scan_pid}" 2>/dev/null || {
      echo "pointcloud_to_laserscan exited during Nav2 startup." >&2
      return 1
    }
    count="$(topic_publisher_count /scan)" || { sleep 1; continue; }
    (( count <= 1 )) || {
      echo "/scan must have exactly one live publisher; found ${count}." >&2
      return 1
    }
    if (( count == 1 )); then
      echo "[OK] /scan has one workflow-owned publisher."
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for /scan from pointcloud_to_laserscan." >&2
  return 1
}

verify_single_map_server_publisher() {
  local info count
  info="$(timeout 5 ros2 topic info /map -v 2>/dev/null)" || {
    echo "Cannot inspect /map publisher ownership." >&2
    return 1
  }
  count="$(awk -F: '/^Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${info}")"
  [[ "${count}" == "1" ]] || {
    echo "/map must have exactly one publisher; found ${count:-unknown}." >&2
    return 1
  }
  awk '
    /^Node name:/ {node=$3; sub(/^\//, "", node)}
    /^Endpoint type:/ {if (node=="map_server" && $3=="PUBLISHER") owned=1}
    END {exit !owned}
  ' <<<"${info}" || {
    echo "/map publisher is not the owned map_server node." >&2
    return 1
  }
}
# END topic ownership

resolve_map_yaml() {
  local requested="${1:-}"
  if [[ -n "${requested}" ]]; then
    [[ "${requested}" == *.yaml ]] && printf '%s\n' "${requested}" || printf '%s.yaml\n' "${requested}"
    return
  fi
  printf '%s\n' /root/r680_nav/maps/mid360_slam2d_latest.yaml
}

MAP_YAML="$(resolve_map_yaml "${1:-}")"
PARAMS_FILE="${2:-/root/r680_nav/config/r680_nav2_params.yaml}"
[[ -f "${MAP_YAML}" ]] || { echo "Map yaml not found: ${MAP_YAML}" >&2; exit 1; }
[[ -f "${PARAMS_FILE}" ]] || { echo "Nav2 params not found: ${PARAMS_FILE}" >&2; exit 1; }

if [[ "${MAP_YAML}" == "/root/r680_nav/maps/mid360_slam2d_latest.yaml" ]]; then
  map_image="$(awk '$1 == "image:" {print $2; exit}' "${MAP_YAML}")"
  [[ "${map_image}" == "mid360_slam2d_latest.pgm" && -s "/root/r680_nav/maps/${map_image}" ]] || {
    echo "Latest map pair is invalid: YAML image=${map_image:-<missing>}" >&2
    exit 1
  }
fi

preflight_required_helpers

echo "=== R680 Nav2 dry-run start ==="
date '+%F %T %Z'
echo "Map: ${MAP_YAML}"
echo "Params: ${PARAMS_FILE}"
echo "Checking FAST-LIO odometry and TF before changing the active workflow..."
timeout 6 ros2 topic echo /Odometry --once --field header >/dev/null 2>&1 || {
  echo "[FAIL] /Odometry was not received within 6s." >&2
  exit 1
}
echo "[OK] /Odometry is alive."
timeout 6 ros2 run tf2_ros tf2_echo camera_init base_link >/tmp/r680_nav2_tf_check.txt 2>&1 || true
grep -q "Translation:" /tmp/r680_nav2_tf_check.txt || {
  echo "[FAIL] TF camera_init -> base_link was not available within 6s." >&2
  tail -20 /tmp/r680_nav2_tf_check.txt >&2 || true
  exit 1
}
echo "[OK] TF camera_init -> base_link is available."

echo "Stopping old Nav2/map_server and 2D-SLAM processes that may conflict..."
startup_attempted=1
/root/r680_nav/scripts/r680_nav2_dryrun_stop.sh --quiet
/root/r680_nav/scripts/r680_mid360_slam2d_stop.sh
sleep 1
reset_ros2_daemon
require_zero_publishers /scan
require_zero_publishers /map

echo "Starting one workflow-owned /scan publisher for the local costmap..."
ensure_owned_scan

echo "Starting map_server..."
nohup /root/r680_nav/scripts/r680_nav_map_server.sh "${MAP_YAML}" \
  >"${LOG_DIR}/r680_nav_map_server_wrapper.log" 2>&1 &
printf '%s\n' "$!" >"${RUN_DIR}/r680_nav_map_server_wrapper.pid"
map_ready=0
for _ in $(seq 1 20); do
  if timeout 3 ros2 topic echo /map --once --field info >/tmp/r680_nav2_map_info.txt 2>/dev/null &&
     grep -Eq "width=[1-9]|width: [1-9]" /tmp/r680_nav2_map_info.txt; then
    map_ready=1
    break
  fi
  sleep 1
done
[[ "${map_ready}" == "1" ]] || {
  echo "[FAIL] /map did not become ready." >&2
  tail -80 "${LOG_DIR}/r680_nav_map_server_wrapper.log" >&2 || true
  exit 1
}
verify_single_map_server_publisher
echo "[OK] /map has exactly one owned map_server publisher."

echo "Starting Nav2 navigation stack..."
nohup /root/r680_nav/scripts/r680_nav_nav2_stack.sh "${PARAMS_FILE}" \
  >"${LOG_DIR}/r680_nav_nav2_stack_wrapper.log" 2>&1 &
printf '%s\n' "$!" >"${RUN_DIR}/r680_nav_nav2_stack_wrapper.pid"
nav2_lifecycle_nodes=(/controller_server /planner_server /bt_navigator /global_costmap/global_costmap /local_costmap/local_costmap /velocity_smoother)
nav_ready=0
for _ in $(seq 1 90); do
  nav_ready=1
  for node_name in "${nav2_lifecycle_nodes[@]}"; do
    node_state="$(timeout 4 ros2 lifecycle get "${node_name}" 2>/dev/null | awk 'NR == 1 {print $1; exit}')"
    [[ "${node_state}" == "active" ]] || { nav_ready=0; break; }
  done
  if [[ "${nav_ready}" == "1" ]]; then
    goal_info="$(timeout 4 ros2 topic info /goal_pose -v 2>/dev/null || true)"
    cmd_info="$(timeout 4 ros2 topic info /cmd_vel -v 2>/dev/null || true)"
    grep -q "Node name: bt_navigator" <<<"${goal_info}" || nav_ready=0
    grep -q "Node name: velocity_smoother" <<<"${cmd_info}" || nav_ready=0
  fi
  [[ "${nav_ready}" == "1" ]] && break
  sleep 1
done
[[ "${nav_ready}" == "1" ]] || {
  echo "[FAIL] Nav2 did not reach active state within the wait window." >&2
  tail -120 "${LOG_DIR}/r680_nav_nav2_stack_wrapper.log" >&2 || true
  exit 1
}
echo "[OK] Nav2 lifecycle active; /goal_pose and /cmd_vel endpoints are ready."

timeout 8 ros2 run tf2_ros tf2_echo map base_link >/tmp/r680_nav2_map_tf_check.txt 2>&1 || true
grep -q "Translation:" /tmp/r680_nav2_map_tf_check.txt || {
  echo "[FAIL] TF map -> base_link is required for Nav2 dry-run." >&2
  tail -20 /tmp/r680_nav2_map_tf_check.txt >&2 || true
  exit 1
}
echo "[OK] TF map -> base_link is available."
startup_success=1

cat <<EOF

=== Nav2 dry-run ready ===
RViz on VM:
  ~/r680_nav2_dryrun_view_rviz.sh
Watch velocity output on RK3588:
  /root/r680_nav/scripts/r680_nav2_cmd_vel_watch.sh
Status:
  /root/r680_nav/scripts/r680_nav2_dryrun_status.sh
Stop:
  /root/r680_nav/scripts/r680_nav2_dryrun_stop.sh
Safety note:
  This dry-run only starts Nav2 and publishes ROS /cmd_vel.
  It does not start any RK3588 -> STM32 serial bridge.
EOF
