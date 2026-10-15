#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STOP_HELPER="${SCRIPT_DIR}/r680_process_stop.sh"

set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

PARAMS_FILE="${1:-/root/r680_nav/config/r680_nav2_params.yaml}"
LOG_DIR="/root/r680_nav/logs"
RUN_DIR="/root/r680_nav/run"
STATIC_TF_PID_FILE="${RUN_DIR}/r680_nav_map_to_camera_init.pid"
mkdir -p "${LOG_DIR}" "${RUN_DIR}"

if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "Nav2 params not found: ${PARAMS_FILE}" >&2
  exit 1
fi

STATIC_TF_LOG="${LOG_DIR}/r680_nav_map_to_camera_init.log"
NAV2_LOG="${LOG_DIR}/r680_nav_navigation.log"
NAV2_PID_FILE="${RUN_DIR}/r680_nav2_navigation.pid"
NAV2_PGID_FILE="${RUN_DIR}/r680_nav2_navigation.pgid"

"${STOP_HELPER}" --pid-file "${STATIC_TF_PID_FILE}" --match "static_transform_publisher" --label "map_to_camera_init static transform" >/dev/null 2>&1 || true
rm -f -- "${STATIC_TF_PID_FILE}"
sleep 1

/opt/ros/humble/lib/tf2_ros/static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id camera_init \
  > "${STATIC_TF_LOG}" 2>&1 &
STATIC_TF_PID=$!
printf '%s\n' "${STATIC_TF_PID}" > "${STATIC_TF_PID_FILE}"

setsid ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=False autostart:=True use_composition:=False use_respawn:=False \
  params_file:="${PARAMS_FILE}" \
  > "${NAV2_LOG}" 2>&1 &
NAV2_PID=$!
printf '%s\n' "${NAV2_PID}" > "${NAV2_PID_FILE}"
NAV2_PGID="$(ps -o pgid= -p "${NAV2_PID}" 2>/dev/null | tr -d ' ')"
if [[ "${NAV2_PGID}" =~ ^[0-9]+$ ]]; then
  printf '%s\n' "${NAV2_PGID}" > "${NAV2_PGID_FILE}"
else
  rm -f -- "${NAV2_PGID_FILE}"
fi

stop_nav2_group() {
  local shell_pgid group_safe=1 pid cmd
  shell_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ')"
  if [[ ! "${NAV2_PGID:-}" =~ ^[0-9]+$ ]] ||
     [[ "${NAV2_PGID}" == "0" || "${NAV2_PGID}" == "${shell_pgid}" ]]; then
    group_safe=0
  else
    while read -r pid; do
      [[ "${pid}" =~ ^[0-9]+$ ]] || continue
      [[ "${pid}" == "$$" ]] && continue
      cmd="$(ps -p "${pid}" -o args= 2>/dev/null || true)"
      case "${cmd}" in
        *"nav2_bringup navigation_launch.py"*|*"/opt/ros/humble/lib/nav2_"*) ;;
        *) group_safe=0; break ;;
      esac
    done < <(ps -eo pid=,pgid= | awk -v wanted="${NAV2_PGID}" '$2 == wanted {print $1}')
  fi

  if (( group_safe == 1 )); then
    kill -TERM -- "-${NAV2_PGID}" 2>/dev/null || true
  else
    kill -TERM "${NAV2_PID}" 2>/dev/null || true
  fi
  for _ in $(seq 1 30); do
    kill -0 "${NAV2_PID}" 2>/dev/null || return 0
    sleep 0.2
  done
  if (( group_safe == 1 )); then
    kill -KILL -- "-${NAV2_PGID}" 2>/dev/null || true
  else
    kill -KILL "${NAV2_PID}" 2>/dev/null || true
  fi
}

cleanup() {
  stop_nav2_group || true
  kill "${STATIC_TF_PID}" 2>/dev/null || true
  wait "${NAV2_PID}" 2>/dev/null || true
  wait "${STATIC_TF_PID}" 2>/dev/null || true
  rm -f -- "${STATIC_TF_PID_FILE}"
  rm -f -- "${NAV2_PID_FILE}" "${NAV2_PGID_FILE}"
}
trap cleanup EXIT INT TERM

echo "Nav2 stack starting..."
echo "  params: ${PARAMS_FILE}"
echo "  static tf log: ${STATIC_TF_LOG}"
echo "  static tf pid: ${STATIC_TF_PID_FILE}"
echo "  nav2 log: ${NAV2_LOG}"
echo "Press Ctrl+C to stop."

wait "${NAV2_PID}"
