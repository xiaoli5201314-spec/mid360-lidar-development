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

MAP_YAML="${1:-/root/r680_nav/maps/mid360_slam2d_latest.yaml}"
LOG_DIR="/root/r680_nav/logs"
RUN_DIR="/root/r680_nav/run"
MAP_PID_FILE="${RUN_DIR}/r680_nav_map_server.pid"
MAP_PGID_FILE="${RUN_DIR}/r680_nav_map_server.pgid"
mkdir -p "${LOG_DIR}" "${RUN_DIR}"

reset_ros2_daemon() {
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 2
}

lifecycle_set_retry() {
  local transition="$1"
  local attempt

  for attempt in 1 2 3; do
    if timeout 10 ros2 lifecycle set /map_server "${transition}"; then
      return 0
    fi

    echo "[WARN] map_server lifecycle '${transition}' failed on attempt ${attempt}; resetting ROS2 daemon..." >&2
    reset_ros2_daemon
    sleep 1
  done

  echo "[FAIL] map_server lifecycle '${transition}' failed after retries." >&2
  return 1
}

if [[ ! -f "${MAP_YAML}" ]]; then
  echo "Map yaml not found: ${MAP_YAML}" >&2
  exit 1
fi

if ros2 node list 2>/dev/null | grep -qx '/map_server'; then
  echo "Refusing map_server start: /map_server is already advertised; stop the owned instance first." >&2
  exit 1
fi
sleep 1
reset_ros2_daemon

setsid ros2 run nav2_map_server map_server --ros-args -p "yaml_filename:=${MAP_YAML}" \
  > "${LOG_DIR}/r680_nav_map_server.log" 2>&1 &
MAP_PID=$!
printf '%s\n' "${MAP_PID}" > "${MAP_PID_FILE}"
MAP_PGID="$(ps -o pgid= -p "${MAP_PID}" 2>/dev/null | tr -d ' ')"
if [[ "${MAP_PGID}" =~ ^[0-9]+$ ]]; then
  printf '%s\n' "${MAP_PGID}" > "${MAP_PGID_FILE}"
else
  rm -f -- "${MAP_PGID_FILE}"
fi

cleanup() {
  shell_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ')"
  if [[ "${MAP_PGID:-}" =~ ^[0-9]+$ && "${MAP_PGID}" != "0" && "${MAP_PGID}" != "${shell_pgid}" ]]; then
    kill -TERM -- "-${MAP_PGID}" 2>/dev/null || true
  else
    kill "${MAP_PID}" 2>/dev/null || true
  fi
  wait "${MAP_PID}" 2>/dev/null || true
  rm -f -- "${MAP_PID_FILE}" "${MAP_PGID_FILE}"
}
trap cleanup EXIT INT TERM

sleep 2
lifecycle_set_retry configure
lifecycle_set_retry activate

echo "R680 Nav2 map_server is active."
echo "Map: ${MAP_YAML}"
echo "Topic: /map"
echo "Log: ${LOG_DIR}/r680_nav_map_server.log"
echo "Press Ctrl+C to stop."

wait "${MAP_PID}"
