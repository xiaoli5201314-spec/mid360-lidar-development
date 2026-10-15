#!/usr/bin/env bash
set -euo pipefail

RUN_DIR=/root/r680_nav/run
LOG_DIR=/root/r680_nav/logs
PID_FILE="${RUN_DIR}/r680_rviz_multi_goal.pid"
PGID_FILE="${RUN_DIR}/r680_rviz_multi_goal.pgid"
SCRIPT=/root/r680_nav/scripts/r680_rviz_multi_goal.py

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

process_birth() {
  [[ -r "/proc/$1/stat" ]] || return 1
  sed 's/^.*) //' "/proc/$1/stat" | awk '{print $20}'
}

owned_process() {
  local pid="$1" birth="$2" arg found=0
  [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  [[ "$(process_birth "${pid}" 2>/dev/null || true)" == "${birth}" ]] || return 1
  while IFS= read -r -d '' arg; do
    [[ "${arg##*/}" == "r680_rviz_multi_goal.py" ||
       "${arg##*/}" == "r680_rviz_multi_goal.sh" ]] && found=1
  done <"/proc/${pid}/cmdline"
  (( found == 1 ))
}

if [[ -r "${PID_FILE}" ]]; then
  old_pid="$(tr -d '\r\n' <"${PID_FILE}")"
  old_birth="$(tr -d '\r\n' <"${PID_FILE}.birth" 2>/dev/null || true)"
  if [[ -n "${old_birth}" ]] && owned_process "${old_pid}" "${old_birth}"; then
    echo "RViz multi-goal queue already running (PID ${old_pid})."
    exit 0
  fi
  rm -f -- "${PID_FILE}" "${PGID_FILE}" "${PID_FILE}.birth"
fi

setsid /root/r680_nav/scripts/r680_rviz_multi_goal.sh \
  >"${LOG_DIR}/r680_rviz_multi_goal.log" 2>&1 &
pid=$!
birth="$(process_birth "${pid}" 2>/dev/null || true)"
[[ -n "${birth}" ]] || { echo "Cannot record multi-goal process identity." >&2; exit 1; }
pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
[[ "${pgid}" == "${pid}" ]] || {
  kill -TERM "${pid}" 2>/dev/null || true
  echo "Multi-goal process did not establish a dedicated process group." >&2
  exit 1
}
printf '%s\n' "${pid}" >"${PID_FILE}"
printf '%s\n' "${pgid}" >"${PGID_FILE}"
printf '%s\n' "${birth}" >"${PID_FILE}.birth"

for _ in {1..40}; do
  owned_process "${pid}" "${birth}" || {
    echo "Multi-goal queue exited during startup; inspect ${LOG_DIR}/r680_rviz_multi_goal.log" >&2
    exit 1
  }
  grep -q "RViz multi-goal ready" "${LOG_DIR}/r680_rviz_multi_goal.log" && {
    echo "RViz multi-goal queue ready (PID ${pid})."
    exit 0
  }
  sleep 0.1
done
echo "Multi-goal queue did not report ready; inspect ${LOG_DIR}/r680_rviz_multi_goal.log" >&2
exit 1
