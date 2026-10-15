#!/usr/bin/env bash
set -euo pipefail

RUN_DIR=/root/r680_nav/run
PID_FILE="${RUN_DIR}/r680_rviz_multi_goal.pid"
PGID_FILE="${RUN_DIR}/r680_rviz_multi_goal.pgid"

[[ -r "${PID_FILE}" ]] || { echo "RViz multi-goal queue is not recorded."; exit 0; }
pid="$(tr -d '\r\n' <"${PID_FILE}")"
birth="$(tr -d '\r\n' <"${PID_FILE}.birth" 2>/dev/null || true)"
pgid="$(tr -d '\r\n' <"${PGID_FILE}" 2>/dev/null || true)"
[[ "${pid}" =~ ^[1-9][0-9]*$ && "${pgid}" == "${pid}" && -n "${birth}" ]] || {
  echo "Invalid multi-goal PID/PGID record; refusing a blind signal." >&2
  exit 1
}
[[ "$(sed 's/^.*) //' "/proc/${pid}/stat" 2>/dev/null | awk '{print $20}' || true)" == "${birth}" ]] || {
  rm -f -- "${PID_FILE}" "${PGID_FILE}" "${PID_FILE}.birth"
  echo "RViz multi-goal queue already stopped (stale record removed)."
  exit 0
}
cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
[[ "${cmdline}" == *"r680_rviz_multi_goal.py"* ||
   "${cmdline}" == *"r680_rviz_multi_goal.sh"* ]] || {
  echo "Refusing to stop unowned PID ${pid}." >&2
  exit 1
}
kill -TERM -- "-${pgid}" 2>/dev/null || true
for _ in {1..60}; do
  kill -0 "${pid}" 2>/dev/null || break
  sleep 0.1
done
if kill -0 "${pid}" 2>/dev/null; then
  echo "RViz multi-goal queue did not stop; use the navigation stop command again." >&2
  exit 1
fi
rm -f -- "${PID_FILE}" "${PGID_FILE}" "${PID_FILE}.birth"
echo "RViz multi-goal queue stopped; route cleared."
