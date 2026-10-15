#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="${OCTOMAP_RUN_DIR:-/root/r680_nav/run/octomap3d}"
PID_FILE="${RUN_DIR}/octomap3d.pid"
PGID_FILE="${RUN_DIR}/octomap3d.pgid"
THROTTLE_PID_FILE="${RUN_DIR}/octomap3d_throttle.pid"
THROTTLE_PGID_FILE="${RUN_DIR}/octomap3d_throttle.pgid"

is_number() {
  [[ "${1:-}" =~ ^[0-9]+$ ]]
}

group_has_owned_process() {
  local pgid="$1"
  ps -eo pgid=,args= 2>/dev/null | awk -v g="$pgid" '
    $1 == g && $0 ~ /(octomap_server|topic_tools.*throttle)/ { found=1 }
    END { exit(found ? 0 : 1) }
  '
}

kill_group() {
  local label="$1" pid_file="$2" pgid_file="$3"
  local pid="" pgid="" cmdline=""

  [[ -s "${pid_file}" ]] && pid="$(tr -dc '0-9' <"${pid_file}")"
  [[ -s "${pgid_file}" ]] && pgid="$(tr -dc '0-9' <"${pgid_file}")"

  # New instances have a dedicated process group. Validate ownership before
  # sending a negative-PID signal so an unrelated user's group is untouched.
  if is_number "${pgid}" && [[ "${pgid}" != "0" ]] && group_has_owned_process "${pgid}"; then
    kill -TERM -- "-${pgid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! pgrep -g "${pgid}" >/dev/null 2>&1; then
        rm -f -- "${pid_file}" "${pgid_file}"
        echo "${label} stopped."
        return 0
      fi
      sleep 0.25
    done
    kill -KILL -- "-${pgid}" 2>/dev/null || true
    sleep 0.25
    if ! pgrep -g "${pgid}" >/dev/null 2>&1; then
      rm -f -- "${pid_file}" "${pgid_file}"
      echo "${label} stopped."
      return 0
    fi
    echo "${label} process group did not stop within 5 seconds (PGID ${pgid})." >&2
    return 1
  fi

  # Backward-compatible cleanup for the original script, which only recorded
  # the ros2 Python launcher PID. Kill its direct children first; do not derive
  # and signal its inherited shell process group.
  if is_number "${pid}" && [[ -r "/proc/${pid}/cmdline" ]]; then
    cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline")"
    if [[ "${cmdline}" == *octomap_server* || "${cmdline}" == *topic_tools* ]]; then
      pkill -TERM -P "${pid}" 2>/dev/null || true
      kill -TERM "${pid}" 2>/dev/null || true
      for _ in $(seq 1 20); do
        if ! kill -0 "${pid}" 2>/dev/null; then
          rm -f -- "${pid_file}" "${pgid_file}"
          echo "${label} stopped (legacy PID cleanup)."
          return 0
        fi
        sleep 0.25
      done
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  fi

  rm -f -- "${pid_file}" "${pgid_file}"
  [[ -n "${pid}" ]] && echo "${label} PID was stale; records removed."
  return 0
}

result=0
kill_group "OctoMap 3D server" "${PID_FILE}" "${PGID_FILE}" || result=1
kill_group "3D input throttle" "${THROTTLE_PID_FILE}" "${THROTTLE_PGID_FILE}" || result=1
exit "${result}"
