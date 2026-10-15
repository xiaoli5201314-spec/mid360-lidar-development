#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "${SCRIPT_DIR}/r680_process_stop.sh" ]]; then
  STOP_HELPER="${SCRIPT_DIR}/r680_process_stop.sh"
elif [[ -x "${SCRIPT_DIR}/../r680_process_stop.sh" ]]; then
  STOP_HELPER="${SCRIPT_DIR}/../r680_process_stop.sh"
else
  echo "Safe process stop helper is missing." >&2
  exit 1
fi

quiet=0
if [[ "${1:-}" == "--quiet" ]]; then
  quiet=1
fi

result=0

STATIC_TF_PID_FILE=/root/r680_nav/run/r680_nav_map_to_camera_init.pid
NAV2_PGID_FILE=/root/r680_nav/run/r680_nav2_navigation.pgid
MAP_PGID_FILE=/root/r680_nav/run/r680_nav_map_server.pgid
"${STOP_HELPER}" --pid-file "${STATIC_TF_PID_FILE}" --match "static_transform_publisher" --label "map_to_camera_init static transform" >/dev/null 2>&1 || result=1
if (( result == 0 )); then rm -f -- "${STATIC_TF_PID_FILE}"; fi

"${STOP_HELPER}" --pid-file /root/r680_nav/run/r680_nav_nav2_stack_wrapper.pid --match "/root/r680_nav/scripts/r680_nav_nav2_stack.sh" --label "Nav2 stack wrapper" >/dev/null 2>&1 || result=1

stop_owned_nav2_group() {
  local pgid shell_pgid safe=1 pid cmd
  [[ -r "${NAV2_PGID_FILE}" ]] || return 0
  pgid="$(tr -d '\r\n' <"${NAV2_PGID_FILE}")"
  [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid Nav2 PGID record." >&2; return 1; }
  shell_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ')"
  [[ "${pgid}" != "${shell_pgid}" ]] || { echo "Refusing to stop the current shell process group." >&2; return 1; }
  while read -r pid; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    [[ "${pid}" == "$$" ]] && continue
    cmd="$(ps -p "${pid}" -o args= 2>/dev/null || true)"
    case "${cmd}" in
      *"nav2_bringup navigation_launch.py"*|*"/opt/ros/humble/lib/nav2_"*) ;;
      *) safe=0; break ;;
    esac
  done < <(ps -eo pid=,pgid= | awk -v wanted="${pgid}" '$2 == wanted {print $1}')
  (( safe == 1 )) || { echo "Nav2 process group ownership check failed." >&2; return 1; }
  kill -TERM -- "-${pgid}" 2>/dev/null || return 1
  sleep 2
  if kill -0 -- "-${pgid}" 2>/dev/null; then
    kill -KILL -- "-${pgid}" 2>/dev/null || return 1
    sleep 1
  fi
  kill -0 -- "-${pgid}" 2>/dev/null && return 1
  return 0
}

stop_owned_nav2_group || result=1
"${STOP_HELPER}" --pid-file /root/r680_nav/run/r680_nav_map_server_wrapper.pid --match "/root/r680_nav/scripts/r680_nav_map_server.sh" --label "map_server wrapper" >/dev/null 2>&1 || result=1

stop_owned_map_group() {
  local pgid shell_pgid safe=1 pid cmd
  [[ -r "${MAP_PGID_FILE}" ]] || return 0
  pgid="$(tr -d '\r\n' <"${MAP_PGID_FILE}")"
  [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid map_server PGID record." >&2; return 1; }
  shell_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ')"
  [[ "${pgid}" != "${shell_pgid}" ]] || { echo "Refusing to stop the current shell process group." >&2; return 1; }
  while read -r pid; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    [[ "${pid}" == "$$" ]] && continue
    cmd="$(ps -p "${pid}" -o args= 2>/dev/null || true)"
    case "${cmd}" in
      *"nav2_map_server map_server"*|*"/opt/ros/humble/lib/nav2_map_server/map_server"*) ;;
      *) safe=0; break ;;
    esac
  done < <(ps -eo pid=,pgid= | awk -v wanted="${pgid}" '$2 == wanted {print $1}')
  (( safe == 1 )) || { echo "map_server process group ownership check failed." >&2; return 1; }
  kill -TERM -- "-${pgid}" 2>/dev/null || return 1
  sleep 2
  if kill -0 -- "-${pgid}" 2>/dev/null; then
    kill -KILL -- "-${pgid}" 2>/dev/null || return 1
    sleep 1
  fi
  kill -0 -- "-${pgid}" 2>/dev/null && return 1
  return 0
}

stop_owned_map_group || result=1
"${STOP_HELPER}" --pid-file /root/r680_nav/run/r680_nav_map_server.pid --match "/opt/ros/humble/lib/nav2_map_server/map_server" --label "map_server" >/dev/null 2>&1 || result=1
if (( result == 0 )); then rm -f /root/r680_nav/run/r680_nav_map_server_wrapper.pid \
      /root/r680_nav/run/r680_nav_nav2_stack_wrapper.pid \
      /root/r680_nav/run/r680_nav2_navigation.pid \
      /root/r680_nav/run/r680_nav2_navigation.pgid \
      /root/r680_nav/run/r680_nav_map_server.pid \
      /root/r680_nav/run/r680_nav_map_server.pgid 2>/dev/null || result=1; fi

if [[ -f /root/r680_nav/run/r680_nav2_dryrun_scan.owned ]]; then
  if /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan_stop.sh >/dev/null 2>&1; then
    rm -f /root/r680_nav/run/r680_nav2_dryrun_scan.pid /root/r680_nav/run/r680_nav2_dryrun_scan.owned
  else
    result=1
  fi
fi

if [[ "${quiet}" != "1" ]]; then
  if (( result == 0 )); then echo "Requested R680 Nav2 dry-run stop."; else echo "R680 Nav2 dry-run stop failed; retain records for inspection." >&2; fi
fi
exit "${result}"
