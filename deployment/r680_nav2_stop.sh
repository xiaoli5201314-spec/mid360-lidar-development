#!/usr/bin/env bash
set -euo pipefail
[[ "$#" -eq 0 || ( "$#" -eq 1 && "${1:-}" == --quiet ) ]] || exit 2
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
TX_ROOT=/root/r680_nav/run
TX_NAME=r680_nav2_localized
TX_SCRIPT=r680_nav2_start.sh
cancel_start_transaction
PID_FILE=/root/r680_nav/run/r680_nav2_localized.pid
PGID_FILE=/root/r680_nav/run/r680_nav2_localized.pgid
[[ -f "${PID_FILE}" ]] || exit 0
pid="$(tr -d '\r\n' <"${PID_FILE}")"
[[ "${pid}" =~ ^[1-9][0-9]*$ && "${pid}" -gt 1 ]] || { echo "Invalid localized PID." >&2; exit 1; }
pgid="${pid}"
if [[ -f "${PGID_FILE}" ]]; then
  pgid="$(tr -d '\r\n' <"${PGID_FILE}")"
fi
[[ "${pgid}" == "${pid}" ]] || { echo "Localized PID/PGID mismatch." >&2; exit 1; }
if [[ -r "/proc/${pid}/cmdline" ]]; then
  leader_state="$(ps -o stat= -p "${pid}" 2>/dev/null | tr -d ' ')" || {
    echo "Unable to query localized leader state; retaining PID/PGID." >&2
    exit 1
  }
  [[ -n "${leader_state}" ]] || {
    echo "Localized leader state query returned no state; retaining PID/PGID." >&2
    exit 1
  }
  if [[ "${leader_state}" != Z* ]]; then
    cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline")"
    [[ "${cmdline}" == *"nav2_bringup bringup_launch.py"* ]] ||
      { echo "Refusing unowned localized PID ${pid}." >&2; exit 1; }
    actual_pgid="$(ps -o pgid= -p "${pid}" | tr -d ' ')"
    [[ "${actual_pgid}" == "${pgid}" ]] || { echo "Localized process group changed." >&2; exit 1; }
  fi
fi
# Validate surviving group members even if the launch leader already exited.
# Process discovery is security-sensitive here: an unreadable member must not
# be treated as absent, because that could make an unrelated process group
# appear owned and allow its PID records to be removed.
validate_group_members() {
  local table members member group state cmd
  table="$(ps -eo pid=,pgid= 2>/dev/null)" || {
    echo "Unable to query localized process group members; retaining PID/PGID." >&2
    return 1
  }
  [[ -n "${table//[[:space:]]/}" ]] || {
    echo "Localized process-group query returned no members; retaining PID/PGID." >&2
    return 1
  }
  members="$(awk -v wanted="${pgid}" '$2==wanted {print $1, $2}' <<<"${table}")" || {
    echo "Unable to filter localized process group members; retaining PID/PGID." >&2
    return 1
  }
  if [[ -z "${members//[[:space:]]/}" ]]; then
    kill -0 -- "-${pgid}" 2>/dev/null && {
      echo "Localized group members disappeared from query; retaining PID/PGID." >&2
      return 1
    }
    return 0
  fi
  while read -r member group; do
    [[ -n "${member:-}" ]] || continue
    [[ "${member}" =~ ^[1-9][0-9]*$ && "${group:-}" == "${pgid}" ]] || {
      echo "Invalid localized process-group query; retaining PID/PGID." >&2
      return 1
    }
    state="$(ps -o stat= -p "${member}" 2>/dev/null | tr -d ' ')" || {
      echo "Unable to query state for localized member ${member}; retaining PID/PGID." >&2
      return 1
    }
    [[ -n "${state}" ]] || {
      echo "Localized member ${member} disappeared during validation; retaining PID/PGID." >&2
      return 1
    }
    [[ "${state}" == Z* ]] && continue
    [[ -r "/proc/${member}/cmdline" ]] || {
      echo "Unable to read localized member ${member}; retaining PID/PGID." >&2
      return 1
    }
    cmd="$(tr '\0' ' ' <"/proc/${member}/cmdline")" || {
      echo "Unable to read localized member ${member}; retaining PID/PGID." >&2
      return 1
    }
    [[ -n "${cmd}" ]] || {
      echo "Empty command line for localized member ${member}; retaining PID/PGID." >&2
      return 1
    }
    case "${cmd}" in
      *"nav2_bringup bringup_launch.py"*|*"/opt/ros/humble/lib/nav2_"*|*"/opt/ros/humble/lib/rclcpp_components/component_container"*) ;;
      *) echo "Refusing unowned member ${member} in localized group ${pgid}." >&2; return 1 ;;
    esac
  done <<<"${members}"
}
validate_group_members || exit 1
group_alive() {
  local table members member group state group_id
  table="$(ps -eo pgid=,stat= 2>/dev/null)" || return 2
  [[ -n "${table//[[:space:]]/}" ]] || return 2
  members="$(awk -v wanted="${pgid}" '$1==wanted {print $1, $2}' <<<"${table}")" || return 2
  if [[ -z "${members//[[:space:]]/}" ]]; then
    # A zombie leader may have no remaining group members. If kill(0) still
    # sees the group, the process listing was incomplete: fail closed.
    kill -0 -- "-${pgid}" 2>/dev/null && return 2
    return 1
  fi
  while read -r group_id state; do
    [[ "${group_id}" =~ ^[0-9]+$ && -n "${state}" ]] || return 2
  done <<<"${members}"
  awk -v wanted="${pgid}" '$1==wanted && $2 !~ /^Z/ {found=1} END {exit !found}' <<<"${table}"
}
if group_alive; then
  kill -TERM -- "-${pgid}" || { echo "Failed to signal localized group." >&2; exit 1; }
  for _ in {1..40}; do
    group_alive || {
      rc=$?
      (( rc != 1 )) && { echo "Unable to query localized process group; retaining PID/PGID." >&2; exit 1; }
      break
    }
    sleep 0.25
  done
else
  rc=$?
  (( rc != 1 )) && { echo "Unable to query localized process group; retaining PID/PGID." >&2; exit 1; }
fi
if group_alive; then
  echo "Localized group did not exit; retaining PID/PGID for inspection." >&2
  exit 1
else
  rc=$?
  (( rc != 1 )) && { echo "Unable to query localized process group; retaining PID/PGID." >&2; exit 1; }
fi
rm -f -- "${PID_FILE}" "${PGID_FILE}"
[[ "${1:-}" == --quiet ]] || echo "Stopped owned localized Nav2 process group."
