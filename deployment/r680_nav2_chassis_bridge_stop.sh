#!/usr/bin/env bash
set -euo pipefail
[[ "$#" -eq 0 ]] || exit 2
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
TX_NAME=r680_nav2_chassis_bridge
TX_SCRIPT=r680_nav2_chassis_bridge_start.sh
cancel_start_transaction
PID_FILE=/root/r680_nav/run/r680_nav2_chassis_bridge.pid
PGID_FILE=/root/r680_nav/run/r680_nav2_chassis_bridge.pgid
[[ -f "${PID_FILE}" ]] || exit 0
pid="$(tr -d '\r\n' <"${PID_FILE}")"
[[ "${pid}" =~ ^[1-9][0-9]*$ && "${pid}" -gt 1 ]] ||
  { echo "Invalid bridge PID; use physical emergency stop." >&2; exit 1; }
bridge_alive() {
  local state
  [[ -d "/proc/${pid}" ]] || return 1
  if ! kill -0 "${pid}" 2>/dev/null; then
    [[ -d "/proc/${pid}" ]] || return 1
    return 2
  fi
  if ! state="$(ps -o stat= -p "${pid}" 2>/dev/null)"; then
    [[ -d "/proc/${pid}" ]] || return 1
    return 2
  fi
  state="${state//[[:space:]]/}"
  if [[ -z "${state}" ]]; then
    [[ -d "/proc/${pid}" ]] || return 1
    return 2
  fi
  [[ "${state}" == Z* ]] && return 1
  return 0
}
if bridge_alive; then bridge_rc=0; else bridge_rc=$?; fi
(( bridge_rc == 2 )) && { echo "Unable to determine bridge state; use physical emergency stop." >&2; exit 1; }
if (( bridge_rc == 0 )); then
  cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline")"
  [[ " ${cmdline} " == *" /root/r680_nav/scripts/r680_ackermann_bridge.py "* &&
     " ${cmdline} " == *" --write "* ]] ||
    { echo "Refusing unowned bridge PID ${pid}; use physical emergency stop." >&2; exit 1; }
  actual_pgid="$(ps -o pgid= -p "${pid}" | tr -d ' ')"
  [[ -r "${PGID_FILE}" && "$(tr -d '\r\n' <"${PGID_FILE}")" == "${pid}" &&
     "${actual_pgid}" == "${pid}" ]] ||
    { echo "Bridge PID/PGID mismatch; use physical emergency stop." >&2; exit 1; }
  kill -TERM "${pid}" ||
    { echo "Cannot signal bridge; use physical emergency stop." >&2; exit 1; }
  # SIGTERM allows Python finally to send three zero frames and close serial.
  for _ in {1..48}; do
    bridge_alive || break
    sleep 0.25
  done
fi
if bridge_alive; then bridge_rc=0; else bridge_rc=$?; fi
(( bridge_rc == 2 )) && { echo "Unable to determine bridge state; use physical emergency stop." >&2; exit 1; }
if (( bridge_rc == 0 )); then
  echo "Bridge shutdown timed out. Press physical emergency stop NOW; PID retained." >&2
  exit 1
fi
rm -f -- "${PID_FILE}" "${PGID_FILE}"
echo "Bridge stopped; serial handle released."
