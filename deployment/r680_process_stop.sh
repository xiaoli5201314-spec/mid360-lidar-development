#!/usr/bin/env bash
# Safely stop only processes whose /proc command line contains an explicit token.
set -u

SELF_PID=$$
PARENT_PID=${PPID:-0}
TERM_WAIT_SECONDS=${R680_STOP_TERM_WAIT_SECONDS:-5}
KILL_WAIT_SECONDS=${R680_STOP_KILL_WAIT_SECONDS:-1}

usage() {
  echo "Usage: $0 --token TOKEN --label LABEL | --pid-file FILE --match TOKEN --label LABEL" >&2
}

token=""
label="process"
pid_file=""
mode=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --token) [[ $# -ge 2 ]] || { usage; exit 2; }; mode=token; token=$2; shift 2 ;;
    --match) [[ $# -ge 2 ]] || { usage; exit 2; }; token=$2; shift 2 ;;
    --pid-file) [[ $# -ge 2 ]] || { usage; exit 2; }; mode=pid; pid_file=$2; shift 2 ;;
    --label) [[ $# -ge 2 ]] || { usage; exit 2; }; label=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done

[[ -n "$token" && -n "$mode" ]] || { usage; exit 2; }

declare -a candidates=()
if [[ "$mode" == pid ]]; then
  [[ -r "$pid_file" ]] || exit 0
  while IFS= read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    candidates+=("$pid")
  done < "$pid_file"
else
  for proc_dir in /proc/[0-9]*; do
    [[ -d "$proc_dir" ]] || continue
    candidates+=("${proc_dir##*/}")
  done
fi

declare -a targets=()
cmdline_matches() {
  local pid=$1 cmdline
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  cmdline=$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true)
  [[ -n "$cmdline" && "$cmdline" == *"$token"* ]]
}

for pid in "${candidates[@]}"; do
  [[ "$pid" =~ ^[0-9]+$ ]] || continue
  (( pid == SELF_PID || pid == PARENT_PID )) && continue
  # Keep the explicit /proc/${pid}/cmdline path for reliable command-line matching.
  cmdline_matches "$pid" || continue
  targets+=("$pid")
done

for pid in "${targets[@]}"; do
  kill -TERM "$pid" 2>/dev/null || true
done

if ((${#targets[@]})); then
  for ((i = 0; i < TERM_WAIT_SECONDS * 10; i++)); do
    alive=0
    for pid in "${targets[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive=1 || true
    done
    (( alive == 0 )) && break
    sleep 0.1
  done
  for pid in "${targets[@]}"; do
    # Re-check immediately before escalation in case a PID was recycled.
    kill -0 "$pid" 2>/dev/null && cmdline_matches "$pid" && kill -KILL "$pid" 2>/dev/null || true
  done
  sleep "$KILL_WAIT_SECONDS"
fi

failed=0
for pid in "${targets[@]}"; do
  if kill -0 "$pid" 2>/dev/null && cmdline_matches "$pid"; then
    echo "Failed to stop ${label}: PID ${pid} is still running." >&2
    failed=1
  fi
done

echo "Requested ${label} stop."
exit "${failed}"
