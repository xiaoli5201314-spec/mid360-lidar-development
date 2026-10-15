#!/usr/bin/env bash
set -eo pipefail

MARKER_FILE="${R680_TIME_MARKER:-/root/r680_nav/time/last_good_epoch}"
mkdir -p "$(dirname "${MARKER_FILE}")"

now_epoch="$(date +%s)"
if ! [[ "${now_epoch}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: cannot read current system time." >&2
  exit 42
fi

if [[ -f "${MARKER_FILE}" ]]; then
  last_epoch="$(tr -dc '0-9' < "${MARKER_FILE}" | head -c 20 || true)"
  if [[ -n "${last_epoch}" ]] && [[ "${last_epoch}" =~ ^[0-9]+$ ]]; then
    # If the board clock moved backwards behind the last known-good time,
    # refuse to start SLAM. A backward jump corrupts ROS TF/FAST-LIO timing.
    if (( now_epoch + 300 < last_epoch )); then
      echo "ERROR: RK3588 clock looks stale." >&2
      echo "  current epoch : ${now_epoch} ($(date -d "@${now_epoch}" '+%F %T %Z' 2>/dev/null || date))" >&2
      echo "  last good     : ${last_epoch} ($(date -d "@${last_epoch}" '+%F %T %Z' 2>/dev/null || true))" >&2
      echo "Please sync RK3588 time first, then restart Livox + FAST-LIO before mapping." >&2
      exit 42
    fi
  fi
fi

printf '%s\n' "${now_epoch}" > "${MARKER_FILE}"
date '+%F %T %Z' > "${MARKER_FILE}.txt"
echo "RK3588 time check OK: $(cat "${MARKER_FILE}.txt")"
