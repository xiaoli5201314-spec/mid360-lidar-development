#!/usr/bin/env bash
set -euo pipefail

PID_FILE="/tmp/r680_display_accumulated_cloud.pid"
if [[ ! -f "${PID_FILE}" ]]; then
  echo "Accumulated display cloud is not running."
  exit 0
fi

pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
  kill "${pid}"
  for _ in {1..20}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    echo "Process did not stop within 2 seconds; leaving it untouched."
    exit 1
  fi
  echo "Stopped accumulated display cloud: PID ${pid}"
else
  echo "Stale accumulated display cloud PID file removed."
fi
rm -f "${PID_FILE}"
