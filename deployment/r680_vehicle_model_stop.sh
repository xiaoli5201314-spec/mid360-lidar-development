#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${R680_VEHICLE_MODEL_RUN_DIR:-$HOME/.r680_vehicle_model}"
PID_FILE="${RUN_DIR}/robot_state_publisher.pid"

if [[ ! -f "${PID_FILE}" ]]; then
  echo "Vehicle model is not running."
  exit 0
fi

pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
  kill "${pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  kill -9 "${pid}" 2>/dev/null || true
fi
rm -f "${PID_FILE}"
echo "Vehicle model stopped."
