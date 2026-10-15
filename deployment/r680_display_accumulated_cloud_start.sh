#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

SCRIPT="/root/r680_nav/scripts/r680_display_accumulated_cloud.py"
PID_FILE="/tmp/r680_display_accumulated_cloud.pid"
LOG_DIR="/root/r680_nav/logs"
LOG_FILE="${LOG_DIR}/r680_display_accumulated_cloud.log"

mkdir -p "${LOG_DIR}"
if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ "${old_pid}" =~ ^[0-9]+$ ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "Accumulated display cloud already running: PID ${old_pid}"
    exit 0
  fi
  rm -f "${PID_FILE}"
fi

nohup python3 "${SCRIPT}" \
  --ros-args \
  -p input_topic:=/cloud_registered_body \
  -p output_topic:=/r680/display/cloud_accumulated \
  -p target_frame:=camera_init \
  -p voxel_size:=0.08 \
  -p max_points:=120000 \
  -p min_height:=-2.0 \
  -p max_height:=3.0 \
  -p max_range:=35.0 \
  -p publish_hz:=2.0 \
  >>"${LOG_FILE}" 2>&1 &
echo $! > "${PID_FILE}"
echo "Started accumulated display cloud: PID $(cat "${PID_FILE}")"
echo "Topic: /r680/display/cloud_accumulated"
echo "Log: ${LOG_FILE}"
