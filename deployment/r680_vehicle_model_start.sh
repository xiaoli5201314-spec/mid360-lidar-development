#!/usr/bin/env bash
set -euo pipefail

# Start the visual-only model once on the VM. This publishes robot_description
# and base_link child TFs; it never opens a serial device or controls motors.
set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

MODEL_FILE="${R680_VEHICLE_MODEL_FILE:-$HOME/r680_vehicle_description.urdf}"
RUN_DIR="${R680_VEHICLE_MODEL_RUN_DIR:-$HOME/.r680_vehicle_model}"
PID_FILE="${RUN_DIR}/robot_state_publisher.pid"
LOG_FILE="${RUN_DIR}/robot_state_publisher.log"
PARAM_FILE="${RUN_DIR}/robot_state_publisher_params.yaml"

if [[ ! -s "${MODEL_FILE}" ]]; then
  echo "Vehicle model not found: ${MODEL_FILE}" >&2
  echo "Copy r680_vehicle_description.urdf to the VM or set R680_VEHICLE_MODEL_FILE." >&2
  exit 1
fi

mkdir -p "${RUN_DIR}"

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ "${old_pid}" =~ ^[0-9]+$ ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "Vehicle model already running (pid ${old_pid})."
    exit 0
  fi
  rm -f "${PID_FILE}"
fi

# Pass the URDF through a YAML block scalar.  Supplying a multiline XML string
# as `-p robot_description:=...` is rejected by rcl's parameter parser on
# Humble; a params file preserves the XML exactly and is reproducible.
{
  printf '/**:\n'
  printf '  ros__parameters:\n'
  printf '    robot_description: |\n'
  sed 's/^/      /' "${MODEL_FILE}"
} >"${PARAM_FILE}"
nohup ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file "${PARAM_FILE}" \
  >"${LOG_FILE}" 2>&1 &
model_pid=$!
echo "${model_pid}" >"${PID_FILE}"
sleep 1
if ! kill -0 "${model_pid}" 2>/dev/null; then
  echo "robot_state_publisher exited during startup." >&2
  tail -40 "${LOG_FILE}" >&2 || true
  rm -f "${PID_FILE}"
  exit 1
fi
echo "Vehicle model started (pid ${model_pid})."
