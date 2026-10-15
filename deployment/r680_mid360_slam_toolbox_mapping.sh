#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
source /userdata/livox/ros2_ws/install/setup.bash
source /userdata/fastlio_ws/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

PARAMS_FILE="${1:-/root/r680_nav/config/r680_mid360_slam_toolbox_params.yaml}"
LOG_DIR="/root/r680_nav/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/r680_mid360_slam_toolbox_mapping.log"

if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "slam_toolbox params not found: ${PARAMS_FILE}" >&2
  exit 1
fi

echo "Starting slam_toolbox online async mapping:"
echo "  params: ${PARAMS_FILE}"
echo "  input : /scan"
echo "  output: /map"
echo "  frames: map -> camera_init -> body"
echo "  log   : ${LOG_FILE}"
echo "Press Ctrl+C to stop."

exec ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:="${PARAMS_FILE}" \
  2>&1 | tee "${LOG_FILE}"
