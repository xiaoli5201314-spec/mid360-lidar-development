#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STOP_HELPER="${SCRIPT_DIR}/r680_process_stop.sh"

set +u
source /opt/ros/humble/setup.bash
source /userdata/livox/ros2_ws/install/setup.bash
source /userdata/fastlio_ws/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

LOG_DIR="/root/r680_nav/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/r680_mid360_pointcloud_to_scan.log"
RELAY_LOG_FILE="${LOG_DIR}/r680_scan_qos_relay.log"
CLOUD_IN="${CLOUD_IN:-/cloud_registered_body}"
SCAN_OUT="${SCAN_OUT:-/scan}"
RAW_SCAN_OUT="${RAW_SCAN_OUT:-/r680/scan_best_effort}"
SCAN_FRAME="${SCAN_FRAME:-body}"
SCAN_MIN_HEIGHT="${SCAN_MIN_HEIGHT:--0.20}"
SCAN_MAX_HEIGHT="${SCAN_MAX_HEIGHT:-0.40}"
SCAN_RANGE_MIN="${SCAN_RANGE_MIN:-0.30}"
SCAN_RANGE_MAX="${SCAN_RANGE_MAX:-15.0}"
RESTAMP_SCAN_NOW="${RESTAMP_SCAN_NOW:-0}"
RELAY_SCRIPT="${SCRIPT_DIR}/r680_scan_qos_relay.py"

[[ -r "${RELAY_SCRIPT}" ]] || {
  echo "Required scan QoS relay is missing: ${RELAY_SCRIPT}" >&2
  exit 1
}

"${STOP_HELPER}" --token "pointcloud_to_laserscan_node" --label "pointcloud_to_laserscan" >/dev/null 2>&1
"${STOP_HELPER}" --token "r680_scan_qos_relay.py" --label "scan QoS relay" >/dev/null 2>&1
sleep 1

echo "Starting pointcloud_to_laserscan:"
echo "  input : ${CLOUD_IN} (sensor_msgs/msg/PointCloud2)"
echo "  raw output: ${RAW_SCAN_OUT} (best effort)"
echo "  output: ${SCAN_OUT} (reliable LaserScan)"
echo "  frame : ${SCAN_FRAME}"
echo "  z band: [${SCAN_MIN_HEIGHT}, ${SCAN_MAX_HEIGHT}] m"
echo "  range: [${SCAN_RANGE_MIN}, ${SCAN_RANGE_MAX}] m"
echo "  restamp: ${RESTAMP_SCAN_NOW}"
echo "  log   : ${LOG_FILE}"
echo "  relay log: ${RELAY_LOG_FILE}"
echo "Press Ctrl+C to stop."

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:="${CLOUD_IN}" \
  -r scan:="${RAW_SCAN_OUT}" \
  -p target_frame:="${SCAN_FRAME}" \
  -p queue_size:=1 \
  -p transform_tolerance:=0.2 \
  -p min_height:="${SCAN_MIN_HEIGHT}" \
  -p max_height:="${SCAN_MAX_HEIGHT}" \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0174533 \
  -p scan_time:=0.1 \
  -p range_min:="${SCAN_RANGE_MIN}" \
  -p range_max:="${SCAN_RANGE_MAX}" \
  -p use_inf:=true \
  -p inf_epsilon:=1.0 \
  >"${LOG_FILE}" 2>&1 &
CONVERTER_PID=$!

RELAY_ARGS=(--input-topic "${RAW_SCAN_OUT}" --output-topic "${SCAN_OUT}")
if [[ "${RESTAMP_SCAN_NOW}" == "1" ]]; then
  RELAY_ARGS+=(--restamp-now)
fi
python3 -u "${RELAY_SCRIPT}" "${RELAY_ARGS[@]}" \
  >"${RELAY_LOG_FILE}" 2>&1 &
RELAY_PID=$!

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  kill -TERM "${CONVERTER_PID}" "${RELAY_PID}" 2>/dev/null || true
  wait "${CONVERTER_PID}" 2>/dev/null || true
  wait "${RELAY_PID}" 2>/dev/null || true
  exit "${rc}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

sleep 1
kill -0 "${CONVERTER_PID}" 2>/dev/null || {
  echo "pointcloud_to_laserscan exited during startup; see ${LOG_FILE}" >&2
  exit 1
}
kill -0 "${RELAY_PID}" 2>/dev/null || {
  echo "Scan QoS relay exited during startup; see ${RELAY_LOG_FILE}" >&2
  exit 1
}

set +e
wait -n "${CONVERTER_PID}" "${RELAY_PID}"
CHAIN_RC=$?
set -e
echo "A scan-chain process exited unexpectedly (status ${CHAIN_RC}); see ${LOG_FILE} and ${RELAY_LOG_FILE}" >&2
exit 1
