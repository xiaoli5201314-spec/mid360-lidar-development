#!/usr/bin/env bash
set -euo pipefail

# Independent 3D mapping path. It consumes FAST-LIO's registered cloud and
# never stops/restarts the existing 2D mapping or Nav2 processes. The input
# is first rate-limited with the ROS 2 topic_tools throttle component so the
# ARM board is not forced to fuse every 10 Hz, 15k-point frame.
set +u
source /opt/ros/humble/setup.bash
[[ -f /userdata/livox/ros2_ws/install/setup.bash ]] && source /userdata/livox/ros2_ws/install/setup.bash
[[ -f /userdata/fastlio_ws/install/setup.bash ]] && source /userdata/fastlio_ws/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

WORKSPACE="${OCTOMAP_WORKSPACE:-/userdata/nav2_3d_ws}"
RUN_DIR="${OCTOMAP_RUN_DIR:-/root/r680_nav/run/octomap3d}"
LOG_DIR="${OCTOMAP_LOG_DIR:-/root/r680_nav/logs}"
PID_FILE="${RUN_DIR}/octomap3d.pid"
PGID_FILE="${RUN_DIR}/octomap3d.pgid"
THROTTLE_PID_FILE="${RUN_DIR}/octomap3d_throttle.pid"
THROTTLE_PGID_FILE="${RUN_DIR}/octomap3d_throttle.pgid"
LOG_FILE="${LOG_DIR}/r680_octomap3d.log"
THROTTLE_LOG_FILE="${LOG_DIR}/r680_octomap3d_throttle.log"
CLOUD_IN="${OCTOMAP_CLOUD_IN:-${OCTOMAP_CLOUD_TOPIC:-/cloud_registered_body}}"
THROTTLED_CLOUD="${OCTOMAP_THROTTLED_CLOUD:-/r680/octomap/cloud}"
INPUT_RATE="${OCTOMAP_INPUT_RATE:-3.0}"
FRAME_ID="${OCTOMAP_FRAME_ID:-camera_init}"
BASE_FRAME_ID="${OCTOMAP_BASE_FRAME_ID:-body}"
RESOLUTION="${OCTOMAP_RESOLUTION:-0.10}"
MAX_RANGE="${OCTOMAP_MAX_RANGE:-30.0}"

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

if ! ros2 pkg prefix octomap_server >/dev/null 2>&1; then
  echo "octomap_server is not installed in the current ROS environment." >&2
  echo "Install ros-humble-octomap-server or build it in ${WORKSPACE}." >&2
  exit 1
fi
if ! ros2 pkg prefix topic_tools >/dev/null 2>&1; then
  echo "topic_tools is required for the bounded 3D input rate." >&2
  echo "Install ros-humble-topic-tools, then retry." >&2
  exit 1
fi

TOPIC_INFO="$(ros2 topic info "${CLOUD_IN}" 2>&1 || true)"
if ! grep -q 'Publisher count: [1-9]' <<<"${TOPIC_INFO}"; then
  echo "No live publisher found on ${CLOUD_IN}; start Livox + FAST-LIO first." >&2
  exit 1
fi

if [[ -s "${PID_FILE}" && -s "${THROTTLE_PID_FILE}" ]] && \
   kill -0 "$(tr -dc '0-9' <"${PID_FILE}")" 2>/dev/null && \
   kill -0 "$(tr -dc '0-9' <"${THROTTLE_PID_FILE}")" 2>/dev/null; then
  echo "OctoMap already running (PID $(tr -dc '0-9' <"${PID_FILE}")); input throttle is active."
  exit 0
fi

# Clean up a stale/newer instance before launching. The stop script validates
# ownership and kills complete process groups; this also handles installations
# made with the older single-PID version of this script.
if [[ -s "${PID_FILE}" || -s "${THROTTLE_PID_FILE}" ]]; then
  "$(dirname "$0")/r680_octomap3d_stop.sh"
fi
rm -f -- "${PID_FILE}" "${PGID_FILE}" "${THROTTLE_PID_FILE}" "${THROTTLE_PGID_FILE}"

# Do not silently create a second server if an older/manual instance was
# started without our PID records. Duplicate latched publishers make RViz
# flicker and double the OctoMap CPU load, so require an explicit cleanup.
EXISTING_OCTOMAP_NODES="$(ros2 node list 2>/dev/null | grep -c '^/octomap_server$' || true)"
if [[ "${EXISTING_OCTOMAP_NODES}" != "0" ]]; then
  echo "Existing /octomap_server node detected (${EXISTING_OCTOMAP_NODES}); stop it before starting a new instance." >&2
  exit 1
fi

echo "Starting independent OctoMap 3D mapping..."
echo "  workspace: ${WORKSPACE}"
echo "  cloud_in: ${CLOUD_IN}"
echo "  throttled_cloud: ${THROTTLED_CLOUD}"
echo "  input_rate: ${INPUT_RATE} Hz"
echo "  frame_id: ${FRAME_ID}"
echo "  base_frame_id: ${BASE_FRAME_ID}"
echo "  resolution: ${RESOLUTION} m"

# Each component gets its own session/process group. Recording both the PID
# and PGID is intentional: 'ros2 run' is a Python launcher whose native ROS
# child otherwise survives a simple 'kill PID'.
nohup setsid ros2 run topic_tools throttle messages "${CLOUD_IN}" "${INPUT_RATE}" "${THROTTLED_CLOUD}" \
  >"${THROTTLE_LOG_FILE}" 2>&1 &
THROTTLE_PID=$!
THROTTLE_PGID="$(ps -o pgid= -p "${THROTTLE_PID}" | tr -d ' ')"
printf '%s\n' "${THROTTLE_PID}" >"${THROTTLE_PID_FILE}"
printf '%s\n' "${THROTTLE_PGID}" >"${THROTTLE_PGID_FILE}"

nohup setsid ros2 run octomap_server octomap_server_node --ros-args \
  -p resolution:="${RESOLUTION}" \
  -p frame_id:="${FRAME_ID}" \
  -p base_frame_id:="${BASE_FRAME_ID}" \
  -p sensor_model.max_range:="${MAX_RANGE}" \
  -r cloud_in:="${THROTTLED_CLOUD}" \
  >"${LOG_FILE}" 2>&1 &
PID=$!
PGID="$(ps -o pgid= -p "${PID}" | tr -d ' ')"
printf '%s\n' "${PID}" >"${PID_FILE}"
printf '%s\n' "${PGID}" >"${PGID_FILE}"

sleep 3
if ! kill -0 "${THROTTLE_PID}" 2>/dev/null; then
  echo "3D throttle exited during startup; see ${THROTTLE_LOG_FILE}" >&2
  "$(dirname "$0")/r680_octomap3d_stop.sh"
  exit 1
fi
if ! kill -0 "${PID}" 2>/dev/null; then
  echo "OctoMap exited during startup; see ${LOG_FILE}" >&2
  "$(dirname "$0")/r680_octomap3d_stop.sh"
  exit 1
fi

echo "OctoMap started (PID ${PID}, PGID ${PGID})."
echo "Input throttle started (PID ${THROTTLE_PID}, PGID ${THROTTLE_PGID})."
echo "Topics: /octomap_binary, /octomap_full and /occupied_cells_vis_array"
echo "Logs: ${LOG_FILE} and ${THROTTLE_LOG_FILE}"
