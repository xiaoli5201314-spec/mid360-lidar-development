#!/usr/bin/env bash
set -Eeuo pipefail
set +u
source /opt/ros/humble/setup.bash
if [[ -f /userdata/nav2_3d_ws/install/setup.bash ]]; then source /userdata/nav2_3d_ws/install/setup.bash; fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
RUN_DIR="${OCTOMAP_RUN_DIR:-/root/r680_nav/run/octomap3d}"
PID_FILE="${RUN_DIR}/octomap3d.pid"
PGID_FILE="${RUN_DIR}/octomap3d.pgid"
THROTTLE_PID_FILE="${RUN_DIR}/octomap3d_throttle.pid"
THROTTLE_PGID_FILE="${RUN_DIR}/octomap3d_throttle.pgid"
ok=1
if [[ -s "${PID_FILE}" ]]; then
  pid="$(tr -dc '0-9' <"${PID_FILE}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then echo "process: active (PID ${pid})"; else echo "process: inactive (stale PID file)"; ok=0; fi
else
  echo "process: inactive (no PID file)"; ok=0
fi
if [[ -s "${THROTTLE_PID_FILE}" ]]; then
  throttle_pid="$(tr -dc '0-9' <"${THROTTLE_PID_FILE}")"
  if [[ -n "${throttle_pid}" ]] && kill -0 "${throttle_pid}" 2>/dev/null; then echo "input throttle: active (PID ${throttle_pid})"; else echo "input throttle: inactive (stale PID file)"; ok=0; fi
else
  echo "input throttle: inactive (no PID file)"; ok=0
fi
echo "nodes matching octomap_server:"
OCTOMAP_NODE_COUNT="$(ros2 node list 2>/dev/null | grep -c '^/octomap_server$' || true)"
if [[ "${OCTOMAP_NODE_COUNT}" == "1" ]]; then
  echo "  /octomap_server (exactly one node)"
else
  echo "  expected exactly one, found ${OCTOMAP_NODE_COUNT}"
  ok=0
fi
topic_has_publisher() {
  local topic="$1" info
  info="$(ros2 topic info "${topic}" 2>&1 || true)"
  grep -Eq 'Publisher count: 1([[:space:]]|$)' <<<"${info}"
}
for topic in /cloud_registered_body /r680/display/cloud_accumulated /r680/octomap/cloud /octomap_binary /occupied_cells_vis_array; do
  if topic_has_publisher "${topic}"; then echo "topic ${topic}: publisher available"; else echo "topic ${topic}: publisher unavailable"; ok=0; fi
done
if [[ "${ok}" == "1" ]]; then echo "OCTOMAP3D_STATUS=OK"; else echo "OCTOMAP3D_STATUS=NOT_READY"; exit 1; fi
