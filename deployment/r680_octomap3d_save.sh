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

MAP_DIR="${OCTOMAP_MAP_DIR:-/userdata/nav2_3d_maps}"
FINAL_BT="${MAP_DIR}/mid360_octomap_latest.bt"
FINAL_OT="${MAP_DIR}/mid360_octomap_latest.ot"
STAMP="$(date +%Y%m%d_%H%M%S)_$$"
TMP_BT="${MAP_DIR}/.mid360_octomap_latest.${STAMP}.bt"
TMP_OT="${MAP_DIR}/.mid360_octomap_latest.${STAMP}.ot"
TIMEOUT_SEC="${OCTOMAP_SAVE_TIMEOUT:-30}"

mkdir -p "${MAP_DIR}"
cleanup() { rm -f -- "${TMP_BT}" "${TMP_OT}"; }
trap cleanup EXIT

if ! ros2 pkg prefix octomap_server >/dev/null 2>&1; then
  echo "octomap_server is not installed." >&2
  exit 1
fi
OCTOMAP_TOPIC_INFO="$(ros2 topic info /octomap_binary 2>&1 || true)"
if ! grep -q 'Publisher count: [1-9]' <<<"${OCTOMAP_TOPIC_INFO}"; then
  echo "No /octomap_binary publisher; start r680_octomap3d_start.sh first." >&2
  exit 1
fi

save_one() {
  local output="$1"
  echo "Saving OctoMap ($(basename "${output}")) ..."
  timeout "${TIMEOUT_SEC}" ros2 run octomap_server octomap_saver_node --ros-args \
    -p octomap_path:="${output}"
  if [[ ! -s "${output}" ]]; then
    echo "OctoMap saver produced no non-empty file: ${output}" >&2
    exit 1
  fi
}

save_one "${TMP_BT}"
save_one "${TMP_OT}"

# Publish neither half-written file: replace both outputs only after both
# temporary saves succeeded. The fixed names are what Nav2/operator commands use.
mv -f -- "${TMP_BT}" "${FINAL_BT}"
mv -f -- "${TMP_OT}" "${FINAL_OT}"
find "${MAP_DIR}" -maxdepth 1 -type f \
  \( -name 'mid360_octomap_*.bt' -o -name 'mid360_octomap_*.ot' \) \
  ! -name 'mid360_octomap_latest.bt' \
  ! -name 'mid360_octomap_latest.ot' \
  -delete

echo "3D maps saved:"
echo "  ${FINAL_BT}"
echo "  ${FINAL_OT}"

