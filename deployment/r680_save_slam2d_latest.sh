#!/usr/bin/env bash
set -Eeuo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

MAP_DIR="/root/r680_nav/maps"
FINAL_BASE="${MAP_DIR}/mid360_slam2d_latest"
STAMP="$(date +%Y%m%d_%H%M%S)_$$"
TMP_BASE="${MAP_DIR}/.mid360_slam2d_latest.${STAMP}"
POINTER_TMP="${MAP_DIR}/.latest_slam2d_map.txt.${STAMP}"

mkdir -p "${MAP_DIR}"

cleanup() {
  rm -f -- "${TMP_BASE}.pgm" "${TMP_BASE}.yaml" "${POINTER_TMP}"
}
trap cleanup EXIT

echo "Saving active /map to the fixed latest map..."
echo "Temporary basename: ${TMP_BASE}"

# BEGIN map saver retry
MAP_SAVE_TIMEOUT="${R680_MAP_SAVE_TIMEOUT:-30.0}"
MAP_SAVE_ATTEMPTS="${R680_MAP_SAVE_ATTEMPTS:-2}"
save_ok=0
for ((attempt = 1; attempt <= MAP_SAVE_ATTEMPTS; attempt++)); do
  rm -f -- "${TMP_BASE}.pgm" "${TMP_BASE}.yaml"
  echo "Map save attempt ${attempt}/${MAP_SAVE_ATTEMPTS} (DDS wait ${MAP_SAVE_TIMEOUT}s)..."
  if timeout 25 ros2 run nav2_map_server map_saver_cli \
    -f "${TMP_BASE}" \
    --ros-args \
    -p map_subscribe_transient_local:=true \
    -p save_map_timeout:="${MAP_SAVE_TIMEOUT}"; then
    save_ok=1
    break
  fi
  echo "Map saver did not receive /map on attempt ${attempt}; retrying without touching the existing latest map." >&2
  if (( attempt < MAP_SAVE_ATTEMPTS )); then
    sleep 1
  fi
done

if (( save_ok != 1 )); then
  echo "Map saver failed after ${MAP_SAVE_ATTEMPTS} attempts; existing latest map was preserved." >&2
  exit 1
fi
# END map saver retry

if [[ ! -s "${TMP_BASE}.pgm" || ! -s "${TMP_BASE}.yaml" ]]; then
  echo "Map saver did not produce both non-empty files." >&2
  exit 1
fi

if ! grep -q '^image:' "${TMP_BASE}.yaml"; then
  echo "Saved map YAML has no image entry." >&2
  exit 1
fi

# The saver writes the temporary image basename; normalize it before moving both files.
sed -i "s|^image:.*$|image: $(basename "${FINAL_BASE}.pgm")|" "${TMP_BASE}.yaml"

mv -f -- "${TMP_BASE}.pgm" "${FINAL_BASE}.pgm"
mv -f -- "${TMP_BASE}.yaml" "${FINAL_BASE}.yaml"

# The active navigation map is intentionally a single fixed pair. Remove
# older timestamped/previous outputs only after the new pair is in place.
find "${MAP_DIR}" -maxdepth 1 -type f \
  \( -name 'mid360_slam2d_*.pgm' -o -name 'mid360_slam2d_*.yaml' \) \
  ! -name 'mid360_slam2d_latest.pgm' \
  ! -name 'mid360_slam2d_latest.yaml' \
  -delete

printf '%s\n' "${FINAL_BASE}" > "${POINTER_TMP}"
mv -f -- "${POINTER_TMP}" "${MAP_DIR}/latest_slam2d_map.txt"

echo "Latest map saved:"
echo "  ${FINAL_BASE}.yaml"
echo "  ${FINAL_BASE}.pgm"
echo "Pointer: ${MAP_DIR}/latest_slam2d_map.txt"
echo "Active map policy: one latest YAML/PGM pair; older timestamped active maps removed."
