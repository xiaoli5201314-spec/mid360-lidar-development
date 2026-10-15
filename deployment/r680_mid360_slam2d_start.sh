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

reject_active_chassis() {
  local pid cmd
  if [[ -r /root/r680_nav/run/r680_nav2_chassis_bridge.pid ]]; then
    pid="$(tr -d '\r\n' </root/r680_nav/run/r680_nav2_chassis_bridge.pid)"
    if [[ "${pid}" =~ ^[1-9][0-9]*$ ]] && kill -0 "${pid}" 2>/dev/null; then
      echo "Refusing to start a non-chassis workflow while the real chassis stack is active; run r680_nav2_chassis_stop.sh first." >&2
      return 1
    fi
  fi
  for proc in /proc/[0-9]*/cmdline; do
    [[ -r "${proc}" ]] || continue
    cmd="$(tr '\0' ' ' <"${proc}" 2>/dev/null || true)"
    if [[ " ${cmd} " == *" /root/r680_nav/scripts/r680_ackermann_bridge.py "* &&
          " ${cmd} " == *" --write "* ]]; then
      echo "Refusing to start a non-chassis workflow while the real chassis stack is active; run r680_nav2_chassis_stop.sh first." >&2
      return 1
    fi
  done
}

reject_active_chassis

if [[ "${R680_SKIP_TIME_CHECK:-0}" != "1" ]]; then
  /root/r680_nav/scripts/r680_check_3588_time_sane.sh
fi

ensure_core_services() {
  local unit
  # Mapping depends on the live FAST-LIO cloud and odometry chain.  Start only
  # units that are actually inactive; do not restart a healthy Livox/TF/FAST-
  # LIO service just because the user started a new 2D mapping session.
  for unit in mid360-static-tf.service mid360-livox-driver.service fastlio-mapping.service; do
    if systemctl is-active --quiet "${unit}"; then
      echo "Core service already active: ${unit}"
    else
      echo "Starting inactive core service: ${unit}"
      systemctl start "${unit}"
    fi
  done

  for unit in mid360-static-tf.service mid360-livox-driver.service fastlio-mapping.service; do
    if ! systemctl is-active --quiet "${unit}"; then
      echo "Required core service is not active: ${unit}" >&2
      return 1
    fi
  done
}

wait_for_unique_topic_publisher() {
  local topic="$1" info count
  for _ in $(seq 1 "${R680_TOPIC_WAIT_SEC:-20}"); do
    info="$(ros2 topic info "${topic}" 2>&1 || true)"
    count="$(awk -F: '/Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${info}")"
    if [[ "${count}" =~ ^[0-9]+$ ]] && (( count > 1 )); then
      echo "Multiple publishers detected on ${topic} (Publisher count: ${count})." >&2
      echo "Stop the owned mapping chain, then retry: /root/r680_nav/scripts/r680_mid360_slam2d_stop.sh" >&2
      echo "Exact topic info:" >&2
      printf '%s\n' "${info}" >&2
      return 1
    fi
    if [[ "${count}" == "1" ]]; then
      echo "[OK] ${topic} has one publisher."
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for a publisher on ${topic}; helper may still be starting." >&2
  echo "Exact topic info after bounded wait:" >&2
  printf '%s\n' "${info}" >&2
  return 1
}

require_exec() {
  local path="$1"
  [[ -x "${path}" ]] || {
    echo "Required mapping helper is missing: ${path}" >&2
    return 1
  }
}

for helper in \
  /root/r680_nav/scripts/r680_nav2_dryrun_stop.sh \
  /root/r680_nav/scripts/r680_nav_nav2_stop.sh \
  /root/r680_nav/scripts/r680_mid360_3d_mapping_stop.sh \
  /root/r680_nav/scripts/r680_mid360_slam_toolbox_stop.sh \
  /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan_stop.sh \
  /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan.sh \
  /root/r680_nav/scripts/r680_mid360_slam_toolbox_mapping.sh; do
  require_exec "${helper}"
done

echo "Checking Livox/TF/FAST-LIO core services..."
echo "Checking mapping helper closure..."
echo "Stopping old Nav2/map_server/3D/2D-SLAM processes that may conflict..."
/root/r680_nav/scripts/r680_nav2_dryrun_stop.sh --quiet
/root/r680_nav/scripts/r680_nav_nav2_stop.sh
/root/r680_nav/scripts/r680_mid360_3d_mapping_stop.sh
/root/r680_nav/scripts/r680_mid360_slam_toolbox_stop.sh
/root/r680_nav/scripts/r680_mid360_pointcloud_to_scan_stop.sh
sleep 1
ensure_core_services

LOG_DIR="/root/r680_nav/logs"
mkdir -p "${LOG_DIR}"

echo "Starting legacy 2D mapping chain:"
echo "  /cloud_registered_body -> pointcloud_to_laserscan -> /scan"
echo "  /scan -> slam_toolbox -> /map"
echo "  No /cmd_vel bridge is started here, so the chassis will not move."

CLOUD_IN="${CLOUD_IN:-/cloud_registered_body}" \
SCAN_OUT="${SCAN_OUT:-/scan}" \
SCAN_FRAME="${SCAN_FRAME:-body}" \
nohup /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan.sh \
  >"${LOG_DIR}/r680_mid360_pointcloud_to_scan.nohup" 2>&1 &

sleep 4

nohup /root/r680_nav/scripts/r680_mid360_slam_toolbox_mapping.sh \
  >"${LOG_DIR}/r680_mid360_slam_toolbox_mapping.nohup" 2>&1 &

sleep 6

echo "Checking deterministic mapping topic ownership..."
wait_for_unique_topic_publisher /scan
wait_for_unique_topic_publisher /map
echo "Logs:"
echo "  ${LOG_DIR}/r680_mid360_pointcloud_to_scan.log"
echo "  ${LOG_DIR}/r680_mid360_slam_toolbox_mapping.log"
