#!/usr/bin/env bash
set -euo pipefail
[[ "$#" -eq 0 ]] || exit 2
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
echo "Mode:"
if [[ -r /root/r680_nav/run/r680_nav2_chassis.mode ]]; then
  tr -d '\r' </root/r680_nav/run/r680_nav2_chassis.mode
else
  echo "not recorded"
fi
echo "Fixed serial link:"
ls -l /dev/serial/by-id/YOUR_CHASSIS_SERIAL 2>&1 || true
for file in /root/r680_nav/run/r680_nav2_chassis_bridge.pid /root/r680_nav/run/r680_nav2_localized.pid; do
  echo "PID record: ${file}"
  if [[ -r "${file}" ]]; then
    pid="$(tr -d '\r\n' <"${file}")"
    if [[ "${pid}" =~ ^[1-9][0-9]*$ ]]; then
      ps -p "${pid}" -o pid=,pgid=,stat=,args= || true
    else
      echo "Invalid PID record."
    fi
  else
    echo "not recorded"
  fi
done
for node in /map_server /amcl /controller_server /planner_server /bt_navigator \
  /global_costmap/global_costmap /local_costmap/local_costmap /velocity_smoother; do
  printf '%s: ' "${node}"
  timeout 3 ros2 lifecycle get "${node}" 2>&1 || true
done
for topic in /Odometry /scan /plan /cmd_vel; do
  echo "Topic: ${topic}"
  timeout 3 ros2 topic info "${topic}" -v 2>&1 || true
done
echo "TF map -> base_link (requires initial pose):"
timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true
echo "Bridge log:"
tail -n 40 /root/r680_nav/logs/r680_nav2_chassis_bridge.log 2>&1 || true
