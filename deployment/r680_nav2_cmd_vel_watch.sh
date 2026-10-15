#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

duration="${1:-0}"

echo "Watching /cmd_vel."
echo "After setting a 2D Goal Pose in RViz, Nav2 should publish linear.x / angular.z here."
echo "Press Ctrl+C to stop."
echo

if [[ "${duration}" == "0" ]]; then
  ros2 topic echo /cmd_vel
else
  timeout "${duration}" ros2 topic echo /cmd_vel
fi
