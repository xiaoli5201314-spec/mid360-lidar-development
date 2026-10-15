#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

echo "=== R680 Nav2 dry-run status ==="
date '+%F %T %Z'
echo

echo "Processes:"
pgrep -af "nav2|map_server|static_transform_publisher.*camera_init|fast_lio|fastlio|livox|pointcloud_to_laserscan|slam_toolbox" || true
echo

echo "Topics:"
ros2 topic list 2>/dev/null | sort | egrep "cmd_vel|goal|map|plan|scan|costmap|Odometry|tf" || true
echo

echo "/map info:"
timeout 4 ros2 topic echo /map --once --field info 2>/dev/null || echo "  /map not available"
echo

echo "/cmd_vel topic:"
ros2 topic info /cmd_vel 2>/dev/null || echo "  /cmd_vel not available yet"
echo

echo "/plan topic:"
ros2 topic info /plan 2>/dev/null || echo "  /plan not available yet"
echo

echo "TF map -> base_link:"
timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -30 || true
echo

echo "Lifecycle nodes:"
ros2 lifecycle nodes 2>/dev/null || true
