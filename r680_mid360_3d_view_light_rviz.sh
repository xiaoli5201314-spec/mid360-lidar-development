#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
RVIZ_CFG="${1:-$HOME/r680_mid360_3d_view_light.rviz}"
[[ -f "${RVIZ_CFG}" ]] || { echo "RViz config not found: ${RVIZ_CFG}" >&2; exit 1; }
exec rviz2 --display-config "${RVIZ_CFG}"
