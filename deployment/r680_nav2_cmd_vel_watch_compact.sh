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

echo "Compact /cmd_vel watcher."
echo "Format: vx = forward speed m/s, wz = yaw angular speed rad/s."
echo "Press Ctrl+C to stop watching."
echo

python3 - "${duration}" <<'PY'
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

duration = float(sys.argv[1])
start = time.monotonic()
last_print = 0.0
latest = None

def callback(msg: Twist) -> None:
    global last_print, latest
    latest = msg
    now = time.monotonic()
    if now - last_print >= 1.0:
        last_print = now
        stamp = time.strftime("%H:%M:%S")
        print(
            f"[{stamp}] vx={msg.linear.x:+.3f} m/s, "
            f"wz={msg.angular.z:+.3f} rad/s",
            flush=True,
        )

rclpy.init()
node = rclpy.create_node("r680_cmd_vel_compact_watch")
node.create_subscription(Twist, "/cmd_vel", callback, 10)

try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if duration > 0 and time.monotonic() - start >= duration:
            break
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
