#!/usr/bin/env python3
"""Republish LaserScan with a reliable ROS 2 publisher.

The Humble ``pointcloud_to_laserscan`` 2.1.0 node publishes with
``SensorDataQoS`` (best effort) and does not expose an output-QoS parameter.
This small relay keeps the sensor-side subscription best effort, while
offering the navigation-facing ``/scan`` topic as reliable.  The isolated
dynamic-obstacle profile can optionally restamp each scan at relay time;
otherwise a delayed FAST-LIO stamp can be older than Nav2's TF cache and be
dropped before obstacle marking.
"""

from __future__ import annotations

import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def sensor_input_qos() -> QoSProfile:
    """QoS compatible with the upstream sensor-data publisher."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def navigation_output_qos() -> QoSProfile:
    """Reliable, volatile, latest-scan-only output for Nav2/AMCL."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class ScanQosRelay(Node):
    """Forward the latest scan from a best-effort topic to a reliable one."""

    def __init__(
        self, input_topic: str, output_topic: str, restamp_now: bool = False
    ) -> None:
        super().__init__("r680_scan_qos_relay")
        self._restamp_now = restamp_now
        self._publisher = self.create_publisher(
            LaserScan, output_topic, navigation_output_qos()
        )
        self._subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self._forward,
            sensor_input_qos(),
        )
        mode = "restamp-now" if restamp_now else "preserve-stamp"
        self.get_logger().info(
            f"Relaying {input_topic} (best effort) -> {output_topic} "
            f"(reliable, {mode})"
        )

    def _forward(self, message: LaserScan) -> None:
        if self._restamp_now:
            # Dynamic mode opts into receive-time stamps so a delayed scan is
            # not rejected as older than the available TF cache.
            message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/r680/scan_best_effort")
    parser.add_argument("--output-topic", default="/scan")
    parser.add_argument(
        "--restamp-now",
        action="store_true",
        help="replace each outgoing header stamp with relay receive time",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ScanQosRelay(args.input_topic, args.output_topic, args.restamp_now)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # SIGINT/SIGTERM can already have shut down the context.
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
