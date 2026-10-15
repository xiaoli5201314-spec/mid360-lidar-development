#!/usr/bin/env python3
import argparse
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudGrabber(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("mid360_save_one_cloud")
        self.msg = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(PointCloud2, topic, self._callback, qos)

    def _callback(self, msg: PointCloud2) -> None:
        self.msg = msg


def grab_once(topic: str, timeout_s: float):
    node = PointCloudGrabber(topic)
    end_time = time.time() + timeout_s
    while time.time() < end_time and rclpy.ok() and node.msg is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    msg = node.msg
    node.destroy_node()
    return msg


def pointcloud2_to_array(msg: PointCloud2) -> np.ndarray:
    names = [field.name for field in msg.fields]
    fields = ["x", "y", "z"]
    if "intensity" in names:
        fields.append("intensity")

    points = pc2.read_points(msg, field_names=fields, skip_nans=True)

    if isinstance(points, np.ndarray):
        if points.dtype.names:
            cols = [points[name].astype(np.float32, copy=False) for name in fields]
            arr = np.stack(cols, axis=1)
        else:
            arr = points.astype(np.float32, copy=False)
    else:
        arr = np.array(list(points), dtype=np.float32)

    if arr.ndim == 1 and arr.size:
        arr = arr.reshape(1, -1)
    if arr.size == 0:
        arr = np.empty((0, len(fields)), dtype=np.float32)
    if arr.shape[1] == 3:
        arr = np.c_[arr, np.zeros((arr.shape[0], 1), dtype=np.float32)]
    return arr[:, :4]


def write_ascii_pcd(path: str, arr: np.ndarray) -> None:
    count = int(arr.shape[0])
    with open(path, "w", encoding="ascii") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION 0.7\n")
        handle.write("FIELDS x y z intensity\n")
        handle.write("SIZE 4 4 4 4\n")
        handle.write("TYPE F F F F\n")
        handle.write("COUNT 1 1 1 1\n")
        handle.write(f"WIDTH {count}\n")
        handle.write("HEIGHT 1\n")
        handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {count}\n")
        handle.write("DATA ascii\n")
        np.savetxt(handle, arr, fmt="%.5f %.5f %.5f %.3f")


def write_metadata(path: str, topic: str, frame_id: str, arr: np.ndarray) -> None:
    count = int(arr.shape[0])
    if count:
        mins = arr[:, :3].min(axis=0)
        maxs = arr[:, :3].max(axis=0)
    else:
        mins = np.zeros(3)
        maxs = np.zeros(3)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(f"topic={topic}\n")
        handle.write(f"frame_id={frame_id}\n")
        handle.write(f"points={count}\n")
        handle.write(f"min_xyz={mins[0]:.3f},{mins[1]:.3f},{mins[2]:.3f}\n")
        handle.write(f"max_xyz={maxs[0]:.3f},{maxs[1]:.3f},{maxs[2]:.3f}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/Laser_map")
    parser.add_argument("--fallback-topic", default="/cloud_registered")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--output-dir", default="/userdata/mid360_maps")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    rclpy.init()
    try:
        topic = args.topic
        msg = grab_once(topic, args.timeout)
        if msg is None and args.fallback_topic:
            topic = args.fallback_topic
            msg = grab_once(topic, args.timeout)
        if msg is None:
            raise RuntimeError(
                f"No PointCloud2 message received from {args.topic} or {args.fallback_topic}"
            )

        arr = pointcloud2_to_array(msg)
        if arr.shape[0] == 0:
            raise RuntimeError(f"Received empty point cloud from {topic}")

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_topic = topic.strip("/").replace("/", "_")
        pcd_path = os.path.join(args.output_dir, f"mid360_{safe_topic}_{stamp}.pcd")
        meta_path = pcd_path + ".txt"

        write_ascii_pcd(pcd_path, arr)
        write_metadata(meta_path, topic, msg.header.frame_id, arr)

        print(f"PCD_PATH={pcd_path}")
        print(f"META_PATH={meta_path}")
        print(f"TOPIC={topic}")
        print(f"FRAME={msg.header.frame_id}")
        print(f"POINTS={arr.shape[0]}")
        print(f"SIZE_BYTES={os.path.getsize(pcd_path)}")
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
