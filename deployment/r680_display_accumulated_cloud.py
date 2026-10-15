#!/usr/bin/env python3
"""Publish a bounded, accumulated display cloud for RViz.

The node intentionally sits on the visualization side of the pipeline.  It
does not modify FAST-LIO, slam_toolbox, Nav2, or the source point cloud.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Deque, Dict, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


VoxelKey = Tuple[int, int, int]


class AccumulatedDisplayCloud(Node):
    def __init__(self) -> None:
        super().__init__("r680_display_accumulated_cloud")

        self.declare_parameter("input_topic", "/cloud_registered_body")
        self.declare_parameter("output_topic", "/r680/display/cloud_accumulated")
        self.declare_parameter("target_frame", "camera_init")
        self.declare_parameter("voxel_size", 0.08)
        self.declare_parameter("max_points", 120000)
        self.declare_parameter("min_height", -2.0)
        self.declare_parameter("max_height", 3.0)
        self.declare_parameter("max_range", 35.0)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("tf_timeout_ms", 80)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.voxel_size = max(0.01, float(self.get_parameter("voxel_size").value))
        self.max_points = max(1000, int(self.get_parameter("max_points").value))
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.max_range = max(1.0, float(self.get_parameter("max_range").value))
        publish_hz = max(0.2, float(self.get_parameter("publish_hz").value))
        tf_timeout_ms = max(1, int(self.get_parameter("tf_timeout_ms").value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._lock = threading.Lock()
        self._voxels: Dict[VoxelKey, Tuple[float, float, float, float]] = {}
        self._order: Deque[VoxelKey] = deque()
        self._last_warn_ns = 0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._sub = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, sensor_qos
        )
        self._pub = self.create_publisher(PointCloud2, self.output_topic, output_qos)
        self._timer = self.create_timer(1.0 / publish_hz, self._publish)
        self._tf_timeout = Duration(nanoseconds=tf_timeout_ms * 1_000_000)

        self.get_logger().info(
            f"Accumulating {self.input_topic} -> {self.output_topic} "
            f"in {self.target_frame}; voxel={self.voxel_size:.3f} m, "
            f"max_points={self.max_points}"
        )

    def _warn_throttled(self, message: str) -> None:
        now = self.get_clock().now().nanoseconds
        if now - self._last_warn_ns > 5_000_000_000:
            self._last_warn_ns = now
            self.get_logger().warning(message)

    def _cloud_callback(self, msg: PointCloud2) -> None:
        source_frame = msg.header.frame_id or self.target_frame
        names = {field.name for field in msg.fields}
        if not {"x", "y", "z"}.issubset(names):
            self._warn_throttled("Input cloud has no x/y/z fields")
            return
        field_names = ["x", "y", "z"]
        has_intensity = "intensity" in names
        if has_intensity:
            field_names.append("intensity")

        try:
            rows = pc2.read_points(
                msg, field_names=field_names, skip_nans=True
            )
            # Humble's sensor_msgs_py may return either a structured NumPy
            # array or an iterator of tuples depending on the installed patch
            # level. Normalize both forms without assuming a packed dtype.
            if getattr(rows, "dtype", None) is not None and rows.dtype.names:
                array = np.column_stack(
                    [np.asarray(rows[name], dtype=np.float32) for name in field_names]
                )
            else:
                array = np.asarray(list(rows), dtype=np.float32)
        except (TypeError, ValueError, IndexError) as exc:
            self._warn_throttled(f"Unable to decode input cloud: {exc}")
            return
        if array.size == 0:
            return
        if array.ndim == 1:
            array = array.reshape(1, -1)
        if not has_intensity:
            array = np.column_stack((array[:, :3], np.zeros(array.shape[0], dtype=np.float32)))

        # Do not pass Livox's full PointCloud2 message to do_transform_cloud:
        # its extra fields can violate the dtype assertion in tf2_sensor_msgs.
        # Transform only the fields used by this display node.
        if source_frame != self.target_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    rclpy.time.Time.from_msg(msg.header.stamp),
                    timeout=self._tf_timeout,
                )
            except (TransformException, ValueError, RuntimeError) as exc:
                # FAST-LIO cloud stamps can be a few hundred milliseconds
                # ahead of the latest TF on a Wi-Fi link. For visualization,
                # the latest transform is preferable to dropping the frame.
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        source_frame,
                        rclpy.time.Time(),
                        timeout=self._tf_timeout,
                    )
                except (TransformException, ValueError, RuntimeError):
                    self._warn_throttled(
                        f"Waiting for {source_frame} -> {self.target_frame} TF: {exc}"
                    )
                    return
            array[:, :3] = self._transform_xyz(array[:, :3], transform)

        xyz = array[:, :3]
        intensity = np.nan_to_num(array[:, 3], nan=0.0, posinf=0.0, neginf=0.0)
        distances = np.linalg.norm(xyz[:, :2], axis=1)
        valid = (
            np.isfinite(xyz).all(axis=1)
            & (xyz[:, 2] >= self.min_height)
            & (xyz[:, 2] <= self.max_height)
            & (distances <= self.max_range)
        )
        xyz = xyz[valid]
        intensity = intensity[valid]
        if xyz.size == 0:
            return

        keys = np.floor(xyz / self.voxel_size).astype(np.int64)
        with self._lock:
            for point, level, key_array in zip(xyz, intensity, keys):
                key = (int(key_array[0]), int(key_array[1]), int(key_array[2]))
                if key in self._voxels:
                    self._voxels[key] = (
                        float(point[0]), float(point[1]), float(point[2]), float(level)
                    )
                    continue
                self._voxels[key] = (
                    float(point[0]), float(point[1]), float(point[2]), float(level)
                )
                self._order.append(key)
                while len(self._voxels) > self.max_points and self._order:
                    old_key = self._order.popleft()
                    self._voxels.pop(old_key, None)

    @staticmethod
    def _transform_xyz(xyz: np.ndarray, transform) -> np.ndarray:
        """Apply geometry_msgs/TransformStamped to an N-by-3 float array."""
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        q = np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=np.float64)
        norm = float(np.linalg.norm(q))
        if norm < 1e-12:
            raise ValueError("TF contains a zero-length quaternion")
        x, y, z, w = q / norm
        matrix = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        offset = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
        return (xyz.astype(np.float64) @ matrix.T + offset).astype(np.float32)

    def _publish(self) -> None:
        with self._lock:
            points = list(self._voxels.values())
        if not points:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self._pub.publish(pc2.create_cloud(header, fields, points))


def main(args=None) -> int:
    rclpy.init(args=args)
    node = AccumulatedDisplayCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
