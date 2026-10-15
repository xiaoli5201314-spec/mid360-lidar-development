#!/usr/bin/env python3
"""Fail closed unless live odometry and laser scans are timely and sustained.

This process is read-only: it only subscribes to ROS 2 topics and never
publishes motion, opens the chassis serial device, or changes parameters.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
import time
from typing import Sequence


@dataclass(frozen=True)
class Assessment:
    ok: bool
    detail: str
    rate_hz: float
    header_age: float
    receive_silence: float


@dataclass(frozen=True)
class PoseAssessment:
    ok: bool
    detail: str


@dataclass
class Samples:
    received_monotonic: list[float] = field(default_factory=list)
    header_epochs: list[float] = field(default_factory=list)
    positions_xyz: list[tuple[float, float, float]] = field(default_factory=list)
    orientations_xyzw: list[tuple[float, float, float, float]] = field(default_factory=list)


def assess_odometry_poses(
    positions_xyz: Sequence[Sequence[float]],
    orientations_xyzw: Sequence[Sequence[float]],
    *,
    max_abs_position: float = 1000.0,
    min_quaternion_norm: float = 0.50,
    max_quaternion_norm: float = 1.50,
) -> PoseAssessment:
    """Reject non-finite/diverged poses before allowing a real chassis arm."""
    if not positions_xyz or not orientations_xyzw:
        return PoseAssessment(False, "Odometry pose: no pose samples received")
    if len(positions_xyz) != len(orientations_xyzw):
        return PoseAssessment(False, "Odometry pose: position/orientation count mismatch")
    for index, (position, quaternion) in enumerate(zip(positions_xyz, orientations_xyzw), start=1):
        if len(position) != 3:
            return PoseAssessment(False, f"Odometry pose sample {index}: position must have 3 values")
        if len(quaternion) != 4:
            return PoseAssessment(False, f"Odometry pose sample {index}: quaternion must have 4 values")
        if not all(math.isfinite(float(value)) for value in position):
            return PoseAssessment(False, f"Odometry pose sample {index}: non-finite position")
        if any(abs(float(value)) > max_abs_position for value in position):
            return PoseAssessment(
                False,
                f"Odometry pose sample {index}: position exceeds +/-{max_abs_position:.1f} m",
            )
        if not all(math.isfinite(float(value)) for value in quaternion):
            return PoseAssessment(False, f"Odometry pose sample {index}: non-finite quaternion")
        norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
        if not min_quaternion_norm <= norm <= max_quaternion_norm:
            return PoseAssessment(
                False,
                f"Odometry pose sample {index}: quaternion norm {norm:.6f} outside "
                f"[{min_quaternion_norm:.2f}, {max_quaternion_norm:.2f}]",
            )
    return PoseAssessment(True, f"Odometry pose: OK samples={len(positions_xyz)}")


def assess_samples(
    name: str,
    received_monotonic: Sequence[float],
    header_epochs: Sequence[float],
    *,
    now_monotonic: float,
    now_epoch: float,
    window_seconds: float,
    min_hz: float,
    max_header_age: float,
    max_receive_silence: float,
) -> Assessment:
    """Assess one topic window using receive rate and message header age."""
    if not received_monotonic or not header_epochs:
        return Assessment(False, f"{name}: no samples received", 0.0, math.inf, math.inf)
    if len(received_monotonic) != len(header_epochs):
        return Assessment(False, f"{name}: sample/header count mismatch", 0.0, math.inf, math.inf)
    values = [*received_monotonic, *header_epochs, now_monotonic, now_epoch]
    if not all(math.isfinite(value) for value in values):
        return Assessment(False, f"{name}: non-finite timing data", 0.0, math.inf, math.inf)

    span = received_monotonic[-1] - received_monotonic[0]
    rate_hz = ((len(received_monotonic) - 1) / span) if len(received_monotonic) >= 2 and span > 0 else 0.0
    header_age = now_epoch - header_epochs[-1]
    receive_silence = now_monotonic - received_monotonic[-1]
    minimum_samples = max(2, math.ceil(min_hz * window_seconds * 0.60))

    if rate_hz < min_hz:
        return Assessment(
            False,
            f"{name}: rate {rate_hz:.2f} Hz is below {min_hz:.2f} Hz",
            rate_hz,
            header_age,
            receive_silence,
        )
    if len(received_monotonic) < minimum_samples:
        return Assessment(
            False,
            f"{name}: rate coverage is too short ({len(received_monotonic)} < {minimum_samples} samples)",
            rate_hz,
            header_age,
            receive_silence,
        )
    if receive_silence > max_receive_silence:
        return Assessment(
            False,
            f"{name}: stream has been silent for {receive_silence:.3f} s",
            rate_hz,
            header_age,
            receive_silence,
        )
    if header_age < -0.10:
        return Assessment(
            False,
            f"{name}: latest header is {-header_age:.3f} s in the future; check clock synchronization",
            rate_hz,
            header_age,
            receive_silence,
        )
    if header_age > max_header_age:
        return Assessment(
            False,
            f"{name}: header age {header_age:.3f} s exceeds {max_header_age:.3f} s",
            rate_hz,
            header_age,
            receive_silence,
        )
    return Assessment(
        True,
        f"{name}: OK rate={rate_hz:.2f} Hz header_age={header_age:.3f} s silence={receive_silence:.3f} s",
        rate_hz,
        header_age,
        receive_silence,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--min-odometry-hz", type=float, default=5.0)
    parser.add_argument("--max-odometry-age", type=float, default=0.30)
    parser.add_argument("--min-scan-hz", type=float, default=5.0)
    parser.add_argument("--max-scan-age", type=float, default=0.40)
    parser.add_argument("--max-receive-silence", type=float, default=0.50)
    parser.add_argument("--max-odometry-position", type=float, default=1000.0)
    parser.add_argument("--min-quaternion-norm", type=float, default=0.50)
    parser.add_argument("--max-quaternion-norm", type=float, default=1.50)
    args = parser.parse_args()
    for key, value in vars(args).items():
        if value <= 0:
            parser.error(f"--{key.replace('_', '-')} must be positive")
    return args


def main() -> int:
    args = parse_args()
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
    except ImportError as exc:
        print(f"REALTIME PREFLIGHT FAIL: ROS 2 Python dependency unavailable: {exc}")
        return 2

    odometry = Samples()
    scan = Samples()
    rclpy.init(args=[])
    node = Node("r680_nav_realtime_preflight")

    def record_header(target: Samples, message: object) -> None:
        target.received_monotonic.append(time.monotonic())
        stamp = message.header.stamp
        target.header_epochs.append(float(stamp.sec) + float(stamp.nanosec) * 1e-9)

    def record_odometry(message: object) -> None:
        record_header(odometry, message)
        pose = message.pose.pose
        odometry.positions_xyz.append(
            (float(pose.position.x), float(pose.position.y), float(pose.position.z))
        )
        odometry.orientations_xyzw.append(
            (
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
        )

    node.create_subscription(
        Odometry,
        "/Odometry",
        record_odometry,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        LaserScan,
        "/scan",
        lambda message: record_header(scan, message),
        qos_profile_sensor_data,
    )

    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.10, remaining))
        now_monotonic = time.monotonic()
        now_epoch = time.time()
        odometry_result = assess_samples(
            "/Odometry",
            odometry.received_monotonic,
            odometry.header_epochs,
            now_monotonic=now_monotonic,
            now_epoch=now_epoch,
            window_seconds=args.duration,
            min_hz=args.min_odometry_hz,
            max_header_age=args.max_odometry_age,
            max_receive_silence=args.max_receive_silence,
        )
        scan_result = assess_samples(
            "/scan",
            scan.received_monotonic,
            scan.header_epochs,
            now_monotonic=now_monotonic,
            now_epoch=now_epoch,
            window_seconds=args.duration,
            min_hz=args.min_scan_hz,
            max_header_age=args.max_scan_age,
            max_receive_silence=args.max_receive_silence,
        )
        pose_result = assess_odometry_poses(
            odometry.positions_xyz,
            odometry.orientations_xyzw,
            max_abs_position=args.max_odometry_position,
            min_quaternion_norm=args.min_quaternion_norm,
            max_quaternion_norm=args.max_quaternion_norm,
        )
        results = [odometry_result, scan_result, pose_result]
    finally:
        node.destroy_node()
        rclpy.shutdown()

    for result in results:
        print(result.detail)
    if all(result.ok for result in results):
        print("REALTIME PREFLIGHT PASS: chassis arming may continue")
        return 0
    print("REALTIME PREFLIGHT FAIL: chassis remains disarmed")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
