#!/usr/bin/env python3
"""ROS 2 to WHEELTEC C50X Akm_Car serial bridge.

The STM32 firmware already converts Vx/Vz to the Ackermann steering angle.
This node therefore forwards the existing ROS Twist convention instead of
duplicating the steering calculation on RK3588.

Safety defaults:
  * serial writes are disabled unless --write is supplied;
  * non-zero frames require --allow-nonzero when writing;
  * in-place rotation is rejected for Ackermann kinematics;
  * stale commands are replaced with a zero frame;
  * live writes require continuously fresh, bounded FAST-LIO odometry.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import errno
import os
import select
import logging
import math
import signal
import struct
import sys
import time
from typing import Iterable, List, Optional, Sequence, Tuple

try:  # Linux-only fallback; kept optional so protocol tests run on Windows.
    import fcntl
    import termios
except ImportError:  # pragma: no cover - only used on non-POSIX hosts
    fcntl = None  # type: ignore
    termios = None  # type: ignore


FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
COMMAND_SIZE = 11
TELEMETRY_SIZE = 24


def bcc(data: Iterable[int]) -> int:
    """Return the byte-wise XOR used by the C50X firmware."""

    value = 0
    for item in data:
        value ^= int(item) & 0xFF
    return value


def _finite(value: float) -> float:
    return float(value) if math.isfinite(float(value)) else 0.0


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class CommandValues:
    """Sanitized ROS velocity values in SI units."""

    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class OdometryHealth:
    """Result of the live odometry gate used immediately before serial I/O."""

    safe: bool
    reason: str


@dataclass(frozen=True)
class OdometrySample:
    """Latest odometry timing and position fields needed by the live gate."""

    received_monotonic: float
    header_epoch: float
    position_xyz: Tuple[float, float, float]


def assess_odometry_health(
    sample: Optional[OdometrySample],
    *,
    now_monotonic: float,
    now_epoch: float,
    receive_timeout: float = 0.50,
    max_header_age: float = 0.30,
    future_tolerance: float = 0.10,
    max_abs_position: float = 1000.0,
) -> OdometryHealth:
    """Fail closed when the latest FAST-LIO pose is absent, stale, or diverged."""

    if sample is None:
        return OdometryHealth(False, "no odometry received")

    timing = (sample.received_monotonic, sample.header_epoch, now_monotonic, now_epoch)
    if not all(math.isfinite(float(value)) for value in timing):
        return OdometryHealth(False, "non-finite odometry timing")

    receive_silence = now_monotonic - sample.received_monotonic
    if receive_silence < 0.0:
        return OdometryHealth(False, "odometry receive time is in the future")
    if receive_silence > receive_timeout:
        return OdometryHealth(False, f"odometry stream silent for {receive_silence:.3f} s")

    header_age = now_epoch - sample.header_epoch
    if header_age < -future_tolerance:
        return OdometryHealth(False, f"odometry header is {-header_age:.3f} s in the future")
    if header_age > max_header_age:
        return OdometryHealth(False, f"odometry header age is {header_age:.3f} s")

    if not all(math.isfinite(float(value)) for value in sample.position_xyz):
        return OdometryHealth(False, "odometry position is non-finite")
    if any(abs(float(value)) > max_abs_position for value in sample.position_xyz):
        return OdometryHealth(
            False,
            f"odometry position exceeds +/-{max_abs_position:.1f} m",
        )
    return OdometryHealth(True, "odometry fresh and bounded")


def sanitize_twist(
    vx: float,
    vy: float,
    wz: float,
    *,
    max_linear_x: float = 0.26,
    max_angular_z: float = 0.19,
    minimum_turning_radius: float = 1.40,
    allow_nonzero: bool = True,
    allow_reverse: bool = False,
    allow_in_place_rotation: bool = False,
    in_place_epsilon: float = 0.001,
) -> CommandValues:
    """Apply the safety and Ackermann constraints to a Twist command.

    The C50X Akm_Car has no lateral velocity.  Its firmware also maps a
    non-zero Vz to steering only when Vx is non-zero, so an in-place rotation
    request is converted to a zero command instead of being sent repeatedly.
    """

    max_linear_x = max(0.0, float(max_linear_x))
    max_angular_z = max(0.0, float(max_angular_z))
    minimum_turning_radius = float(minimum_turning_radius)
    if not math.isfinite(minimum_turning_radius) or minimum_turning_radius <= 0.0:
        minimum_turning_radius = 1.40

    clean_vx = _clamp(_finite(vx), -max_linear_x, max_linear_x)
    clean_vy = _finite(vy)
    clean_wz = _clamp(_finite(wz), -max_angular_z, max_angular_z)

    # Ackermann chassis cannot translate sideways.
    clean_vy = 0.0

    # Nav2 keeps reversing disabled.  A supervised manual controller may
    # opt in explicitly while retaining every other Ackermann constraint.
    if clean_vx < 0.0 and not allow_reverse:
        clean_vx = clean_vy = clean_wz = 0.0

    # The Akm_Car firmware returns zero steering when Vx is zero.  Do not
    # keep publishing a physically meaningless in-place rotation request.
    if not allow_in_place_rotation and abs(clean_vx) <= in_place_epsilon:
        clean_vx = clean_vy = clean_wz = 0.0

    if abs(clean_vx) > in_place_epsilon:
        turn_limit = abs(clean_vx) / minimum_turning_radius
        clean_wz = _clamp(clean_wz, -turn_limit, turn_limit)

    if not allow_nonzero:
        clean_vx = clean_vy = clean_wz = 0.0

    return CommandValues(clean_vx, clean_vy, clean_wz)


def encode_command(
    vx: float,
    vy: float,
    wz: float,
    *,
    max_linear_x: float = 0.26,
    max_angular_z: float = 0.19,
    minimum_turning_radius: float = 1.40,
    allow_nonzero: bool = True,
    allow_reverse: bool = False,
    allow_in_place_rotation: bool = False,
) -> bytes:
    """Encode one 11-byte C50X ROS control frame.

    Bytes 3..8 are signed big-endian int16 values in mm/s and mrad/s.  The
    layout matches ``Cmd_Vel_Callback`` in the WHEELTEC ROS source.
    """

    values = sanitize_twist(
        vx,
        vy,
        wz,
        max_linear_x=max_linear_x,
        max_angular_z=max_angular_z,
        minimum_turning_radius=minimum_turning_radius,
        allow_nonzero=allow_nonzero,
        allow_reverse=allow_reverse,
        allow_in_place_rotation=allow_in_place_rotation,
    )

    x_mm_s = int(round(values.vx * 1000.0))
    y_mm_s = int(round(values.vy * 1000.0))
    z_mrad_s = int(round(values.wz * 1000.0))
    # Quantize the linear component first, then derive the largest legal
    # angular wire unit from that same integer value. Independent rounding
    # can otherwise relax the radius constraint on the actual frame.
    radius = float(minimum_turning_radius)
    if not math.isfinite(radius) or radius <= 0.0:
        radius = 1.40
    angular_wire_limit = int(math.floor(max(0.0, float(max_angular_z)) * 1000.0 + 1e-12))
    radius_wire_limit = int(math.floor(abs(x_mm_s) / radius + 1e-12)) if x_mm_s else 0
    z_mrad_s = math.copysign(
        min(abs(z_mrad_s), angular_wire_limit, radius_wire_limit), z_mrad_s
    )
    z_mrad_s = int(z_mrad_s)
    payload = bytes([FRAME_HEADER, 0x00, 0x00]) + struct.pack(">hhh", x_mm_s, y_mm_s, z_mrad_s)
    return payload + bytes([bcc(payload), FRAME_TAIL])


def format_frame(frame: bytes) -> str:
    return " ".join(f"{item:02X}" for item in frame)


@dataclass(frozen=True)
class Telemetry:
    """One 24-byte C50X feedback frame, decoded to native units."""

    flag_stop: int
    vx_m_s: float
    vy_m_s: float
    wz_rad_s: float
    accel_x_raw: int
    accel_y_raw: int
    accel_z_raw: int
    gyro_x_raw: int
    gyro_y_raw: int
    gyro_z_raw: int
    voltage_v: float


class TelemetryParser:
    """Incremental parser for the firmware's 24-byte feedback frame."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[Telemetry]:
        self._buffer.extend(data)
        frames: List[Telemetry] = []

        while True:
            header_at = self._buffer.find(bytes([FRAME_HEADER]))
            if header_at < 0:
                self._buffer.clear()
                break
            if header_at:
                del self._buffer[:header_at]
            if len(self._buffer) < TELEMETRY_SIZE:
                break

            candidate = bytes(self._buffer[:TELEMETRY_SIZE])
            if candidate[-1] != FRAME_TAIL or candidate[-2] != bcc(candidate[:-2]):
                # Drop one byte and rescan. This recovers from noise and
                # command frames appearing on a shared serial stream.
                del self._buffer[0]
                continue

            del self._buffer[:TELEMETRY_SIZE]
            values = struct.unpack(">hhhhhhhhhh", candidate[2:22])
            frames.append(
                Telemetry(
                    flag_stop=candidate[1],
                    vx_m_s=values[0] / 1000.0,
                    vy_m_s=values[1] / 1000.0,
                    wz_rad_s=values[2] / 1000.0,
                    accel_x_raw=values[3],
                    accel_y_raw=values[4],
                    accel_z_raw=values[5],
                    gyro_x_raw=values[6],
                    gyro_y_raw=values[7],
                    gyro_z_raw=values[8],
                    voltage_v=values[9] / 1000.0,
                )
            )

        return frames


class PosixSerial:
    """Minimal 8N1 non-blocking serial adapter using the Python stdlib."""

    def __init__(self, port: str, baudrate: int) -> None:
        if termios is None or fcntl is None:
            raise RuntimeError("the stdlib serial fallback is only available on Linux")
        self._fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            # Prevent a second process from sharing the control UART.  A
            # shared stream can interleave 11-byte command frames and leave
            # the STM32 parser consuming stale/non-zero bytes.
            if hasattr(fcntl, "flock"):
                fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            attrs = termios.tcgetattr(self._fd)
            attrs[0] = 0  # iflag: raw input
            attrs[1] = 0  # oflag: raw output
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            if hasattr(termios, "CRTSCTS"):
                attrs[2] &= ~termios.CRTSCTS
            attrs[3] = 0  # lflag: raw local mode
            speed = getattr(termios, f"B{int(baudrate)}", None)
            if speed is None:
                raise ValueError(f"unsupported baudrate: {baudrate}")
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
            termios.tcflush(self._fd, termios.TCIOFLUSH)
        except Exception:
            os.close(self._fd)
            raise

    @property
    def in_waiting(self) -> int:
        if self._fd < 0:
            return 0
        available = struct.pack("I", 0)
        available = fcntl.ioctl(self._fd, getattr(termios, "FIONREAD", 0x541B), available)
        return struct.unpack("I", available)[0]

    def read(self, size: int) -> bytes:
        if size <= 0:
            return b""
        try:
            return os.read(self._fd, size)
        except BlockingIOError:
            return b""

    def write(self, data: bytes) -> int:
        sent = 0
        deadline = time.monotonic() + 0.2
        while sent < len(data):
            try:
                sent += os.write(self._fd, data[sent:])
            except OSError as exc:
                if exc.errno != errno.EAGAIN or time.monotonic() >= deadline:
                    raise
                select.select([], [self._fd], [], max(0.0, deadline - time.monotonic()))
        return sent

    def flush(self) -> None:
        termios.tcdrain(self._fd)

    def close(self) -> None:
        if self._fd >= 0:
            if hasattr(fcntl, "flock"):
                try:
                    fcntl.flock(self._fd, fcntl.LOCK_UN)
                except OSError:
                    pass
            os.close(self._fd)
            self._fd = -1


def open_serial(port: str, baudrate: int):
    """Open pyserial when present, otherwise use the stdlib Linux adapter."""

    try:
        import serial  # type: ignore
    except ImportError:
        return PosixSerial(port, baudrate)

    # Open with DTR/RTS low.  The WCH USB-UART bridge exposes these modem
    # control lines, and the Akm_Car board can reset when they are asserted
    # during open.  A reset starts its built-in 10-second self-test, which
    # briefly commands forward motion even though the ROS command is zero.
    # Opening first with port=None lets us set the line states before the
    # actual open operation.
    serial_kwargs = dict(
        port=None,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=0.2,
        dsrdtr=False,
        rtscts=False,
    )
    try:
        serial_kwargs["exclusive"] = True
        connection = serial.Serial(**serial_kwargs)
    except TypeError:
        # Keep compatibility with older pyserial releases that do not expose
        # the exclusive keyword.
        serial_kwargs.pop("exclusive", None)
        connection = serial.Serial(**serial_kwargs)

    def set_modem_line(name: str) -> None:
        try:
            setattr(connection, name, False)
        except OSError as exc:
            # Pseudo-terminals and a few USB-UART drivers do not implement
            # modem-control ioctls.  They are still valid 8N1 data endpoints;
            # on real WCH hardware the lines are supported and remain low.
            if exc.errno not in (errno.ENOTTY, errno.EINVAL):
                raise

    set_modem_line("dtr")
    set_modem_line("rts")
    connection.port = port
    connection.open()
    set_modem_line("dtr")
    set_modem_line("rts")
    connection.reset_input_buffer()
    connection.reset_output_buffer()
    return connection


class ControlFileLock:
    """Cross-process owner lock for every live chassis-control path."""

    def __init__(self, path: str) -> None:
        self.path = path
        self._handle = None

    def acquire(self) -> None:
        if self._handle is not None or not self.path:
            return
        if fcntl is None:
            raise RuntimeError("control-file locking requires POSIX fcntl")
        parent = os.path.dirname(self.path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        handle = open(self.path, "a+b")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            handle.close()
            if exc.errno in (errno.EACCES, errno.EAGAIN):
                raise RuntimeError("another chassis controller already holds the control lock") from exc
            raise
        handle.seek(0)
        handle.truncate()
        handle.write(f"{os.getpid()}\n".encode("ascii"))
        handle.flush()
        self._handle = handle

    def close(self) -> None:
        handle = self._handle
        self._handle = None
        if handle is None:
            return
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()


class AckermannBridgeNode:
    """Small ROS 2 node kept separate so protocol code remains testable."""

    def __init__(self, node, args: argparse.Namespace) -> None:
        self.node = node
        self.args = args
        self._serial = None
        self._control_lock = None
        self._parser = TelemetryParser()
        self._latest = CommandValues(0.0, 0.0, 0.0)
        self._last_cmd_monotonic = time.monotonic()
        self._last_report_monotonic = 0.0
        self._last_feedback_report_monotonic = 0.0
        self._startup_deadline = time.monotonic() + args.startup_delay if args.write else None
        self._odom_sample: Optional[OdometrySample] = None
        self._odom_health = OdometryHealth(
            not args.write,
            "odometry gate not required in dry-run" if not args.write else "no odometry received",
        )
        self._last_transmitted = CommandValues(0.0, 0.0, 0.0)

        try:
            if args.write and args.control_lock:
                self._control_lock = ControlFileLock(args.control_lock)
                self._control_lock.acquire()

            from geometry_msgs.msg import Twist
            from rclpy.qos import HistoryPolicy, QoSProfile

            command_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)

            # Only a live serial bridge needs the odometry gate.  Keeping this
            # subscription out of dry-run mode preserves protocol previews on
            # hosts that do not have nav_msgs installed.
            if args.write:
                from nav_msgs.msg import Odometry
                from rclpy.qos import qos_profile_sensor_data

                self._odom_subscription = node.create_subscription(
                    Odometry,
                    args.odom_topic,
                    self._odom_callback,
                    qos_profile_sensor_data,
                )

            self._subscription = node.create_subscription(
                Twist,
                args.cmd_topic,
                self._cmd_callback,
                command_qos,
            )
            self._timer = node.create_timer(1.0 / args.rate, self._tick)
        except Exception:
            lock = self._control_lock
            self._control_lock = None
            if lock is not None:
                lock.close()
            raise
        node.get_logger().info(
            f"Ackermann bridge: topic={args.cmd_topic} device={args.device} "
            f"baud={args.baudrate} write={args.write} allow_nonzero={args.allow_nonzero} "
            f"allow_reverse={args.allow_reverse}"
        )
        if not args.write:
            node.get_logger().info("DRY-RUN: no bytes will be written to the serial port.")
        elif args.startup_delay > 0.0:
            node.get_logger().info(f"WRITE startup delay: {args.startup_delay:.1f} s; serial remains closed.")
        elif not args.allow_nonzero:
            node.get_logger().warn("WRITE ENABLED but non-zero frames are blocked; only zero frames can be sent.")
        else:
            node.get_logger().info(
                f"LIVE odometry gate: topic={args.odom_topic} receive_timeout={args.odom_timeout:.2f}s "
                f"max_header_age={args.max_odom_header_age:.2f}s "
                f"future_tolerance={args.odom_future_tolerance:.2f}s "
                f"max_abs_position={args.max_odom_abs_position:.1f}m"
            )

    def _cmd_callback(self, message) -> None:
        self._latest = sanitize_twist(
            message.linear.x,
            message.linear.y,
            message.angular.z,
            max_linear_x=self.args.max_linear_x,
            max_angular_z=self.args.max_angular_z,
            minimum_turning_radius=self.args.minimum_turning_radius,
            allow_nonzero=True,
            allow_reverse=self.args.allow_reverse,
            allow_in_place_rotation=self.args.allow_in_place_rotation,
        )
        self._last_cmd_monotonic = time.monotonic()

    def _odom_callback(self, message) -> None:
        """Record one pose and stop immediately when it fails the safety gate."""

        stamp = message.header.stamp
        header_epoch = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        position = message.pose.pose.position
        self._odom_sample = OdometrySample(
            received_monotonic=time.monotonic(),
            header_epoch=header_epoch,
            position_xyz=(float(position.x), float(position.y), float(position.z)),
        )
        health = self._assess_odometry()
        was_safe = self._odom_health.safe
        self._odom_health = health
        if health.safe and not was_safe:
            self.node.get_logger().info("Odometry gate recovered; live non-zero output is permitted.")
        elif not health.safe and was_safe:
            self.node.get_logger().warn(f"Odometry gate blocked live output: {health.reason}")
            self._send_immediate_zero()

    def _assess_odometry(self, *, now_monotonic: Optional[float] = None, now_epoch: Optional[float] = None):
        return assess_odometry_health(
            self._odom_sample,
            now_monotonic=time.monotonic() if now_monotonic is None else now_monotonic,
            now_epoch=time.time() if now_epoch is None else now_epoch,
            receive_timeout=self.args.odom_timeout,
            max_header_age=self.args.max_odom_header_age,
            future_tolerance=self.args.odom_future_tolerance,
            max_abs_position=self.args.max_odom_abs_position,
        )

    def _send_immediate_zero(self) -> None:
        """Send one neutral frame on a health transition, without chatter."""

        if self._serial is None or self._last_transmitted == CommandValues(0.0, 0.0, 0.0):
            return
        args = getattr(self, "args", None)
        if args is None:
            zero = encode_command(0.0, 0.0, 0.0)
        else:
            zero = encode_command(
                0.0,
                0.0,
                0.0,
                minimum_turning_radius=args.minimum_turning_radius,
                allow_reverse=args.allow_reverse,
                allow_in_place_rotation=args.allow_in_place_rotation,
            )
        self._serial.write(zero)
        self._last_transmitted = CommandValues(0.0, 0.0, 0.0)

    def _tick(self) -> None:
        now = time.monotonic()
        if self.args.write and self._serial is None:
            assert self._startup_deadline is not None
            if now < self._startup_deadline:
                return
            try:
                self._serial = open_serial(self.args.device, self.args.baudrate)
            except Exception as exc:
                raise RuntimeError(f"cannot open serial device {self.args.device}: {exc}") from exc
            # Commands seen while the UART was deliberately closed are not
            # safe to replay when it opens.  Require a subsequent Twist.
            self._latest = CommandValues(0.0, 0.0, 0.0)
            self._last_cmd_monotonic = now

        stale = now - self._last_cmd_monotonic > self.args.cmd_timeout
        if self.args.write:
            odom_health = self._assess_odometry(now_monotonic=now, now_epoch=time.time())
            if odom_health != self._odom_health:
                was_safe = self._odom_health.safe
                self._odom_health = odom_health
                if not odom_health.safe and was_safe:
                    self.node.get_logger().warn(f"Odometry gate blocked live output: {odom_health.reason}")
                elif odom_health.safe and not was_safe:
                    self.node.get_logger().info("Odometry gate recovered; live non-zero output is permitted.")
        odom_blocked = self.args.write and not self._odom_health.safe
        values = CommandValues(0.0, 0.0, 0.0) if stale or odom_blocked else self._latest
        transmitted = sanitize_twist(
            values.vx,
            values.vy,
            values.wz,
            max_linear_x=self.args.max_linear_x,
            max_angular_z=self.args.max_angular_z,
            minimum_turning_radius=self.args.minimum_turning_radius,
            allow_nonzero=(not self.args.write) or self.args.allow_nonzero,
            allow_reverse=self.args.allow_reverse,
            allow_in_place_rotation=self.args.allow_in_place_rotation,
        )
        frame = encode_command(
            transmitted.vx,
            transmitted.vy,
            transmitted.wz,
            max_linear_x=self.args.max_linear_x,
            max_angular_z=self.args.max_angular_z,
            minimum_turning_radius=self.args.minimum_turning_radius,
            allow_nonzero=True,
            allow_reverse=self.args.allow_reverse,
            allow_in_place_rotation=self.args.allow_in_place_rotation,
        )

        if self._serial is not None:
            self._serial.write(frame)
            self._last_transmitted = transmitted

        if now - self._last_report_monotonic >= self.args.log_period:
            state = (
                f"odom-blocked({self._odom_health.reason})->zero"
                if odom_blocked
                else ("stale->zero" if stale else "active")
            )
            self.node.get_logger().info(
                f"{state} vx={transmitted.vx:+.3f} m/s wz={transmitted.wz:+.3f} rad/s "
                f"tx=[{format_frame(frame)}]"
            )
            self._last_report_monotonic = now

        self._read_feedback(now)

    def _read_feedback(self, now: float) -> None:
        if self._serial is None:
            return
        waiting = getattr(self._serial, "in_waiting", 0)
        if waiting <= 0:
            return
        frames = self._parser.feed(self._serial.read(waiting))
        if not frames or now - self._last_feedback_report_monotonic < self.args.log_period:
            return
        latest = frames[-1]
        self.node.get_logger().info(
            f"feedback stop={latest.flag_stop} vx={latest.vx_m_s:+.3f} m/s "
            f"wz={latest.wz_rad_s:+.3f} rad/s voltage={latest.voltage_v:.3f} V"
        )
        self._last_feedback_report_monotonic = now

    def close(self) -> None:
        serial = self._serial
        self._serial = None
        try:
            if serial is not None:
                args = getattr(self, "args", None)
                if args is None:
                    zero = encode_command(0.0, 0.0, 0.0)
                else:
                    zero = encode_command(
                        0.0,
                        0.0,
                        0.0,
                        minimum_turning_radius=args.minimum_turning_radius,
                        allow_reverse=args.allow_reverse,
                        allow_in_place_rotation=args.allow_in_place_rotation,
                    )
                for _ in range(3):
                    try:
                        serial.write(zero)
                    except Exception:
                        pass
                    try:
                        serial.flush()
                    except Exception:
                        pass
                    time.sleep(0.02)
        finally:
            if serial is not None:
                try:
                    serial.close()
                except Exception:
                    pass
            control_lock = getattr(self, "_control_lock", None)
            self._control_lock = None
            if control_lock is not None:
                try:
                    control_lock.close()
                except Exception:
                    pass


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/serial/by-id/YOUR_CHASSIS_SERIAL")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--cmd-timeout", type=float, default=0.5)
    parser.add_argument("--odom-timeout", type=float, default=0.50)
    parser.add_argument("--max-odom-header-age", type=float, default=0.30)
    parser.add_argument("--odom-future-tolerance", type=float, default=0.10)
    parser.add_argument("--max-odom-abs-position", type=float, default=1000.0)
    parser.add_argument("--log-period", type=float, default=1.0)
    parser.add_argument("--max-linear-x", type=float, default=0.26)
    parser.add_argument("--max-angular-z", type=float, default=0.19)
    parser.add_argument("--minimum-turning-radius", type=float, default=1.40)
    parser.add_argument("--startup-delay", type=float, default=6.0)
    parser.add_argument("--control-lock", default="")
    parser.add_argument("--write", action="store_true", help="open the serial port and write frames")
    parser.add_argument(
        "--allow-nonzero",
        action="store_true",
        help="allow non-zero frames; required together with --write for motion",
    )
    parser.add_argument(
        "--allow-reverse",
        action="store_true",
        help="allow negative Vx commands; opt-in for the isolated reverse Nav2 mode",
    )
    parser.add_argument(
        "--allow-in-place-rotation",
        action="store_true",
        help="do not suppress Vz when Vx is zero (not valid for normal Akm_Car use)",
    )
    return parser


def validate_configuration(args: argparse.Namespace) -> None:
    """Reject unsafe numeric configuration before ROS initialization."""

    numeric_fields = (
        "rate",
        "cmd_timeout",
        "odom_timeout",
        "max_odom_header_age",
        "odom_future_tolerance",
        "max_odom_abs_position",
        "max_linear_x",
        "max_angular_z",
        "minimum_turning_radius",
        "startup_delay",
        "log_period",
    )
    for name in numeric_fields:
        try:
            value = float(getattr(args, name))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite number") from exc
        if not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")

    if args.rate < 20.0:
        raise ValueError("rate must be at least 20 Hz")
    if args.cmd_timeout <= 0.0 or args.cmd_timeout > 0.5:
        raise ValueError("cmd_timeout must be greater than 0 and no more than 0.5 s")
    if args.odom_timeout <= 0.0:
        raise ValueError("odom_timeout must be positive")
    if args.max_odom_header_age <= 0.0:
        raise ValueError("max_odom_header_age must be positive")
    if args.odom_future_tolerance < 0.0:
        raise ValueError("odom_future_tolerance must not be negative")
    if args.max_odom_abs_position <= 0.0:
        raise ValueError("max_odom_abs_position must be positive")
    if args.max_linear_x <= 0.0:
        raise ValueError("max_linear_x must be positive")
    if args.max_angular_z <= 0.0:
        raise ValueError("max_angular_z must be positive")
    if args.minimum_turning_radius <= 0.0:
        raise ValueError("minimum_turning_radius must be positive")
    if args.startup_delay < 0.0:
        raise ValueError("startup_delay must not be negative")
    if args.log_period <= 0.0:
        raise ValueError("log_period must be positive")
    if not str(args.odom_topic).strip():
        raise ValueError("odom_topic must not be empty")
    if args.write:
        if args.max_linear_x > 0.26:
            raise ValueError("live max_linear_x must not exceed 0.26 m/s")
        if args.max_angular_z > 0.19:
            raise ValueError("live max_angular_z must not exceed 0.19 rad/s")
        if args.minimum_turning_radius < 1.40:
            raise ValueError("live minimum_turning_radius must be at least 1.40 m")
        if args.startup_delay < 6.0:
            raise ValueError("live startup_delay must be at least 6.0 s")
        if args.odom_timeout > 0.50:
            raise ValueError("live odom_timeout must not exceed 0.50 s")
        if args.max_odom_header_age > 0.30:
            raise ValueError("live max_odom_header_age must not exceed 0.30 s")
        if args.odom_future_tolerance > 0.10:
            raise ValueError("live odom_future_tolerance must not exceed 0.10 s")
        if args.max_odom_abs_position > 1000.0:
            raise ValueError("live max_odom_abs_position must not exceed 1000.0 m")
        if args.allow_in_place_rotation:
            raise ValueError("live allow_in_place_rotation is not permitted")


def _handle_sigterm(_signum, _frame) -> None:
    """Route SIGTERM through main's finally block; never do I/O here."""

    raise KeyboardInterrupt


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _argument_parser()
    parsed, ros_args = parser.parse_known_args(argv)

    try:
        validate_configuration(parsed)
    except ValueError as exc:
        parser.error(str(exc))

    if parsed.allow_nonzero and not parsed.write:
        # This flag is harmless in dry-run, but spelling it out avoids making
        # a later command-line copy accidentally look like a live command.
        logging.getLogger(__name__).warning("--allow-nonzero has no effect without --write")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        print("rclpy is required to run the ROS 2 node", file=sys.stderr)
        return 2

    rclpy.init(args=ros_args)
    node = Node("r680_ackermann_cmd_bridge")
    bridge: Optional[AckermannBridgeNode] = None
    previous_sigterm_handler = signal.signal(signal.SIGTERM, _handle_sigterm)
    try:
        bridge = AckermannBridgeNode(node, parsed)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # pragma: no cover - hardware/runtime path
        if rclpy.ok():
            node.get_logger().error(f"bridge stopped: {exc}")
        else:
            print(f"bridge stopped: {exc}", file=sys.stderr)
        return 1
    finally:
        if bridge is not None:
            bridge.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_sigterm_handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
