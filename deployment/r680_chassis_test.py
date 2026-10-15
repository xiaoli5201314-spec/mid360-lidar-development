#!/usr/bin/env python3
"""Bounded raised-chassis motion test for the C50X USART3 link.

This is an intentional hardware test tool, not a ROS bridge.  It sends the
validated 11-byte command frame at 20 Hz and always sends zero frames before
closing the UART.  Keep the vehicle raised for this test; the limits are
diagnostic limits, not a ground-driving interface.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import sys
import time
from typing import Optional, Sequence

from r680_ackermann_bridge import encode_command, format_frame, open_serial


DEFAULT_DEVICE = "/dev/serial/by-id/YOUR_CHASSIS_SERIAL"
DEFAULT_BAUDRATE = 115200
TEST_RATE_HZ = 20.0
MAX_TEST_VX_MPS = 0.12
MAX_TEST_WZ_RADPS = 0.20
MIN_DURATION_S = 0.1
MAX_DURATION_S = 10.0


@dataclass(frozen=True)
class TestRequest:
    vx_m_s: float
    wz_rad_s: float
    duration_s: float


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限数值")
    return value


def validate_test_request(vx: float, wz: float, duration: float) -> TestRequest:
    """Validate one bounded test request without opening the serial port."""

    vx = _finite(vx, "vx")
    wz = _finite(wz, "wz")
    duration = _finite(duration, "duration")
    if abs(vx) > MAX_TEST_VX_MPS:
        raise ValueError(f"|vx| 不能超过 {MAX_TEST_VX_MPS:g} m/s（架空测试上限）")
    if abs(wz) > MAX_TEST_WZ_RADPS:
        raise ValueError(f"|wz| 不能超过 {MAX_TEST_WZ_RADPS:g} rad/s（架空测试上限）")
    if duration < MIN_DURATION_S or duration > MAX_DURATION_S:
        raise ValueError(
            f"时间必须在 {MIN_DURATION_S:g}～{MAX_DURATION_S:g} 秒（架空测试范围）"
        )
    if abs(vx) < 0.001 and abs(wz) >= 0.001:
        raise ValueError("阿克曼底盘不能原地旋转，请同时给出非零 vx")
    return TestRequest(vx, wz, duration)


def _send_zero(serial_port) -> None:
    zero = encode_command(0.0, 0.0, 0.0)
    for _ in range(3):
        written = serial_port.write(zero)
        serial_port.flush()
        if written != len(zero):
            raise RuntimeError(f"停止帧写入不完整：{written}/{len(zero)}")
        time.sleep(0.02)


def run_test(request: TestRequest, device: str, baudrate: int) -> int:
    frame = encode_command(request.vx_m_s, 0.0, request.wz_rad_s)
    print("用途：架空底盘低速诊断，不要在落地状态执行")
    print(f"设备： {device}")
    print(f"串口：{baudrate} 8N1")
    print(f"vx={request.vx_m_s:+.3f} m/s, wz={request.wz_rad_s:+.3f} rad/s, duration={request.duration_s:.2f} s")
    print(f"测试帧： {format_frame(frame)}")

    serial_port = open_serial(device, baudrate)
    tx_frames = 0
    period = 1.0 / TEST_RATE_HZ
    start = time.monotonic()
    next_tx = start
    try:
        while True:
            now = time.monotonic()
            if now - start >= request.duration_s:
                break
            if now >= next_tx:
                written = serial_port.write(frame)
                serial_port.flush()
                if written != len(frame):
                    raise RuntimeError(f"控制帧写入不完整：{written}/{len(frame)}")
                tx_frames += 1
                next_tx += period
                if next_tx < now - period:
                    next_tx = now + period
            time.sleep(0.001)
    finally:
        try:
            _send_zero(serial_port)
        finally:
            serial_port.close()

    print(f"已发送测试帧： {tx_frames}")
    print("已发送零速并关闭串口")
    return 0


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("vx", type=float, help="前进速度，单位 m/s")
    parser.add_argument("wz", type=float, help="角速度，单位 rad/s")
    parser.add_argument("duration", type=float, help="持续时间，单位 s")
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _argument_parser().parse_args(argv)
    try:
        request = validate_test_request(args.vx, args.wz, args.duration)
        return run_test(request, args.device, args.baudrate)
    except (ValueError, OSError, RuntimeError) as exc:
        print(f"拒绝执行：{exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C；已尝试发送零速并关闭串口", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
