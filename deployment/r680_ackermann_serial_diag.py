#!/usr/bin/env python3
"""Safe zero-only diagnostic for the WHEELTEC C50X USART3 link.

This utility deliberately has no non-zero command option.  It is intended to
be run with the motor power off before any physical motion test.  It verifies
that the UART opens with DTR/RTS low, sends only the validated zero frame, and
reports raw/decoded STM32 feedback statistics.
"""

from __future__ import annotations

import argparse
import time

from r680_ackermann_bridge import TelemetryParser, encode_command, open_serial


MOTION_VX_THRESHOLD = 0.01
MOTION_WZ_THRESHOLD = 0.02


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device",
        default="/dev/serial/by-id/YOUR_CHASSIS_SERIAL",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--period", type=float, default=0.1)
    args = parser.parse_args()

    if args.duration <= 0 or args.period <= 0:
        parser.error("--duration and --period must be positive")

    zero = encode_command(0.0, 0.0, 0.0)
    print("zero_frame=", zero.hex(" ").upper(), "len=", len(zero))

    serial_port = open_serial(args.device, args.baudrate)
    try:
        print(
            "opened:",
            "dtr=", getattr(serial_port, "dtr", "n/a"),
            "rts=", getattr(serial_port, "rts", "n/a"),
            "exclusive=", getattr(serial_port, "exclusive", "n/a"),
        )
        parser_feedback = TelemetryParser()
        start = time.monotonic()
        next_tx = start
        tx_frames = 0
        rx_bytes = 0
        feedback = []
        raw = bytearray()

        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            if now >= next_tx:
                written = serial_port.write(zero)
                serial_port.flush()
                if written != len(zero):
                    raise RuntimeError(f"short write: {written}/{len(zero)} bytes")
                tx_frames += 1
                next_tx += args.period

            waiting = int(getattr(serial_port, "in_waiting", 0) or 0)
            if waiting > 0:
                data = serial_port.read(min(waiting, 4096))
                rx_bytes += len(data)
                raw.extend(data)
                feedback.extend(parser_feedback.feed(data))
            time.sleep(0.002)
    finally:
        serial_port.close()

    # The firmware feedback is quantized and can report a few mm/s or a few
    # mrad/s while stationary.  Treat only values above these thresholds as
    # actual motion; strict equality would turn harmless sensor jitter into a
    # false alarm.
    zero_feedback = [
        item
        for item in feedback
        if abs(item.vx_m_s) < MOTION_VX_THRESHOLD
        and abs(item.vy_m_s) < MOTION_VX_THRESHOLD
        and abs(item.wz_rad_s) < MOTION_WZ_THRESHOLD
    ]
    nonzero_feedback = [item for item in feedback if item not in zero_feedback]
    print("tx_frames=", tx_frames)
    print("rx_bytes=", rx_bytes)
    print("valid_feedback=", len(feedback))
    print("zero_feedback=", len(zero_feedback))
    print("nonzero_feedback=", len(nonzero_feedback))
    if feedback:
        print(
            "feedback_unique=",
            sorted(
                {
                    (
                        round(item.vx_m_s, 3),
                        round(item.vy_m_s, 3),
                        round(item.wz_rad_s, 3),
                        item.flag_stop,
                    )
                    for item in feedback
                }
            ),
        )
    print("raw_prefix=", raw[:96].hex(" ").upper())

    if nonzero_feedback:
        print("RESULT=CHECK_NONZERO_FEEDBACK")
        return 2
    if not feedback:
        print("RESULT=NO_VALID_FEEDBACK")
        return 3
    print("RESULT=ZERO_LINK_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
