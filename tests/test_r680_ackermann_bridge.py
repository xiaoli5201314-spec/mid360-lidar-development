import struct
import sys
import types
from pathlib import Path

import pytest

BRIDGE_DIR = Path(__file__).resolve().parents[1] / "deployment"
sys.path.insert(0, str(BRIDGE_DIR))

from r680_ackermann_bridge import (  # noqa: E402
    FRAME_HEADER,
    FRAME_TAIL,
    TelemetryParser,
    bcc,
    encode_command,
    sanitize_twist,
)
import r680_ackermann_bridge as bridge  # noqa: E402


def test_zero_command_matches_c50x_layout():
    frame = encode_command(0.0, 0.0, 0.0)
    assert frame == bytes([FRAME_HEADER, 0, 0, 0, 0, 0, 0, 0, 0, FRAME_HEADER, FRAME_TAIL])
    assert len(frame) == 11
    assert frame[9] == bcc(frame[:9])


def test_command_uses_signed_big_endian_millimeter_units():
    frame = encode_command(
        0.125,
        -0.125,
        0.2,
        max_linear_x=1.0,
        max_angular_z=1.0,
        minimum_turning_radius=0.2,
    )
    assert frame[3:9] == struct.pack(">hhh", 125, 0, 200)
    assert frame[9] == bcc(frame[:9])


@pytest.mark.parametrize("vx,wz", [(0.004, 0.19), (0.12, 0.19), (0.12, -0.19), (0.26, 0.19), (0.26, -0.19)])
def test_encoded_wire_frame_preserves_minimum_turning_radius(vx, wz):
    """The decoded integer wire units must satisfy the live Ackermann radius."""
    frame = encode_command(vx, 0.0, wz)
    wire_vx, _wire_vy, wire_wz = struct.unpack(">hhh", frame[3:9])
    assert wire_vx >= 0
    if wire_vx == 0:
        assert wire_wz == 0
    else:
        assert abs(wire_wz) <= wire_vx / 1.40


def test_ackermann_suppresses_in_place_rotation_and_lateral_speed():
    values = sanitize_twist(0.0, 0.4, 1.0)
    assert values.vx == 0.0
    assert values.vy == 0.0
    assert values.wz == 0.0


def test_reverse_and_low_speed_commands_become_complete_zero_commands():
    reverse = sanitize_twist(-0.01, 0.3, 0.04)
    low_speed = sanitize_twist(0.001, 0.3, 0.04)
    assert reverse == sanitize_twist(0.0, 0.0, 0.0)
    assert low_speed == sanitize_twist(0.0, 0.0, 0.0)


def test_reverse_is_available_only_when_explicitly_enabled():
    blocked = sanitize_twist(-0.12, 0.0, -0.2, max_linear_x=0.26, max_angular_z=1.0)
    allowed = sanitize_twist(
        -0.12,
        0.0,
        -0.2,
        max_linear_x=0.26,
        max_angular_z=1.0,
        minimum_turning_radius=1.40,
        allow_reverse=True,
    )
    assert blocked == sanitize_twist(0.0, 0.0, 0.0)
    assert allowed.vx == pytest.approx(-0.12)
    assert allowed.wz == pytest.approx(-0.12 / 1.40)


def test_reverse_wire_frame_uses_signed_speed_and_radius_limit():
    frame = encode_command(
        -0.12,
        0.0,
        -0.2,
        max_linear_x=0.26,
        max_angular_z=1.0,
        minimum_turning_radius=1.40,
        allow_reverse=True,
    )
    wire_vx, _wire_vy, wire_wz = struct.unpack(">hhh", frame[3:9])
    assert wire_vx == -120
    assert wire_wz == -85
    assert abs(wire_wz) <= abs(wire_vx) / 1.40


def test_turn_rate_is_limited_by_radius_and_angular_limit():
    radius_limited = sanitize_twist(0.26, 0.0, 0.30, max_angular_z=0.30)
    angular_limited = sanitize_twist(0.26, 0.0, 0.30)
    assert radius_limited.wz == pytest.approx(0.1857142, abs=1e-7)
    assert angular_limited.wz == pytest.approx(0.1857142, abs=1e-7)


@pytest.mark.parametrize("vx", [0.0, 0.075, 0.15, 0.18, 0.22, 0.26])
def test_bridge_preserves_dynamic_linear_speed_without_minimum_uplift(vx):
    """The bridge caps unsafe maxima but must not replace Nav2's chosen speed."""
    values = sanitize_twist(vx, 0.0, 0.0)
    assert values.vx == pytest.approx(vx)
    assert values.vy == 0.0
    assert values.wz == 0.0


def test_limits_and_write_gate_are_applied():
    values = sanitize_twist(1.0, 0.0, -2.0, max_linear_x=0.05, max_angular_z=0.2, minimum_turning_radius=0.2)
    assert values.vx == 0.05
    assert values.wz == -0.2
    assert sanitize_twist(0.05, 0.0, 0.2, allow_nonzero=False) == sanitize_twist(0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    "overrides",
    [
        {"rate": 19.99},
        {"rate": float("nan")},
        {"cmd_timeout": 0.0},
        {"cmd_timeout": 0.51},
        {"max_linear_x": 0.0},
        {"max_angular_z": float("inf")},
        {"minimum_turning_radius": -1.0},
    ],
)
def test_validate_configuration_rejects_unsafe_numeric_values(overrides):
    args = bridge._argument_parser().parse_args([])
    for name, value in overrides.items():
        setattr(args, name, value)
    with pytest.raises(ValueError):
        bridge.validate_configuration(args)


@pytest.mark.parametrize(
    "overrides",
    [
        {"max_linear_x": 0.261},
        {"max_angular_z": 0.191},
        {"minimum_turning_radius": 1.39},
        {"startup_delay": 5.99},
        {"odom_timeout": 0.501},
        {"max_odom_header_age": 0.301},
        {"odom_future_tolerance": 0.101},
        {"max_odom_abs_position": 1000.1},
        {"allow_in_place_rotation": True},
    ],
)
def test_validate_configuration_rejects_live_mode_relaxations(overrides):
    args = bridge._argument_parser().parse_args(["--write"])
    for name, value in overrides.items():
        setattr(args, name, value)
    with pytest.raises(ValueError):
        bridge.validate_configuration(args)


def test_live_defaults_match_grounded_vehicle_profile():
    args = bridge._argument_parser().parse_args([])
    assert args.max_linear_x == 0.26
    assert args.max_angular_z == 0.19
    bridge.validate_configuration(args)


def _bridge_args(**overrides):
    args = bridge._argument_parser().parse_args([])
    args.write = True
    args.allow_nonzero = True
    args.startup_delay = 6.0
    for name, value in overrides.items():
        setattr(args, name, value)
    return args


def test_write_bridge_accepts_control_lock_argument():
    args = bridge._argument_parser().parse_args(["--write", "--control-lock", "/tmp/r680-control.lock"])
    assert args.control_lock.endswith("r680-control.lock")


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, _message):
        self.messages.append(_message)

    def warn(self, _message):
        self.messages.append(_message)


class _Node:
    def __init__(self):
        self.subscription_qos = None
        self.subscriptions = []
        self.logger = _Logger()

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscription_qos = qos
        self.subscriptions.append((message_type, topic, callback, qos))
        return object()

    def create_timer(self, _period, _callback):
        return object()

    def get_logger(self):
        return self.logger


class _Serial:
    def __init__(self, write_failures=(), flush_failures=()):
        self.frames = []
        self.closed = False
        self._write_failures = set(write_failures)
        self._flush_failures = set(flush_failures)
        self._write_count = 0
        self._flush_count = 0

    @property
    def in_waiting(self):
        return 0

    def write(self, frame):
        self._write_count += 1
        self.frames.append(frame)
        if self._write_count in self._write_failures:
            raise OSError("injected write failure")

    def flush(self):
        self._flush_count += 1
        if self._flush_count in self._flush_failures:
            raise OSError("injected flush failure")

    def close(self):
        self.closed = True


def _install_ros_message_stubs(monkeypatch):
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = type("Twist", (), {})
    geometry_msgs.msg = geometry_msgs_msg
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = type("Odometry", (), {})
    nav_msgs.msg = nav_msgs_msg
    rclpy = types.ModuleType("rclpy")
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    rclpy_qos.qos_profile_sensor_data = types.SimpleNamespace(name="sensor_data")

    class _QoSProfile:
        def __init__(self, *, history, depth):
            self.history = history
            self.depth = depth

    rclpy_qos.QoSProfile = _QoSProfile
    monkeypatch.setitem(sys.modules, "geometry_msgs", geometry_msgs)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs_msg)
    monkeypatch.setitem(sys.modules, "nav_msgs", nav_msgs)
    monkeypatch.setitem(sys.modules, "nav_msgs.msg", nav_msgs_msg)
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.qos", rclpy_qos)


def _twist(vx=0.09, wz=0.05):
    return types.SimpleNamespace(
        linear=types.SimpleNamespace(x=vx, y=0.0),
        angular=types.SimpleNamespace(z=wz),
    )


def _odometry(stamp_epoch, position=(1.0, 2.0, 0.0)):
    seconds = int(stamp_epoch)
    nanoseconds = int(round((stamp_epoch - seconds) * 1_000_000_000))
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=seconds, nanosec=nanoseconds),
        ),
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(
                    x=position[0],
                    y=position[1],
                    z=position[2],
                ),
            ),
        ),
    )


def test_write_startup_delay_defers_open_and_discards_cached_twist(monkeypatch):
    _install_ros_message_stubs(monkeypatch)
    now = [100.0]
    epoch = [1000.0]
    factory_calls = []
    serial = _Serial()
    monkeypatch.setattr(bridge.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(bridge.time, "time", lambda: epoch[0])
    monkeypatch.setattr(bridge, "open_serial", lambda *_args: factory_calls.append(True) or serial)
    node = _Node()
    subject = bridge.AckermannBridgeNode(node, _bridge_args())
    message = _twist()
    subject._cmd_callback(message)
    now[0] = 105.0
    subject._tick()
    assert factory_calls == []
    assert serial.frames == []
    assert node.subscription_qos.depth == 1
    assert node.subscription_qos.history == "keep_last"

    now[0] = 106.0
    subject._tick()
    assert factory_calls == [True]
    assert serial.frames == [encode_command(0.0, 0.0, 0.0)]

    now[0] = 107.0
    epoch[0] = 1007.0
    subject._odom_callback(_odometry(epoch[0]))
    subject._cmd_callback(message)
    subject._tick()
    assert serial.frames[-1] == encode_command(0.09, 0.0, 0.05)


def test_live_bridge_blocks_motion_until_fresh_odometry_and_times_out_to_zero(monkeypatch):
    """Removing the runtime odometry gate would let stale FAST-LIO drive the chassis."""
    _install_ros_message_stubs(monkeypatch)
    now = [100.0]
    epoch = [1000.0]
    serial = _Serial()
    monkeypatch.setattr(bridge.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(bridge.time, "time", lambda: epoch[0])
    monkeypatch.setattr(bridge, "open_serial", lambda *_args: serial)
    subject = bridge.AckermannBridgeNode(_Node(), _bridge_args())

    now[0] = 106.0
    subject._tick()
    subject._cmd_callback(_twist())
    subject._tick()
    assert serial.frames[-1] == bytes.fromhex("7B 00 00 00 00 00 00 00 00 7B 7D")

    now[0] = 106.10
    epoch[0] = 1000.10
    subject._odom_callback(_odometry(1000.10))
    subject._cmd_callback(_twist())
    subject._tick()
    assert serial.frames[-1] == bytes.fromhex("7B 00 00 00 5A 00 00 00 32 13 7D")

    now[0] = 106.61
    epoch[0] = 1000.61
    subject._cmd_callback(_twist())
    subject._tick()
    assert serial.frames[-1] == bytes.fromhex("7B 00 00 00 00 00 00 00 00 7B 7D")


@pytest.mark.parametrize(
    "header_epoch,position",
    [
        (999.69, (1.0, 2.0, 0.0)),
        (1000.11, (1.0, 2.0, 0.0)),
        (1000.0, (float("nan"), 2.0, 0.0)),
        (1000.0, (1000.1, 2.0, 0.0)),
    ],
)
def test_invalid_odometry_immediately_zeroes_and_blocks_until_recovery(
    monkeypatch, header_epoch, position
):
    """Old, future, non-finite, or diverged poses must stop an already moving bridge."""
    _install_ros_message_stubs(monkeypatch)
    now = [100.0]
    epoch = [1000.0]
    serial = _Serial()
    monkeypatch.setattr(bridge.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(bridge.time, "time", lambda: epoch[0])
    monkeypatch.setattr(bridge, "open_serial", lambda *_args: serial)
    subject = bridge.AckermannBridgeNode(_Node(), _bridge_args())

    now[0] = 106.0
    subject._tick()
    subject._odom_callback(_odometry(1000.0))
    subject._cmd_callback(_twist())
    subject._tick()
    assert serial.frames[-1] == bytes.fromhex("7B 00 00 00 5A 00 00 00 32 13 7D")

    frames_before_fault = len(serial.frames)
    subject._odom_callback(_odometry(header_epoch, position))
    assert len(serial.frames) == frames_before_fault + 1
    assert serial.frames[-1] == bytes.fromhex("7B 00 00 00 00 00 00 00 00 7B 7D")

    subject._cmd_callback(_twist())
    subject._tick()
    assert serial.frames[-1] == bytes.fromhex("7B 00 00 00 00 00 00 00 00 7B 7D")

    now[0] = 106.05
    epoch[0] = 1000.05
    subject._odom_callback(_odometry(1000.05))
    subject._cmd_callback(_twist())
    subject._tick()
    assert serial.frames[-1] == bytes.fromhex("7B 00 00 00 5A 00 00 00 32 13 7D")


def test_write_bridge_holds_and_releases_common_control_lock(monkeypatch):
    _install_ros_message_stubs(monkeypatch)
    events = []

    class _Lock:
        def __init__(self, path):
            events.append(("init", path))

        def acquire(self):
            events.append(("acquire",))

        def close(self):
            events.append(("close",))

    monkeypatch.setattr(bridge, "ControlFileLock", _Lock)
    monkeypatch.setattr(bridge, "open_serial", lambda *_args: _Serial())
    subject = bridge.AckermannBridgeNode(
        _Node(),
        _bridge_args(control_lock="/tmp/r680-control.lock"),
    )
    assert events == [("init", "/tmp/r680-control.lock"), ("acquire",)]
    subject.close()
    assert events[-1] == ("close",)


def test_dry_run_never_opens_serial_and_logs_safety_limited_preview(monkeypatch):
    _install_ros_message_stubs(monkeypatch)
    now = [100.0]
    factory_calls = []
    monkeypatch.setattr(bridge.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(bridge, "open_serial", lambda *_args: factory_calls.append(True))
    node = _Node()
    subject = bridge.AckermannBridgeNode(node, _bridge_args(write=False))
    subject._cmd_callback(
        types.SimpleNamespace(
            linear=types.SimpleNamespace(x=1.0, y=0.0),
            angular=types.SimpleNamespace(z=1.0),
        )
    )
    subject._tick()
    expected_frame = bridge.format_frame(encode_command(0.26, 0.0, 0.19))
    assert factory_calls == []
    assert any("DRY-RUN" in message for message in node.logger.messages)
    assert any(
        "vx=+0.260 m/s wz=+0.186 rad/s" in message and f"tx=[{expected_frame}]" in message
        for message in node.logger.messages
    )


def test_sigterm_runs_main_finally_close_without_handler_serial_io(monkeypatch):
    _install_ros_message_stubs(monkeypatch)
    now = [100.0]
    serial = _Serial()
    created_nodes = []
    active_handler = {"handler": None}

    class _MainNode(_Node):
        def __init__(self, _name):
            super().__init__()
            self.timer_callback = None
            self.destroyed = False
            created_nodes.append(self)

        def create_timer(self, _period, callback):
            self.timer_callback = callback
            return object()

        def destroy_node(self):
            self.destroyed = True

    rclpy = sys.modules["rclpy"]
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_node.Node = _MainNode
    rclpy_executors.ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
    rclpy.init = lambda **_kwargs: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None

    def spin(node):
        now[0] = 106.0
        node.timer_callback()
        handler = active_handler["handler"]
        assert handler is bridge._handle_sigterm
        writes_before_handler = len(serial.frames)
        try:
            handler(bridge.signal.SIGTERM, None)
        except KeyboardInterrupt:
            assert len(serial.frames) == writes_before_handler
            raise

    rclpy.spin = spin
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)
    monkeypatch.setitem(sys.modules, "rclpy.executors", rclpy_executors)
    monkeypatch.setattr(bridge.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(bridge.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(bridge, "open_serial", lambda *_args: serial)

    def install_handler(_signum, handler):
        previous = active_handler["handler"]
        active_handler["handler"] = handler
        return previous

    monkeypatch.setattr(bridge.signal, "signal", install_handler)
    assert bridge.main(["--write"]) == 0
    zero = encode_command(0.0, 0.0, 0.0)
    assert serial.frames[-3:] == [zero, zero, zero]
    assert serial.closed
    assert created_nodes[0].destroyed


def test_close_writes_three_zero_frames_despite_io_errors_and_is_idempotent(monkeypatch):
    serial = _Serial(write_failures={1}, flush_failures={2})
    subject = object.__new__(bridge.AckermannBridgeNode)
    subject._serial = serial
    monkeypatch.setattr(bridge.time, "sleep", lambda _seconds: None)
    subject.close()
    subject.close()
    assert serial.frames == [encode_command(0.0, 0.0, 0.0)] * 3
    assert serial.closed


def _telemetry_frame(
    vx_mm_s=120,
    vy_mm_s=0,
    wz_mrad_s=-25,
    accel=(1, 2, 3),
    gyro=(4, 5, 6),
    voltage_mv=24000,
    flag_stop=0,
):
    payload = bytes([FRAME_HEADER, flag_stop]) + struct.pack(
        ">hhhhhhhhhh",
        vx_mm_s,
        vy_mm_s,
        wz_mrad_s,
        *accel,
        *gyro,
        voltage_mv,
    )
    return payload + bytes([bcc(payload), FRAME_TAIL])


def test_telemetry_parser_handles_split_input_and_noise():
    parser = TelemetryParser()
    frame = _telemetry_frame()
    assert parser.feed(b"noise" + frame[:7]) == []
    decoded = parser.feed(frame[7:])
    assert len(decoded) == 1
    assert decoded[0].vx_m_s == 0.12
    assert decoded[0].wz_rad_s == -0.025
    assert decoded[0].voltage_v == 24.0


def test_telemetry_parser_rejects_bad_bcc_and_recovers():
    parser = TelemetryParser()
    bad = bytearray(_telemetry_frame())
    bad[5] ^= 0x01
    good = _telemetry_frame(vx_mm_s=-80)
    decoded = parser.feed(bytes(bad) + good)
    assert len(decoded) == 1
    assert decoded[0].vx_m_s == -0.08
