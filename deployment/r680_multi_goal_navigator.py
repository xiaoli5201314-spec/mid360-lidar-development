#!/usr/bin/env python3
"""Send a checked multi-point route to Nav2's NavigateThroughPoses action.

The client only sends a navigation action.  It does not open the chassis UART
and does not publish motion commands.  Nav2 remains the sole path/controller
owner; the existing bridge safety gate and odometry watchdog therefore remain
in effect.

Examples (on RK3588, after the real 2D/3D chassis workflow is armed)::

  r680_multi_goal_navigator.py --point 3.0 0.0 0 --point 3.0 2.0 1.57
  r680_multi_goal_navigator.py --route /root/r680_nav/routes/patrol.yaml --loops 3
  r680_multi_goal_navigator.py --route patrol.yaml --loops 5 --return-to-start

Yaw is in radians by default.  A CLI yaw ending in ``deg`` is accepted too.
The route file may be YAML or JSON and contains either a top-level ``points``
list or is itself a list.  A point is ``[x, y, yaw]`` or a mapping with
``x``, ``y`` and ``yaw``/``yaw_rad``/``yaw_deg`` keys.
"""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import signal
import sys
import time
from typing import Any, Iterable, Sequence


DEFAULT_ACTION = "/navigate_through_poses"
DEFAULT_FRAME = "map"
MAX_EXPANDED_POSES = 256


@dataclass(frozen=True)
class RoutePoint:
    x: float
    y: float
    yaw: float


def _finite_number(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def parse_angle(value: Any, *, name: str = "yaw") -> float:
    """Parse radians, or a string with an explicit ``deg`` suffix."""

    if isinstance(value, str):
        text = value.strip().lower()
        if text.endswith("deg"):
            return math.radians(_finite_number(text[:-3].strip(), name))
        if text.endswith("rad"):
            text = text[:-3].strip()
        value = text
    return _finite_number(value, name)


def point_from_value(value: Any, *, index: int = 0) -> RoutePoint:
    """Convert one JSON/YAML point to a validated route point."""

    if isinstance(value, dict):
        if "x" not in value or "y" not in value:
            raise ValueError(f"point {index + 1} must contain x and y")
        x = _finite_number(value["x"], f"point {index + 1} x")
        y = _finite_number(value["y"], f"point {index + 1} y")
        if "yaw_deg" in value:
            yaw = math.radians(_finite_number(value["yaw_deg"], f"point {index + 1} yaw_deg"))
        elif "yaw_rad" in value:
            yaw = parse_angle(value["yaw_rad"], name=f"point {index + 1} yaw_rad")
        else:
            yaw = parse_angle(value.get("yaw", 0.0), name=f"point {index + 1} yaw")
        return RoutePoint(x, y, yaw)
    if isinstance(value, (list, tuple)) and len(value) in (2, 3):
        x = _finite_number(value[0], f"point {index + 1} x")
        y = _finite_number(value[1], f"point {index + 1} y")
        yaw = parse_angle(value[2] if len(value) == 3 else 0.0, name=f"point {index + 1} yaw")
        return RoutePoint(x, y, yaw)
    raise ValueError(f"point {index + 1} must be [x, y, yaw] or a mapping")


def parse_point_tokens(tokens: Sequence[str]) -> list[RoutePoint]:
    """Parse one CLI ``--point X Y YAW`` occurrence."""

    if len(tokens) != 3:
        raise ValueError("--point requires X Y YAW")
    return [point_from_value(tokens, index=0)]


def _simple_yaml_load(text: str) -> Any:
    """Small fallback for the route format when PyYAML is unavailable.

    The deployed image normally has PyYAML.  This fallback intentionally only
    accepts the documented route shape, rather than pretending to be a YAML
    parser for arbitrary input.
    """

    lines = [line.rstrip() for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")]
    frame_id = DEFAULT_FRAME
    points: list[Any] = []
    in_points = False
    current: dict[str, Any] | None = None

    def flush_current() -> None:
        if current is None:
            return
        if "x" not in current or "y" not in current:
            raise ValueError("each YAML point must contain x and y")
        item: dict[str, Any] = {"x": current["x"], "y": current["y"]}
        for key in ("yaw", "yaw_rad", "yaw_deg"):
            if key in current:
                item[key] = current[key]
                break
        points.append(item)
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("frame_id:"):
            frame_id = stripped.split(":", 1)[1].strip().strip("'\"") or DEFAULT_FRAME
            continue
        if stripped == "points:":
            in_points = True
            continue
        if not in_points:
            continue
        if stripped.startswith("-"):
            flush_current()
            current = {}
            remainder = stripped[1:].strip()
            if remainder.startswith("[") and remainder.endswith("]"):
                values = [v.strip() for v in remainder[1:-1].split(",")]
                points.append(values)
                current = None
            elif remainder:
                key, _, value = remainder.partition(":")
                if key.strip() and value:
                    current[key.strip()] = float(value.strip())
            continue
        if current is not None and ":" in stripped:
            key, _, value = stripped.partition(":")
            current[key.strip()] = float(value.strip())
    if current is not None:
        flush_current()
    if not points:
        raise ValueError("route file has no points (install PyYAML or use JSON)")
    return {"frame_id": frame_id, "points": points}


def load_route(path: str | Path) -> tuple[str, list[RoutePoint]]:
    """Load a documented YAML/JSON route and return ``(frame_id, points)``."""

    route_path = Path(path)
    try:
        text = route_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ValueError(f"cannot read route file {route_path}: {exc}") from exc
    try:
        if route_path.suffix.lower() == ".json":
            document = json.loads(text)
        else:
            try:
                import yaml  # type: ignore
            except ImportError:
                document = _simple_yaml_load(text)
            else:
                document = yaml.safe_load(text)
    except Exception as exc:
        if isinstance(exc, ValueError):
            raise
        raise ValueError(f"cannot parse route file {route_path}: {exc}") from exc
    if isinstance(document, dict):
        frame_id = str(document.get("frame_id", DEFAULT_FRAME))
        raw_points = document.get("points")
    else:
        frame_id = DEFAULT_FRAME
        raw_points = document
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError("route file must contain a non-empty points list")
    points = [point_from_value(value, index=index) for index, value in enumerate(raw_points)]
    if not frame_id.strip():
        raise ValueError("frame_id must not be empty")
    return frame_id.strip(), points


def _without_adjacent_duplicates(points: Iterable[RoutePoint]) -> list[RoutePoint]:
    result: list[RoutePoint] = []
    for point in points:
        if not result or point != result[-1]:
            result.append(point)
    return result


def expand_route(
    points: Sequence[RoutePoint],
    *,
    loops: int = 1,
    return_to_start: bool = False,
    ping_pong: bool = False,
) -> list[RoutePoint]:
    """Expand a finite route without silently dropping any requested point.

    ``loops`` is the number of route traversals.  ``return_to_start`` appends
    the first point to each traversal, so ``A,B --loops 2`` becomes
    ``A,B,A,B,A``.  ``ping_pong`` creates ``A,B,C,B,A`` cycles and is useful
    for a corridor route where the turnaround area is already represented by
    the route points.
    """

    if not points:
        raise ValueError("at least one route point is required")
    if isinstance(loops, bool) or int(loops) != loops or loops < 1:
        raise ValueError("loops must be a positive integer")
    loops = int(loops)
    base = list(points)
    if ping_pong and len(base) > 1:
        base = base + list(reversed(base[1:-1]))
    if return_to_start and base[-1] != base[0]:
        base = base + [base[0]]
    expanded: list[RoutePoint] = []
    for _ in range(loops):
        expanded.extend(base)
    expanded = _without_adjacent_duplicates(expanded)
    if len(expanded) > MAX_EXPANDED_POSES:
        raise ValueError(
            f"expanded route has {len(expanded)} poses; maximum is {MAX_EXPANDED_POSES}"
        )
    return expanded


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    yaw = parse_angle(yaw)
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--point", nargs=3, action="append", metavar=("X", "Y", "YAW"),
                        help="one map-frame point; repeat this option for multiple points")
    parser.add_argument("--route", type=Path, help="YAML/JSON route file")
    parser.add_argument("--frame-id", default=None, help="override route frame (default: map)")
    parser.add_argument("--loops", type=int, default=1, help="number of complete route traversals")
    parser.add_argument("--return-to-start", action="store_true",
                        help="append the first point at the end of every route traversal")
    parser.add_argument("--ping-pong", action="store_true",
                        help="visit route points forward and backward in each traversal")
    parser.add_argument("--action", default=DEFAULT_ACTION,
                        help="Nav2 action name (default: /navigate_through_poses)")
    parser.add_argument("--server-timeout", type=float, default=15.0)
    parser.add_argument("--dry-run", action="store_true", help="validate and print the expanded route only")
    return parser


def collect_route(args: argparse.Namespace) -> tuple[str, list[RoutePoint]]:
    frame_id = DEFAULT_FRAME
    points: list[RoutePoint] = []
    if args.route is not None:
        frame_id, points = load_route(args.route)
    if args.point:
        if args.route is not None:
            raise ValueError("use --route or --point, not both")
        for index, tokens in enumerate(args.point):
            parsed = parse_point_tokens(tokens)
            points.extend(parsed)
    if args.frame_id is not None:
        frame_id = str(args.frame_id).strip()
    if not frame_id:
        raise ValueError("frame id must not be empty")
    if not points:
        raise ValueError("provide at least one --point or --route")
    return frame_id, points


def _make_ros_goal(node: Any, goal_type: Any, frame_id: str, points: Sequence[RoutePoint]) -> Any:
    from geometry_msgs.msg import PoseStamped  # imported only on the ROS target

    stamp = node.get_clock().now().to_msg()
    poses = []
    for point in points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(point.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        poses.append(pose)
    goal = goal_type.Goal()
    goal.poses = poses
    return goal


def run_ros_goal(
    frame_id: str,
    points: Sequence[RoutePoint],
    *,
    action_name: str = DEFAULT_ACTION,
    server_timeout: float = 15.0,
) -> int:
    """Send one action goal and cancel it on Ctrl-C or a non-success result."""

    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateThroughPoses
        from rclpy.action import ActionClient
        from rclpy.executors import ExternalShutdownException
    except ImportError as exc:
        print(f"MULTI-GOAL FAIL: ROS 2 navigation dependencies unavailable: {exc}", file=sys.stderr)
        return 2

    if not action_name.startswith("/"):
        action_name = "/" + action_name
    rclpy.init(args=[])
    node = rclpy.create_node("r680_multi_goal_navigator")
    client = ActionClient(node, NavigateThroughPoses, action_name)
    goal_handle = None
    cancel_requested = False

    def request_cancel(_signum: int, _frame: Any) -> None:
        nonlocal cancel_requested
        cancel_requested = True

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_cancel)
    signal.signal(signal.SIGTERM, request_cancel)
    try:
        print(f"Waiting for Nav2 action server: {action_name}", flush=True)
        if not client.wait_for_server(timeout_sec=max(0.1, server_timeout)):
            print("MULTI-GOAL FAIL: NavigateThroughPoses action server unavailable", file=sys.stderr)
            return 3
        goal = _make_ros_goal(node, NavigateThroughPoses, frame_id, points)
        print(f"Sending {len(points)} route poses in frame '{frame_id}'", flush=True)

        def feedback_callback(message: Any) -> None:
            feedback = message.feedback
            remaining = getattr(feedback, "distance_remaining", None)
            poses_left = getattr(feedback, "number_of_poses_remaining", None)
            parts = []
            if remaining is not None:
                parts.append(f"distance={float(remaining):.2f}m")
            if poses_left is not None:
                parts.append(f"poses_remaining={int(poses_left)}")
            if parts:
                print("FEEDBACK " + " ".join(parts), flush=True)

        send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        while not send_future.done():
            if cancel_requested:
                print("Cancel requested before goal acceptance", flush=True)
                return 130
            rclpy.spin_once(node, timeout_sec=0.10)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("MULTI-GOAL FAIL: Nav2 rejected the route", file=sys.stderr)
            return 4
        print("Route accepted; Nav2 controls replanning and velocity continuously.", flush=True)

        result_future = goal_handle.get_result_async()
        cancel_future = None
        while not result_future.done():
            if cancel_requested and cancel_future is None:
                print("Cancelling route; no new route will be submitted.", flush=True)
                cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_once(node, timeout_sec=0.10)
        result = result_future.result()
        status = getattr(result, "status", None)
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("MULTI-GOAL SUCCEEDED: all route poses completed", flush=True)
            return 0
        if cancel_requested:
            print("MULTI-GOAL CANCELLED: chassis workflow remains under its normal stop/watchdog path", flush=True)
            return 130
        print(f"MULTI-GOAL FAIL: Nav2 action ended with status {status}", file=sys.stderr)
        return 5
    except (KeyboardInterrupt, ExternalShutdownException):
        print("MULTI-GOAL CANCELLED", file=sys.stderr)
        return 130
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.server_timeout <= 0:
        parser.error("--server-timeout must be positive")
    try:
        frame_id, base_points = collect_route(args)
        points = expand_route(
            base_points,
            loops=args.loops,
            return_to_start=args.return_to_start,
            ping_pong=args.ping_pong,
        )
    except ValueError as exc:
        parser.error(str(exc))
    print(f"Route: {len(base_points)} base point(s), expanded to {len(points)} pose(s), loops={args.loops}")
    for index, point in enumerate(points, start=1):
        print(f"  {index:02d}: x={point.x:+.3f} y={point.y:+.3f} yaw={point.yaw:+.3f} rad")
    if args.dry_run:
        print("DRY RUN OK: no ROS action sent")
        return 0
    return run_ros_goal(
        frame_id,
        points,
        action_name=args.action,
        server_timeout=args.server_timeout,
    )


if __name__ == "__main__":
    raise SystemExit(main())
