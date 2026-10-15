#!/usr/bin/env python3
"""RViz waypoint queue for the WHEELTEC-style multi-point workflow.

RViz's standard ``2D Goal Pose`` tool is configured to publish on
``/r680_multi_goal_pose`` (rather than Nav2's single-goal ``/goal_pose``).
This node accepts each pose, draws the complete queue on ``/waypoints`` and
uses one ``NavigateToPose`` action at a time.  A pose is sent only after the
previous action succeeds, so clicks cannot pre-empt one another.  When at
least two poses have been collected, the default behavior is cyclic: after
the last pose succeeds the queue starts again at the first pose.  Use
``--once`` for a finite route.

The node is deliberately action-only.  It does not publish velocity commands
or open the chassis device; Nav2 and the already deployed chassis bridge
remain the only motion owners.  An action failure clears the queue and leaves
the bridge's normal watchdog/stop path in control.
"""

import argparse
from collections import deque
from dataclasses import dataclass
import math
import signal
import sys
from typing import Any, Deque, Optional, Sequence


DEFAULT_ACTION = "/navigate_to_pose"
DEFAULT_POSE_TOPIC = "/r680_multi_goal_pose"
DEFAULT_CLICKED_TOPIC = "/clicked_point"
DEFAULT_MARKER_TOPIC = "/waypoints"
DEFAULT_FRAME = "map"


@dataclass(frozen=True)
class QueuePoint:
    """Small ROS-independent representation used by the offline tests."""

    x: float
    y: float
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    frame_id: str = DEFAULT_FRAME


class WaypointQueue:
    """Queue state machine shared by the ROS node and offline tests.

    ``route`` retains the points in their click order.  ``pending`` contains
    only poses that still need to be sent.  At the end of a cyclic route,
    ``route`` is copied into ``pending`` so the robot returns to the first
    point and then repeats the same order.
    """

    def __init__(self, *, loop: bool = True) -> None:
        self.loop = bool(loop)
        self.route: list[Any] = []
        self.pending: Deque[Any] = deque()

    def add(self, pose: Any, *, active: bool = False) -> Optional[Any]:
        """Append a clicked pose and return a pose that should start now."""

        # ``route`` is intentionally retained after a finite route so that
        # RViz can keep showing its markers.  A later click while idle starts
        # from that new pose (and, once there are two poses, enables the
        # normal cyclic behavior) instead of getting stuck in ``pending``.
        if not active and not self.pending:
            self.route.append(pose)
            return pose
        self.route.append(pose)
        self.pending.append(pose)
        return None

    def succeeded(self) -> Optional[Any]:
        """Advance after one successful action, or return ``None`` at stop."""

        if self.pending:
            return self.pending.popleft()
        if self.loop and len(self.route) > 1:
            self.pending.extend(self.route)
            return self.pending.popleft()
        return None

    def failed(self) -> None:
        """Drop all queued motion after an action failure."""

        self.route.clear()
        self.pending.clear()

    def clear(self) -> None:
        self.failed()


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    yaw = _finite(yaw, "yaw")
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action", default=DEFAULT_ACTION)
    parser.add_argument("--pose-topic", default=DEFAULT_POSE_TOPIC,
                        help="RViz SetGoal topic (default: %(default)s)")
    parser.add_argument("--clicked-point-topic", default=DEFAULT_CLICKED_TOPIC,
                        help="optional RViz Publish Point topic")
    parser.add_argument("--marker-topic", default=DEFAULT_MARKER_TOPIC)
    parser.add_argument("--frame-id", default=DEFAULT_FRAME)
    parser.add_argument("--once", action="store_true",
                        help="stop after the last queued point instead of looping")
    parser.add_argument("--server-timeout", type=float, default=0.0,
                        help="reserved compatibility option; server waiting is asynchronous")
    return parser


def _pose_key(pose: Any) -> tuple[float, float, float, float, float, float, float]:
    return (
        float(pose.pose.position.x),
        float(pose.pose.position.y),
        float(pose.pose.position.z),
        float(pose.pose.orientation.x),
        float(pose.pose.orientation.y),
        float(pose.pose.orientation.z),
        float(pose.pose.orientation.w),
    )


def _make_clicked_pose(node: Any, message: Any, default_frame: str) -> Any:
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header = message.header
    pose.header.frame_id = pose.header.frame_id or default_frame
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = message.point.x
    pose.pose.position.y = message.point.y
    pose.pose.position.z = message.point.z
    # Publish Point has no heading.  A neutral heading is deterministic and
    # keeps the official point-picking workflow usable alongside SetGoal.
    pose.pose.orientation.w = 1.0
    return pose


class RvizMultiGoalNode:
    """ROS 2 adapter around :class:`WaypointQueue`."""

    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        from geometry_msgs.msg import PointStamped, PoseStamped
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from visualization_msgs.msg import MarkerArray

        self._rclpy = rclpy
        self._NavigateToPose = NavigateToPose
        self._MarkerArray = MarkerArray
        self._node = rclpy.create_node("r680_rviz_multi_goal")
        self._args = args
        self._queue = WaypointQueue(loop=not args.once)
        self._active_handle: Any = None
        self._send_future: Any = None
        self._waiting_pose: Any = None
        self._stopping = False
        self._last_marker_count = 0

        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        self._marker_pub = self._node.create_publisher(
            MarkerArray, args.marker_topic, marker_qos)
        self._action_client = ActionClient(self._node, NavigateToPose, args.action)
        self._pose_sub = self._node.create_subscription(
            PoseStamped, args.pose_topic, self._pose_callback, 10)
        self._point_sub = self._node.create_subscription(
            PointStamped, args.clicked_point_topic, self._point_callback, 10)
        self._retry_timer = self._node.create_timer(0.25, self._retry_send)

        self._node.get_logger().info(
            f"RViz multi-goal ready: pose={args.pose_topic} "
            f"point={args.clicked_point_topic} action={args.action} loop={not args.once}")

    @property
    def node(self) -> Any:
        return self._node

    def _pose_callback(self, message: Any) -> None:
        if self._stopping:
            return
        if not message.header.frame_id:
            message.header.frame_id = self._args.frame_id
        self._accept_pose(message)

    def _point_callback(self, message: Any) -> None:
        if self._stopping:
            return
        self._accept_pose(_make_clicked_pose(self._node, message, self._args.frame_id))

    def _accept_pose(self, pose: Any) -> None:
        if not pose.header.frame_id:
            pose.header.frame_id = self._args.frame_id
        # Do not add the exact same click twice in a row (RViz can emit a
        # duplicate when a tool is changed immediately after a click).
        if self._queue.route and _pose_key(self._queue.route[-1]) == _pose_key(pose):
            self._node.get_logger().info("Ignoring adjacent duplicate waypoint")
            return
        active = self._active_handle is not None or self._send_future is not None or self._waiting_pose is not None
        start_now = self._queue.add(pose, active=active)
        self._publish_markers()
        self._node.get_logger().info(
            f"Queued waypoint {len(self._queue.route)} at "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}); "
            f"pending={len(self._queue.pending)}")
        if start_now is not None:
            self._dispatch(start_now)

    def _dispatch(self, pose: Any) -> None:
        if self._stopping:
            return
        self._waiting_pose = pose
        self._try_send()

    def _retry_send(self) -> None:
        self._try_send()

    def _try_send(self) -> None:
        if self._stopping or self._waiting_pose is None:
            return
        if self._active_handle is not None or self._send_future is not None:
            return
        if not self._action_client.server_is_ready():
            return
        pose = self._waiting_pose
        self._waiting_pose = None
        goal = self._NavigateToPose.Goal()
        goal.pose = pose
        self._node.get_logger().info(
            f"Sending waypoint ({pose.pose.position.x:.2f}, "
            f"{pose.pose.position.y:.2f})")
        self._send_future = self._action_client.send_goal_async(goal)
        self._send_future.add_done_callback(self._goal_response)

    def _goal_response(self, future: Any) -> None:
        self._send_future = None
        try:
            handle = future.result()
        except Exception as exc:  # pragma: no cover - ROS middleware path
            self._fail_route(f"goal request failed: {exc}")
            return
        if handle is None or not handle.accepted:
            self._fail_route("Nav2 rejected waypoint")
            return
        self._active_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future: Any) -> None:
        handle = self._active_handle
        self._active_handle = None
        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # pragma: no cover - ROS middleware path
            self._fail_route(f"result unavailable: {exc}")
            return
        from action_msgs.msg import GoalStatus

        if status != GoalStatus.STATUS_SUCCEEDED:
            self._fail_route(f"waypoint action ended with status {status}")
            return
        next_pose = self._queue.succeeded()
        if next_pose is None:
            self._node.get_logger().info("Route complete; bridge should be stopped by Nav2")
            return
        self._dispatch(next_pose)

    def _fail_route(self, reason: str) -> None:
        self._node.get_logger().error(f"Multi-goal stopped: {reason}")
        self._queue.failed()
        self._waiting_pose = None
        self._publish_markers()

    def _publish_markers(self) -> None:
        # During SIGTERM rclpy invalidates the context before the executor
        # unwinds.  Do not publish a final DELETEALL through an invalid
        # publisher; the next process start will publish a fresh queue.
        if not self._rclpy.ok():
            return
        from visualization_msgs.msg import Marker

        array = self._MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for index, pose in enumerate(self._queue.route):
            arrow = Marker()
            arrow.header = pose.header
            arrow.ns = "r680_waypoints"
            arrow.id = index * 3
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose.pose
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.35, 0.07, 0.03
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.0, 1.0, 0.1, 1.0
            array.markers.append(arrow)

            circle = Marker()
            circle.header = pose.header
            circle.ns = "r680_waypoints"
            circle.id = index * 3 + 1
            circle.type = Marker.SPHERE
            circle.action = Marker.ADD
            circle.pose = pose.pose
            circle.scale.x = circle.scale.y = circle.scale.z = 0.12
            circle.color.r, circle.color.g, circle.color.b, circle.color.a = 1.0, 0.1, 0.1, 1.0
            array.markers.append(circle)

            label = Marker()
            label.header = pose.header
            label.ns = "r680_waypoints"
            label.id = index * 3 + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = pose.pose
            label.pose.position.z += 0.22
            label.scale.z = 0.10
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"wp_{index + 1}"
            array.markers.append(label)
        self._marker_pub.publish(array)
        self._last_marker_count = len(self._queue.route)

    def cancel(self) -> None:
        self._stopping = True
        self._queue.clear()
        self._waiting_pose = None
        if self._active_handle is not None:
            try:
                self._active_handle.cancel_goal_async()
            except Exception:  # pragma: no cover - middleware shutdown race
                pass
        self._publish_markers()

    def spin(self) -> int:
        try:
            self._rclpy.spin(self._node)
        except KeyboardInterrupt:
            self.cancel()
            return 130
        except Exception as exc:  # rclpy raises ExternalShutdownException on SIGTERM
            if exc.__class__.__name__ == "ExternalShutdownException":
                self.cancel()
                return 130
            raise
        finally:
            self.cancel()
            self._node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
        return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.server_timeout < 0:
        parser.error("--server-timeout must be non-negative")
    if not args.frame_id.strip():
        parser.error("--frame-id must not be empty")
    try:
        import rclpy
    except ImportError as exc:
        print(f"RVIZ MULTI-GOAL FAIL: ROS 2 dependencies unavailable: {exc}", file=sys.stderr)
        return 2
    rclpy.init(args=[])
    node = RvizMultiGoalNode(args)
    return node.spin()


if __name__ == "__main__":
    raise SystemExit(main())
