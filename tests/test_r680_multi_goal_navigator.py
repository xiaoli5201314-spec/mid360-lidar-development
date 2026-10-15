"""Offline contracts for the multi-point Nav2 action client."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "deployment" / "r680_multi_goal_navigator.py"
WRAPPER = PROJECT / "deployment" / "r680_multi_goal_navigator.sh"
MANIFEST = PROJECT / "deployment" / "MANIFEST.md"
EXAMPLE_ROUTE = PROJECT / "deployment" / "r680_patrol_route.example.yaml"


def load_module():
    spec = importlib.util.spec_from_file_location("r680_multi_goal_navigator", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class MultiGoalNavigatorContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_module()

    def test_parse_point_tokens_converts_xy_yaw(self):
        points = self.mod.parse_point_tokens(["1.0", "-2.5", "90deg"])
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0].x, 1.0)
        self.assertAlmostEqual(points[0].y, -2.5)
        self.assertAlmostEqual(points[0].yaw, 1.57079632679, places=8)

    def test_expand_route_repeats_the_same_order_for_requested_loops(self):
        points = self.mod.RoutePoint(0.0, 0.0, 0.0), self.mod.RoutePoint(1.0, 0.0, 0.0)
        expanded = self.mod.expand_route(points, loops=3)
        self.assertEqual([(p.x, p.y) for p in expanded], [(0.0, 0.0), (1.0, 0.0)] * 3)

    def test_expand_route_supports_return_loop_without_reverse_goal_order(self):
        points = self.mod.RoutePoint(0.0, 0.0, 0.0), self.mod.RoutePoint(1.0, 0.0, 0.0)
        expanded = self.mod.expand_route(points, loops=2, return_to_start=True)
        self.assertEqual(
            [(p.x, p.y) for p in expanded],
            [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
        )

    def test_rejects_zero_or_negative_loops(self):
        with self.assertRaises(ValueError):
            self.mod.expand_route((self.mod.RoutePoint(0.0, 0.0, 0.0),), loops=0)

    def test_rejects_unbounded_route_expansion(self):
        with self.assertRaises(ValueError):
            self.mod.expand_route(
                (
                    self.mod.RoutePoint(0.0, 0.0, 0.0),
                    self.mod.RoutePoint(1.0, 0.0, 0.0),
                ),
                loops=129,
            )

    def test_route_loader_accepts_yaml_like_json_without_ros(self):
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "route.json"
            path.write_text(
                '{"frame_id":"map","points":[{"x":1,"y":2,"yaw_deg":180},[3,4,0.5]]}',
                encoding="utf-8",
            )
            frame_id, points = self.mod.load_route(path)
        self.assertEqual(frame_id, "map")
        self.assertEqual([(p.x, p.y) for p in points], [(1.0, 2.0), (3.0, 4.0)])
        self.assertAlmostEqual(points[0].yaw, 3.14159265359, places=8)

    def test_fallback_yaml_loader_preserves_yaw_deg(self):
        document = self.mod._simple_yaml_load(
            "frame_id: map\npoints:\n  - x: 1\n    y: 2\n    yaw_deg: 180\n"
        )
        self.assertEqual(document["frame_id"], "map")
        point = self.mod.point_from_value(document["points"][0])
        self.assertAlmostEqual(point.yaw, 3.14159265359, places=8)

    def test_module_never_contains_cmd_vel_publisher(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("create_publisher", text)
        self.assertNotIn("/cmd_vel", text)

    def test_wrapper_sources_humble_and_executes_action_client(self):
        text = WRAPPER.read_text(encoding="utf-8")
        self.assertIn("source /opt/ros/humble/setup.bash", text)
        self.assertIn("r680_multi_goal_navigator.py", text)
        self.assertIn('exec python3 -u', text)

    def test_release_manifest_mentions_multi_goal_entrypoints(self):
        text = MANIFEST.read_text(encoding="utf-8")
        self.assertIn("r680_multi_goal_navigator.py", text)
        self.assertIn("r680_multi_goal_navigator.sh", text)

    def test_example_route_is_parseable_and_has_multiple_points(self):
        frame_id, points = self.mod.load_route(EXAMPLE_ROUTE)
        self.assertEqual(frame_id, "map")
        self.assertGreaterEqual(len(points), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
