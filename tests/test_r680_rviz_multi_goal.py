"""Offline contracts for the RViz-compatible multi-goal queue."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "deployment" / "r680_rviz_multi_goal.py"
WRAPPER = PROJECT / "deployment" / "r680_rviz_multi_goal.sh"
MANIFEST = PROJECT / "deployment" / "MANIFEST.md"


def load_module():
    spec = importlib.util.spec_from_file_location("r680_rviz_multi_goal", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class RvizMultiGoalContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_module()

    def test_first_click_starts_and_later_clicks_are_pending(self):
        queue = self.mod.WaypointQueue(loop=True)
        first, second = object(), object()
        self.assertIs(queue.add(first), first)
        self.assertIsNone(queue.add(second, active=True))
        self.assertIs(queue.succeeded(), second)

    def test_completed_route_returns_to_first_and_repeats(self):
        queue = self.mod.WaypointQueue(loop=True)
        points = [object(), object(), object()]
        self.assertIs(queue.add(points[0]), points[0])
        for point in points[1:]:
            queue.add(point, active=True)
        self.assertIs(queue.succeeded(), points[1])
        self.assertIs(queue.succeeded(), points[2])
        self.assertIs(queue.succeeded(), points[0])

    def test_once_route_stops_after_final_point(self):
        queue = self.mod.WaypointQueue(loop=False)
        first, second = object(), object()
        self.assertIs(queue.add(first), first)
        queue.add(second, active=True)
        self.assertIs(queue.succeeded(), second)
        self.assertIsNone(queue.succeeded())

    def test_failure_clears_route(self):
        queue = self.mod.WaypointQueue(loop=True)
        queue.add(object())
        queue.add(object(), active=True)
        queue.failed()
        self.assertFalse(queue.route)
        self.assertFalse(queue.pending)

    def test_contract_does_not_own_velocity_or_serial(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("/cmd_vel", text)
        self.assertNotIn("ttyACM0", text)
        self.assertIn("NavigateToPose", text)
        self.assertIn("/r680_multi_goal_pose", text)

    def test_wrapper_and_manifest_are_deployable(self):
        wrapper = WRAPPER.read_text(encoding="utf-8")
        manifest = MANIFEST.read_text(encoding="utf-8")
        self.assertIn("source /opt/ros/humble/setup.bash", wrapper)
        self.assertIn("r680_rviz_multi_goal.py", wrapper)
        self.assertIn("r680_rviz_multi_goal.py", manifest)


if __name__ == "__main__":
    unittest.main(verbosity=2)
