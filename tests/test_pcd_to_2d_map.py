import tempfile
import unittest
from pathlib import Path
import sys


# ``unittest discover -s <tests-dir>`` places the repository root (rather
# than this test's parent directory) on ``sys.path``.  Resolve the module
# relative to the test file so the suite behaves identically from either the
# repository root or the tests directory.
MODULE_DIR = Path(__file__).resolve().parents[1]
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from pcd_to_2d_map import (
    GridConfig,
    build_grid_from_points,
    iter_pcd_ascii_points,
    write_ros_map,
)


def write_ascii_pcd(path: Path, rows):
    lines = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z intensity",
        "SIZE 4 4 4 4",
        "TYPE F F F F",
        "COUNT 1 1 1 1",
        f"WIDTH {len(rows)}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(rows)}",
        "DATA ascii",
    ]
    lines.extend(f"{x} {y} {z} {i}" for x, y, z, i in rows)
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


class PcdTo2dMapTests(unittest.TestCase):
    def test_reads_ascii_pcd_xyz_intensity_points(self):
        with tempfile.TemporaryDirectory() as tmp:
            pcd = Path(tmp) / "sample.pcd"
            write_ascii_pcd(pcd, [(1.0, 2.0, 0.1, 42.0), (3.0, 4.0, 0.2, 7.0)])

            points = list(iter_pcd_ascii_points(pcd))

        self.assertEqual(points, [(1.0, 2.0, 0.1, 42.0), (3.0, 4.0, 0.2, 7.0)])

    def test_marks_high_points_as_occupied_and_ground_as_free(self):
        points = [
            (0.0, 0.0, 0.0, 10.0),
            (1.0, 0.0, 0.8, 10.0),
        ]
        grid = build_grid_from_points(
            points,
            GridConfig(resolution=1.0, padding=0.0, ground_z=0.0, obstacle_min_height=0.3),
        )

        self.assertEqual(grid.width, 2)
        self.assertEqual(grid.height, 1)
        self.assertEqual(grid.cells, [[254, 0]])
        self.assertEqual(grid.origin, (0.0, 0.0, 0.0))

    def test_pgm_is_written_top_row_first_with_ros_yaml(self):
        points = [
            (0.0, 0.0, 0.0, 10.0),
            (0.0, 1.0, 0.8, 10.0),
        ]
        grid = build_grid_from_points(
            points,
            GridConfig(resolution=1.0, padding=0.0, ground_z=0.0, obstacle_min_height=0.3),
        )

        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "map"
            pgm_path, yaml_path = write_ros_map(grid, out)
            pgm = pgm_path.read_bytes()
            yaml_text = yaml_path.read_text(encoding="utf-8")

        header, payload = pgm.split(b"255\n", 1)
        self.assertIn(b"1 2\n", header)
        self.assertEqual(payload, bytes([0, 254]))
        self.assertIn("image: map.pgm", yaml_text)
        self.assertIn("resolution: 1.0", yaml_text)
        self.assertIn("origin: [0.0, 0.0, 0.0]", yaml_text)


if __name__ == "__main__":
    unittest.main()
