#!/usr/bin/env python3
"""Convert a FAST-LIO/MID-360S PCD point cloud into a ROS-style 2D map.

The output follows the common ROS map_server convention:

* 0   = occupied / obstacle, shown as black in the PGM
* 254 = free / observed traversable, shown as white
* 205 = unknown / not inferred, shown as gray

This is an offline helper.  It intentionally keeps unknown areas unknown instead
of pretending that every empty pixel in the bounding box is safe to drive.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, List, Optional, Sequence, Tuple


Point = Tuple[float, float, float, float]

OCCUPIED = 0
FREE = 254
UNKNOWN = 205


@dataclass(frozen=True)
class GridConfig:
    resolution: float = 0.05
    padding: float = 0.5
    ground_z: Optional[float] = None
    ground_percentile: float = 5.0
    obstacle_min_height: float = 0.30
    obstacle_max_height: float = 2.20
    min_points_per_cell: int = 1
    occupied_dilation_m: float = 0.10


@dataclass(frozen=True)
class OccupancyGrid:
    cells: List[List[int]]
    resolution: float
    origin: Tuple[float, float, float]
    ground_z: float
    width: int
    height: int
    occupied_cells: int
    free_cells: int
    unknown_cells: int
    source_points: int
    used_points: int


def _parse_header(path: Path) -> Tuple[dict, int]:
    header: dict = {}
    data_start = 0
    with path.open("rb") as fh:
        while True:
            line = fh.readline()
            if not line:
                raise ValueError(f"{path} is missing a DATA line")
            data_start = fh.tell()
            text = line.decode("ascii", errors="ignore").strip()
            if not text or text.startswith("#"):
                continue
            key, _, value = text.partition(" ")
            header[key.upper()] = value.strip()
            if key.upper() == "DATA":
                break
    return header, data_start


def iter_pcd_ascii_points(path: Path | str) -> Iterator[Point]:
    """Yield (x, y, z, intensity) points from an ASCII PCD file."""
    p = Path(path)
    header, data_start = _parse_header(p)
    if header.get("DATA", "").lower() != "ascii":
        raise ValueError(f"Only DATA ascii PCD files are supported by this helper: {p}")

    fields = header.get("FIELDS", "").split()
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError(f"PCD file must contain x, y, z fields: {p}")
    x_i, y_i, z_i = fields.index("x"), fields.index("y"), fields.index("z")
    intensity_i = fields.index("intensity") if "intensity" in fields else None

    with p.open("rb") as fh:
        fh.seek(data_start)
        for raw in fh:
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) <= max(x_i, y_i, z_i):
                continue
            x = float(parts[x_i])
            y = float(parts[y_i])
            z = float(parts[z_i])
            intensity = float(parts[intensity_i]) if intensity_i is not None and len(parts) > intensity_i else 0.0
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                yield (x, y, z, intensity)


def percentile(values: Sequence[float], percent: float) -> float:
    if not values:
        raise ValueError("Cannot calculate percentile from an empty value list")
    if percent <= 0:
        return min(values)
    if percent >= 100:
        return max(values)
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * percent / 100.0))
    return ordered[idx]


def estimate_ground_z(points: Iterable[Point], ground_percentile: float) -> float:
    z_values = [p[2] for p in points]
    return percentile(z_values, ground_percentile)


def _make_empty_grid(width: int, height: int, value: int) -> List[List[int]]:
    return [[value for _ in range(width)] for _ in range(height)]


def _cell_index(x: float, y: float, min_x: float, min_y: float, resolution: float) -> Tuple[int, int]:
    return (
        int(math.floor((x - min_x) / resolution)),
        int(math.floor((y - min_y) / resolution)),
    )


def _dilate_occupied(cells: List[List[int]], radius_cells: int) -> None:
    if radius_cells <= 0:
        return
    height = len(cells)
    width = len(cells[0]) if height else 0
    occupied = [
        (x, y)
        for y, row in enumerate(cells)
        for x, value in enumerate(row)
        if value == OCCUPIED
    ]
    offsets = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                offsets.append((dx, dy))
    for x, y in occupied:
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                cells[ny][nx] = OCCUPIED


def build_grid_from_points(points: Iterable[Point], config: GridConfig) -> OccupancyGrid:
    pts = list(points)
    if not pts:
        raise ValueError("No valid points found")
    if config.resolution <= 0:
        raise ValueError("resolution must be positive")
    if config.padding < 0:
        raise ValueError("padding must be non-negative")

    ground_z = config.ground_z
    if ground_z is None:
        ground_z = estimate_ground_z(pts, config.ground_percentile)

    min_x = min(p[0] for p in pts) - config.padding
    max_x = max(p[0] for p in pts) + config.padding
    min_y = min(p[1] for p in pts) - config.padding
    max_y = max(p[1] for p in pts) + config.padding
    width = int(math.floor((max_x - min_x) / config.resolution)) + 1
    height = int(math.floor((max_y - min_y) / config.resolution)) + 1

    if width <= 0 or height <= 0:
        raise ValueError("Calculated map dimensions are invalid")

    observed_counts = [0 for _ in range(width * height)]
    obstacle_counts = [0 for _ in range(width * height)]
    used_points = 0
    min_obstacle_z = ground_z + config.obstacle_min_height
    max_obstacle_z = ground_z + config.obstacle_max_height

    for x, y, z, _intensity in pts:
        gx, gy = _cell_index(x, y, min_x, min_y, config.resolution)
        if not (0 <= gx < width and 0 <= gy < height):
            continue
        used_points += 1
        idx = gy * width + gx
        observed_counts[idx] += 1
        if min_obstacle_z <= z <= max_obstacle_z:
            obstacle_counts[idx] += 1

    cells = _make_empty_grid(width, height, UNKNOWN)
    for gy in range(height):
        row = cells[gy]
        base = gy * width
        for gx in range(width):
            idx = base + gx
            if obstacle_counts[idx] >= config.min_points_per_cell:
                row[gx] = OCCUPIED
            elif observed_counts[idx] >= config.min_points_per_cell:
                row[gx] = FREE

    _dilate_occupied(cells, int(round(config.occupied_dilation_m / config.resolution)))

    occupied_cells = sum(value == OCCUPIED for row in cells for value in row)
    free_cells = sum(value == FREE for row in cells for value in row)
    unknown_cells = width * height - occupied_cells - free_cells
    return OccupancyGrid(
        cells=cells,
        resolution=config.resolution,
        origin=(round(min_x, 6), round(min_y, 6), 0.0),
        ground_z=float(ground_z),
        width=width,
        height=height,
        occupied_cells=occupied_cells,
        free_cells=free_cells,
        unknown_cells=unknown_cells,
        source_points=len(pts),
        used_points=used_points,
    )


def write_ros_map(grid: OccupancyGrid, output_stem: Path | str) -> Tuple[Path, Path]:
    stem = Path(output_stem)
    stem.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = stem.with_suffix(".pgm")
    yaml_path = stem.with_suffix(".yaml")

    with pgm_path.open("wb") as fh:
        fh.write(f"P5\n# Created by pcd_to_2d_map.py\n{grid.width} {grid.height}\n255\n".encode("ascii"))
        for y in range(grid.height - 1, -1, -1):
            fh.write(bytes(grid.cells[y]))

    yaml_text = "\n".join(
        [
            f"image: {pgm_path.name}",
            f"resolution: {grid.resolution}",
            f"origin: [{grid.origin[0]}, {grid.origin[1]}, {grid.origin[2]}]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
            "",
        ]
    )
    yaml_path.write_text(yaml_text, encoding="utf-8")
    return pgm_path, yaml_path


def convert_pcd_to_2d_map(pcd_path: Path | str, output_stem: Path | str, config: GridConfig) -> OccupancyGrid:
    points = iter_pcd_ascii_points(pcd_path)
    grid = build_grid_from_points(points, config)
    write_ros_map(grid, output_stem)
    return grid


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert MID-360S/FAST-LIO ASCII PCD into ROS map_server .pgm + .yaml."
    )
    parser.add_argument("pcd", type=Path, help="Input ASCII .pcd file")
    parser.add_argument("-o", "--output", type=Path, required=True, help="Output path stem, without extension")
    parser.add_argument("--resolution", type=float, default=0.05, help="Map resolution in meters/cell")
    parser.add_argument("--padding", type=float, default=0.5, help="Padding around point cloud bounds in meters")
    parser.add_argument("--ground-z", type=float, default=None, help="Ground z. If omitted, use percentile estimate")
    parser.add_argument("--ground-percentile", type=float, default=5.0, help="Percentile used to estimate ground z")
    parser.add_argument("--obstacle-min-height", type=float, default=0.30, help="Obstacle height above ground in meters")
    parser.add_argument("--obstacle-max-height", type=float, default=2.20, help="Maximum obstacle height above ground")
    parser.add_argument("--min-points-per-cell", type=int, default=1, help="Minimum points for a cell to be observed")
    parser.add_argument("--occupied-dilation-m", type=float, default=0.10, help="Inflate occupied cells in meters")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    config = GridConfig(
        resolution=args.resolution,
        padding=args.padding,
        ground_z=args.ground_z,
        ground_percentile=args.ground_percentile,
        obstacle_min_height=args.obstacle_min_height,
        obstacle_max_height=args.obstacle_max_height,
        min_points_per_cell=args.min_points_per_cell,
        occupied_dilation_m=args.occupied_dilation_m,
    )
    grid = convert_pcd_to_2d_map(args.pcd, args.output, config)
    print(f"saved: {Path(args.output).with_suffix('.pgm')}")
    print(f"saved: {Path(args.output).with_suffix('.yaml')}")
    print(
        "summary: "
        f"{grid.width}x{grid.height}, resolution={grid.resolution}m, "
        f"ground_z={grid.ground_z:.3f}, occupied={grid.occupied_cells}, "
        f"free={grid.free_cells}, unknown={grid.unknown_cells}, points={grid.used_points}/{grid.source_points}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
