#!/usr/bin/env python3
import argparse
import struct
from pathlib import Path

import numpy as np


def load_ascii_pcd_xyz(path: Path) -> np.ndarray:
    with path.open("r", encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if line.strip().lower() == "data ascii":
                break
        data = np.loadtxt(handle, dtype=np.float32, usecols=(0, 1, 2))
    if data.ndim == 1:
        data = data.reshape(1, 3)
    return data


def write_bmp(path: Path, image: np.ndarray) -> None:
    # image is uint8 RGB, top-left origin.
    height, width, _ = image.shape
    row_size = ((width * 3 + 3) // 4) * 4
    pixel_data_size = row_size * height
    file_size = 14 + 40 + pixel_data_size

    with path.open("wb") as handle:
        handle.write(b"BM")
        handle.write(struct.pack("<IHHI", file_size, 0, 0, 14 + 40))
        handle.write(
            struct.pack(
                "<IIIHHIIIIII",
                40,
                width,
                height,
                1,
                24,
                0,
                pixel_data_size,
                2835,
                2835,
                0,
                0,
            )
        )
        padding = b"\x00" * (row_size - width * 3)
        # BMP stores BGR rows from bottom to top.
        for row in image[::-1]:
            bgr = row[:, ::-1]
            handle.write(bgr.tobytes())
            handle.write(padding)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd", type=Path)
    parser.add_argument("bmp", type=Path)
    parser.add_argument("--size", type=int, default=1800)
    parser.add_argument("--point-size", type=int, default=1)
    args = parser.parse_args()

    xyz = load_ascii_pcd_xyz(args.pcd)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if xyz.size == 0:
        raise RuntimeError("No valid points in PCD")

    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]
    margin = 0.05
    min_x, max_x = float(x.min()), float(x.max())
    min_y, max_y = float(y.min()), float(y.max())
    span_x = max(max_x - min_x, 1e-3)
    span_y = max(max_y - min_y, 1e-3)
    span = max(span_x, span_y)
    cx = (min_x + max_x) / 2
    cy = (min_y + max_y) / 2
    min_x = cx - span / 2 - span * margin
    max_x = cx + span / 2 + span * margin
    min_y = cy - span / 2 - span * margin
    max_y = cy + span / 2 + span * margin

    width = height = args.size
    px = ((x - min_x) / (max_x - min_x) * (width - 1)).astype(np.int32)
    py = ((max_y - y) / (max_y - min_y) * (height - 1)).astype(np.int32)
    mask = (px >= 0) & (px < width) & (py >= 0) & (py < height)
    px = px[mask]
    py = py[mask]
    z = z[mask]

    image = np.full((height, width, 3), 18, dtype=np.uint8)

    # Draw a light grid.
    for i in range(0, width, max(width // 20, 1)):
        image[:, i] = (45, 45, 45)
        image[i, :] = (45, 45, 45)

    # Color by height, blue/cyan/green/yellow/red.
    z_low, z_high = np.percentile(z, [2, 98])
    if z_high <= z_low:
        z_high = z_low + 1.0
    t = np.clip((z - z_low) / (z_high - z_low), 0, 1)
    colors = np.empty((t.size, 3), dtype=np.uint8)
    colors[:, 0] = np.clip(255 * np.maximum(0, 2 * t - 0.5), 0, 255)
    colors[:, 1] = np.clip(255 * np.minimum(1, 2 - 2 * np.abs(t - 0.5)), 0, 255)
    colors[:, 2] = np.clip(255 * np.maximum(0, 1.2 - 2 * t), 0, 255)

    point_size = max(1, args.point_size)
    for dx in range(point_size):
        for dy in range(point_size):
            xx = np.clip(px + dx, 0, width - 1)
            yy = np.clip(py + dy, 0, height - 1)
            image[yy, xx] = colors

    write_bmp(args.bmp, image)
    print(f"points={xyz.shape[0]}")
    print(f"bounds_x={min_x:.3f},{max_x:.3f}")
    print(f"bounds_y={min_y:.3f},{max_y:.3f}")
    print(f"preview={args.bmp}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
