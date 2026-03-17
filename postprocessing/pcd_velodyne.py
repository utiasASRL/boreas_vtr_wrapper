#!/usr/bin/env python3
"""
Visualize a Velodyne-style .bin point cloud with fields [x, y, z, i, d, t]
Usage:
  python view_bin_o3d.py /path/to/frame.bin
Optional:
  python view_bin_o3d.py /path/to/frame.bin --color intensity
  python view_bin_o3d.py /path/to/frame.bin --color depth
"""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def load_bin_xyzidt(bin_path: str, fields: int = 6, dtype=np.float32) -> np.ndarray:
    """
    Loads a binary file of float32 records with `fields` floats per point.
    Returns: (N, fields) float32 array.
    """
    bin_path = str(bin_path)
    raw = np.fromfile(bin_path, dtype=dtype)
    if raw.size % fields != 0:
        raise ValueError(
            f"File size not divisible by {fields} floats/point. "
            f"Got {raw.size} floats -> remainder {raw.size % fields}."
        )
    return raw.reshape((-1, fields))


def normalize01(x: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    x = x.astype(np.float64)
    mn, mx = np.nanmin(x), np.nanmax(x)
    return ((x - mn) / (mx - mn + eps)).astype(np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin_path", type=str, help="Path to .bin file containing [x,y,z,i,d,t] float32 records")
    ap.add_argument("--color", choices=["none", "intensity", "depth", "time"], default="intensity",
                    help="How to color points")
    ap.add_argument("--stride", type=int, default=1, help="Downsample by taking every k-th point (for speed)")
    ap.add_argument("--max_points", type=int, default=0,
                    help="If >0, randomly subsample to this many points (after stride)")
    args = ap.parse_args()

    p = Path(args.bin_path)
    if not p.exists():
        raise FileNotFoundError(p)

    pc = load_bin_xyzidt(p, fields=6, dtype=np.float32)
    if args.stride > 1:
        pc = pc[:: args.stride]

    if args.max_points and pc.shape[0] > args.max_points:
        idx = np.random.choice(pc.shape[0], size=args.max_points, replace=False)
        pc = pc[idx]

    xyz = pc[:, 0:3].astype(np.float64)
    intensity = pc[:, 3].astype(np.float64)
    depth = np.linalg.norm(xyz, axis=1)
    t = pc[:, 5].astype(np.float64)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    origin = np.array([0, 0, 0])
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0, origin=origin)
    vis.add_geometry(frame)

    if args.color == "none":
        pass
    else:
        if args.color == "intensity":
            c = normalize01(intensity)
        elif args.color == "depth":
            c = normalize01(depth)
        elif args.color == "time":
            c = normalize01(t)

        # Simple grayscale coloring (Open3D expects Nx3 in [0,1])
        colors = np.stack([c, c, c], axis=1)
        pcd.colors = o3d.utility.Vector3dVector(colors)

    # Optional: show coordinate frame
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0, 0, 0])

    print(f"Loaded {pc.shape[0]} points from {p}")
    print("Fields: x y z i d t  (d is present in file but not used unless you want it)")
    o3d.visualization.draw_geometries([pcd, axes])


if __name__ == "__main__":
    main()