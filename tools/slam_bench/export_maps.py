#!/usr/bin/env python3
"""Export PCD maps from the KISS-SLAM and GLIM benchmark outputs.

KISS-SLAM writes local-map PLYs. GLIM writes each submap as raw
points_compact.bin (N x float32 xyz) plus a data.txt containing
T_world_origin. The script merges the pieces, voxel-downsamples, and writes
PCD files.

Usage:
  export_maps.py \
    --kiss-dir /results/full/kiss/slam_output/<timestamp> \
    --kiss-out /results/full/kiss/kiss_map.pcd \
    --glim-dump /results/full/glim/dump \
    --glim-out /results/full/glim/glim_map.pcd
"""

import argparse
import re
from pathlib import Path

import numpy as np
import open3d as o3d


def read_t_world_origin(path):
    lines = path.read_text().splitlines()
    for i, line in enumerate(lines):
        if line.strip().startswith('T_world_origin'):
            rows = []
            for row in lines[i + 1:i + 5]:
                vals = row.split()
                if len(vals) == 4:
                    rows.append([float(x) for x in vals])
            if len(rows) == 4:
                return np.asarray(rows)
    raise ValueError(f'no T_world_origin in {path}')


def export_kiss(directory, out, voxel):
    plys = sorted(Path(directory).glob('local_maps/plys/*.ply'))
    if not plys:
        raise SystemExit(f'no PLYs under {directory}')
    merged = o3d.geometry.PointCloud()
    for p in plys:
        pc = o3d.io.read_point_cloud(str(p))
        print(f'  {p.name}: {len(pc.points)} points')
        merged += pc
    if voxel > 0:
        merged = merged.voxel_down_sample(voxel)
    o3d.io.write_point_cloud(str(out), merged, write_ascii=False)
    print(f'KISS map: {len(merged.points)} points -> {out}')


def export_glim(dump, out, voxel):
    points = []
    for sub in sorted(Path(dump).glob('[0-9][0-9][0-9][0-9][0-9][0-9]')):
        bin_path = sub / 'points_compact.bin'
        txt_path = sub / 'data.txt'
        if not bin_path.exists() or not txt_path.exists():
            continue
        n = bin_path.stat().st_size // 12
        if n == 0:
            continue
        pts = np.fromfile(bin_path, dtype='<f4').reshape(-1, 3)
        T = read_t_world_origin(txt_path)
        pts = pts @ T[:3, :3].T + T[:3, 3]
        points.append(pts)
        print(f'  {sub.name}: {len(pts)} points')
    if not points:
        raise SystemExit(f'no GLIM submaps under {dump}')
    all_pts = np.vstack(points)
    print(f'GLIM raw: {len(all_pts)} points')
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(all_pts.astype(np.float64))
    if voxel > 0:
        pc = pc.voxel_down_sample(voxel)
    o3d.io.write_point_cloud(str(out), pc, write_ascii=False)
    print(f'GLIM map: {len(pc.points)} points -> {out}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kiss-dir')
    ap.add_argument('--kiss-out')
    ap.add_argument('--glim-dump')
    ap.add_argument('--glim-out')
    ap.add_argument('--voxel', type=float, default=0.0,
                    help='voxel size in meters; 0 keeps the raw merged cloud')
    args = ap.parse_args()

    if args.kiss_dir and args.kiss_out:
        print('KISS-SLAM:')
        export_kiss(args.kiss_dir, args.kiss_out, args.voxel)
    if args.glim_dump and args.glim_out:
        print('GLIM:')
        export_glim(args.glim_dump, args.glim_out, args.voxel)


if __name__ == '__main__':
    main()
