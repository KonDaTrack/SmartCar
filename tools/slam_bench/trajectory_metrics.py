#!/usr/bin/env python3
"""Compare TUM trajectories from the three SLAM candidates.

Usage:
  trajectory_metrics.py --out /results/compare \
      glim=/results/glim/traj_lidar.txt \
      kiss=/results/kiss/poses_tum.txt \
      kinematic=/results/kinematic/poses_tum.txt

The first pose of each trajectory is subtracted so the plot and loop error are
frame-independent. There is no ground truth: path length and loop error only
show whether a run is self-consistent.
"""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load_tum(path):
    a = np.loadtxt(path, comments='#')
    if a.ndim == 1:
        a = a[None, :]
    t = a[:, 0]
    xyz = a[:, 1:4]
    # Remove duplicate timestamps, if any, before interpolation.
    _, idx = np.unique(t, return_index=True)
    return t[idx], xyz[idx]


def metrics(t, xyz):
    xyz = xyz - xyz[0]
    step = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    dt = np.diff(t)
    speed = step / np.maximum(dt, 1e-6)
    return {
        'poses': int(len(t)),
        'duration_s': float(t[-1] - t[0]),
        'path_length_m': float(step.sum()),
        'start_xyz': xyz[0].round(3).tolist(),
        'end_xyz': xyz[-1].round(3).tolist(),
        'loop_error_m': float(np.linalg.norm(xyz[-1] - xyz[0])),
        'z_minmax': [round(float(xyz[:, 2].min()), 3),
                     round(float(xyz[:, 2].max()), 3)],
        'max_speed_mps': float(np.max(speed)) if len(speed) else 0.0,
        'p95_step_m': float(np.percentile(step, 95)) if len(step) else 0.0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('trajectories', nargs='+', help='name=path')
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    results = {}
    plt.figure(figsize=(9, 9))
    for item in args.trajectories:
        name, path = item.split('=', 1)
        t, xyz = load_tum(path)
        m = metrics(t, xyz)
        m['file'] = path
        results[name] = m
        rel = xyz - xyz[0]
        plt.plot(rel[:, 1], rel[:, 0], label=f'{name} ({m["path_length_m"]:.1f} m)',
                 linewidth=1.5)
        plt.scatter([rel[0, 1]], [rel[0, 0]], marker='o', s=40)

    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.xlabel('y [m]')
    plt.ylabel('x [m]')
    plt.title('SLAM trajectories (start-normalized, top view)')
    plt.legend()
    plt.tight_layout()
    plt.savefig(out / 'trajectories_xy.png', dpi=150)

    (out / 'metrics.json').write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))
    print('wrote', out / 'trajectories_xy.png')


if __name__ == '__main__':
    main()
