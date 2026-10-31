#!/usr/bin/env python3
"""Pairwise 2-D deviation between start-normalized TUM trajectories.

Usage:
  pairwise_deviation.py name=path [name=path ...]

No rigid alignment is applied: the comparison is in each trajectory's local
frame after subtracting its first pose. This highlights frame-convention
differences as well as actual trajectory errors.
"""
import sys
from pathlib import Path

import numpy as np


def load(path):
    a = np.loadtxt(path, comments='#')
    _, idx = np.unique(a[:, 0], return_index=True)
    return a[idx, 0], a[idx, 1:3]


def main():
    data = {}
    for item in sys.argv[1:]:
        name, path = item.split('=', 1)
        t, xy = load(path)
        data[name] = (t, xy - xy[0])

    names = list(data)
    t0 = max(data[n][0][0] for n in names)
    t1 = min(data[n][0][-1] for n in names)
    grid = np.arange(t0, t1, 1.0)
    interp = {
        n: np.column_stack([
            np.interp(grid, data[n][0], data[n][1][:, 0]),
            np.interp(grid, data[n][0], data[n][1][:, 1]),
        ]) for n in names
    }
    print(f'common duration: {grid[-1] - grid[0]:.1f} s, {len(grid)} samples')
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            d = np.linalg.norm(interp[a] - interp[b], axis=1)
            print(f'{a:10s} vs {b:10s}: mean {d.mean():7.2f}  '
                  f'median {np.median(d):7.2f}  p95 {np.percentile(d, 95):7.2f}  '
                  f'max {d.max():7.2f}  end {d[-1]:7.2f} m')


if __name__ == '__main__':
    main()
