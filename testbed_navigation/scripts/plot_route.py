#!/usr/bin/env python3
"""
Draw a route recorded by navigation_test.py on the map.

The path is coloured by AMCL's localization error at each point; goals are
numbered in the order they were sent.

  ros2 run testbed_navigation plot_route.py route.csv out.png -- -6,5 2,-3 ...
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402


def load_pgm(path):
    with open(path, 'rb') as f:
        f.readline()
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        return np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)


def main():
    default_map = os.path.join(get_package_share_directory('testbed_bringup'),
                               'maps', 'testbed_world.yaml')
    parser = argparse.ArgumentParser()
    parser.add_argument('csv')
    parser.add_argument('png')
    parser.add_argument('goals', nargs='*', help='goals as x,y, in the order sent')
    parser.add_argument('--map', default=default_map)
    parser.add_argument('--title', default='Navigation tour')
    args = parser.parse_intermixed_args()

    meta = yaml.safe_load(open(args.map))
    img = load_pgm(os.path.join(os.path.dirname(args.map), meta['image']))
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    h, w = img.shape
    rows = list(csv.DictReader(open(args.csv)))
    x = np.array([float(r['x']) for r in rows])
    y = np.array([float(r['y']) for r in rows])
    err = np.array([float(r['localization_error']) for r in rows])
    err = np.nan_to_num(err, nan=0.0)

    fig, ax = plt.subplots(figsize=(7.5, 7), dpi=130)
    ax.imshow(img, cmap='gray', extent=[ox, ox + w * res, oy, oy + h * res], vmin=0, vmax=255)
    pts = ax.scatter(x, y, c=err * 100, cmap='viridis', s=6, vmin=0, vmax=max(30, err.max() * 100))
    fig.colorbar(pts, ax=ax, shrink=0.8, label='AMCL localization error (cm)')
    ax.plot(x[0], y[0], marker='s', color='#d0571b', markersize=9)
    ax.annotate('start', (x[0], y[0]), (x[0] + 0.3, y[0] + 0.35), color='#d0571b', fontsize=9)
    for i, g in enumerate(args.goals, 1):
        gx, gy = map(float, g.split(','))
        ax.plot(gx, gy, marker='*', color='#e03030', markersize=13, markeredgecolor='white')
        ax.annotate(str(i), (gx, gy), (gx + 0.25, gy - 0.6), color='#e03030',
                    fontsize=11, fontweight='bold')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(args.title, fontsize=11)
    ax.set_xticks(range(-10, 11, 2))
    ax.set_yticks(range(-10, 11, 2))
    ax.grid(color='#999', linewidth=0.3, alpha=0.5)
    fig.tight_layout()
    fig.savefig(args.png)
    print('wrote', args.png)


if __name__ == '__main__':
    main()
