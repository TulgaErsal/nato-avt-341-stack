#!/usr/bin/env python3
"""
Obstacle-processor worst-case cluster capacity tool (checkerboard pattern).

The maximum number of obstacle clusters produced by the greedy square
clustering algorithm in obstacles_processor_node.cpp is NOT achieved when the
sector is fully occupied.  When the sector is fully occupied the greedy
algorithm merges adjacent cells into large squares, reducing the cluster count.

The true worst case is a checkerboard occupancy pattern, where no two occupied
cells are adjacent (in x or y).  In that case largest_square() immediately
returns sz=1 for every occupied cell because the three far-corner cells it
checks are always empty.  Every occupied cell becomes its own 1x1 cluster, so
the cluster count equals the number of occupied cells -- the maximum possible
ratio of clusters to cells.

This script:
  1. Fills the sector with a checkerboard pattern (both parities are tried).
  2. Applies the same clustering algorithm used at runtime.
  3. Searches over all fractional grid alignments in [0, cell_size)^2.
  4. Reports and plots the alignment + parity that maximises the cluster count.

Sector inclusion logic and clustering algorithm are identical to those in
obstacles_processor_node.cpp (see sector_cluster_capacity.py for details).

Usage
-----
    python3 sector_cluster_capacity_checkerboard.py <cell_size_m> <range_m> <fov_half_deg>

Arguments
---------
  cell_size_m   Grid cell size [m].
  range_m       Observation range [m].
  fov_half_deg  Half-angle of the field of view [deg].
                The complete FOV spans from -fov_half_deg to +fov_half_deg.

Example
-------
    python3 sector_cluster_capacity_checkerboard.py 1.0 20.0 90
"""

import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


# ---------------------------------------------------------------------------
# Sector inclusion  (same as the other sector_* scripts)
# ---------------------------------------------------------------------------

def _in_sector(cx, cy, range_m, lx, ly, rx, ry):
    if math.hypot(cx, cy) > range_m:
        return False
    if cx * ly - lx * cy < 0:
        return False
    if cx * ry - rx * cy > 0:
        return False
    return True


# ---------------------------------------------------------------------------
# Clustering  (matches cluster_occupied_cells() / largest_square() in C++)
# ---------------------------------------------------------------------------

def _is_occ(obs, xi, yi):
    if xi < 0 or yi < 0 or xi >= len(obs) or yi >= len(obs[0]):
        return False
    return obs[xi][yi]


def _largest_square(obs, xi, yi):
    sz = 0
    while True:
        if (not _is_occ(obs, xi,      yi + sz) or
                not _is_occ(obs, xi + sz, yi     ) or
                not _is_occ(obs, xi + sz, yi + sz)):
            return sz
        sz += 1


def _cluster(obs_copy, cell_size, origin_x, origin_y):
    clusters = []
    for xi in range(len(obs_copy)):
        for yi in range(len(obs_copy[0])):
            if not obs_copy[xi][yi]:
                continue
            sz = _largest_square(obs_copy, xi, yi)
            sz_m = sz * cell_size
            cx = xi * cell_size + origin_x + sz_m / 2.0
            cy = yi * cell_size + origin_y + sz_m / 2.0
            clusters.append((cx, cy, sz_m))
            for i in range(xi, min(len(obs_copy),      xi + sz)):
                for j in range(yi, min(len(obs_copy[0]), yi + sz)):
                    obs_copy[i][j] = False
    return clusters


# ---------------------------------------------------------------------------
# Core: checkerboard fill + cluster, single alignment + parity
# ---------------------------------------------------------------------------

def _build_checkerboard_and_cluster(cell_size, range_m,
                                    lx, ly, rx, ry,
                                    offset_x, offset_y, parity):
    """
    Fill the sector with the checkerboard parity ((xi+yi) % 2 == parity),
    run clustering, and return results.
    """
    half = range_m + cell_size
    origin_x = -half + offset_x
    origin_y = -half + offset_y
    n = int(math.ceil(2.0 * half / cell_size))

    obs = [[False] * n for _ in range(n)]
    cells = []
    for xi in range(n):
        for yi in range(n):
            cx = (xi + 0.5) * cell_size + origin_x
            cy = (yi + 0.5) * cell_size + origin_y
            if not _in_sector(cx, cy, range_m, lx, ly, rx, ry):
                continue
            if (xi + yi) % 2 != parity:
                continue
            obs[xi][yi] = True
            cells.append((cx, cy))

    obs_copy = [col[:] for col in obs]
    clusters = _cluster(obs_copy, cell_size, origin_x, origin_y)
    return cells, clusters, origin_x, origin_y, n


# ---------------------------------------------------------------------------
# Search over alignments and parities
# ---------------------------------------------------------------------------

def max_checkerboard_clusters(cell_size, range_m, fov_half_deg, n_offsets=20):
    """
    Return (max_count, cells, clusters, origin_x, origin_y, n) for the
    alignment and parity that maximises the cluster count.
    """
    theta = math.radians(fov_half_deg)
    lx, ly = math.cos(theta),  math.sin(theta)
    rx, ry = math.cos(theta), -math.sin(theta)

    offsets = np.linspace(0.0, cell_size, n_offsets, endpoint=False)
    best = (0, [], [], 0.0, 0.0, 0)

    for ox in offsets:
        for oy in offsets:
            for parity in (0, 1):
                cells, clusters, ox0, oy0, n = _build_checkerboard_and_cluster(
                    cell_size, range_m, lx, ly, rx, ry, ox, oy, parity)
                if len(clusters) > best[0]:
                    best = (len(clusters), cells, clusters, ox0, oy0, n)

    return best


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def plot_result(cell_size, range_m, fov_half_deg,
                cells, clusters, origin_x, origin_y, n):
    theta = math.radians(fov_half_deg)
    half = range_m + cell_size

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect('equal')

    # Grid lines
    for k in range(n + 2):
        ax.axvline(origin_x + k * cell_size, color='#cccccc', linewidth=0.4, zorder=1)
        ax.axhline(origin_y + k * cell_size, color='#cccccc', linewidth=0.4, zorder=1)

    # Light shade: occupied (checkerboard) cells
    for (cx, cy) in cells:
        ax.add_patch(plt.Rectangle(
            (cx - 0.5 * cell_size, cy - 0.5 * cell_size),
            cell_size, cell_size,
            facecolor='steelblue', alpha=0.20, edgecolor='none', zorder=2))

    # Darker shade: clusters.  In the checkerboard case every cluster is
    # exactly one cell (sz_m == cell_size), so draw them slightly inset so
    # both layers are visible at the same time.
    inset = cell_size * 0.15
    for (cx, cy, sz_m) in clusters:
        side = sz_m - 2 * inset
        ax.add_patch(plt.Rectangle(
            (cx - sz_m / 2.0 + inset, cy - sz_m / 2.0 + inset),
            side, side,
            facecolor='steelblue', alpha=0.70,
            edgecolor='#1a3a6b', linewidth=0.6, zorder=3))

    # Sector boundary
    ax.plot([0, range_m * math.cos(theta)],
            [0,  range_m * math.sin(theta)],
            color='navy', linewidth=1.8, zorder=4)
    ax.plot([0, range_m * math.cos(theta)],
            [0, -range_m * math.sin(theta)],
            color='navy', linewidth=1.8, zorder=4)
    ax.add_patch(mpatches.Arc(
        (0, 0), 2 * range_m, 2 * range_m,
        angle=0, theta1=-fov_half_deg, theta2=fov_half_deg,
        color='navy', linewidth=1.8, zorder=4))

    # Vehicle
    ax.plot(0, 0, 'ro', markersize=7, zorder=5)
    arrow_len = max(cell_size * 1.5, range_m * 0.08)
    ax.annotate('', xy=(arrow_len, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle='->', color='red', lw=2.0), zorder=5)

    ax.set_xlim(-half * 0.3, half)
    ax.set_ylim(-half, half)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(
        f'Obstacle processor worst-case cluster capacity  (checkerboard)\n'
        f'cell size = {cell_size} m   range = {range_m} m   '
        f'FOV = \u00b1{fov_half_deg}\u00b0\n'
        f'Occupied cells: {len(cells)}     Maximum clusters: {len(clusters)}',
        fontsize=11)
    ax.legend(handles=[
        mpatches.Patch(facecolor='steelblue', alpha=0.20,
                       label='Occupied cells (checkerboard)'),
        mpatches.Patch(facecolor='steelblue', alpha=0.70,
                       edgecolor='#1a3a6b', label='Clusters (each = 1 cell)'),
    ], loc='upper right', fontsize=9)
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Worst-case cluster count using a checkerboard occupancy pattern.')
    parser.add_argument('cell_size_m',  type=float, help='Grid cell size [m]')
    parser.add_argument('range_m',      type=float, help='Observation range [m]')
    parser.add_argument('fov_half_deg', type=float,
                        help='Half-angle of the field of view [deg] '
                             '(full FOV spans -fov_half_deg to +fov_half_deg)')
    args = parser.parse_args()

    if args.cell_size_m <= 0:
        raise ValueError('cell_size_m must be positive')
    if args.range_m <= 0:
        raise ValueError('range_m must be positive')
    if not (0.0 < args.fov_half_deg <= 180.0):
        raise ValueError('fov_half_deg must be in (0, 180]')

    n_clusters, cells, clusters, ox, oy, n = max_checkerboard_clusters(
        args.cell_size_m, args.range_m, args.fov_half_deg)

    print(f'Occupied cells (checkerboard): {len(cells)}')
    print(f'Maximum clusters:              {n_clusters}')

    # Sanity check: in a pure checkerboard every cluster must be 1x1
    non_unit = [c for c in clusters if abs(c[2] - args.cell_size_m) > 1e-9]
    if non_unit:
        print(f'WARNING: {len(non_unit)} cluster(s) larger than 1 cell '
              f'(unexpected -- checkerboard should isolate all cells)')

    plot_result(args.cell_size_m, args.range_m, args.fov_half_deg,
                cells, clusters, ox, oy, n)


if __name__ == '__main__':
    main()
