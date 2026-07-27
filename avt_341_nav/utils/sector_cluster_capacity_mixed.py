#!/usr/bin/env python3
"""
Obstacle-processor worst-case cluster capacity tool (mixed full/half-row pattern).

This script analyses a two-row repeating occupancy pattern that produces more
clusters than either a fully-occupied or a pure-checkerboard sector:

    Even rows  (yi % 2 == 0): every cell is occupied.
    Odd  rows  (yi % 2 == 1): every other cell is occupied (half-row).

Why this is worse than a pure checkerboard
------------------------------------------
Consider a fully-occupied cell at (xi, yi_even).  The clustering algorithm
calls largest_square(), which at sz=1 checks three corners:

    obs[xi    ][yi_even + 1]  -- half-row, column xi     (may be occupied)
    obs[xi + 1][yi_even    ]  -- full row,  column xi+1  (always occupied)
    obs[xi + 1][yi_even + 1]  -- half-row, column xi+1  (always EMPTY,
                                   opposite parity to xi)

The third corner is always empty, so largest_square() returns sz=1 for every
cell in the full row.  Cells in the half-row are isolated for the same reason.
Every occupied cell therefore becomes its own 1x1 cluster.

Cluster count comparison per row-pair of width W cells
    Pure checkerboard:  W/2  +  W/2  =   W   clusters  (W occupied cells)
    Mixed full/half:      W  +  W/2  = 3W/2  clusters  (3W/2 occupied cells)

The mixed pattern gives 3/2 x more clusters than the checkerboard, at the
cost of 3/2 x more occupied cells.  From the perspective of max_obstacle_number
dimensioning, the mixed pattern is the true worst case.

The script searches over all fractional grid alignments in [0, cell_size)^2
and both half-row parities (even or odd xi in the sparse rows), then reports
and plots the configuration that maximises the cluster count.

Usage
-----
    python3 sector_cluster_capacity_mixed.py <cell_size_m> <range_m> <fov_half_deg>

Arguments
---------
  cell_size_m   Grid cell size [m].
  range_m       Observation range [m].
  fov_half_deg  Half-angle of the field of view [deg].
                The complete FOV spans from -fov_half_deg to +fov_half_deg.

Example
-------
    python3 sector_cluster_capacity_mixed.py 1.0 20.0 90
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
# Core: mixed pattern fill + cluster, single alignment + parity
# ---------------------------------------------------------------------------

def _build_mixed_and_cluster(cell_size, range_m,
                              lx, ly, rx, ry,
                              offset_x, offset_y, half_row_parity):
    """
    Fill the sector with the mixed pattern and cluster it.

    Pattern (yi relative to the grid, i.e. the obs-array index):
      Even rows (yi % 2 == 0): all xi occupied.
      Odd  rows (yi % 2 == 1): xi occupied iff xi % 2 == half_row_parity.

    Returns (full_row_cells, half_row_cells, clusters, origin_x, origin_y, n).
    """
    half = range_m + cell_size
    origin_x = -half + offset_x
    origin_y = -half + offset_y
    n = int(math.ceil(2.0 * half / cell_size))

    obs = [[False] * n for _ in range(n)]
    full_cells = []   # cells from fully-occupied rows
    half_cells = []   # cells from half-occupied rows

    for xi in range(n):
        for yi in range(n):
            cx = (xi + 0.5) * cell_size + origin_x
            cy = (yi + 0.5) * cell_size + origin_y
            if not _in_sector(cx, cy, range_m, lx, ly, rx, ry):
                continue
            if yi % 2 == 0:
                # Full row
                obs[xi][yi] = True
                full_cells.append((cx, cy))
            else:
                # Half row: every other xi
                if xi % 2 == half_row_parity:
                    obs[xi][yi] = True
                    half_cells.append((cx, cy))

    obs_copy = [col[:] for col in obs]
    clusters = _cluster(obs_copy, cell_size, origin_x, origin_y)
    return full_cells, half_cells, clusters, origin_x, origin_y, n


# ---------------------------------------------------------------------------
# Search over alignments and parities
# ---------------------------------------------------------------------------

def max_mixed_clusters(cell_size, range_m, fov_half_deg, n_offsets=20):
    """
    Return (max_count, full_cells, half_cells, clusters, origin_x, origin_y, n)
    for the alignment and half-row parity that maximise the cluster count.
    """
    theta = math.radians(fov_half_deg)
    lx, ly = math.cos(theta),  math.sin(theta)
    rx, ry = math.cos(theta), -math.sin(theta)

    offsets = np.linspace(0.0, cell_size, n_offsets, endpoint=False)
    best = (0, [], [], [], 0.0, 0.0, 0)

    for ox in offsets:
        for oy in offsets:
            for parity in (0, 1):
                fc, hc, clusters, ox0, oy0, n = _build_mixed_and_cluster(
                    cell_size, range_m, lx, ly, rx, ry, ox, oy, parity)
                if len(clusters) > best[0]:
                    best = (len(clusters), fc, hc, clusters, ox0, oy0, n)

    return best


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def plot_result(cell_size, range_m, fov_half_deg,
                full_cells, half_cells, clusters,
                origin_x, origin_y, n):
    theta = math.radians(fov_half_deg)
    half = range_m + cell_size

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect('equal')

    # Grid lines
    for k in range(n + 2):
        ax.axvline(origin_x + k * cell_size, color='#cccccc', linewidth=0.4, zorder=1)
        ax.axhline(origin_y + k * cell_size, color='#cccccc', linewidth=0.4, zorder=1)

    # Light shade: full-row cells (slightly darker tint to distinguish row type)
    for (cx, cy) in full_cells:
        ax.add_patch(plt.Rectangle(
            (cx - 0.5 * cell_size, cy - 0.5 * cell_size),
            cell_size, cell_size,
            facecolor='steelblue', alpha=0.30, edgecolor='none', zorder=2))

    # Light shade: half-row cells (lighter tint)
    for (cx, cy) in half_cells:
        ax.add_patch(plt.Rectangle(
            (cx - 0.5 * cell_size, cy - 0.5 * cell_size),
            cell_size, cell_size,
            facecolor='steelblue', alpha=0.15, edgecolor='none', zorder=2))

    # Darker shade: clusters (inset so both layers remain visible)
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

    n_occ = len(full_cells) + len(half_cells)
    ax.set_xlim(-half * 0.3, half)
    ax.set_ylim(-half, half)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(
        f'Obstacle processor worst-case cluster capacity  (full + half-row pattern)\n'
        f'cell size = {cell_size} m   range = {range_m} m   '
        f'FOV = \u00b1{fov_half_deg}\u00b0\n'
        f'Occupied cells: {n_occ}  '
        f'(full rows: {len(full_cells)}, half rows: {len(half_cells)})     '
        f'Maximum clusters: {len(clusters)}',
        fontsize=10)
    ax.legend(handles=[
        mpatches.Patch(facecolor='steelblue', alpha=0.30,
                       label='Occupied cells \u2013 full rows'),
        mpatches.Patch(facecolor='steelblue', alpha=0.15,
                       label='Occupied cells \u2013 half rows'),
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
        description='Worst-case cluster count using a full-row / half-row '
                    'alternating occupancy pattern.')
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

    result = max_mixed_clusters(args.cell_size_m, args.range_m, args.fov_half_deg)
    n_clusters, full_cells, half_cells, clusters, ox, oy, n = result

    n_occ = len(full_cells) + len(half_cells)
    print(f'Occupied cells (full rows):  {len(full_cells)}')
    print(f'Occupied cells (half rows):  {len(half_cells)}')
    print(f'Total occupied cells:        {n_occ}')
    print(f'Maximum clusters:            {n_clusters}')

    non_unit = [c for c in clusters if abs(c[2] - args.cell_size_m) > 1e-9]
    if non_unit:
        print(f'WARNING: {len(non_unit)} cluster(s) larger than 1 cell '
              f'(unexpected for this pattern)')

    plot_result(args.cell_size_m, args.range_m, args.fov_half_deg,
                full_cells, half_cells, clusters, ox, oy, n)


if __name__ == '__main__':
    main()
