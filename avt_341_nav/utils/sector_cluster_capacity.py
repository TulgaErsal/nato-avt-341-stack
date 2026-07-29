#!/usr/bin/env python3
"""
Obstacle-processor cluster capacity tool.

Extends sector_cell_capacity.py by additionally applying the greedy square
clustering algorithm from cluster_occupied_cells() in obstacles_processor_node.cpp.

A fully-occupied sector (every grid cell in the sector marked as an obstacle)
is built for each candidate grid alignment, clustered with the same algorithm
used at runtime, and the alignment that yields the MAXIMUM number of output
clusters is reported.  This gives the worst-case obstacle count that the MPC
planner would receive for the given sensor parameters.

Cluster inclusion logic (matches obstacles_processor_node.cpp):
  - A cell is in the sector if:
      1.  distance(vehicle, cell_center) <= range_m
      2.  cross(cell_vec, left_boundary).z  >= 0
      3.  cross(cell_vec, right_boundary).z <= 0
    where left_boundary  = [cos(theta),  sin(theta)]
          right_boundary = [cos(theta), -sin(theta)],  theta = fov_half_deg [rad]

Clustering logic (matches cluster_occupied_cells() / largest_square() in C++):
  - Scan the obs[xi][yi] grid left-to-right, bottom-to-top.
  - At each occupied cell, grow the largest axis-aligned square whose three
    far corners (top, right, diagonal) are all occupied.
  - Record the square as one cluster and mark all its cells as consumed.

Usage
-----
    python3 sector_cluster_capacity.py <cell_size_m> <range_m> <fov_half_deg>

Arguments
---------
  cell_size_m   Grid cell size [m].
  range_m       Observation range [m].
  fov_half_deg  Half-angle of the field of view [deg].
                The complete FOV spans from -fov_half_deg to +fov_half_deg.

Example
-------
    python3 sector_cluster_capacity.py 1.0 20.0 90
"""

import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


# ---------------------------------------------------------------------------
# Sector inclusion  (identical to sector_cell_capacity.py)
# ---------------------------------------------------------------------------

def _in_sector(cx, cy, range_m, lx, ly, rx, ry):
    if math.hypot(cx, cy) > range_m:
        return False
    if cx * ly - lx * cy < 0:    # outside left boundary
        return False
    if cx * ry - rx * cy > 0:    # outside right boundary
        return False
    return True


# ---------------------------------------------------------------------------
# Clustering  (translated from obstacles_processor_node.cpp)
# ---------------------------------------------------------------------------

def _is_occ(obs, xi, yi):
    if xi < 0 or yi < 0 or xi >= len(obs) or yi >= len(obs[0]):
        return False
    return obs[xi][yi]


def _largest_square(obs, xi, yi):
    """Greedy corner-check square growth, matching largest_square() in C++."""
    sz = 0
    while True:
        if (not _is_occ(obs, xi,      yi + sz) or
                not _is_occ(obs, xi + sz, yi     ) or
                not _is_occ(obs, xi + sz, yi + sz)):
            return sz
        sz += 1


def _cluster(obs_copy, cell_size, origin_x, origin_y):
    """
    Run cluster_occupied_cells() on a copy of the obs grid.

    Returns a list of (center_x, center_y, side_m) tuples, one per cluster.
    """
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
            # Consume all cells belonging to this cluster
            for i in range(xi, min(len(obs_copy),      xi + sz)):
                for j in range(yi, min(len(obs_copy[0]), yi + sz)):
                    obs_copy[i][j] = False
    return clusters


# ---------------------------------------------------------------------------
# Grid builder + alignment search
# ---------------------------------------------------------------------------

def _build_and_cluster(cell_size, range_m, lx, ly, rx, ry, offset_x, offset_y):
    """Build occupied grid with given fractional offset and cluster it."""
    half = range_m + cell_size
    origin_x = -half + offset_x
    origin_y = -half + offset_y
    n = int(math.ceil(2.0 * half / cell_size))

    # Build obs grid: obs[xi][yi] = True if cell center is in sector
    obs = [[False] * n for _ in range(n)]
    cells = []
    for xi in range(n):
        for yi in range(n):
            cx = (xi + 0.5) * cell_size + origin_x
            cy = (yi + 0.5) * cell_size + origin_y
            if _in_sector(cx, cy, range_m, lx, ly, rx, ry):
                obs[xi][yi] = True
                cells.append((cx, cy))

    # Deep copy before clustering (cluster modifies the grid in-place)
    obs_copy = [col[:] for col in obs]
    clusters = _cluster(obs_copy, cell_size, origin_x, origin_y)

    return cells, clusters, origin_x, origin_y, n


def max_clusters_in_sector(cell_size, range_m, fov_half_deg, n_offsets=20):
    """
    Search over fractional grid alignments and return the configuration that
    yields the maximum number of clusters from a fully-occupied sector.

    Returns (max_count, cells, clusters, origin_x, origin_y, n).
    """
    theta = math.radians(fov_half_deg)
    lx, ly = math.cos(theta), math.sin(theta)
    rx, ry = math.cos(theta), -math.sin(theta)

    offsets = np.linspace(0.0, cell_size, n_offsets, endpoint=False)
    best = (0, [], [], 0.0, 0.0, 0)

    for ox in offsets:
        for oy in offsets:
            cells, clusters, ox0, oy0, n = _build_and_cluster(
                cell_size, range_m, lx, ly, rx, ry, ox, oy)
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

    # Light shade: every occupied cell in the sector
    for (cx, cy) in cells:
        ax.add_patch(plt.Rectangle(
            (cx - 0.5 * cell_size, cy - 0.5 * cell_size),
            cell_size, cell_size,
            facecolor='steelblue', alpha=0.20, edgecolor='none', zorder=2))

    # Darker shade: each cluster square
    for (cx, cy, sz_m) in clusters:
        ax.add_patch(plt.Rectangle(
            (cx - sz_m / 2.0, cy - sz_m / 2.0),
            sz_m, sz_m,
            facecolor='steelblue', alpha=0.70,
            edgecolor='#1a3a6b', linewidth=0.7, zorder=3))

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
        f'Obstacle processor cluster capacity\n'
        f'cell size = {cell_size} m   range = {range_m} m   '
        f'FOV = \u00b1{fov_half_deg}\u00b0\n'
        f'Cells in sector: {len(cells)}     Maximum clusters: {len(clusters)}',
        fontsize=11)
    ax.legend(handles=[
        mpatches.Patch(facecolor='steelblue', alpha=0.20, label='Occupied cells'),
        mpatches.Patch(facecolor='steelblue', alpha=0.70,
                       edgecolor='#1a3a6b', label='Clusters'),
    ], loc='upper right', fontsize=9)
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Count clusters produced by the obstacle processor '
                    'from a fully-occupied sector.')
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

    n_clusters, cells, clusters, ox, oy, n = max_clusters_in_sector(
        args.cell_size_m, args.range_m, args.fov_half_deg)

    print(f'Cells in sector:  {len(cells)}')
    print(f'Maximum clusters: {n_clusters}')

    plot_result(args.cell_size_m, args.range_m, args.fov_half_deg,
                cells, clusters, ox, oy, n)


if __name__ == '__main__':
    main()
