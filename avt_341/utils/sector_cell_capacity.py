#!/usr/bin/env python3
"""
Obstacle-processor sector capacity tool.

Computes the maximum number of grid cells that can fall inside the observation
sector used by obstacles_processor_node.cpp, using the same inclusion logic:

  A cell is INCLUDED if all three conditions hold:
    1. distance(vehicle, cell_center) <= range_m
    2. cross(cell_vec, left_boundary).z  >= 0   (within left FOV edge)
    3. cross(cell_vec, right_boundary).z <= 0   (within right FOV edge)

  where:
    left_boundary  = [cos(theta), sin(theta)]
    right_boundary = [cos(theta), -sin(theta)]
    theta          = fov_half_deg converted to radians

The vehicle sits at the origin and faces +X.  Because the count depends on
how the grid is aligned relative to the vehicle, the script searches over all
fractional offsets in [0, cell_size) x [0, cell_size) and reports the MAXIMUM
achievable count together with a plot for that best-case alignment.

Usage
-----
    python3 sector_cell_capacity.py <cell_size_m> <range_m> <fov_half_deg>

Arguments
---------
  cell_size_m   Grid cell size [m].
  range_m       Observation range [m].
  fov_half_deg  Half-angle of the field of view [deg].
                The complete FOV spans from -fov_half_deg to +fov_half_deg.

Example
-------
    python3 sector_cell_capacity.py 1.0 20.0 45
"""

import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def _count_in_sector(cell_size, range_m, theta_rad, offset_x, offset_y):
    """Count cell centers inside the sector for a given grid offset.

    The grid origin is chosen so that cell centers are at:
        cx = offset_x + (i + 0.5) * cell_size
        cy = offset_y + (j + 0.5) * cell_size
    for all integer i, j such that the center is within the bounding box
    [-range_m - cell_size, range_m + cell_size]^2.

    Returns (count, list_of_(cx, cy)).
    """
    lx = math.cos(theta_rad)
    ly = math.sin(theta_rad)
    rx = math.cos(theta_rad)
    ry = -math.sin(theta_rad)

    half_side = range_m + cell_size
    origin = -half_side
    n = int(math.ceil(2.0 * half_side / cell_size))

    inside = []
    for i in range(n):
        for j in range(n):
            cx = offset_x + (i + 0.5) * cell_size + origin
            cy = offset_y + (j + 0.5) * cell_size + origin
            # 1. Range check (obstacles_processor skips if distance > range)
            if math.hypot(cx, cy) > range_m:
                continue
            # 2. Left boundary: skip if cross(cell_vec, left).z < 0
            if cx * ly - lx * cy < 0:
                continue
            # 3. Right boundary: skip if cross(cell_vec, right).z > 0
            if cx * ry - rx * cy > 0:
                continue
            inside.append((cx, cy))
    return len(inside), inside


def max_cells_in_sector(cell_size, range_m, fov_half_deg, n_offsets=20):
    """Return (max_count, best_cells) over a grid of fractional offsets."""
    theta_rad = math.radians(fov_half_deg)
    offsets = np.linspace(0.0, cell_size, n_offsets, endpoint=False)

    best_count = 0
    best_cells = []
    for ox in offsets:
        for oy in offsets:
            count, cells = _count_in_sector(cell_size, range_m, theta_rad, ox, oy)
            if count > best_count:
                best_count = count
                best_cells = cells

    return best_count, best_cells


def plot_sector(cell_size, range_m, fov_half_deg, inside_cells):
    theta_rad = math.radians(fov_half_deg)
    half_side = range_m + cell_size

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')

    # Grid lines
    origin = -half_side
    n = int(math.ceil(2.0 * half_side / cell_size))
    # Collect the actual grid extents from the best-alignment cells so the
    # lines align with the shaded cells.
    if inside_cells:
        cx0, cy0 = inside_cells[0]
        # Grid lines relative to the first cell center
        dx = (cx0 + half_side - 0.5 * cell_size) % cell_size  # offset within cell
    else:
        dx, _ = 0.0, 0.0
    # Draw grid from origin aligned with the best cells
    x_start = origin
    y_start = origin
    for k in range(n + 2):
        xv = x_start + k * cell_size
        ax.axvline(xv, color='#cccccc', linewidth=0.4, zorder=1)
    for k in range(n + 2):
        yv = y_start + k * cell_size
        ax.axhline(yv, color='#cccccc', linewidth=0.4, zorder=1)

    # Shade cells inside sector
    for (cx, cy) in inside_cells:
        rect = plt.Rectangle(
            (cx - 0.5 * cell_size, cy - 0.5 * cell_size),
            cell_size, cell_size,
            facecolor='steelblue', alpha=0.35, edgecolor='none', zorder=2)
        ax.add_patch(rect)

    # Sector boundary: two radial lines + arc
    ax.plot([0, range_m * math.cos(theta_rad)],
            [0, range_m * math.sin(theta_rad)],
            color='navy', linewidth=1.8, zorder=3, label='FOV boundary')
    ax.plot([0, range_m * math.cos(theta_rad)],
            [0, -range_m * math.sin(theta_rad)],
            color='navy', linewidth=1.8, zorder=3)
    arc = mpatches.Arc(
        (0, 0), 2 * range_m, 2 * range_m,
        angle=0, theta1=-fov_half_deg, theta2=fov_half_deg,
        color='navy', linewidth=1.8, zorder=3)
    ax.add_patch(arc)

    # Vehicle marker + heading arrow
    ax.plot(0, 0, 'ro', markersize=7, zorder=5, label='Vehicle')
    arrow_len = max(cell_size * 1.5, range_m * 0.08)
    ax.annotate('', xy=(arrow_len, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle='->', color='red', lw=2.0), zorder=5)

    ax.set_xlim(-half_side * 0.3, half_side)
    ax.set_ylim(-half_side, half_side)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(
        f'Obstacle processor sector capacity\n'
        f'cell size = {cell_size} m   range = {range_m} m   '
        f'FOV = \u00b1{fov_half_deg}\u00b0\n'
        f'Maximum cells in sector: {len(inside_cells)}',
        fontsize=11)
    ax.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Count grid cells inside the obstacle-processor observation sector.')
    parser.add_argument('cell_size_m',   type=float, help='Grid cell size [m]')
    parser.add_argument('range_m',       type=float, help='Observation range [m]')
    parser.add_argument('fov_half_deg',  type=float,
                        help='Half-angle of the field of view [deg] '
                             '(full FOV spans -fov_half_deg to +fov_half_deg)')
    args = parser.parse_args()

    if args.cell_size_m <= 0:
        raise ValueError('cell_size_m must be positive')
    if args.range_m <= 0:
        raise ValueError('range_m must be positive')
    if not (0.0 < args.fov_half_deg <= 180.0):
        raise ValueError('fov_half_deg must be in (0, 180]')

    max_count, best_cells = max_cells_in_sector(
        args.cell_size_m, args.range_m, args.fov_half_deg)

    print(f'Maximum cells in sector: {max_count}')
    plot_sector(args.cell_size_m, args.range_m, args.fov_half_deg, best_cells)


if __name__ == '__main__':
    main()
