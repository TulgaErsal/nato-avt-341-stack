#!/usr/bin/env python3
"""
Trajectory-corridor obstacle culling illustration.

Demonstrates the effect of the corridor culling feature implemented in
avt_341_mpc_planner_node.cpp: only obstacles whose center lies within
corridor_half_width of the MPC trajectory polyline are forwarded to the
NLP solver.  Obstacles outside the corridor are dropped before the solve.

The Python culling logic below is an exact translation of
CullObstaclesToCorridor() / SegmentDistSq() from the C++ node.

Two plots are shown side-by-side:
  Left  — without culling: all obstacles visible to the sensor are shown.
  Right — with    culling: only corridor obstacles are shown; culled
          obstacles are rendered faded so the spatial context is preserved.

Usage
-----
    python3 corridor_culling_demo.py [corridor_half_width]

Arguments
---------
  corridor_half_width  Half-width of the trajectory corridor [m].
                       Default: 10.0 (matches the node default).

Example
-------
    python3 corridor_culling_demo.py 10.0
"""

import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


# ---------------------------------------------------------------------------
# Culling logic — exact translation of the C++ implementation
# ---------------------------------------------------------------------------

def _segment_dist_sq(px, py, ax, ay, bx, by):
    """
    Squared distance from point (px, py) to segment (ax,ay)-(bx,by).

    Matches SegmentDistSq() in avt_341_mpc_planner_node.cpp.
    """
    dx, dy = bx - ax, by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-12:
        ex, ey = px - ax, py - ay
        return ex * ex + ey * ey
    t = ((px - ax) * dx + (py - ay) * dy) / len_sq
    t = max(0.0, min(1.0, t))
    cx = ax + t * dx - px
    cy = ay + t * dy - py
    return cx * cx + cy * cy


def cull_to_corridor(obstacles, path, half_width):
    """
    Return the subset of obstacles within half_width of the path polyline.

    Matches CullObstaclesToCorridor() in avt_341_mpc_planner_node.cpp.

    Parameters
    ----------
    obstacles : list of (x, y, size) tuples
    path      : list of (x, y) tuples with at least 2 points
    half_width : float

    Returns
    -------
    kept   : list of (x, y, size) tuples inside the corridor
    culled : list of (x, y, size) tuples outside the corridor
    """
    if len(path) < 2:
        return list(obstacles), []   # no valid path yet — pass all through

    threshold_sq = half_width * half_width
    kept, culled = [], []

    for ox, oy, sz in obstacles:
        in_corridor = False
        for j in range(len(path) - 1):
            d_sq = _segment_dist_sq(ox, oy,
                                    path[j][0],     path[j][1],
                                    path[j+1][0],   path[j+1][1])
            if d_sq <= threshold_sq:
                in_corridor = True
                break
        (kept if in_corridor else culled).append((ox, oy, sz))

    return kept, culled


# ---------------------------------------------------------------------------
# Corridor boundary polygon
# ---------------------------------------------------------------------------

def _corridor_polygon(path, half_width):
    """
    Build the left and right offset polylines of *path* at distance
    *half_width*.  The offset direction at each node is the average of the
    unit normals of adjacent segments (miter join), which matches the
    visual effect of the circular-distance metric used in culling.
    """
    pts = np.array(path)
    n = len(pts)
    normals = []
    for i in range(n):
        if i == 0:
            d = pts[1] - pts[0]
        elif i == n - 1:
            d = pts[-1] - pts[-2]
        else:
            d1 = pts[i] - pts[i-1];  d1 /= (np.linalg.norm(d1) + 1e-12)
            d2 = pts[i+1] - pts[i];  d2 /= (np.linalg.norm(d2) + 1e-12)
            d = d1 + d2
        length = np.linalg.norm(d)
        if length < 1e-9:
            normals.append(np.array([0.0, 1.0]))
        else:
            nx, ny = -d[1] / length, d[0] / length
            normals.append(np.array([nx, ny]))

    normals = np.array(normals)
    left  = pts + half_width * normals
    right = pts - half_width * normals
    # closed polygon: left boundary forward + right boundary backward
    poly = np.vstack([left, right[::-1]])
    return poly


# ---------------------------------------------------------------------------
# Scene generation
# ---------------------------------------------------------------------------

def _make_scene(rng, n_obs=250):
    """
    Generate a curved MPC trajectory and a random obstacle field.

    The path is an S-curve going forward in x.
    Obstacles are scattered across the bounding area.
    """
    # MPC trajectory: S-curve from (0,0) to (60, 0)
    t = np.linspace(0, 1, 25)
    px = 60.0 * t
    py = 12.0 * np.sin(2.0 * np.pi * t * 0.75)
    path = list(zip(px, py))

    # Obstacles: random centers in the scene bounding box
    xs = rng.uniform(-5, 65, n_obs)
    ys = rng.uniform(-20, 20, n_obs)
    sizes = rng.uniform(0.5, 2.0, n_obs)
    obstacles = list(zip(xs, ys, sizes))

    return path, obstacles


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _draw_scene(ax, path, kept, culled, half_width, show_culled, title):
    """Draw one panel of the comparison figure."""
    pts = np.array(path)

    # Corridor shading
    poly = _corridor_polygon(path, half_width)
    ax.fill(poly[:, 0], poly[:, 1],
            color='steelblue', alpha=0.12, zorder=1, label='Corridor')
    ax.plot(poly[:len(path), 0], poly[:len(path), 1],
            color='steelblue', linewidth=0.8, linestyle='--', zorder=2)
    ax.plot(poly[len(path):, 0], poly[len(path):, 1],
            color='steelblue', linewidth=0.8, linestyle='--', zorder=2)

    # Culled obstacles (faded) — drawn even in the "without culling" panel
    # so the spatial context is identical in both plots.
    if show_culled:
        for ox, oy, sz in culled:
            ax.add_patch(plt.Rectangle(
                (ox - sz / 2, oy - sz / 2), sz, sz,
                facecolor='#aaaaaa', alpha=0.35, edgecolor='#888888',
                linewidth=0.5, zorder=3))
    else:
        # Without culling: culled obstacles are shown the same as kept ones
        for ox, oy, sz in culled:
            ax.add_patch(plt.Rectangle(
                (ox - sz / 2, oy - sz / 2), sz, sz,
                facecolor='tomato', alpha=0.75, edgecolor='#8b0000',
                linewidth=0.5, zorder=3))

    # Kept (in-corridor) obstacles
    for ox, oy, sz in kept:
        ax.add_patch(plt.Rectangle(
            (ox - sz / 2, oy - sz / 2), sz, sz,
            facecolor='tomato', alpha=0.85, edgecolor='#8b0000',
            linewidth=0.6, zorder=4))

    # MPC trajectory
    ax.plot(pts[:, 0], pts[:, 1],
            color='navy', linewidth=2.0, zorder=5, label='MPC trajectory')
    ax.plot(pts[:, 0], pts[:, 1],
            'o', color='navy', markersize=3, zorder=5)

    # Vehicle marker at path start
    ax.plot(pts[0, 0], pts[0, 1], 'ro', markersize=8, zorder=6)
    if len(path) >= 2:
        dx = pts[1, 0] - pts[0, 0]
        dy = pts[1, 1] - pts[0, 1]
        arrow_len = 3.0
        scale = arrow_len / (math.hypot(dx, dy) + 1e-12)
        ax.annotate('', xy=(pts[0, 0] + dx * scale, pts[0, 1] + dy * scale),
                    xytext=(pts[0, 0], pts[0, 1]),
                    arrowprops=dict(arrowstyle='->', color='red', lw=2.0),
                    zorder=6)

    n_shown = len(kept) + (0 if show_culled else len(culled))
    ax.set_title(title + f'\n{n_shown} obstacles passed to NLP', fontsize=10)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_aspect('equal')
    ax.set_xlim(-10, 70)
    ax.set_ylim(-25, 25)

    legend_handles = [
        mpatches.Patch(facecolor='tomato', alpha=0.85,
                       edgecolor='#8b0000', label='Obstacle (active in NLP)'),
        plt.Line2D([0], [0], color='navy', linewidth=2, label='MPC trajectory'),
        mpatches.Patch(facecolor='steelblue', alpha=0.25,
                       linestyle='--', label=f'Corridor (±{half_width:.0f} m)'),
    ]
    if show_culled:
        legend_handles.insert(1, mpatches.Patch(
            facecolor='#aaaaaa', alpha=0.45, edgecolor='#888888',
            label='Obstacle (culled — not in NLP)'))
    ax.legend(handles=legend_handles, loc='upper right', fontsize=8)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Illustrate trajectory-corridor obstacle culling '
                    '(matches avt_341_mpc_planner_node.cpp).')
    parser.add_argument('corridor_half_width', type=float, nargs='?',
                        default=10.0,
                        help='Half-width of the trajectory corridor [m] '
                             '(default: 10.0)')
    args = parser.parse_args()

    half_width = args.corridor_half_width
    if half_width <= 0:
        raise ValueError('corridor_half_width must be positive')

    rng = np.random.default_rng(seed=42)
    path, obstacles = _make_scene(rng)

    kept, culled = cull_to_corridor(obstacles, path, half_width)

    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle(
        f'Trajectory-corridor obstacle culling  '
        f'(corridor half-width = {half_width:.1f} m)\n'
        f'Total obstacles in sensor FOV: {len(obstacles)}     '
        f'Kept: {len(kept)}     Culled: {len(culled)}',
        fontsize=12)

    _draw_scene(ax_left,  path, kept, culled, half_width,
                show_culled=False,
                title='Without culling')
    _draw_scene(ax_right, path, kept, culled, half_width,
                show_culled=True,
                title='With culling')

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
