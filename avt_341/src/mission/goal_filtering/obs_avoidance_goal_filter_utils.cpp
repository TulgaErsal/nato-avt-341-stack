#include <cmath>
#include <iostream>
#include <algorithm>
#include <climits>
#include <tuple>
#include "avt_341/mission/goal_filtering/obs_avoidance_goal_filter_utils.hpp"

namespace avt_341::mission {

using Eigen::MatrixXi;
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::VectorXi;
using std::string;

// Check if a point collides (obstacle or padding)
bool isInCollision(const MatrixXi& grid, const Vector2d& pt) {
    int x = std::round(pt[0]), y = std::round(pt[1]);
    if (x < 0 || y < 0 || x >= grid.rows() || y >= grid.cols()) 
        return true;
    int v = grid(x, y);
    return (v == OCC_OBSTACLE || v == OCC_PADDING);
}

// Bresenham line‐of‐sight intersection
bool pointsIntersect(const MatrixXi& grid, const Vector2d& p1, const Vector2d& p2) {
    int x1 = std::round(p1[0]), y1 = std::round(p1[1]);
    int x2 = std::round(p2[0]), y2 = std::round(p2[1]);
    int dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
    int sx = (x1 < x2 ? 1 : -1), sy = (y1 < y2 ? 1 : -1);
    int err = dx - dy;
    while (x1 != x2 || y1 != y2) {
        if (grid(x1, y1) == OCC_OBSTACLE) 
            return true;
        int e2 = 2*err;
        if (e2 > -dy) {
            err -= dy; 
            x1 += sx;
        }
        if (e2 < dx) { 
            err += dx; 
            y1 += sy; 
        }
    }
    return grid(x2, y2) == OCC_OBSTACLE;
}

// Extract a PATCH_PAD_WIDTH–padded block around contact
std::tuple<MatrixXi, Vector2i, Vector2i>
extractPatch(const MatrixXi& grid, const Vector2d& contact, const int patch_pad_width) {
    int x = static_cast<int>(std::round(contact[0]));
    int y = static_cast<int>(std::round(contact[1]));
    const int R = static_cast<int>(grid.rows());
    const int C = static_cast<int>(grid.cols());

    x = std::min(std::max(x, 0), R - 1);
    y = std::min(std::max(y, 0), C - 1);

    int xs = std::max(0, x - patch_pad_width);
    int ys = std::max(0, y - patch_pad_width);
    int xe = std::min(R, x + patch_pad_width);
    int ye = std::min(C, y + patch_pad_width);

    if (xe <= xs) xe = std::min(R, xs + 1);
    if (ye <= ys) ye = std::min(C, ys + 1);

    MatrixXi patch = grid.block(xs, ys, xe - xs, ye - ys);
    Vector2i origin(xs, ys);
    // Vector2i padding(x - xs, y - ys);
    Vector2i padding(0, 0);
    return {patch, origin, padding};
}

// Rotate patch & extract the row at patch_point
std::tuple<VectorXi, MatrixXi, Vector2d>
getRowFromPatch(const MatrixXi& patch, Vector2d& patch_point, const Vector2d& patch_center, double angle) {
    MatrixXi rot = patch;
    Vector2d pt = patch_point;

    if (angle != 0.0) {
        int R = patch.rows(), C = patch.cols();
        MatrixXi tmp(R, C); tmp.setConstant(OCC_OBSTACLE);

        Eigen::Matrix2d M;
        M << std::cos(angle), -std::sin(angle),
             std::sin(angle),  std::cos(angle);

        Vector2d ctr(R/2.0, C/2.0);
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                Vector2d o(i - ctr[0], j - ctr[1]);
                Vector2d rc = M*o + ctr;
                int ri = static_cast<int>(std::round(rc[0]));
                int rj = static_cast<int>(std::round(rc[1]));
                if (ri >= 0 && ri < R && rj >= 0 && rj < C)
                    tmp(ri, rj) = patch(i, j);
            }
        }
        rot = tmp;
        Vector2d rel = patch_point - patch_center;
        pt = M*rel + Vector2d(R/2.0, C/2.0);
    }

    // clamp rotated point inside patch bounds
    pt[0] = std::min(std::max(pt[0], 0.0), double(rot.rows() - 1));
    pt[1] = std::min(std::max(pt[1], 0.0), double(rot.cols() - 1));

    int ridx = static_cast<int>(std::round(pt[0]));
    // transpose to return a VectorXi (column vector)
    return {rot.row(ridx).transpose(), rot, pt};
}

// Scan left/right for first OCC_FREE
std::tuple<std::string, int, bool>
getDistance(const Eigen::VectorXi& row, int idx, const std::string& dir) {
    const int L = row.size();

    auto scan_left = [&]() -> int {
        for (int d = 1; idx - d >= 0; ++d) if (row[idx - d] == OCC_FREE) return d;
        return -1;
    };
    auto scan_right = [&]() -> int {
        for (int d = 1; idx + d < L; ++d) if (row[idx + d] == OCC_FREE) return d;
        return -1;
    };

    int ldist = scan_left();
    int rdist = scan_right();

    // deadlock if no free on either side
    if (ldist < 0 && rdist < 0) return {dir, 0, true};

    std::string direction = dir;
    int dist = 0;
    bool deadlock = false;

    if (dir.empty()) {
        // choose side by sum of values along span
        auto sum_left = [&]() -> long long {
            if (ldist < 0) return LLONG_MAX;
            long long s = 0;
            for (int k = idx - ldist; k < idx; ++k) s += row[k];
            return s;
        }();
        auto sum_right = [&]() -> long long {
            if (rdist < 0) return LLONG_MAX;
            long long s = 0;
            for (int k = idx + 1; k <= idx + rdist; ++k) s += row[k];
            return s;
        }();

        if (sum_left <= sum_right) { 
            direction = "left";  
            dist = (ldist < 0 ? 0 : ldist); 
        }
        else { 
            direction = "right"; 
            dist = (rdist < 0 ? 0 : rdist); 
        }
    } 
    else if (dir == "left") {
        if (ldist < 0) {
            deadlock = true; 
        } else {
        dist = ldist;
        }
    } 
    else if (dir == "right") {
        if (rdist < 0) {
            deadlock = true;
        } else {
            dist = rdist;
        }
    } 
    else {
        deadlock = true; // unexpected direction
    }
    return {direction, dist, deadlock};
}


// Find first OCC_FREE in row from cur_idx in direction
int getFirstFeasibleIndex(const Eigen::VectorXi& row, int row_idx_prev, int row_idx, const std::string& dir) {
    const int L = row.size();
    if (row_idx_prev < 0 || row_idx_prev >= L || row_idx < 0 || row_idx >= L) {
        return -1;
    }

    auto sum_range = [&](int a, int b) -> int {
        if (a >= b) {
            return 0;
        }
        int s = 0; 
        for (int k = a; k < b; ++k) {
            s += row[k];
        }
        return s;
    };

    if (dir == "left") {
        // quick accept: prev is FREE and just to its right is PADDING
        if (row_idx_prev + 1 < L && row[row_idx_prev] == OCC_FREE && row[row_idx_prev + 1] == OCC_PADDING)
            return row_idx_prev;

        // scan left from current index
        for (int i = row_idx; i >= 0; --i) {
            if (row[i] == OCC_PADDING || row[i] == OCC_OBSTACLE) continue;
            if (row[i] == OCC_FREE) {
                if (sum_range(row_idx_prev, i) == 0) {
                    return i;
                }
            }
        }
    } 
    else if (dir == "right") {
        // quick accept: prev is FREE and just to its left is PADDING
        if (row_idx_prev - 1 >= 0 && row[row_idx_prev] == OCC_FREE && row[row_idx_prev - 1] == OCC_PADDING)
            return row_idx_prev;

        // scan right from current index
        for (int i = row_idx; i < L; ++i) {
            if (row[i] == OCC_PADDING || row[i] == OCC_OBSTACLE) continue;
            if (row[i] == OCC_FREE) {
                if (sum_range(i, row_idx_prev) == 0) {
                    return i;
                }
            }
        }
    } 
    else {
        return -1;
    }

    return -1;
}

// Deadlock if count(OCC_OBSTACLE) ≥ MIN_OBSTACLE_WIDTH
bool isInDeadlock(const VectorXi& row, int prev, int cur, int min_obstacle_width) {
    int s = std::min(prev, cur);
    int e = std::max(prev, cur);
    int cnt = 0;
    for (int i = s; i <= e; ++i) {
        if (row[i] == OCC_OBSTACLE) {
            ++cnt;
        }
    }
    return cnt >= min_obstacle_width;
}

// Collision avoidance
std::tuple<Vector2d,string,int,bool>
avoidCollision(const std::tuple<MatrixXi, Vector2d, Vector2d, Vector2i, Vector2i>& pd,
               double angle, const VectorXi& row, const string& dir)
{
    auto [patch, pp, pc, orig, pad] = pd;
    int idx = std::round(pp[1]);
    auto [dsel, dist, dead] = getDistance(row, idx, dir);
    if (!dead) {
        pp[1] = (dsel == "left" ? idx - dist : idx + dist);
    }
    if (angle != 0.0) {
        Eigen::Matrix2d M;
        M << std::cos(angle), std::sin(angle),
            -std::sin(angle), std::cos(angle);
        Vector2d center(patch.rows()/2.0, patch.cols()/2.0);
        pp = M*(pp - center) + Vector2d(pc[0], pc[1]);
    }
    Vector2d out = pp + Vector2d(orig[0], orig[1]) - Vector2d(pad[0], pad[1]);
    return {out, dsel, std::round(pp[1]), dead};
}

// Intersection avoidance
std::tuple<Eigen::Vector2d, std::string, int, bool>
avoidIntersection(const std::tuple<Eigen::MatrixXi, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2i, Eigen::Vector2i>& patch_data,
                  double angle, const Eigen::VectorXi& row, int prev_idx, const std::string& dir, int min_obstacle_width) {
    
                    auto [patch, patch_pt, center, origin, padding] = patch_data;
    int cur_idx = static_cast<int>(std::round(patch_pt[1]));

    if (prev_idx < 0) {
        prev_idx = cur_idx;
    }
    int new_idx = getFirstFeasibleIndex(row, prev_idx, cur_idx, dir);
    bool deadlock = (new_idx == -1 || isInDeadlock(row, prev_idx, new_idx, min_obstacle_width));

    if (!deadlock) {
        patch_pt[1] = new_idx;
    }

    // rotate back if needed (your existing inverse-rotation code)
    if (angle != 0.0) {
        Eigen::Matrix2d M;
        M << std::cos(angle), std::sin(angle),
             -std::sin(angle), std::cos(angle);
        Eigen::Vector2d center_rc(patch.rows() / 2.0, patch.cols() / 2.0);
        patch_pt = M * (patch_pt - center_rc) + center;
    }

    Eigen::Vector2d out_pt = patch_pt + origin.cast<double>() - padding.cast<double>();
    return {out_pt, dir, new_idx, deadlock};
}

} // namespace avt_341::mission
