#include "avt_341/planning/global/fastmarching.h"
#include "avt_341/planning/global/astar_cell.h"

#include <queue>
#include <chrono>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace avt_341 {
namespace planning {

// Optimized clearance penalty function
static inline float clearance_penalty(float d, float r, const std::string& option) {
  if (option == "linear") {
    const float R_inf = 3.0f;
    if (d >= R_inf) return 0.0f;
    return (R_inf - d) / (R_inf - r);
  }
  else if (option == "quadratic") {
    float ratio = r / d;
    return ratio * ratio;
  }
  else if (option == "exponential") {
    return std::exp(-2.0f * (d - r));
  }
  else if (option == "repulsive_potential") {
    const float R_inf = 3.0f;
    if (d >= R_inf) return 0.0f;
    float inv_d = 1.0f / d;
    float inv_R = 1.0f / R_inf;
    float diff = inv_d - inv_R;
    return diff * diff;
  }
  else if (option == "wall_hugging") {
    return d * d;
  }
  return 0.0f;
}

std::vector<Point> FastMarching::PlanPath(avt_341::msg::OccupancyGrid* grid,
                                         avt_341::msg::OccupancyGrid* segmentation_grid,
                                         Point goal,
                                         Point position) {
    if (grid->info.height <= 0 || grid->info.width <= 0) {
        return path_world_;
    }

    int w = grid->info.width;
    int h = grid->info.height;

    // Allocate/Resize maps if needed. Astar::AllocateMap handles base maps.
    if (width_ != w || height_ != h) {
        AllocateMap(h, w, 0);
        edt_flat_.assign(w * h, 0.0f);
        costs_flat_.assign(w * h, INF);
        base_weights_tmp_.assign(w * h, 0.0f);
        
        // EDT work buffers
        edt_work_f_.resize(std::max(w, h));
        edt_work_d_row_.resize(std::max(w, h));
        edt_work_v_.resize(std::max(w, h));
        edt_work_z_.resize(std::max(w, h) + 1);
        edt_work_dist_sq_.assign(w * h, 1e10f);
    }
    
    SetCornerCoords(grid->info.origin.position.x, grid->info.origin.position.y);
    SetMapRes(grid->info.resolution);

    Index goal_idx = PointToIndex(goal);
    Index start_idx = PointToIndex(position);

    // Clamp indexes
    goal_idx.ix = std::max(0, std::min(w - 1, goal_idx.ix));
    goal_idx.iy = std::max(0, std::min(h - 1, goal_idx.iy));
    start_idx.ix = std::max(0, std::min(w - 1, start_idx.ix));
    start_idx.iy = std::max(0, std::min(h - 1, start_idx.iy));

    SetGoal(goal_idx);
    SetStart(start_idx);

    // Fast map copy and weight init
    int n_cells = w * h;
    for (int i = 0; i < n_cells; ++i) {
        int ix = i % w;
        int iy = i / w;
        double x_grid = grid->info.origin.position.x + ix * grid->info.resolution;
        double y_grid = grid->info.origin.position.y + iy * grid->info.resolution;
        
        float occ = (float)grid->data[i];
        float seg = 100.0f - (float)GetGridValue(segmentation_grid, x_grid, y_grid);
        
        map_[ix][iy] = occ;
        base_weights_tmp_[i] = w_distance_ * Astar::EdgeDistanceCost + w_occupancy_ * occ + w_segmentation_ * seg;
    }
    
    float adjusted_safety_margin = safety_margin_ + (map_res_ * 0.5f);
    ComputeEDT();

    if (verbose_) {
        std::cout << "[FastMarching] Safety margin (input/adjusted): " << safety_margin_ << "/" << adjusted_safety_margin << "m" << std::endl;
    }

    // Combine base weights with EDT-based clearance
    for (int i = 0; i < n_cells; ++i) {
        float d = edt_flat_[i];
        int ix = i % w;
        int iy = i / w;
        
        if (d <= adjusted_safety_margin || map_[ix][iy] > 0) {
            weights_[i] = INF;
        } else {
            float base_w = base_weights_tmp_[i];
            if (base_w >= INF) {
                weights_[i] = INF;
            } else {
                weights_[i] = base_w + w_distance_ * clearance_penalty(d, adjusted_safety_margin, clearance_penalty_type_);
            }
        }
    }

    if (!Solve()) {
        if (verbose_) std::cerr << "WARNING: Fast Marching failed to find path" << std::endl;
    }

    return path_world_;
}

void FastMarching::ComputeEDT() {
    int w = width_;
    int h = height_;
    const float INF_EDT = 1e10f;
    
    edt_work_dist_sq_.assign(w * h, INF_EDT);

    // Column pass
    for (int x = 0; x < w; ++x) {
        float last_occ = -INF_EDT;
        for (int y = 0; y < h; ++y) {
            if (map_[x][y] > 0) last_occ = (float)y;
            if (last_occ != -INF_EDT) {
                float dist = (float)y - last_occ;
                edt_work_dist_sq_[y * w + x] = dist * dist;
            }
        }
        last_occ = INF_EDT;
        for (int y = h - 1; y >= 0; --y) {
            if (map_[x][y] > 0) last_occ = (float)y;
            if (last_occ != INF_EDT) {
                float dist = last_occ - (float)y;
                float dsq = dist * dist;
                if (dsq < edt_work_dist_sq_[y * w + x]) {
                    edt_work_dist_sq_[y * w + x] = dsq;
                }
            }
        }
    }

    // Row pass (parabola lower envelope)
    for (int y = 0; y < h; ++y) {
        int k = 0;
        edt_work_v_[0] = 0;
        edt_work_z_[0] = -INF_EDT;
        edt_work_z_[1] = INF_EDT;

        for (int q = 1; q < w; ++q) {
            float f_q = edt_work_dist_sq_[y * w + q];
            if (f_q >= INF_EDT) continue;
            
            float s;
            while (true) {
                int v_k = edt_work_v_[k];
                float f_vk = edt_work_dist_sq_[y * w + v_k];
                s = ((f_q + q * q) - (f_vk + v_k * v_k)) / (2.0f * (q - v_k));
                if (s <= edt_work_z_[k]) {
                    k--;
                } else {
                    break;
                }
                if (k < 0) break;
            }
            k++;
            if (k < 0) k = 0;
            edt_work_v_[k] = q;
            edt_work_z_[k] = s;
            edt_work_z_[k+1] = INF_EDT;
        }

        int cur_k = 0;
        for (int q = 0; q < w; ++q) {
            while (edt_work_z_[cur_k + 1] < q) cur_k++;
            int v_k = edt_work_v_[cur_k];
            float f_vk = edt_work_dist_sq_[y * w + v_k];
            float dx = (float)(q - v_k);
            float dsq = dx * dx + f_vk;
            edt_flat_[y * w + q] = std::sqrt(dsq) * map_res_;
        }
    }
}

bool FastMarching::Solve() {
    std::fill(paths_.begin(), paths_.end(), -1);
    std::fill(costs_flat_.begin(), costs_flat_.end(), INF);

    // Goal is our source in FMM (matching Python logic)
    costs_flat_[goal_] = 0.0f;
    
    std::priority_queue<AStarCell> pq;
    pq.emplace(goal_, 0.0f);

    bool solution_found = false;
    const int w = width_;
    const int h = height_;

    // Neighbor offsets
    const int dx[] = {-1, 1, 0, 0, -1, 1, -1, 1};
    const int dy[] = {0, 0, -1, 1, -1, -1, 1, 1};
    const int n_neigh = search_diagonals_ ? 8 : 4;

    while (!pq.empty()) {
        AStarCell top = pq.top();
        pq.pop();

        int current_idx = top.idx;
        if (top.cost > costs_flat_[current_idx]) continue;

        if (current_idx == start_) {
            solution_found = true;
            break; // Early exit! Huge performance win
        }

        int cx = current_idx % w;
        int cy = current_idx / w;

        for (int i = 0; i < n_neigh; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

            int n_idx = ny * w + nx;
            float weight = weights_[n_idx];
            if (weight >= INF) continue;

            // FMM Update Rule
            float val_dx = INF, val_dy = INF;
            if (nx > 0) val_dx = std::min(val_dx, costs_flat_[n_idx - 1]);
            if (nx < w - 1) val_dx = std::min(val_dx, costs_flat_[n_idx + 1]);
            if (ny > 0) val_dy = std::min(val_dy, costs_flat_[n_idx - w]);
            if (ny < h - 1) val_dy = std::min(val_dy, costs_flat_[n_idx + w]);

            float new_cost;
            float diff = std::abs(val_dx - val_dy);
            if (diff < weight) {
                float sum = val_dx + val_dy;
                new_cost = (sum + std::sqrt(2.0f * weight * weight - diff * diff)) * 0.5f;
            } else {
                new_cost = std::min(val_dx, val_dy) + weight;
            }

            if (new_cost < costs_flat_[n_idx]) {
                costs_flat_[n_idx] = new_cost;
                paths_[n_idx] = current_idx;
                pq.emplace(n_idx, new_cost);
            }
        }
    }

    if (solution_found) {
        return ExtractPath();
    }
    return false;
}

bool FastMarching::ExtractPath() {
    path_.clear();
    path_world_.clear();
    
    int curr = start_;
    int max_iters = width_ * height_;
    int count = 0;
    
    while (curr != goal_ && curr >= 0 && count < max_iters) {
        path_.push_back(FoldIndex(curr));
        path_world_.push_back(IndexToPoint(FoldIndex(curr)));
        
        // Find neighbor with minimum arrival time
        int w = width_;
        int h = height_;
        int cx = curr % w;
        int cy = curr / w;
        
        int best_neigh = -1;
        float min_val = costs_flat_[curr];
        
        const int dx[] = {-1, 1, 0, 0, -1, 1, -1, 1};
        const int dy[] = {0, 0, -1, 1, -1, -1, 1, 1};
        const int n_neigh = search_diagonals_ ? 8 : 4;
        
        for (int i = 0; i < n_neigh; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];
            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                int n_idx = ny * w + nx;
                if (costs_flat_[n_idx] < min_val) {
                    min_val = costs_flat_[n_idx];
                    best_neigh = n_idx;
                }
            }
        }
        
        if (best_neigh == -1) break;
        curr = best_neigh;
        count++;
    }
    
    if (curr == goal_) {
        path_.push_back(FoldIndex(curr));
        path_world_.push_back(IndexToPoint(FoldIndex(curr)));
        return true;
    }
    
    return false;
}

float FastMarching::Distance(const Point& p1, const Point& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace planning
} // namespace avt_341
