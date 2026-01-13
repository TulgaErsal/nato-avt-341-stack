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

    costs_flat_[goal_] = 0.0f;
    std::priority_queue<AStarCell> pq;
    pq.emplace(goal_, 0.0f, 0.0f);

    const int w = width_;
    const int h = height_;
    const int n_neigh = search_diagonals_ ? 8 : 4;
    const int dx[] = {-1, 1, 0, 0, -1, 1, -1, 1};
    const int dy[] = {0, 0, -1, 1, -1, -1, 1, 1};

    while (!pq.empty()) {
        AStarCell top = pq.top();
        pq.pop();

        int idx = top.idx;
        if (top.g > costs_flat_[idx]) continue;
        if (idx == start_) return ExtractPath();

        int cx = idx % w;
        int cy = idx / w;

        // Standard FMM propagates to neighbors
        for (int i = 0; i < n_neigh; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

            int n_idx = ny * w + nx;
            float weight = weights_[n_idx];
            if (weight >= INF) continue;

            // --- EIKONAL UPDATE LOGIC ---
            // Find the minimum arrival times from the 4-cardinal neighbors of the TARGET node (n_idx)
            float T_x = INF;
            if (nx > 0) T_x = std::min(T_x, costs_flat_[n_idx - 1]);
            if (nx < w - 1) T_x = std::min(T_x, costs_flat_[n_idx + 1]);

            float T_y = INF;
            if (ny > 0) T_y = std::min(T_y, costs_flat_[n_idx - w]);
            if (ny < h - 1) T_y = std::min(T_y, costs_flat_[n_idx + w]);

            float new_cost;
            if (std::abs(T_x - T_y) < weight) {
                // Solution to (T-Tx)^2 + (T-Ty)^2 = W^2
                float sum = T_x + T_y;
                new_cost = (sum + std::sqrt(2.0f * weight * weight - (T_x - T_y) * (T_x - T_y))) * 0.5f;
            } else {
                // Wave is moving mostly along one axis
                new_cost = std::min(T_x, T_y) + weight;
            }

            if (new_cost < costs_flat_[n_idx]) {
                costs_flat_[n_idx] = new_cost;
                paths_[n_idx] = idx; 
                pq.emplace(n_idx, new_cost, new_cost);
            }
        }
    }
    return false;
}

bool FastMarching::ExtractPath() {
    path_.clear();
    path_world_.clear();
    
    // Check YAML-loaded parameter
    if (path_integration_mode_ == "discrete") {
        return ExtractPathDiscrete();
    } else {
        return ExtractPathGradientDescent();
    }
}

bool FastMarching::ExtractPathDiscrete() {
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

bool FastMarching::ExtractPathGradientDescent() {
    // Start at the 'start' position (tracking back to goal)
    // In your Solve(), goal has cost 0, so we move "downhill" to goal.
    Point curr_p = IndexToPoint(FoldIndex(start_));
    int max_iters = width_ * height_ * 2;
    float step_size = map_res_ * 0.5f; 
    
    for (int i = 0; i < max_iters; ++i) {
        if (!std::isfinite(curr_p.x) || !std::isfinite(curr_p.y)) break;
        path_world_.push_back(curr_p);
        
        // 1. Calculate Gradient at current continuous point
        // Using Central Difference: grad = ((T(x+1)-T(x-1))/2, (T(y+1)-T(y-1))/2)
        Point grad = GetGradient(curr_p);
        
        float mag = std::sqrt(grad.x * grad.x + grad.y * grad.y);
        if (mag < 1e-5 || !std::isfinite(mag)) break; // Reached the goal (minima) or invalid state

        // 2. Move "downhill" (negative gradient)
        curr_p.x -= (grad.x / mag) * step_size;
        curr_p.y -= (grad.y / mag) * step_size;

        // 3. Termination Check
        if (Distance(curr_p, IndexToPoint(FoldIndex(goal_))) < map_res_) {
            path_world_.push_back(IndexToPoint(FoldIndex(goal_)));
            return true;
        }
    }
    return !path_world_.empty();
}

float FastMarching::Distance(const Point& p1, const Point& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

Point FastMarching::GetGradient(Point p) {
    // Convert world coordinates to float grid coordinates
    float gx = (p.x - llx_) / map_res_;
    float gy = (p.y - lly_) / map_res_;

    // Get integer coordinates of the top-left cell
    int x0 = static_cast<int>(std::floor(gx));
    int y0 = static_cast<int>(std::floor(gy));
    
    // Safety bounds check for a 2x2 neighborhood
    if (x0 < 1 || x0 >= width_ - 2 || y0 < 1 || y0 >= height_ - 2) {
        return {0, 0}; 
    }

    // Local interpolation weights
    float sx = gx - x0;
    float sy = gy - y0;

    // Helper to get cost at integer grid coords. 
    // We treat INF as a large finite value for gradient math to avoid NaNs.
    auto T = [&](int ix, int iy) { 
        float v = costs_flat_[iy * width_ + ix];
        return (v >= INF * 0.5f) ? 1e6f : v; 
    };

    // Calculate Central Differences at the 4 corners of the current cell
    // grad_x = (T[x+1] - T[x-1]) / 2
    float g00_x = (T(x0+1, y0) - T(x0-1, y0)) * 0.5f;
    float g10_x = (T(x0+2, y0) - T(x0, y0)) * 0.5f;
    float g01_x = (T(x0+1, y0+1) - T(x0-1, y0+1)) * 0.5f;
    float g11_x = (T(x0+2, y0+1) - T(x0, y0+1)) * 0.5f;

    float g00_y = (T(x0, y0+1) - T(x0, y0-1)) * 0.5f;
    float g10_y = (T(x0+1, y0+1) - T(x0+1, y0-1)) * 0.5f;
    float g01_y = (T(x0, y0+2) - T(x0, y0)) * 0.5f;
    float g11_y = (T(x0+1, y0+2) - T(x0+1, y0)) * 0.5f;

    // Bilinear interpolation of the gradients
    float grad_x = (1-sy)*( (1-sx)*g00_x + sx*g10_x ) + sy*( (1-sx)*g01_x + sx*g11_x );
    float grad_y = (1-sy)*( (1-sx)*g00_y + sx*g10_y ) + sy*( (1-sx)*g01_y + sx*g11_y );

    return {grad_x / map_res_, grad_y / map_res_};
}

} // namespace planning
} // namespace avt_341
