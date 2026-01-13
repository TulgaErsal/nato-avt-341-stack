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
    const float R_inf = 5.0f;
    if (d >= R_inf) return 0.0f;
    return 10.0f * (R_inf - d) / (R_inf - r);
  }
  else if (option == "quadratic") {
    float ratio = r / d;
    return 10.0f * ratio * ratio;
  }
  else if (option == "exponential") {
    return 10.0f * std::exp(-2.0f * (d - r));
  }
  else if (option == "repulsive_potential") {
    const float R_inf = 5.0f;
    if (d >= R_inf) return 0.0f;
    float inv_d = 1.0f / d;
    float inv_R = 1.0f / R_inf;
    float diff = inv_d - inv_R;
    return 20.0f * diff * diff;
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
        
        if (d <= adjusted_safety_margin || map_[ix][iy] > obstacle_threshold_) {
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
            if (map_[x][y] > obstacle_threshold_) last_occ = (float)y;
            if (last_occ != -INF_EDT) {
                float dist = (float)y - last_occ;
                edt_work_dist_sq_[y * w + x] = dist * dist;
            }
        }
        last_occ = INF_EDT;
        for (int y = h - 1; y >= 0; --y) {
            if (map_[x][y] > obstacle_threshold_) last_occ = (float)y;
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
        return ExtractPathGradientDescent(costs_flat_.data());
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

float FastMarching::Distance(const Point& p1, const Point& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool FastMarching::ExtractPathGradientDescent(float* costs) {
  path_.clear();
  path_world_.clear();

  int step_number = 0;
  int steps_per_path_point = 10;
  float step_size = map_res_ / (float) steps_per_path_point;
  //  int current_idx = goal_;
  int current_idx = start_;
  auto current_position = IndexToPoint(FoldIndex(current_idx));
  auto start_position = IndexToPoint(FoldIndex(start_));
  path_world_.push_back(current_position);

  //  while (current_idx != start_) {
  while (current_idx != goal_) {
    if (!IsInMap(FoldIndex(current_idx))) {
      std::cerr << "Iterating path, coordinate not valid.\n";
      return false;
    }
    if (step_number > 2000) {
      return false;
    }
    auto gradient = BilinearInterpolateGradient(costs, current_position);
    auto gradient_normalized = Normalize(gradient);
    //    current_position.x += -gradient_normalized.x * step_size;
    //    current_position.y += -gradient_normalized.y * step_size;
    current_position.x += gradient_normalized.x * step_size;
    current_position.y += gradient_normalized.y * step_size;
    current_idx = FlattenIndex(PointToIndex(current_position));

    auto current_dist = Distance(current_position, start_position);
    //    std::cout << step_number << ": (" << current_position.x << ", " << current_position.y << "),  \t dist: " << current_dist
    //              << "\t Grad: [" << gradient_normalized.x << ", " << gradient_normalized.y << "]\n";
    if (step_number % steps_per_path_point == 0) {
      path_world_.push_back(current_position);
    }
    step_number++;
  }
  //  path_world_.push_back(current_position);

  //  std::reverse(path_.begin(), path_.end());
  //  std::reverse(path_world_.begin(), path_world_.end());

  return true;
}

float FastMarching::HandleGradientNaNs(float cost_1, float cost_0) {
  if (!std::isfinite(cost_1) && !std::isfinite(cost_0)) {
    if (cost_0 == cost_1) {
      return 0;
    } else if (cost_0 > cost_1) {
      return INF;
    } else {
      return -INF;
    }
  } else {
    return (cost_0 - cost_1) / (2 * map_res_);
  }
}

float FastMarching::ComputeGradientY(const float* costs, const Index& index) {
  int up = FlattenIndex(index.ix, index.iy + 1);
  int down = FlattenIndex(index.ix, index.iy - 1);
  return HandleGradientNaNs(costs[up], costs[down]);
}

float FastMarching::ComputeGradientX(const float* costs, const Index& index) {
  int right = FlattenIndex(index.ix + 1, index.iy);
  int left = FlattenIndex(index.ix - 1, index.iy);
  return HandleGradientNaNs(costs[right], costs[left]);
}

Vec2 FastMarching::BilinearInterpolateGradient(const float* costs, const Point& position) {
  // Convert position to grid indices
  Index idx = PointToIndex(position);

  // Calculate the relative position within the cell
  Point cell_center = IndexToPoint(idx);
  float dx = position.x - cell_center.x;
  float dy = position.y - cell_center.y;

  // Determine neighboring indices dynamically
  int ix0 = (dx < 0) ? idx.ix - 1 : idx.ix;
  int ix1 = ix0 + 1;
  int iy0 = (dy < 0) ? idx.iy - 1 : idx.iy;
  int iy1 = iy0 + 1;

  // Ensure indices are within bounds
  ix0 = std::max(1, std::min(ix0, static_cast<int>(width_ - 2)));
  ix1 = std::max(1, std::min(ix1, static_cast<int>(width_ - 2)));
  iy0 = std::max(1, std::min(iy0, static_cast<int>(height_ - 2)));
  iy1 = std::max(1, std::min(iy1, static_cast<int>(height_ - 2)));

  float grad_y22 = ComputeGradientY(costs, {ix1, iy1});
  // Calculate gradients at surrounding points
  float grad_x11 = ComputeGradientX(costs, {ix0, iy0});
  float grad_y11 = ComputeGradientY(costs, {ix0, iy0});

  float grad_x21 = ComputeGradientX(costs, {ix1, iy0});
  float grad_y21 = ComputeGradientY(costs, {ix1, iy0});

  float grad_x12 = ComputeGradientX(costs, {ix0, iy1});
  float grad_y12 = ComputeGradientY(costs, {ix0, iy1});

  float grad_x22 = ComputeGradientX(costs, {ix1, iy1});

  // Check for inf or -inf values
  auto hasInf = [](float value) {
    return std::isinf(value);
  };

  if (hasInf(grad_x11) || hasInf(grad_y11) || hasInf(grad_x21) || hasInf(grad_y21) || hasInf(grad_x12)
    || hasInf(grad_y12) || hasInf(grad_x22) || hasInf(grad_y22)) {
  // Fallback to gradient at the cell corresponding to the position
    float fallback_grad_x = ComputeGradientX(costs, {idx.ix, idx.iy});
    float fallback_grad_y = ComputeGradientY(costs, {idx.ix, idx.iy});
    return {fallback_grad_x, fallback_grad_y};
  }

  // Get positions of the grid points
  Point p11 = IndexToPoint({ix0, iy0});
  Point p21 = IndexToPoint({ix1, iy0});
  Point p12 = IndexToPoint({ix0, iy1});
  Point p22 = IndexToPoint({ix1, iy1});

  // Calculate interpolation weights
  float w11 = ((p22.x - position.x) * (p22.y - position.y)) / (map_res_ * map_res_);
  float w21 = ((position.x - p12.x) * (p12.y - position.y)) / (map_res_ * map_res_);
  float w12 = ((p21.x - position.x) * (position.y - p21.y)) / (map_res_ * map_res_);
  float w22 = ((position.x - p11.x) * (position.y - p11.y)) / (map_res_ * map_res_);

  // Interpolate gradients
  float grad_x = w11 * grad_x11 + w21 * grad_x21 + w12 * grad_x12 + w22 * grad_x22;
  float grad_y = w11 * grad_y11 + w21 * grad_y21 + w12 * grad_y12 + w22 * grad_y22;

  return {grad_x, grad_y};
}

Vec2 FastMarching::Normalize(const Vec2& v) {
  if (std::isfinite(v.x) && std::isfinite(v.y)) {
    if (v.x == 0 && v.y == 0) {
      return {1, 0};
    }
    float mag = sqrt(v.x * v.x + v.y * v.y);
    return {v.x / mag, v.y / mag};
  }
  float x{0}, y{0};
  if (v.x == INF) {
    x = 1;
  } else if (v.x == -INF) {
    x = -1;
  }
  if (v.y == INF) {
    y = 1;
  } else if (v.y == -INF) {
    y = -1;
  }
  return Normalize({x, y});
}


} // namespace planning
} // namespace avt_341
