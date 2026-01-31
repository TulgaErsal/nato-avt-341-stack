#include "avt_341/planning/global/fastmarching.h"
#include "avt_341/planning/global/astar_cell.h"

#include <queue>
#include <chrono>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace avt_341 {
namespace planning {

float FastMarching::clearance_penalty(float d, float r, const std::string& option) {
  if (option == "linear") {
    if (d >= clearance_penalty_range_) return 0.0f;
    return clearance_penalty_scale_ * (clearance_penalty_range_ - d) / (clearance_penalty_range_ - r);
  }
  else if (option == "quadratic") {
    if (d >= clearance_penalty_range_) return 0.0f;
    float ratio = (clearance_penalty_range_ - d) / (clearance_penalty_range_ - r);
    return clearance_penalty_scale_ * ratio * ratio;
  }
  else if (option == "exponential") {
    if (d >= clearance_penalty_range_) return 0.0f;
    float val_at_d = std::exp(-clearance_penalty_exponent_ * (d - r));
    float val_at_range = std::exp(-clearance_penalty_exponent_ * (clearance_penalty_range_ - r));
    return clearance_penalty_scale_ * (val_at_d - val_at_range) / (1.0f - val_at_range);
  }
  else if (option == "repulsive_potential") {
    if (d >= clearance_penalty_range_) return 0.0f;
    float inv_d = 1.0f / d;
    float inv_R = 1.0f / clearance_penalty_range_;
    float diff = inv_d - inv_R;
    return clearance_penalty_scale_ * diff * diff;
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
        shifts_.assign(w * h, {0.0f, 0.0f});
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
    
    float adjusted_safety_margin = safety_margin_global_ + (map_res_ * 0.5f);
    ComputeEDT();

    if (verbose_) {
        std::cout << "[FastMarching] Safety margin (input/adjusted): " << safety_margin_global_ << "/" << adjusted_safety_margin << "m" << std::endl;
    }

    shifts_.assign(n_cells, {0.0f, 0.0f});

    // Combine base weights with EDT-based clearance and compute shifts
    for (int i = 0; i < n_cells; ++i) {
        float d = edt_flat_[i];
        int ix = i % w;
        int iy = i / w;
        
        // A cell is INF only if it is fully consumed by the safety margin.
        // A cell must be marked as an obstacle if a shift to clear the safety margin exceeds half a cell size.
        if (map_[ix][iy] > obstacle_threshold_ || d < safety_margin_global_) {
            weights_[i] = INF;
        } else {
            if (d < adjusted_safety_margin) {
                // Node is within adjusted safety margin but can be shifted.
                // Calculate gradient direction of EDT (points away from obstacle)
                float gx = 0.0f;
                float gy = 0.0f;
                if (ix > 0 && ix < w - 1) gx = (edt_flat_[i+1] - edt_flat_[i-1]);
                else if (ix == 0 && w > 1) gx = (edt_flat_[i+1] - d) * 2.0f;
                else if (ix == w - 1 && w > 1) gx = (d - edt_flat_[i-1]) * 2.0f;

                if (iy > 0 && iy < h - 1) gy = (edt_flat_[i+w] - edt_flat_[i-w]);
                else if (iy == 0 && h > 1) gy = (edt_flat_[i+w] - d) * 2.0f;
                else if (iy == h - 1 && h > 1) gy = (d - edt_flat_[i-w]) * 2.0f;

                float mag = std::sqrt(gx * gx + gy * gy);
                if (mag > 1e-6f) {
                    float shift_len = adjusted_safety_margin - d;
                    shifts_[i].x = (gx / mag) * shift_len;
                    shifts_[i].y = (gy / mag) * shift_len;
                }
            }

            float base_w = base_weights_tmp_[i];
            if (base_w >= INF) {
                weights_[i] = INF;
            } else {
                weights_[i] = base_w + w_distance_ * clearance_penalty(std::max(d, adjusted_safety_margin), adjusted_safety_margin, clearance_penalty_type_);
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
            Point target_p = GetShiftedPoint(n_idx);

            float T_x = INF;
            float h_x = map_res_;
            if (nx > 0) {
                float val = costs_flat_[n_idx - 1];
                if (val < T_x) {
                    T_x = val;
                    h_x = Distance(target_p, GetShiftedPoint(n_idx - 1));
                }
            }
            if (nx < w - 1) {
                float val = costs_flat_[n_idx + 1];
                if (val < T_x) {
                    T_x = val;
                    h_x = Distance(target_p, GetShiftedPoint(n_idx + 1));
                }
            }

            float T_y = INF;
            float h_y = map_res_;
            if (ny > 0) {
                float val = costs_flat_[n_idx - w];
                if (val < T_y) {
                    T_y = val;
                    h_y = Distance(target_p, GetShiftedPoint(n_idx - w));
                }
            }
            if (ny < h - 1) {
                float val = costs_flat_[n_idx + w];
                if (val < T_y) {
                    T_y = val;
                    h_y = Distance(target_p, GetShiftedPoint(n_idx + w));
                }
            }

            float f = weight / map_res_;
            float new_cost = INF;

            if (T_x < INF && T_y < INF) {
                float a = 1.0f / (h_x * h_x);
                float b = 1.0f / (h_y * h_y);
                float c = f * f;

                bool two_axis = false;
                if (T_x > T_y) {
                    if (T_x - T_y < f * h_y) two_axis = true;
                } else {
                    if (T_y - T_x < f * h_x) two_axis = true;
                }

                if (two_axis) {
                    float sum_ab = a + b;
                    float term1 = (a * T_x + b * T_y) / sum_ab;
                    float disc = c * sum_ab - a * b * (T_x - T_y) * (T_x - T_y);
                    if (disc < 0) disc = 0;
                    new_cost = term1 + std::sqrt(disc) / sum_ab;
                } else {
                    new_cost = std::min(T_x + f * h_x, T_y + f * h_y);
                }
            } else if (T_x < INF) {
                new_cost = T_x + f * h_x;
            } else if (T_y < INF) {
                new_cost = T_y + f * h_y;
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
    
    bool found = false;
    // Check YAML-loaded parameter
    if (path_extraction_method_ == "discrete") {
        found = ExtractPathDiscrete();
        // apply LOS smoothing
        if (found && los_max_iterations_ > 0) {
            std::vector<Index> smoothed_path;
            PostSmoothing(path_, smoothed_path);
            for (int i = 1; i < los_max_iterations_; i++) {
                std::vector<Index> tmp = smoothed_path;
                smoothed_path.clear();
                PostSmoothing(tmp, smoothed_path);
            }
            path_ = smoothed_path;
            path_world_.clear();
            for (const auto& idx : path_) {
                path_world_.push_back(GetShiftedPoint(FlattenIndex(idx)));
            }
        }
    } else if (path_extraction_method_ == "hybrid") {
        found = ExtractPathGradientDescent(costs_flat_.data());
        if (!found) {
            found = ExtractPathDiscrete();
        }
    } else {
        found = ExtractPathGradientDescent(costs_flat_.data());
    }

    if (found && clipping_distance_ > 0.0f && !path_world_.empty()) {
        Point start_pos = IndexToPoint(FoldIndex(start_));
        
        while (path_world_.size() > 1 && Distance(path_world_[0], start_pos) < clipping_distance_) {
            path_world_.erase(path_world_.begin());
            if (!path_.empty()) {
                path_.erase(path_.begin());
            }
        }
    }
    
    return found;
}

bool FastMarching::ExtractPathDiscrete() {
    path_.clear();
    path_world_.clear();
    
    int curr = start_;
    int max_iters = width_ * height_;
    int count = 0;
    
    while (curr != goal_ && curr >= 0 && count < max_iters) {
        path_.push_back(FoldIndex(curr));
        path_world_.push_back(GetShiftedPoint(curr));
        
        // Find neighbor with minimum arrival time
        int w = width_;
        int h = height_;
        int cx = curr % w;
        int cy = curr / w;
        
        int best_neigh = -1;
        float max_descent = 0.0f;
        Point p_curr = GetShiftedPoint(curr);
        
        const int dx[] = {-1, 1, 0, 0, -1, 1, -1, 1};
        const int dy[] = {0, 0, -1, 1, -1, -1, 1, 1};
        const int n_neigh = search_diagonals_ ? 8 : 4;
        
        for (int i = 0; i < n_neigh; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];
            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                int n_idx = ny * w + nx;
                float val_n = costs_flat_[n_idx];
                if (val_n < costs_flat_[curr]) {
                    float dist = Distance(p_curr, GetShiftedPoint(n_idx));
                    float descent = (costs_flat_[curr] - val_n) / dist;
                    if (descent > max_descent) {
                        max_descent = descent;
                        best_neigh = n_idx;
                    }
                }
            }
        }
        
        if (best_neigh == -1) break;
        curr = best_neigh;
        count++;
    }
    
    if (curr == goal_) {
        path_.push_back(FoldIndex(curr));
        path_world_.push_back(GetShiftedPoint(curr));
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
  int steps_per_path_point = gradient_descent_steps_per_point_;
  float step_size = map_res_ / (float) steps_per_path_point;
  int current_idx = start_;
  auto current_position = IndexToPoint(FoldIndex(current_idx));
  auto start_position = IndexToPoint(FoldIndex(start_));
  path_world_.push_back(current_position);

  while (current_idx != goal_) {
    if (!IsInMap(FoldIndex(current_idx))) {
      std::cerr << "Iterating path, coordinate not valid.\n";
      return false;
    }
    if (step_number > gradient_descent_max_steps_) {
      return false;
    }
    auto gradient = BilinearInterpolateGradient(costs, current_position);
    auto gradient_normalized = Normalize(gradient);
    current_position.x += gradient_normalized.x * step_size;
    current_position.y += gradient_normalized.y * step_size;
    current_idx = FlattenIndex(PointToIndex(current_position));

    auto current_dist = Distance(current_position, start_position);
    if (step_number % steps_per_path_point == 0) {
      path_world_.push_back(current_position);
    }
    step_number++;
  }

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

bool FastMarching::LineOfSight(const Index& i0, const Index& i1) {
  int x0 = i0.ix;
  int y0 = i0.iy;
  int x1 = i1.ix;
  int y1 = i1.iy;
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    if (weights_[y0 * width_ + x0] >= INF) return false;
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
  return true;
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


Point FastMarching::GetShiftedPoint(int idx) const {
  Point p = IndexToPoint(FoldIndex(idx));
  p.x += shifts_[idx].x;
  p.y += shifts_[idx].y;
  return p;
}

} // namespace planning
} // namespace avt_341
