#include "avt_341/planning/global/fastmarching.h"
#include "avt_341/planning/global/astar_cell.h"

#include <queue>
#include <chrono>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace avt_341 {
namespace planning {

// Clearance penalty function: penalizes paths close to safety margin
static float clearance_penalty(float d, float r, const std::string& option) {
  if (option == "linear") {
    const float R_inf = 3.0f;  // Influence radius: penalty is 0 beyond this
    if (d >= R_inf) {
      return 0.0f;
    }
    return (R_inf - d) / (R_inf - r);
  }
  else if (option == "quadratic") {
    return std::pow(r / d, 2.0f);
  }
  else if (option == "exponential") {
    const float k = 2.0f;
    return std::exp(-k * (d - r));
  }
  else if (option == "repulsive_potential") {
    const float R_inf = 3.0f; // Influence radius: penalty is 0 beyond this
    if (d >= R_inf) {
      return 0.0f;
    }
    return std::pow(1.0f/d - 1.0f/R_inf, 2.0f);
  }
  else {
    return 0.0f;
  }
}

std::vector<Point> FastMarching::PlanPath(avt_341::msg::OccupancyGrid* grid,
                                         avt_341::msg::OccupancyGrid* segmentation_grid,
                                         Point goal,
                                         Point position) {
    if (grid->info.height <= 0 || grid->info.width <= 0) {
        return path_world_;
    }

    SetCornerCoords(grid->info.origin.position.x, grid->info.origin.position.y);
    SetMapRes(grid->info.resolution);

    Index goal_idx = PointToIndex(goal);
    Index start_idx = PointToIndex(position);

    if (start_idx.ix < 0) start_idx.ix = 0;
    if (start_idx.ix >= grid->info.width) start_idx.ix = grid->info.width - 1;
    if (start_idx.iy < 0) start_idx.iy = 0;
    if (start_idx.iy >= grid->info.height) start_idx.iy = grid->info.height - 1;
    if (goal_idx.ix < 0) goal_idx.ix = 0;
    if (goal_idx.ix >= grid->info.width) goal_idx.ix = grid->info.width - 1;
    if (goal_idx.iy < 0) goal_idx.iy = 0;
    if (goal_idx.iy >= grid->info.height) goal_idx.iy = grid->info.height - 1;

    AllocateMap(grid->info.height, grid->info.width, 0);
    SetGoal(goal_idx);
    SetStart(start_idx);

    int n = 0;
    for (int iy = 0; iy < height_; iy++) {
        for (int ix = 0; ix < width_; ix++) {
            double x_grid = grid->info.origin.position.x + ix * grid->info.resolution;
            double y_grid = grid->info.origin.position.y + iy * grid->info.resolution;
            SetMapValue({ix, iy}, grid->data[n], 100 - GetGridValue(segmentation_grid, x_grid, y_grid));
            n++;
        }
    }
    
    
    // Adjust safety margin by adding half of grid resolution
    float adjusted_safety_margin = safety_margin_ + (map_res_ * 0.5f);
    
    // Compute EDT for safety margin logic
    ComputeEDT();

    if (verbose_) {
        std::cout << "[FastMarching] Safety margin (input): " << safety_margin_ << "m" << std::endl;
        std::cout << "[FastMarching] Safety margin (adjusted): " << adjusted_safety_margin << "m" << std::endl;
    }

    // Store original weights (from occupancy/segmentation) and apply EDT-based clearance
    std::vector<float> base_weights(weights_.size());
    std::copy(weights_.begin(), weights_.end(), base_weights.begin());

    // Recompute weights using EDT and safety margin (matching Python logic)
    for (int ix = 0; ix < width_; ix++) {
        for (int iy = 0; iy < height_; iy++) {
            float d = edt_map_[ix][iy];
            int flat_idx = FlattenIndex(ix, iy);
            
            // If too close to obstacle or is an obstacle itself, make impassable
            if (d <= adjusted_safety_margin || map_[ix][iy] > 0) {
                weights_[flat_idx] = INF;
            } else {
                // Start with base weight (includes occupancy and segmentation costs)
                float base_w = base_weights[flat_idx];
                
                // If base weight was already infinite (obstacle), keep it
                if (base_w >= INF) {
                    weights_[flat_idx] = INF;
                } else {
                    // Preserve base cost and add clearance penalty
                    float w = base_w;
                    w += w_distance_ * clearance_penalty(d, adjusted_safety_margin, clearance_penalty_type_);
                    weights_[flat_idx] = w;
                }
            }
        }
    }

    bool solved = Solve();
    if (!solved) {
        if (verbose_) std::cerr << "WARNING: Fast Marching path planner failed to solve map" << std::endl;
    }

    return path_world_;
}

void FastMarching::ComputeEDT() {
    int w = width_;
    int h = height_;
    
    if (w <= 0 || h <= 0) return;

    const float INF_VAL = 1e5f;
    
    std::vector<std::vector<float>> dist_sq(w, std::vector<float>(h, INF_VAL));
    
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            if (map_[x][y] > 0) { 
                dist_sq[x][y] = 0.0f;
            } else {
                dist_sq[x][y] = INF_VAL * INF_VAL;
            }
        }
    }
    
    // Column pass
    for (int x = 0; x < w; ++x) {
        std::vector<float> f(h);
        for(int y=0; y<h; ++y) f[y] = (map_[x][y] > 0) ? 0.0f : INF_VAL;

        for (int y = 1; y < h; ++y) f[y] = std::min(f[y], f[y-1] + 1.0f);
        for (int y = h-2; y >= 0; --y) f[y] = std::min(f[y], f[y+1] + 1.0f);
        
        for(int y=0; y<h; ++y) dist_sq[x][y] = f[y] * f[y];
    }

    // Row pass
    std::vector<std::vector<float>> new_dist(w, std::vector<float>(h));
    
    std::vector<int> v(w + 1);
    std::vector<float> z(w + 2);

    for (int y = 0; y < h; ++y) {
        std::vector<float> d_row(w);
        for(int x=0; x<w; ++x) d_row[x] = dist_sq[x][y];
        
        int k = 0;
        v[0] = 0;
        z[0] = -1.0f * INF_VAL * INF_VAL;
        z[1] = +1.0f * INF_VAL * INF_VAL;
        
        for (int q = 1; q < w; ++q) {
            float s;
            while (true) {
                 float dist_q = d_row[q];
                 float dist_v = d_row[v[k]];
                 
                 float num = (dist_q + (float)q*q) - (dist_v + (float)v[k]*v[k]);
                 float den = 2.0f * q - 2.0f * v[k];
                 s = num / den;
                 
                 if (s <= z[k]) {
                     k--;
                     if (k < 0) {
                        k = 0;
                        s = -1.0f * INF_VAL;
                        break; 
                     }
                     continue;
                 }
                 break;
            }
            k++;
            v[k] = q;
            z[k] = s;
            z[k+1] = +1.0f * INF_VAL * INF_VAL;
        }
        
        int k_curr = 0;
        for (int q = 0; q < w; ++q) {
            while (z[k_curr+1] < q)
                k_curr++;
            
            float dx = (float)(q - v[k_curr]);
            new_dist[q][y] = dx*dx + d_row[v[k_curr]];
        }
    }
    
    // Convert to meters
    edt_map_.resize(w, std::vector<float>(h));
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            edt_map_[x][y] = std::sqrt(new_dist[x][y]) * map_res_;
        }
    }
}

bool FastMarching::Solve() {
  auto t_1 = std::chrono::system_clock::now();
  std::fill(paths_.begin(), paths_.end(), -1);

  AStarCell start_node(goal_, 0.);
  AStarCell goal_node(start_, 0.);

  delete[] costs_;
  costs_ = new float[height_ * width_];
  for (int i = 0; i < height_ * width_; ++i) {
    costs_[i] = INF;
  }
  costs_[start_node.idx] = 0.;

  std::priority_queue<AStarCell> nodes_to_visit;
  nodes_to_visit.push(start_node);
  const int N_adj = 8;
  int* neighbors = new int[N_adj];

  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    AStarCell current = nodes_to_visit.top();

    if (current.idx == goal_node.idx) {
      solution_found = true;
    }

    nodes_to_visit.pop();

    neighbors[0] = HasDown(current.idx) ? Down(current.idx) : -1;
    neighbors[1] = HasLeft(current.idx) ? Left(current.idx) : -1;
    neighbors[2] = HasUp(current.idx) ? Up(current.idx) : -1;
    neighbors[3] = HasRight(current.idx) ? Right(current.idx) : -1;
    neighbors[4] = HasDown(current.idx) && HasLeft(current.idx) ? DownLeft(current.idx) : -1;
    neighbors[5] = HasDown(current.idx) && HasRight(current.idx) ? DownRight(current.idx) : -1;
    neighbors[6] = HasUp(current.idx) && HasLeft(current.idx) ? UpLeft(current.idx) : -1;
    neighbors[7] = HasUp(current.idx) && HasRight(current.idx) ? UpRight(current.idx) : -1;

    for (int i = 0; i < N_adj; ++i) {
      int current_neighbor = neighbors[i];
      
      if (current_neighbor >= 0 && costs_[current_neighbor] == INF) {

        if (weights_[current_neighbor] >= INF) {
          continue;
        }

        float dy_down = HasDown(current_neighbor) ? costs_[Down(current_neighbor)] : INF;
        float dy_up = HasUp(current_neighbor) ? costs_[Up(current_neighbor)] : INF;
        float val_dy = std::min(dy_down, dy_up);
        
        float dx_left = HasLeft(current_neighbor) ? costs_[Left(current_neighbor)] : INF;
        float dx_right = HasRight(current_neighbor) ? costs_[Right(current_neighbor)] : INF;
        float val_dx = std::min(dx_left, dx_right);

        float weight = weights_[current_neighbor];
        float neighbor_weight = map_res_ * weight;

        float discriminant = 2.0f * std::pow(neighbor_weight, 2) - std::pow(val_dx - val_dy, 2);
        float new_cost;
        
        if (discriminant >= 0) {
          new_cost = (val_dx + val_dy + std::sqrt(discriminant)) / 2.0f;
        } else {
          new_cost = std::min(val_dx + neighbor_weight, val_dy + neighbor_weight);
        }

        if (new_cost < costs_[current_neighbor]) {
          costs_[current_neighbor] = new_cost;
          paths_[current_neighbor] = current.idx;
          nodes_to_visit.emplace(current_neighbor, new_cost);
        }
      }
    }
  }
  
  auto t_now = std::chrono::system_clock::now();
  auto calc_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_1);

  if (solution_found) {
    solution_found = ExtractPath(costs_);
  }

  delete[] neighbors;

  return solution_found;
}

bool FastMarching::ExtractPath(float* costs) {
  path_.clear();
  path_world_.clear();

  int current_idx = start_;
  Index current = FoldIndex(current_idx);
  
  Point current_position = IndexToPoint(current);
  path_world_.push_back(current_position);

  int max_steps = height_ * width_;
  int step_count = 0;

  while (current_idx != goal_ && step_count < max_steps) {
    int parent_idx = paths_[current_idx];
    
    if (parent_idx < 0) {
      if (verbose_) std::cerr << "FastMarching: Path extraction failed - no parent at idx " << current_idx << std::endl;
      return false;
    }

    current_idx = parent_idx;
    current = FoldIndex(current_idx);
    current_position = IndexToPoint(current);
    path_world_.push_back(current_position);
    
    step_count++;
  }

  if (current_idx != goal_) {
    if (verbose_) std::cerr << "FastMarching: Path extraction failed - did not reach goal" << std::endl;
    return false;
  }

  return true;
}

float FastMarching::Distance(const Point& p1, const Point& p2) {
  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  return sqrt(dx * dx + dy * dy);
}

}
}
