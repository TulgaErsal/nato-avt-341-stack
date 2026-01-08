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
static float clearance_penalty(float distance, float safety_margin) {
  // Exponential penalty as we get closer to safety margin
  // Returns 0 when far from obstacles, increases as we approach safety_margin
  if (distance <= safety_margin) {
    return 1e9f; // Infinite penalty
  }
  
  float buffer = 2.0f * safety_margin; // Comfortable clearance distance
  if (distance >= buffer) {
    return 0.0f; // No penalty when far enough
  }
  
  // Exponential penalty in the transition zone
  float normalized = (distance - safety_margin) / safety_margin;
  return std::exp(-normalized * 2.0f) - std::exp(-2.0f);
}

bool FastMarching::Solve() {
  auto t_1 = std::chrono::system_clock::now();
  std::fill(paths_.begin(), paths_.end(), -1);

  // Solve backwards from goal to start (as per Python implementation)
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
  const int N_adj = 8; // Always use 8-connected for FMM
  int* neighbors = new int[N_adj];

  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    AStarCell current = nodes_to_visit.top();

    // Optimization: stop if we reached the actual start
    if (current.idx == goal_node.idx) {
      solution_found = true;
      // Could break here, but continuing fills the entire cost map
    }

    nodes_to_visit.pop();

    // Find 8-connected neighbors
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
      
      // Only process unvisited cells (cost == INF)
      if (current_neighbor >= 0 && costs_[current_neighbor] == INF) {

        // Skip infinite weight cells (obstacles)
        if (weights_[current_neighbor] >= INF) {
          continue;
        }

        // For each new cell, find the min cost in both x and y directions
        float dy_down = HasDown(current_neighbor) ? costs_[Down(current_neighbor)] : INF;
        float dy_up = HasUp(current_neighbor) ? costs_[Up(current_neighbor)] : INF;
        float val_dy = std::min(dy_down, dy_up);
        
        float dx_left = HasLeft(current_neighbor) ? costs_[Left(current_neighbor)] : INF;
        float dx_right = HasRight(current_neighbor) ? costs_[Right(current_neighbor)] : INF;
        float val_dx = std::min(dx_left, dx_right);

        // Get weight for this cell
        float weight = weights_[current_neighbor];
        float neighbor_weight = map_res_ * weight;

        // FMM update formula
        float discriminant = 2.0f * std::pow(neighbor_weight, 2) - std::pow(val_dx - val_dy, 2);
        float new_cost;
        
        if (discriminant >= 0) {
          new_cost = (val_dx + val_dy + std::sqrt(discriminant)) / 2.0f;
        } else {
          new_cost = std::min(val_dx + neighbor_weight, val_dy + neighbor_weight);
        }

        // Update cost and parent
        if (new_cost < costs_[current_neighbor]) {
          costs_[current_neighbor] = new_cost;
          paths_[current_neighbor] = current.idx; // Store parent for backtracking
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

  // Discrete parent backtracking from start to goal
  int current_idx = start_;
  Index current = FoldIndex(current_idx);
  
  Point current_position = IndexToPoint(current);
  path_world_.push_back(current_position);

  int max_steps = height_ * width_;
  int step_count = 0;

  // Follow parent pointers from start to goal
  while (current_idx != goal_ && step_count < max_steps) {
    int parent_idx = paths_[current_idx];
    
    // Safety check for unreachable cells
    if (parent_idx < 0) {
      std::cerr << "FastMarching: Path extraction failed - no parent at idx " << current_idx << std::endl;
      return false;
    }

    current_idx = parent_idx;
    current = FoldIndex(current_idx);
    current_position = IndexToPoint(current);
    path_world_.push_back(current_position);
    
    step_count++;
  }

  if (current_idx != goal_) {
    std::cerr << "FastMarching: Path extraction failed - did not reach goal" << std::endl;
    return false;
  }

  // Path is built from start to goal, no need to reverse
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

float FastMarching::Distance(const Point& p1, const Point& p2) {
  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  float dist = sqrt(dx * dx + dy * dy);
  return dist;
}

}
}
