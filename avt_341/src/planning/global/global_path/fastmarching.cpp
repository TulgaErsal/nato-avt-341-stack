#include "avt_341/planning/global/fastmarching.h"
#include "avt_341/planning/global/astar_cell.h"

#include <queue>
#include <chrono>

namespace avt_341 {
namespace planning {

bool FastMarching::Solve() {
  auto t_1 = std::chrono::system_clock::now();
  std::fill(paths_.begin(), paths_.end(), -1);

//  AStarCell start_node(start_, 0.);
  AStarCell start_node(goal_, 0.);
//  AStarCell goal_node(goal_, 0.);
  AStarCell goal_node(start_, 0.);
//  auto start_position = IndexToPoint(FoldIndex(start_));
//  auto goal_position = IndexToPoint(FoldIndex(goal_));
//  std::cout << "Start " << start_node.idx << " (" << start_position[0] << ", " << start_position[1] << "), dist: "
//            << "\n";
//  std::cout << "Goal: " << goal_node.idx << " (" << goal_position[0] << ", " << goal_position[1] << "), dist: " << "\n";

  delete[] costs_;
  costs_ = new float[height_ * width_];
  for (int i = 0; i < height_ * width_; ++i) {
    costs_[i] = INF;
  }
  costs_[start_node.idx] = 0.;

  std::priority_queue<AStarCell> nodes_to_visit;
  nodes_to_visit.push(start_node);
  const int N_adj = search_diagonals_ ? 8 : 4;
  int* neighbors = new int[N_adj];

  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    // .top() doesn't actually remove the node
    AStarCell current = nodes_to_visit.top();

    if (current == goal_node) {
      solution_found = true;
      break;
    }

    nodes_to_visit.pop();

    // check bounds and find up to four neighbors
    neighbors[0] = HasDown(current.idx) ? Down(current.idx) : -1;
    neighbors[1] = HasLeft(current.idx) ? Left(current.idx) : -1;
    neighbors[2] = HasUp(current.idx) ? Up(current.idx) : -1;
    neighbors[3] = HasRight(current.idx) ? Right(current.idx) : -1;

    for (int i = 0; i < N_adj; ++i) {
      int current_neighbor = neighbors[i];
      if (current_neighbor >= 0 && costs_[current_neighbor] == INF) {

        // For each new cell to explore, find the cost_diff in both x and y directions (dx and dy)
        float dy_down = HasDown(current_neighbor) ? costs_[Down(current_neighbor)] : INF;
        float dy_up = HasUp(current_neighbor) ? costs_[Up(current_neighbor)] : INF;
        float dy = std::min(dy_down, dy_up);
        float dx_left = HasLeft(current_neighbor) ? costs_[Left(current_neighbor)] : INF;
        float dx_right = HasRight(current_neighbor) ? costs_[Right(current_neighbor)] : INF;
        float dx = std::min(dx_left, dx_right);

        // Calculate new cost based on dx, dy and the weights
        int weight = weights_[current_neighbor];
        // Set unknown value to weight 2, slightly higher than perfect traversability
        if (weight == 51) {
          weight = 2;
        }
        float neighbor_weight = map_res_ * weight;
        float discriminant = 2.0 * std::pow(neighbor_weight, 2) - std::pow(dx - dy, 2);
        float new_cost;
        if (discriminant >= 0) {
          new_cost = (dx + dy + std::sqrt(discriminant)) / 2.0f;
        } else {
          new_cost = std::min(dx + neighbor_weight, dy + neighbor_weight);
        }

        // Update cost grid neighbor cells with new costs of neighbors paths with lower expected cost are explored first
        costs_[neighbors[i]] = new_cost;
        nodes_to_visit.emplace(neighbors[i], new_cost);
        paths_[neighbors[i]] = current.idx;
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

float FastMarching::Distance(const Point& p1, const Point& p2) {
  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  float dist = sqrt(dx * dx + dy * dy);
  return dist;
}

}
}
