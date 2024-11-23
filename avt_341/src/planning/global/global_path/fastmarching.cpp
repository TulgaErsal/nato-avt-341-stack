#include "avt_341/planning/global/fastmarching.h"
#include "avt_341/planning/global/astar_cell.h"

#include <queue>
#include <chrono>

namespace avt_341 {
namespace planning {

bool FastMarching::Solve() {
  auto t_1 = std::chrono::system_clock::now();
  std::fill(paths_.begin(), paths_.end(), -1);

  AStarCell start_node(start_, 0.);
  auto start_position = IndexToPoint(FoldIndex(start_));
  AStarCell goal_node(goal_, 0.);
  auto goal_position = IndexToPoint(FoldIndex(goal_));
//  std::cout << "Start " << start_node.idx << " (" << start_position[0] << ", " << start_position[1] << "), dist: "
//            << "\n";
//  std::cout << "Goal: " << goal_node.idx << " (" << goal_position[0] << ", " << goal_position[1] << "), dist: " << "\n";

  float* costs = new float[height_ * width_];
  for (int i = 0; i < height_ * width_; ++i) {
    costs[i] = INF;
  }
  costs[start_] = 0.;

  std::priority_queue<AStarCell> nodes_to_visit;
  nodes_to_visit.push(start_node);
  const int N_adj = search_diagonals_ ? 8 : 4;
  int* neighbors = new int[N_adj];

  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    // .top() doesn't actually remove the node
    AStarCell current = nodes_to_visit.top();

    if (current == goal_node) {
//      std::cout << "got to goal node: " << current.idx << "\n";
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
      if (current_neighbor >= 0 && costs[current_neighbor] == INF) {

        // For each new cell to explore, find the cost_diff in both x and y directions (dx and dy)
        float dy_down = HasDown(current_neighbor) ? costs[Down(current_neighbor)] : INF;
        float dy_up = HasUp(current_neighbor) ? costs[Up(current_neighbor)] : INF;
        float dy = std::min(dy_down, dy_up);
        float dx_left = HasLeft(current_neighbor) ? costs[Left(current_neighbor)] : INF;
        float dx_right = HasRight(current_neighbor) ? costs[Right(current_neighbor)] : INF;
        float dx = std::min(dx_left, dx_right);

        // Calculate new cost based on dx, dy and the weights
        float neighbor_weight = map_res_ * weights_[current_neighbor];
        float discriminant = 2.0 * std::pow(neighbor_weight, 2) - std::pow(dx - dy, 2);
        float new_cost;
        if (discriminant >= 0) {
          new_cost = (dx + dy + std::sqrt(discriminant)) / 2.0f;
        } else {
          new_cost = std::min(dx + neighbor_weight, dy + neighbor_weight);
        }

        // Update cost grid neighbor cells with new costs of neighbors paths with lower expected cost are explored first
        costs[neighbors[i]] = new_cost;
        nodes_to_visit.emplace(neighbors[i], new_cost);
        paths_[neighbors[i]] = current.idx;
      }
    }
  }
  auto t_now = std::chrono::system_clock::now();
  auto calc_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_1);
  std::cout << "FM time: " << calc_duration_ms.count() << " ms\n";

  if (solution_found) {
    solution_found = ExtractPath(costs);
  }

  delete[] costs;
  delete[] neighbors;

  return solution_found;
}

bool FastMarching::ExtractPath(float* costs) {
  path_.clear();
  path_world_.clear();

  int step_number = 0;
  float step_size = map_res_;
  int current_idx = goal_;
  auto current_position = IndexToPoint(FoldIndex(current_idx));
  path_world_.push_back(current_position);

  auto start_position = IndexToPoint(FoldIndex(start_));
//  std::cout << "(" << start_position[0] << ", " << start_position[1] << "), dist: " << "\n";

  while (current_idx != start_) {
    if (step_number > 2000) {
      return false;
    }
    auto gradient = get_gradient(current_position, costs);
    auto gradient_normalized = Normalize(gradient);
    current_position[0] += -gradient_normalized[0] * step_size;
    current_position[1] += -gradient_normalized[1] * step_size;
    current_idx = FlattenIndex(PointToIndex(current_position[0], current_position[1]));

    auto current_dist = Distance(current_position, start_position);
//    std::cout << "(" << current_position[0] << ", " << current_position[1] << "),  \t dist: " << current_dist
//              << "\t Grad: [" << gradient_normalized[0] << ", " << gradient_normalized[1] << "]\n";

    step_number++;
    path_world_.push_back(current_position);
  }

  std::reverse(path_.begin(), path_.end());
  std::reverse(path_world_.begin(), path_world_.end());

  return true;
}

std::vector<float> FastMarching::get_gradient(const std::vector<float>& position, const float* costs) {
  float pos_x_right = position[0] + 0.5 * map_res_;
  float pos_x_left = position[0] - 0.5 * map_res_;
  float pos_y_up = position[1] + 0.5 * map_res_;
  float pos_y_down = position[1] - 0.5 * map_res_;
  auto idx_x_right = PointToIndex(pos_x_right, position[1]);
  auto idx_x_left = PointToIndex(pos_x_left, position[1]);
  auto idx_y_up = PointToIndex(position[0], pos_y_up);
  auto idx_y_down = PointToIndex(position[0], pos_y_down);
  float cost_x_right = IsInMap(idx_x_right) ? costs[FlattenIndex(idx_x_right)] : INF;
  float cost_x_left = IsInMap(idx_x_left) ? costs[FlattenIndex(idx_x_left)] : INF;
  float cost_y_up = IsInMap(idx_y_up) ? costs[FlattenIndex(idx_y_up)] : INF;
  float cost_y_down = IsInMap(idx_y_down) ? costs[FlattenIndex(idx_y_down)] : INF;
  return {cost_x_right - cost_x_left, cost_y_up - cost_y_down};
}

std::vector<float> FastMarching::Normalize(const std::vector<float>& v) {
  if (std::isfinite(v[0]) && std::isfinite(v[1])) {
    float mag = sqrt(v[0] * v[0] + v[1] * v[1]);
    return {v[0] / mag, v[1] / mag};
  }
  float x{0}, y{0};
  if (v[0] == INF) {
    x = 1;
  } else if (v[0] == -INF) {
    x = -1;
  }
  if (v[1] == INF) {
    y = 1;
  } else if (v[1] == -INF) {
    y = -1;
  }
  return Normalize({x, y});
}

float FastMarching::Distance(const std::vector<float>& v1, const std::vector<float>& v2) {
  float dx = v1[0] - v2[0];
  float dy = v1[1] - v2[1];
  float dist = sqrt(dx * dx + dy * dy);
  return dist;
}

}
}
