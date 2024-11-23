#include "avt_341/planning/global/fastmarching.h"
#include "avt_341/planning/global/astar_cell.h"

#include <queue>
#include <chrono>

namespace avt_341 {
namespace planning {


bool FastMarching::Solve() {
  auto t_1 = std::chrono::system_clock::now();
  std::fill(paths_.begin(), paths_.end(),-1);

  const float INF = std::numeric_limits<float>::infinity();

  AStarCell start_node(start_, 0.);
  AStarCell goal_node(goal_, 0.);

  float* costs = new float[height_ * width_];
  for (int i = 0; i < height_ * width_; ++i)
    costs[i] = INF;
  costs[start_] = 0.;

  std::priority_queue<AStarCell> nodes_to_visit;
  nodes_to_visit.push(start_node);
  const int N_adj = search_diagonals_ ? 8 : 4;
  int* neighbors = new int[N_adj];

  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    // .top() doesn't actually remove the node
    AStarCell cur = nodes_to_visit.top();

    if (cur == goal_node) {
      solution_found = true;
      break;
    }

    nodes_to_visit.pop();

    // check bounds and find up to four neighbors
    bool has_down = (cur.idx / width_ > 0);
    bool has_left = (cur.idx % width_ > 0);
    bool has_up = (cur.idx / width_ + 1 < height_);
    bool has_right = (cur.idx % width_ + 1 < width_);
    neighbors[0] = has_down ? (cur.idx - width_) : -1;
    neighbors[1] = has_left ? (cur.idx - 1) : -1;
    neighbors[2] = has_up ? (cur.idx + width_) : -1;
    neighbors[3] = has_right ? (cur.idx + 1) : -1;

    for (int i = 0; i < N_adj; ++i) {
      int current_neighbor = neighbors[i];
      if (current_neighbor >= 0 && costs[current_neighbor] == INF) {

        // For each new cell to explore, find the cost_diff in both x and y directions (dx and dy)
        bool neighbor_has_down = (current_neighbor / width_ > 0);
        bool neighbor_has_left = (current_neighbor % width_ > 0);
        bool neighbor_has_up = (current_neighbor / width_ + 1 < height_);
        bool neighbor_has_right = (current_neighbor % width_ + 1 < width_);
        float dy_down = neighbor_has_down ? costs[(current_neighbor - width_)] : INF;
        float dy_up = neighbor_has_up ? costs[(current_neighbor + width_)] : INF;
        float dy = std::min(dy_down, dy_up);
        float dx_left = neighbor_has_left ? costs[(current_neighbor - 1)] : INF;
        float dx_right = neighbor_has_right ? costs[(current_neighbor + 1)] : INF;
        float dx = std::min(dx_left, dx_right);

        // Calculate new cost based on dx, dy and the weights
        float neighbor_weight = weights_[neighbors[i]];
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
        paths_[neighbors[i]] = cur.idx;
      }
    }
  }
  auto t_now = std::chrono::system_clock::now();
  auto calc_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_1);
  std::cout << "FM time: " << calc_duration_ms.count() << "\n";

  delete[] costs;
  delete[] neighbors;

  if (solution_found){
    solution_found = ExtractPath();
  }
  return solution_found;
}

}
}
