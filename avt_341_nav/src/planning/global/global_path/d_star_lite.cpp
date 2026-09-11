#include "avt_341_nav/planning/global/d_star_lite.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace avt_341_nav {
namespace planning {

DStarLite::DStarLite(float w_distance,
                     float w_occupancy,
                     float w_segmentation,
                     bool search_diagonals,
                     int los_max_iterations,
                     bool los_break_on_first)
  : Astar(w_distance, w_occupancy, w_segmentation, search_diagonals, los_max_iterations, los_break_on_first) {
  km_ = 0.0f;
  s_start_ = -1;
  s_goal_ = -1;
  s_last_ = -1;
}

DStarLite::~DStarLite() {
}

void DStarLite::Initialize() {
  open_list_.clear();
  km_ = 0.0f;
  g_.assign(height_ * width_, INF);
  rhs_.assign(height_ * width_, INF);
  open_keys_.assign(height_ * width_, {INF, INF});
  in_open_.assign(height_ * width_, false);
  rhs_[s_goal_] = 0.0f;
  DStarLiteKey k = CalculateKey(s_goal_);
  open_list_.insert({s_goal_, k});
  open_keys_[s_goal_] = k;
  in_open_[s_goal_] = true;
  s_last_ = s_start_;
}

DStarLiteKey DStarLite::CalculateKey(int s) {
  float min_g_rhs = std::min(g_[s], rhs_[s]);
  float k1 = min_g_rhs + HeuristicDStar(s_start_, s) + km_;
  float k2 = min_g_rhs;
  return {k1, k2};
}

void DStarLite::UpdateVertex(int u) {
  if (u != s_goal_) {
    float min_rhs = INF;
    std::vector<int> neighbors = GetNeighbors(u);
    for (int v : neighbors) {
      float cost = Cost(u, v);
      if (cost < INF && g_[v] < INF) {
        min_rhs = std::min(min_rhs, cost + g_[v]);
      }
    }
    rhs_[u] = min_rhs;
  }

  if (in_open_[u]) {
    open_list_.erase({u, open_keys_[u]});
    in_open_[u] = false;
  }

  if (g_[u] != rhs_[u]) {
    DStarLiteKey k = CalculateKey(u);
    open_list_.insert({u, k});
    open_keys_[u] = k;
    in_open_[u] = true;
  }
}

void DStarLite::ComputeShortestPath() {
  while (!open_list_.empty() && (open_list_.begin()->key < CalculateKey(s_start_) || rhs_[s_start_] != g_[s_start_])) {
    DStarNode current = *open_list_.begin();
    open_list_.erase(open_list_.begin());
    in_open_[current.index] = false;

    int u = current.index;
    DStarLiteKey k_old = current.key;
    DStarLiteKey k_new = CalculateKey(u);

    if (k_old < k_new) {
      open_list_.insert({u, k_new});
      open_keys_[u] = k_new;
      in_open_[u] = true;
    } else if (g_[u] > rhs_[u]) {
      g_[u] = rhs_[u];
      std::vector<int> neighbors = GetNeighbors(u);
      for (int s : neighbors) {
        UpdateVertex(s);
      }
    } else {
      g_[u] = INF;
      std::vector<int> neighbors = GetNeighbors(u);
      neighbors.push_back(u); // Update u itself as well
      for (int s : neighbors) {
        UpdateVertex(s);
      }
    }
  }
}

float DStarLite::Cost(int u, int v) {
  // Check the base Astar weights_. If weights_[v] is >= INF, it's an obstacle.
  if (weights_[v] >= INF) return INF;
  
  // Also check map occupancy as a fallback
  Index iv = FoldIndex(v);
  if (map_[iv.ix][iv.iy] >= 100.0f) return INF;
  
  // Calculate Euclidean distance (grid steps)
  Index iu = FoldIndex(u);
  float dx = std::abs((float)iu.ix - (float)iv.ix);
  float dy = std::abs((float)iu.iy - (float)iv.iy);
  float step_dist = (dx > 0 && dy > 0) ? 1.41421356f : 1.0f;
  
  // Total cost = (base distance cost + terrain penalty) * step distance
  return (w_distance_ + weights_[v]) * step_dist;
}

float DStarLite::HeuristicDStar(int u, int v) {
  Index iu = FoldIndex(u);
  Index iv = FoldIndex(v);
  float dx = std::abs((float)iu.ix - (float)iv.ix);
  float dy = std::abs((float)iu.iy - (float)iv.iy);
  
  // Use Chebychev or octile distance for grid
  if (search_diagonals_) {
    // Octile distance
    return w_distance_ * (std::max(dx, dy) + (1.41421356f - 1.0f) * std::min(dx, dy));
  } else {
    // Manhattan distance
    return w_distance_ * (dx + dy);
  }
}

std::vector<int> DStarLite::GetNeighbors(int u) {
  std::vector<int> neighbors;
  if (HasUp(u)) neighbors.push_back(Up(u));
  if (HasDown(u)) neighbors.push_back(Down(u));
  if (HasLeft(u)) neighbors.push_back(Left(u));
  if (HasRight(u)) neighbors.push_back(Right(u));
  if (search_diagonals_) {
    if (HasUp(u) && HasLeft(u)) neighbors.push_back(UpLeft(u));
    if (HasUp(u) && HasRight(u)) neighbors.push_back(UpRight(u));
    if (HasDown(u) && HasLeft(u)) neighbors.push_back(DownLeft(u));
    if (HasDown(u) && HasRight(u)) neighbors.push_back(DownRight(u));
  }
  return neighbors;
}

std::vector<Point> DStarLite::PlanPath(nav_msgs::msg::OccupancyGrid* grid,
                                       nav_msgs::msg::OccupancyGrid* segmentation_grid,
                                       Point goal,
                                       Point position) {
  if (grid->info.height <= 0 || grid->info.width <= 0) {
    return path_world_;
  }

  SetCornerCoords(grid->info.origin.position.x, grid->info.origin.position.y);
  SetMapRes(grid->info.resolution);

  Index goal_idx = PointToIndex(goal);
  Index start_idx = PointToIndex(position);

  // Bounds check
  auto clamp_idx = [&](Index& idx) {
    if (idx.ix < 0) idx.ix = 0;
    if (idx.ix >= width_) idx.ix = width_ - 1;
    if (idx.iy < 0) idx.iy = 0;
    if (idx.iy >= height_) idx.iy = height_ - 1;
  };

  // Pre-allocation if size changed
  bool size_changed = (height_ != grid->info.height || width_ != grid->info.width);
  if (size_changed) {
    AllocateMap(grid->info.height, grid->info.width, 0);
  }
  
  clamp_idx(start_idx);
  clamp_idx(goal_idx);

  s_start_ = FlattenIndex(start_idx);
  s_goal_ = FlattenIndex(goal_idx);
  
  // Set goal and start in Astar base (for visualization/utilities)
  SetGoal(goal_idx);
  SetStart(start_idx);

  {
    auto recording = RecordSection(planner_sections::GRID_INGEST);
    int n = 0;
    for (int iy = 0; iy < height_; iy++) {
      for (int ix = 0; ix < width_; ix++) {
        double x_grid = grid->info.origin.position.x + ix * grid->info.resolution;
        double y_grid = grid->info.origin.position.y + iy * grid->info.resolution;
        SetMapValue({ix, iy}, grid->data[n], GetGridValue(segmentation_grid, x_grid, y_grid));
        n++;
      }
    }
  }

  // For this global planner, we re-initialize D* Lite every time a new map comes
  // because we don't track which cells changed specifically from the caller.
  // D* Lite behaves like reverse A* in this case.
  {
    auto recording = RecordSection(planner_sections::SOLVE);
    Initialize();
    Solve();
  }

  return path_world_;
}

bool DStarLite::Solve() {
  ComputeShortestPath();
  
  if (g_[s_start_] >= INF) {
    return false;
  }
  
  return ExtractPath();
}

bool DStarLite::ExtractPath() {
  path_.clear();
  path_world_.clear();
  
  int curr = s_start_;
  path_.push_back(FoldIndex(curr));
  
  int max_iters = height_ * width_;
  int iters = 0;
  
  while (curr != s_goal_ && iters < max_iters) {
    std::vector<int> neighbors = GetNeighbors(curr);
    float min_cost = INF;
    int next_node = -1;
    
    for (int v : neighbors) {
      float cost = Cost(curr, v) + g_[v];
      if (cost < min_cost) {
        min_cost = cost;
        next_node = v;
      }
    }
    
    if (next_node == -1) break;
    
    curr = next_node;
    path_.push_back(FoldIndex(curr));
    iters++;
  }
  
  if (curr != s_goal_) return false;
  
  // Path world conversion (simple version)
  for (auto& idx : path_) {
    path_world_.push_back(IndexToPoint(idx));
  }
  
  return true;
}

} // namespace planning
} // namespace avt_341_nav
