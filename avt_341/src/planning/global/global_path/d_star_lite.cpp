#include "avt_341/planning/global/d_star_lite.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace avt_341 {
namespace planning {

DStarLite::DStarLite(std::shared_ptr<avt_341::visualization::VisualizerBase> visualizer,
                     float w_distance,
                     float w_occupancy,
                     float w_segmentation,
                     bool search_diagonals,
                     int los_max_iterations,
                     bool los_break_on_first)
  : Astar(visualizer, w_distance, w_occupancy, w_segmentation, search_diagonals, los_max_iterations, los_break_on_first) {
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
  rhs_[s_goal_] = 0.0f;
  open_list_.insert({s_goal_, CalculateKey(s_goal_)});
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

  // Remove u from open list if it's there
  auto it = std::find_if(open_list_.begin(), open_list_.end(), [u](const DStarNode& node) {
    return node.index == u;
  });
  if (it != open_list_.end()) {
    open_list_.erase(it);
  }

  if (g_[u] != rhs_[u]) {
    open_list_.insert({u, CalculateKey(u)});
  }
}

void DStarLite::ComputeShortestPath() {
  while (!open_list_.empty() && (open_list_.begin()->key < CalculateKey(s_start_) || rhs_[s_start_] != g_[s_start_])) {
    DStarNode current = *open_list_.begin();
    open_list_.erase(open_list_.begin());

    int u = current.index;
    DStarLiteKey k_old = current.key;
    DStarLiteKey k_new = CalculateKey(u);

    if (k_old < k_new) {
      open_list_.insert({u, k_new});
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
  // Use weights_ as cost. If weights_[v] is very large, it's an obstacle.
  // We should check map_[ix][iy] for high occupancy if weights_ doesn't explicitly store "is obstacle".
  // Astar uses weights_ for cost, and weights_ are initialized to include w_occupancy * val_height.
  // If map_[v] >= 100, we treat it as infinite cost.
  Index iv = FoldIndex(v);
  if (map_[iv.ix][iv.iy] >= 100.0f) return INF;
  
  // Astar weight model
  float move_cost = weights_[v];
  
  // Account for diagonal distance
  Index iu = FoldIndex(u);
  if (iu.ix != iv.ix && iu.iy != iv.iy) {
    // Both iv.ix and iv.iy are different, means diagonal move
    // This assumes only 1-step neighbors are checked
    move_cost *= 1.41421356f;
  }
  
  return move_cost;
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

std::vector<Point> DStarLite::PlanPath(avt_341::msg::OccupancyGrid* grid,
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

  int n = 0;
  for (int iy = 0; iy < height_; iy++) {
    for (int ix = 0; ix < width_; ix++) {
      double x_grid = grid->info.origin.position.x + ix * grid->info.resolution;
      double y_grid = grid->info.origin.position.y + iy * grid->info.resolution;
      SetMapValue({ix, iy}, grid->data[n], 100 - GetGridValue(segmentation_grid, x_grid, y_grid));
      n++;
    }
  }

  // For this global planner, we re-initialize D* Lite every time a new map comes
  // because we don't track which cells changed specifically from the caller.
  // D* Lite behaves like reverse A* in this case.
  Initialize();
  bool solved = Solve();

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
} // namespace avt_341
