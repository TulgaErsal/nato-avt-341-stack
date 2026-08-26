#include <queue>
#include <limits>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>
// project includes
#include "avt_341_nav/planning/global/astar.h"
#include "avt_341_nav/planning/global/astar_cell.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
//#include "avt_341/planning/global/dubins_smoothing.h"

namespace avt_341_nav {
namespace planning {

Astar::Astar(float w_distance,
             float w_occupancy,
             float w_segmentation,
             bool search_diagonals,
             int los_max_iterations,
             bool los_break_on_first,
             float no_segmentation_data_cost)
  : w_distance_(w_distance),
    w_occupancy_(w_occupancy),
    w_segmentation_(w_segmentation),
    no_segmentation_data_cost_(no_segmentation_data_cost),
    search_diagonals_(search_diagonals),
    los_max_iterations_(los_max_iterations),
    los_break_on_first_(los_break_on_first) {
  dfac_ = 0;
  width_ = 0;
  height_ = 0;
}

Astar::~Astar() {

}

Index Astar::FoldIndex(int n) const {
  int ix = n % width_;
  int iy = (int) floor((1.0 * n) / (1.0 * width_));
  return {ix, iy};
}

void Astar::AllocateMap(int h, int w, int init_val) {
  height_ = h;
  width_ = w;
  weights_.clear();
  weights_.resize(height_ * width_, init_val);
  paths_.clear();
  paths_.resize(height_ * width_, -1);
  for (int i = 0; i < width_; i++) {
    for (int j = 0; j < height_; j++) {
      if (i == 0 || j == 0 ||
      i == (width_ - 1) ||
      j == (height_ - 1)) {
        int n = FlattenIndex(i, j);
        weights_[n] = 0;
      }
    }
  }

  // resize(width_, column) would leave pre-existing columns at their old height, which
  // writes out of bounds when the grid dimensions change between calls (e.g. costmap crop)
  map_.resize(width_);
  for (auto& column : map_) {
    column.assign(height_, 0.0f);
  }
}

void Astar::SetMapValue(const Index& index, int val_height, int val_seg) {
  weights_[FlattenIndex(index)] = w_occupancy_ * static_cast<float>(val_height)
    + w_segmentation_ * static_cast<float>(val_seg);
  map_[index.ix][index.iy] = (float) val_height;   // only used for obstacles (dilation, line of sight)
}

bool Astar::LineOfSight(const Index& i0, const Index& i1) {
  // see: https://news.movel.ai/theta-star/

  int x0 = i0.ix;
  int y0 = i0.iy;
  int x1 = i1.ix;
  int y1 = i1.iy;

  int dx = i1.ix - i0.ix;
  int dy = i1.iy - i0.iy;

  int f = 0;

  int sx, sy;
  if (dx < 0) {
    dx = -dx;
    sx = -1;
  } else {
    sx = 1;
  }

  if (dy < 0) {
    dy = -dy;
    sy = -1;
  } else {
    sy = 1;
  }

  if (dx >= dy) {
    while (x0 != x1) {
      f = f + dy;
      if (f >= dx) {
        if (map_[x0 + ((sx - 1) / 2)][y0 + ((sy - 1) / 2)] > 0) {
          return false;
        }
        y0 = y0 + sy;
        f = f - dx;
      }
      if (f != 0 && map_[x0 + ((sx - 1) / 2)][y0 + ((sy - 1) / 2)] > 0) {
        return false;
      }
      if (dy == 0 && map_[x0 + ((sx - 1) / 2)][y0] > 0 && map_[x0 + ((sx - 1) / 2)][y0 - 1] > 0) {
        return false;
      }
      x0 = x0 + sx;
    }
  } else {
    while (y0 != y1) {
      f = f + dx;
      if (f >= dy) {
        if (map_[x0 + ((sx - 1) / 2)][y0 + ((sy - 1) / 2)] > 0) {
          return false;
        }
        x0 = x0 + sx;
        f = f - dy;
      }
      if (f != 0 && map_[x0 + ((sx - 1) / 2)][y0 + ((sy - 1) / 2)] > 0) {
        return false;
      }
      if (dx == 0 && map_[x0][y0 + ((sy - 1) / 2)] > 0 && map_[x0 - 1][y0 + ((sy - 1) / 2)] > 0) {
        return false;
      }
      y0 = y0 + sy;
    }
  }

  return true;
}

// For path of length n
void Astar::PostSmoothing(const std::vector<Index>& in_path, std::vector<Index>& out_path) {
  if (in_path.size() > 2) {
    int k = 0;
    out_path.push_back(in_path[0]);
    if (los_break_on_first_) {
      for (int i = 1; i < in_path.size() - 1; i++) {
        if (!LineOfSight(out_path[k], in_path[i + 1])) {
          k++;
          out_path.push_back(in_path[i]);
        }
      }
      out_path.push_back(in_path.back());
    } else {
      // Find last in line of sight
      while (k < in_path.size() - 1) {
        int i = in_path.size() - 1;
        while (i > k + 1 && !LineOfSight(in_path[k], in_path[i])) {
          i--;
        }
        out_path.push_back(in_path[i]);
        k = i;
      }
    }
  } else {
    out_path = in_path;
  }
}

float Astar::Heuristic(const Index& i0, const Index& i1) const {
  const int dx = std::abs(i1.ix - i0.ix);
  const int dy = std::abs(i1.iy - i0.iy);

  constexpr float D  = 1.0f;
  constexpr float D2 = 1.41421356237f; // sqrt(2)

  float h;
  if (search_diagonals_) {
    const int dmin = std::min(dx, dy);
    const int dmax = std::max(dx, dy);
    // Octile distance: (dmax-dmin)*1 + dmin*sqrt(2)
    h = D * float(dmax - dmin) + D2 * float(dmin);
  } else {
    // Manhattan distance
    h = D * float(dx + dy);
  }

  return w_distance_ * h;
}

bool Astar::Solve() {
  std::fill(paths_.begin(), paths_.end(), -1);

  AStarCell start_node(start_, 0.0f, Heuristic(FoldIndex(start_), FoldIndex(goal_)));
  AStarCell goal_node(goal_, 0.0f, 0.0f);

  auto* costs = new float[height_ * width_];
  for (int i = 0; i < height_ * width_; ++i) {
    costs[i] = INF;
  }
  costs[start_] = 0.;

  std::vector<bool> closed(height_ * width_, false);
  std::priority_queue<AStarCell> nodes_to_visit;
  nodes_to_visit.push(start_node);
  const int N_adj = search_diagonals_ ? 8 : 4;
  int* neighbors = new int[N_adj];

  constexpr float kCardinalStep = 1.0f;
  constexpr float kDiagonalStep = 1.41421356237f; // sqrt(2)

  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    // .top() doesn't actually remove the node
    AStarCell current = nodes_to_visit.top();

    if (current == goal_node) {
      solution_found = true;
      break;
    }
    
    nodes_to_visit.pop();
    if (closed[current.idx]) continue;
    closed[current.idx] = true;
    if (current.g > costs[current.idx]) continue;

    // check bounds and find up to eight neighbors
    neighbors[0] = HasDown(current.idx) ? Down(current.idx) : -1;
    neighbors[1] = HasLeft(current.idx) ? Left(current.idx) : -1;
    neighbors[2] = HasUp(current.idx) ? Up(current.idx) : -1;
    neighbors[3] = HasRight(current.idx) ? Right(current.idx) : -1;
    if (search_diagonals_) {
      neighbors[4] = HasDown(current.idx) && HasLeft(current.idx) ? DownLeft(current.idx) : -1;
      neighbors[5] = HasDown(current.idx) && HasRight(current.idx) ? DownRight(current.idx) : -1;
      neighbors[6] = HasUp(current.idx) && HasLeft(current.idx) ? UpLeft(current.idx) : -1;
      neighbors[7] = HasUp(current.idx) && HasRight(current.idx) ? UpRight(current.idx) : -1;
    }
    for (int i = 0; i < N_adj; ++i) {
      const int nb = neighbors[i];
      if (nb < 0) continue;

      const bool is_diag = search_diagonals_ && (i >= 4);
      const float step_cost = is_diag ? kDiagonalStep : kCardinalStep;

      // Diagonal moves cost more than cardinal moves.
      float g_new = costs[current.idx] + step_cost * w_distance_ + weights_[nb];

      if (g_new < costs[nb]) {
        costs[nb] = g_new;
        float f_new = g_new + Heuristic(FoldIndex(nb), FoldIndex(goal_));
        nodes_to_visit.emplace(nb, g_new, f_new);
        paths_[nb] = current.idx;
      }
    }
  }

  delete[] costs;
  delete[] neighbors;

  if (solution_found) {
    solution_found = ExtractPath();
  }
  return solution_found;
}

bool Astar::ExtractPath() {
  path_.clear();
  path_world_.clear();
  int path_idx = goal_;
  Point point;
  while (path_idx != start_) {
    Index c = FoldIndex(path_idx);
    path_.push_back(c);
    path_idx = paths_[path_idx];
  }

  //smooth path out
  std::vector<Index> path_smoothed;
  PostSmoothing(path_, path_smoothed);
  for (int k = 1; k < los_max_iterations_; k++) {
    std::vector<Index> path_smoothed_it;
    PostSmoothing(path_smoothed, path_smoothed_it);
    // pre-emptive break if unchanged
    if (path_smoothed.size() == path_smoothed_it.size()
      && std::equal(path_smoothed.begin(), path_smoothed.end(), path_smoothed_it.begin())) {
      break;
    }
    path_smoothed = path_smoothed_it;
  }

  // put the smoothed path in world coordinates
  path_world_pre_fill_.clear();
  for (auto index: path_smoothed) {
    point = IndexToPoint(index);
    path_world_pre_fill_.push_back(point);
  }

  if (path_world_pre_fill_.empty()) return false;

  // the smoothed path may have much fewer points. Fill in the missing parts
  std::vector<Point> filled_in_path_world;
  for (int i = 0; i < path_world_pre_fill_.size() - 1; i++) {
    float px = path_world_pre_fill_[i].x;
    float py = path_world_pre_fill_[i].y;
    float dx = path_world_pre_fill_[i + 1].x - px;
    float dy = path_world_pre_fill_[i + 1].y - py;
    double d = sqrt(dx * dx + dy * dy);
    double ltx = map_res_ * dx / d;
    double lty = map_res_ * dy / d;
    while (d > 2.0 * map_res_) {
      filled_in_path_world.push_back({px, py});
      px += ltx;
      py += lty;
      dx = path_world_pre_fill_[i + 1].x - px;
      dy = path_world_pre_fill_[i + 1].y - py;
      d = sqrt(dx * dx + dy * dy);
    }
  }
  path_world_ = filled_in_path_world;

  std::reverse(path_.begin(), path_.end());
  std::reverse(path_world_.begin(), path_world_.end());

  /*if (dubins_smoothing_) {
    DubinsSmoothing dubins(path_world_);
    dubins.SmoothPath(dubins_radius_, map_res_/5.0f);
    path_world_ = dubins.GetPath();
  }*/

  return true;
}

int Astar::GetGridValue(nav_msgs::msg::OccupancyGrid* grid, double x, double y) const {
  int seg_val = static_cast<int>(no_segmentation_data_cost_);
  if (x >= grid->info.origin.position.x && x < grid->info.origin.position.x + grid->info.width * grid->info.resolution
    && y >= grid->info.origin.position.y
    && y < grid->info.origin.position.y + grid->info.height * grid->info.resolution) {
    int xj = (x - grid->info.origin.position.x) / grid->info.resolution;
    int yj = (y - grid->info.origin.position.y) / grid->info.resolution;
    int m = xj + yj * grid->info.width;
    seg_val = grid->data[m];
  }
  return seg_val;
}

std::vector<Point> Astar::PlanPath(nav_msgs::msg::OccupancyGrid* grid,
                                   nav_msgs::msg::OccupancyGrid* grid_segmentation,
                                   Point goal,
                                   Point position) {
  if (grid->info.height <= 0 || grid->info.width <= 0) {
    return path_world_;
  }

  bool has_segmentation = grid_segmentation->info.height > 0 && grid_segmentation->info.width > 0;
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
  Point goal_r;
  goal_r = GetCurrentGoal();
  {
    auto recording = RecordSection(planner_sections::GRID_INGEST);
    int n = 0;
    for (int iy = 0; iy < height_; iy++) {
      for (int ix = 0; ix < width_; ix++) {
        double x_grid = grid->info.origin.position.x + ix * grid->info.resolution;
        double y_grid = grid->info.origin.position.y + iy * grid->info.resolution;
        SetMapValue({ix, iy}, grid->data[n], GetGridValue(grid_segmentation, x_grid, y_grid));
        n++;
      }
    }
  }

  // dilate
  if (dfac_ > 0) {
    auto recording = RecordSection(planner_sections::DILATION);
    std::vector<std::vector<float> > dmap = map_;
    for (int i = dfac_; i < width_ - dfac_; i++) {
      for (int j = dfac_; j < height_ - dfac_; j++) {
        int val = 0;
        for (int ii = i - dfac_; ii <= i + dfac_; ii++) {
          for (int jj = j - dfac_; jj <= j + dfac_; jj++) {
            if (map_[ii][jj] > 0) val = 100;
          }
        }
        dmap[i][j] = val;
      }
    }
    for (int i = 1; i < width_ - 1; i++) {
      for (int j = 1; j < height_ - 1; j++) {
        double x_grid = grid->info.origin.position.x + i * grid->info.resolution;
        double y_grid = grid->info.origin.position.y + j * grid->info.resolution;
        SetMapValue({i, j}, dmap[i][j], GetGridValue(grid_segmentation, x_grid, y_grid));
      }
    }
  }

  bool solved;
  {
    auto recording = RecordSection(planner_sections::SOLVE);
    solved = Solve();
  }

  if (!solved) {
    std::cerr << "WARNING: Path planner failed to solve map " << std::endl;
  }

  return path_world_;
}


} // namespace planning
} // namespace avt_341_nav