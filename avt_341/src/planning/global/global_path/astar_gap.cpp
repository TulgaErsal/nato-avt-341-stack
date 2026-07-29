#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <vector>
// project includes
#include "avt_341/planning/global/astar_gap.h"

namespace avt_341 {
namespace planning {

void AstarGap::ComputeClearance() {
  const int w = width_;
  const int h = height_;
  const float INF_EDT = 1e10f;

  clearance_.assign(w * h, 0.0f);
  std::vector<float> dist_sq(w * h, INF_EDT);

  // Column pass: squared distance (in cells) to the nearest obstacle in each column.
  for (int x = 0; x < w; ++x) {
    float last_occ = -INF_EDT;
    for (int y = 0; y < h; ++y) {
      if (map_[x][y] > 0.0f) last_occ = static_cast<float>(y);
      if (last_occ != -INF_EDT) {
        float d = static_cast<float>(y) - last_occ;
        dist_sq[y * w + x] = d * d;
      }
    }
    last_occ = INF_EDT;
    for (int y = h - 1; y >= 0; --y) {
      if (map_[x][y] > 0.0f) last_occ = static_cast<float>(y);
      if (last_occ != INF_EDT) {
        float d = last_occ - static_cast<float>(y);
        float dsq = d * d;
        if (dsq < dist_sq[y * w + x]) dist_sq[y * w + x] = dsq;
      }
    }
  }

  // Row pass: lower envelope of parabolas (Felzenszwalb-Huttenlocher).
  std::vector<int> v(std::max(w, h));
  std::vector<float> z(std::max(w, h) + 1);
  for (int y = 0; y < h; ++y) {
    int k = 0;
    v[0] = 0;
    z[0] = -INF_EDT;
    z[1] = INF_EDT;
    for (int q = 1; q < w; ++q) {
      float f_q = dist_sq[y * w + q];
      if (f_q >= INF_EDT) continue;
      float s;
      while (true) {
        int v_k = v[k];
        float f_vk = dist_sq[y * w + v_k];
        s = ((f_q + q * q) - (f_vk + v_k * v_k)) / (2.0f * (q - v_k));
        if (s <= z[k]) {
          k--;
        } else {
          break;
        }
        if (k < 0) break;
      }
      k++;
      if (k < 0) k = 0;
      v[k] = q;
      z[k] = s;
      z[k + 1] = INF_EDT;
    }
    int cur_k = 0;
    for (int q = 0; q < w; ++q) {
      while (z[cur_k + 1] < q) cur_k++;
      int v_k = v[cur_k];
      float f_vk = dist_sq[y * w + v_k];
      float dx = static_cast<float>(q - v_k);
      float dsq = dx * dx + f_vk;
      // convert from cells to meters
      clearance_[y * w + q] = std::sqrt(dsq) * map_res_;
    }
  }
}

void AstarGap::ApplyGapPenalty() {
  if (vehicle_width_ <= 0.0f || w_clearance_ <= 0.0f) return;

  ComputeClearance();

  const float half_w = 0.5f * vehicle_width_;
  // Distance at which the penalty fades to zero. Default to one vehicle width
  // of breathing room if the configured range is not strictly above half_w.
  float range = (clearance_range_ > half_w) ? clearance_range_ : vehicle_width_;
  if (range <= half_w) range = half_w + map_res_;  // guard against degenerate input
  const float inv_span = 1.0f / (range - half_w);

  const int n_cells = width_ * height_;
  for (int i = 0; i < n_cells; ++i) {
    const float d = clearance_[i];
    if (d < range) {
      // penalty: 0 at d == range, 1 at d == half_w, and > 1 for tighter gaps
      const float penalty = (range - d) * inv_span;
      weights_[i] += w_clearance_ * penalty;
    }
  }
}

// bool AstarGap::LineOfSight(const Index& i0, const Index& i1) {
//   // Preserve the base hard-obstacle line-of-sight semantics exactly.
//   if (!Astar::LineOfSight(i0, i1)) return false;

//   // Before clearance is computed (or with the penalty disabled) behave exactly
//   // like the base planner.
//   if (vehicle_width_ <= 0.0f || clearance_.empty()) return true;

//   // Don't let smoothing straighten the path closer to an obstacle than the
//   // vehicle's half-width. This keeps the clearance-aware route produced by the
//   // gap penalty from being collapsed back onto the shortest taut path.
//   const float min_clearance = 0.5f * vehicle_width_;

//   const int dx = i1.ix - i0.ix;
//   const int dy = i1.iy - i0.iy;
//   const int n_cells = std::max(std::abs(dx), std::abs(dy));
//   if (n_cells == 0) return true;

//   // Sample at sub-cell resolution so no tight cell along the segment is skipped.
//   // Endpoints are path nodes the smoother keeps regardless, so only the cells
//   // the shortcut passes *through* are checked.
//   const int n_steps = 2 * n_cells;
//   for (int s = 1; s < n_steps; ++s) {
//     const float t = static_cast<float>(s) / static_cast<float>(n_steps);
//     const int ix = static_cast<int>(std::lround(i0.ix + t * dx));
//     const int iy = static_cast<int>(std::lround(i0.iy + t * dy));
//     if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) continue;
//     if (clearance_[iy * width_ + ix] < min_clearance) return false;
//   }
//   return true;
// }

// diable the line of sight check for now, since it is not used in the current implementation
// bool AstarGap::LineOfSight(const Index& i0, const Index& i1) {
//   // Path smoothing disabled for astar_gap: never report line of sight between
//   // non-adjacent path nodes, so PostSmoothing cannot shortcut any of them and
//   // the raw grid-search path is returned unmodified.
//   return false;
// }

bool AstarGap::ExtractPath() {
  path_.clear();
  path_world_.clear();

  int path_idx = goal_;
  while (path_idx != start_) {
    path_.push_back(FoldIndex(path_idx));
    path_idx = paths_[path_idx];
  }

  // No PostSmoothing pass: each grid step is already at most one diagonal
  // cell apart, so the raw search path needs no fill-in either.
  path_world_pre_fill_.clear();
  for (const auto& index : path_) {
    path_world_pre_fill_.push_back(IndexToPoint(index));
  }

  if (path_world_pre_fill_.empty()) return false;

  path_world_ = path_world_pre_fill_;

  std::reverse(path_.begin(), path_.end());
  std::reverse(path_world_.begin(), path_world_.end());

  return true;
}

std::vector<Point> AstarGap::PlanPath(avt_341::msg::OccupancyGrid* grid,
                                      avt_341::msg::OccupancyGrid* grid_segmentation,
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
      SetMapValue({ix, iy}, grid->data[n], 100 - GetGridValue(grid_segmentation, x_grid, y_grid));
      n++;
    }
  }

  // dilate
  if (dfac_ > 0) {
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
        SetMapValue({i, j}, dmap[i][j], 100 - GetGridValue(grid_segmentation, x_grid, y_grid));
      }
    }
  }

  // Soft vehicle-width gap penalty (the only addition over base Astar::PlanPath).
  ApplyGapPenalty();

  auto t0 = std::chrono::steady_clock::now();

  bool solved = Solve();

  auto t1 = std::chrono::steady_clock::now();

  std::cout << "A* (gap) solve time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
            << " ms" << std::endl;

  if (!solved) {
    std::cerr << "WARNING: Path planner (gap) failed to solve map " << std::endl;
  }

  return path_world_;
}

} // namespace planning
} // namespace avt_341
