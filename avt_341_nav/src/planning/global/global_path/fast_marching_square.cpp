#include "avt_341_nav/planning/global/fast_marching_square.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace avt_341_nav {
namespace planning {

std::vector<Point> FastMarchingSquare::PlanPath(nav_msgs::msg::OccupancyGrid* grid,
                                                nav_msgs::msg::OccupancyGrid* segmentation_grid,
                                                Point goal,
                                                Point position) {
    if (grid->info.height <= 0 || grid->info.width <= 0) {
        return path_world_;
    }

    int w = grid->info.width;
    int h = grid->info.height;

    // Allocate/Resize maps if needed.
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
    auto clamp = [&](Index& idx) {
        idx.ix = std::max(0, std::min(w - 1, idx.ix));
        idx.iy = std::max(0, std::min(h - 1, idx.iy));
    };
    clamp(goal_idx);
    clamp(start_idx);

    SetGoal(goal_idx);
    SetStart(start_idx);

    // Initialize map binary obstacle info
    // For FM2, we mainly care about "is obstacle" vs "is free".
    // We can also use segmentation as obstacles if needed.
    int n_cells = w * h;
    {
        auto recording = RecordSection(planner_sections::GRID_INGEST);
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
    }

    // 1. Compute Distance Map (EDT) - The first "Fast Marching" step (or solving Eikonal for distance)
    {
        auto recording = RecordSection(planner_sections::EDT);
        ComputeEDT();
    }

    float adjusted_safety_margin = safety_margin_global_ + (map_res_ * 0.5f);

    if (verbose_) {
        std::cout << "[FastMarchingSquare] Safety margin (input/adjusted): " << safety_margin_global_ << "/" << adjusted_safety_margin << "m" << std::endl;
    }

    shifts_.assign(n_cells, {0.0f, 0.0f});

    const float transition_buffer = 3.0f * map_res_; // 3-cell smooth transition
    const float w_penalty = 5.0f; // Magnitude of the safety push

    auto clearance_recording = RecordSection(planner_sections::CLEARANCE_SHIFTS);
    for (int i = 0; i < n_cells; ++i) {
        float d = edt_flat_[i];
        int ix = i % width_;
        int iy = i / width_;
        
        // A cell is INF only if it is fully consumed by the safety margin.
        // A cell must be marked as an obstacle if a shift to clear the safety margin exceeds half a cell size.
        if (map_[ix][iy] > obstacle_threshold_ || d < safety_margin_global_) {
            weights_[i] = INF;
        } else {
            if (d < adjusted_safety_margin) {
                // Calculate gradient direction of EDT
                float gx = 0.0f;
                float gy = 0.0f;
                if (ix > 0 && ix < width_ - 1) gx = (edt_flat_[i+1] - edt_flat_[i-1]);
                else if (ix == 0 && width_ > 1) gx = (edt_flat_[i+1] - d) * 2.0f;
                else if (ix == width_ - 1 && width_ > 1) gx = (d - edt_flat_[i-1]) * 2.0f;

                if (iy > 0 && iy < height_ - 1) gy = (edt_flat_[i+width_] - edt_flat_[i-width_]);
                else if (iy == 0 && height_ > 1) gy = (edt_flat_[i+width_] - d) * 2.0f;
                else if (iy == height_ - 1 && height_ > 1) gy = (d - edt_flat_[i-width_]) * 2.0f;

                float mag = std::sqrt(gx * gx + gy * gy);
                if (mag > 1e-6f) {
                    float shift_len = adjusted_safety_margin - d;
                    shifts_[i].x = (gx / mag) * shift_len;
                    shifts_[i].y = (gy / mag) * shift_len;
                }
            }

            float effective_d = std::max(d, adjusted_safety_margin);

            if (effective_d < adjusted_safety_margin + transition_buffer) {
                float dist_into_buffer = (effective_d - adjusted_safety_margin);
                float ratio = dist_into_buffer / transition_buffer;
                weights_[i] = base_weights_tmp_[i] + w_penalty * std::pow(1.0f - ratio, 2); 
            } else {
                weights_[i] = base_weights_tmp_[i];
            }
        }
    }

    clearance_recording.reset();

    // 3. Solve Path (Second Fast Marching step)
    {
        auto recording = RecordSection(planner_sections::SOLVE);
        if (!Solve()) {
            if (verbose_) std::cerr << "WARNING: Fast Marching Square failed to find path" << std::endl;
        }
    }

    return path_world_;
}

} // namespace planning
} // namespace avt_341_nav
