#include "avt_341/planning/global/fast_marching_square.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace avt_341 {
namespace planning {

std::vector<Point> FastMarchingSquare::PlanPath(avt_341::msg::OccupancyGrid* grid,
                                                avt_341::msg::OccupancyGrid* segmentation_grid,
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
    for (int i = 0; i < n_cells; ++i) {
        int ix = i % w;
        int iy = i / w;
        double x_grid = grid->info.origin.position.x + ix * grid->info.resolution;
        double y_grid = grid->info.origin.position.y + iy * grid->info.resolution;
        
        float occ = (float)grid->data[i];
        float seg_val = (float)GetGridValue(segmentation_grid, x_grid, y_grid);
        // Treat high occupancy or low segmentation class as obstacle
        map_[ix][iy] = occ; 
    }
    
    // 1. Compute Distance Map (EDT) - The first "Fast Marching" step (or solving Eikonal for distance)
    ComputeEDT();
    
    float adjusted_safety_margin = safety_margin_ + (map_res_ * 0.5f);

    if (verbose_) {
        std::cout << "[FastMarchingSquare] Safety margin: " << adjusted_safety_margin << "m" << std::endl;
    }

    // 2. Compute Velocity/Slowness Map
    // FM2 Principal: Velocity V(x) ~ Distance(x).
    // Metric W(x) = 1/V(x) = 1/Distance(x).
    // We normalize to avoid tiny numbers.
    
    float max_dist = 0.0f;
    for (float d : edt_flat_) {
        if (d < 1e9f && d > max_dist) max_dist = d;
    }
    if (max_dist < 0.1f) max_dist = 1.0f;

    for (int i = 0; i < n_cells; ++i) {
        float d = edt_flat_[i];
        int ix = i % w;
        int iy = i / w;
        
        if (d <= adjusted_safety_margin || map_[ix][iy] > 50) { // Obstacle threshold
            weights_[i] = INF;
        } else {
            // W = (max_dist / d) * w_distance
            // This creates a potential well centered at max distance from everything.
            // We can raise to power to influence how strongly it centers.
            // Pure FM2 uses linear relation usually.
            // We add a small epsilon to d to act as saturation.
            
            const float sat_dist = 5.0f;
            float effect_dist = std::min(d, sat_dist);
            
            float denominator = std::max(effect_dist, 0.01f);
            float weight = (max_dist / denominator);
            
            // Apply w_distance_ as a scaling factor
            weights_[i] = weight * w_distance_;
        }
    }

    // 3. Solve Path (Second Fast Marching step)
    if (!Solve()) {
        if (verbose_) std::cerr << "WARNING: Fast Marching Square failed to find path" << std::endl;
    }

    return path_world_;
}

} // namespace planning
} // namespace avt_341
