#ifndef FAST_MARCHING_SQUARE_H
#define FAST_MARCHING_SQUARE_H

#include "avt_341/planning/global/fastmarching.h"

namespace avt_341 {
namespace planning {

/**
 * @brief Fast Marching Square (FM2) planner.
 * 
 * This planner implements the FM2 method where the wave propagation velocity
 * is proportional to the distance from the nearest obstacle. This results in
 * paths that naturally stay in the center of corridors (Voronoi-like paths)
 * and are smooth.
 * 
 * Ref: "Path Planning for Autonomous Mobile Robots using the FM2 Method"
 */
class FastMarchingSquare : public FastMarching {
public:
  FastMarchingSquare(std::shared_ptr<avt_341::visualization::VisualizerBase> visualizer,
                     float w_distance,
                     float w_occupancy,
                     float w_segmentation,
                     bool search_diagonals,
                     int los_max_iterations,
                     bool los_break_on_first,
                     float safety_margin,
                     std::string clearance_penalty_type,
                     bool verbose)
      : FastMarching(std::move(visualizer), w_distance, w_occupancy, w_segmentation,
                     search_diagonals, los_max_iterations, los_break_on_first,
                     safety_margin, clearance_penalty_type, verbose) {}

  virtual ~FastMarchingSquare() = default;

  /**
   * @brief Overrides PlanPath to implement the FM2 weight calculation.
   */
  std::vector<Point> PlanPath(avt_341::msg::OccupancyGrid* grid,
                              avt_341::msg::OccupancyGrid* segmentation_grid,
                              Point goal,
                              Point position) override;
};

} // namespace planning
} // namespace avt_341

#endif // FAST_MARCHING_SQUARE_H
