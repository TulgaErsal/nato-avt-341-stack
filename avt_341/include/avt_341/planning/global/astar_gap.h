#ifndef ASTAR_GAP_H
#define ASTAR_GAP_H

#include <utility>
#include <vector>
#include <memory>

#include "avt_341/planning/global/astar.h"

namespace avt_341 {
namespace planning {

/**
 * A* planner variant that adds a soft, vehicle-width-aware "gap" penalty.
 *
 * For every cell, the distance to the nearest obstacle (its clearance) is
 * computed with an exact Euclidean distance transform. Cells whose clearance is
 * small relative to the vehicle's half-width receive an additive cost penalty,
 * so the planner prefers wider gaps between obstacles.
 *
 * The penalty is intentionally soft: a gap narrower than the vehicle is never
 * marked impassable, so if a tight gap is the only route the planner may still
 * take it, just at a higher cost. This keeps the base Astar untouched -- all of
 * the gap logic lives in this subclass, which overrides PlanPath.
 */
class AstarGap : public Astar {
public:
  AstarGap(std::shared_ptr<avt_341::visualization::VisualizerBase> visualizer,
           float w_distance,
           float w_occupancy,
           float w_segmentation,
           bool search_diagonals,
           int los_max_iterations,
           bool los_break_on_first,
           float vehicle_width,
           float w_clearance,
           float clearance_range)
    : Astar(std::move(visualizer),
            w_distance,
            w_occupancy,
            w_segmentation,
            search_diagonals,
            los_max_iterations,
            los_break_on_first),
      vehicle_width_(vehicle_width),
      w_clearance_(w_clearance),
      clearance_range_(clearance_range) {}

  ~AstarGap() override = default;

  /// Set the vehicle width (meters). <= 0 disables the gap penalty.
  void SetVehicleWidth(float width) { vehicle_width_ = width; }

  /// Set the weight applied to the gap penalty. <= 0 disables it.
  void SetClearanceWeight(float weight) { w_clearance_ = weight; }

  /// Set the clearance (meters) beyond which the gap penalty is zero.
  void SetClearanceRange(float range) { clearance_range_ = range; }

  /**
   * Build the cost map (identical to Astar::PlanPath), then add the soft
   * vehicle-width gap penalty before solving.
   */
  std::vector<Point> PlanPath(avt_341::msg::OccupancyGrid* grid,
                              avt_341::msg::OccupancyGrid* segmentation_grid,
                              Point goal,
                              Point position) override;

protected:
  /**
   * Compute the distance (meters) from every cell to the nearest obstacle in
   * map_, storing the result in clearance_. Uses an exact Euclidean distance
   * transform (Felzenszwalb-Huttenlocher), the same approach used by the Fast
   * Marching planner.
   */
  void ComputeClearance();

  /**
   * Add the soft gap penalty into weights_ using the current map_ obstacles.
   * No-op unless both vehicle_width_ and w_clearance_ are > 0.
   */
  void ApplyGapPenalty();

  /**
   * Clearance-aware line of sight used by the post-smoothing pass.
   *
   * First applies the base Astar hard-obstacle line-of-sight test. If that
   * passes, it additionally rejects any shortcut that would route the path
   * within half the vehicle width of an obstacle. Without this, PostSmoothing
   * straightens the path using only hard-obstacle checks and undoes the
   * clearance-respecting bulge produced by the gap penalty -- which made
   * astar_gap behave almost identically to plain astar. Falls back to the base
   * behavior when the gap penalty is disabled or clearance_ has not been
   * computed yet.
   */
  // bool LineOfSight(const Index& i0, const Index& i1) override;

  bool ExtractPath() override;

  float vehicle_width_;    ///< vehicle width [m]; <= 0 disables the penalty
  float w_clearance_;      ///< weight on the gap penalty; <= 0 disables it
  float clearance_range_;  ///< clearance [m] beyond which the penalty is zero

  /// distance (m) from each cell to the nearest obstacle, indexed like weights_
  std::vector<float> clearance_;
};

} // namespace planning
} // namespace avt_341

#endif // ASTAR_GAP_H
