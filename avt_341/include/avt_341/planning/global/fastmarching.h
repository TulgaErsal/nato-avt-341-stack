#ifndef FASTMARCHING_H
#define FASTMARCHING_H

#include <utility>
#include <vector>

#include "avt_341/planning/global/astar.h"

namespace avt_341 {
namespace planning {
class FastMarching : public Astar {
public:
  FastMarching(std::shared_ptr<avt_341::visualization::VisualizerBase> visualizer,
               float w_distance,
               float w_occupancy,
               float w_segmentation,
               bool search_diagonals,
               int los_max_iterations,
               bool los_break_on_first,
               float safety_margin,
               std::string clearance_penalty_type,
               bool verbose) : Astar(std::move(visualizer),
                                            w_distance,
                                            w_occupancy,
                                            w_segmentation,
                                            search_diagonals,
                                            los_max_iterations,
                                            los_break_on_first),
                                      safety_margin_(safety_margin),
                                      clearance_penalty_type_(clearance_penalty_type),
                                      verbose_(verbose) {}

  /**
   * Solve the FM map. Returns true if a path was found.
   */
  bool Solve() override;

  std::vector<Point> PlanPath(avt_341::msg::OccupancyGrid* grid,
                              avt_341::msg::OccupancyGrid* segmentation_grid,
                              Point goal,
                              Point position) override;

  float* ExtractCosts() {
    return costs_;
  }

protected:
  void ComputeEDT();
  
  float safety_margin_;
  std::string clearance_penalty_type_;
  bool verbose_;
  std::vector<std::vector<float>> edt_map_;

private:
  bool ExtractPath(float* costs);
  static float Distance(const Point& p1, const Point& p2);

  float* costs_ = nullptr;
};

}
}

#endif