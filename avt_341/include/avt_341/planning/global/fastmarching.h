#ifndef FASTMARCHING_H
#define FASTMARCHING_H

#include <utility>
#include <vector>
#include <memory>
#include <string>

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
               std::string path_integration_mode,
               bool verbose) : Astar(std::move(visualizer),
                                            w_distance,
                                            w_occupancy,
                                            w_segmentation,
                                            search_diagonals,
                                            los_max_iterations,
                                            los_break_on_first),
                                      safety_margin_(safety_margin),
                                      clearance_penalty_type_(clearance_penalty_type),
                                      verbose_(verbose),
                                      path_integration_mode_(path_integration_mode) {}

  virtual ~FastMarching() = default;

  /**
   * Solve the FM map. Returns true if a path was found.
   */
  bool Solve() override;

  std::vector<Point> PlanPath(avt_341::msg::OccupancyGrid* grid,
                              avt_341::msg::OccupancyGrid* segmentation_grid,
                              Point goal,
                              Point position) override;

  float* ExtractCosts() {
    return costs_flat_.data();
  }

protected:
  void ComputeEDT();
  
  std::string path_integration_mode_;
  float safety_margin_;
  std::string clearance_penalty_type_;
  bool verbose_;
  
  // Persistent buffers to avoid reallocations
  std::vector<float> edt_flat_;
  std::vector<float> costs_flat_;
  std::vector<float> base_weights_tmp_;
  
  // Temporary buffers for EDT computation
  std::vector<float> edt_work_f_;
  std::vector<float> edt_work_d_row_;
  std::vector<int> edt_work_v_;
  std::vector<float> edt_work_z_;
  std::vector<float> edt_work_dist_sq_;

private:
  bool ExtractPath();
  bool ExtractPathDiscrete();
  bool ExtractPathGradientDescent();
  Point GetGradient(Point p);
  static float Distance(const Point& p1, const Point& p2);
};

} // namespace planning
} // namespace avt_341

#endif // FASTMARCHING_H