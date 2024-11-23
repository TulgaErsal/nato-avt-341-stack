#ifndef FASTMARCHING_H
#define FASTMARCHING_H

#include <utility>

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
               bool los_break_on_first) : Astar(std::move(visualizer),
                                                w_distance,
                                                w_occupancy,
                                                w_segmentation,
                                                search_diagonals,
                                                los_max_iterations,
                                                los_break_on_first) {}

  /**
   * Solve the FM map. Returns true if a path was found.
   */
  bool Solve() override;
  bool ExtractPath(float* costs);
  std::vector<float> get_gradient(const std::vector<float>& position, const float* costs);
  static std::vector<float> Normalize(const std::vector<float>& v);
  float Distance(const std::vector<float>& v1, const std::vector<float>& v2);
};

}
}

#endif