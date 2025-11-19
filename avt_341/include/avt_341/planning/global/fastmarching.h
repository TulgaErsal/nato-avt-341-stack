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

  float* ExtractCosts() {
    return costs_;
  }

private:
  bool ExtractPath(float* costs);
  static Vec2 Normalize(const Vec2& v);
  static float Distance(const Point& p1, const Point& p2);
  float HandleGradientNaNs(float cost_0, float cost_1);
  float ComputeGradientX(const float* costs, const Index& index);
  float ComputeGradientY(const float* costs, const Index& index);
  Vec2 BilinearInterpolateGradient(const float* costs, const Point& position);

  float* costs_ = nullptr;
};

}
}

#endif