#ifndef FASTMARCHING_H
#define FASTMARCHING_H

#include <utility>
#include <vector>
#include <memory>
#include <string>

#include "avt_341_nav/planning/global/astar.h"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace avt_341_nav {
namespace planning {

class FastMarching : public Astar {
public:
  FastMarching(float w_distance,
               float w_occupancy,
               float w_segmentation,
               bool search_diagonals,
               int los_max_iterations,
               bool los_break_on_first,
               float safety_margin_global,
               float safety_margin_soft,
               std::string clearance_penalty_type,
               std::string path_extraction_method,
               float obstacle_threshold,
               float clearance_penalty_scale,
               float clearance_penalty_range,
               float clearance_penalty_exponent,
               int gradient_descent_max_steps,
               int gradient_descent_steps_per_point,
               float clipping_distance,
               bool verbose) : Astar(w_distance,
                                            w_occupancy,
                                            w_segmentation,
                                            search_diagonals,
                                            los_max_iterations,
                                            los_break_on_first),
                                      safety_margin_global_(safety_margin_global),
                                      safety_margin_soft_(safety_margin_soft),
                                      clearance_penalty_type_(clearance_penalty_type),
                                      verbose_(verbose),
                                      path_extraction_method_(path_extraction_method),
                                      obstacle_threshold_(obstacle_threshold),
                                      clearance_penalty_scale_(clearance_penalty_scale),
                                      clearance_penalty_range_(clearance_penalty_range),
                                      clearance_penalty_exponent_(clearance_penalty_exponent),
                                      gradient_descent_max_steps_(gradient_descent_max_steps),
                                      gradient_descent_steps_per_point_(gradient_descent_steps_per_point),
                                      clipping_distance_(clipping_distance) {}

  virtual ~FastMarching() = default;

  /**
   * Solve the FM map. Returns true if a path was found.
   */
  bool Solve() override;

  std::vector<Point> PlanPath(nav_msgs::msg::OccupancyGrid* grid,
                              nav_msgs::msg::OccupancyGrid* segmentation_grid,
                              Point goal,
                              Point position) override;

  float* ExtractCosts() {
    return costs_flat_.data();
  }

protected:
  void ComputeEDT();
  
  std::string path_extraction_method_;
  float safety_margin_global_;
  float safety_margin_soft_;
  std::string clearance_penalty_type_;
  float obstacle_threshold_;
  float clearance_penalty_scale_;
  float clearance_penalty_range_;
  float clearance_penalty_exponent_;
  int gradient_descent_max_steps_;
  int gradient_descent_steps_per_point_;
  float clipping_distance_;
  bool verbose_;
  
  float clearance_penalty(float d, float r, const std::string& option);
  
  // Persistent buffers to avoid reallocations
  std::vector<float> edt_flat_;
  std::vector<float> costs_flat_;
  std::vector<float> base_weights_tmp_;
  std::vector<Vec2> shifts_;
  
  // Temporary buffers for EDT computation
  std::vector<float> edt_work_f_;
  std::vector<float> edt_work_d_row_;
  std::vector<int> edt_work_v_;
  std::vector<float> edt_work_z_;
  std::vector<float> edt_work_dist_sq_;

private:
  bool ExtractPath();
  bool ExtractPathDiscrete();
  bool ExtractPathGradientDescent(float* costs);
  float HandleGradientNaNs(float cost_1, float cost_0);
  float ComputeGradientX(const float* costs, const Index& index);
  float ComputeGradientY(const float* costs, const Index& index);
  bool LineOfSight(const Index& i0, const Index& i1) override;
  Vec2 BilinearInterpolateGradient(const float* costs, const Point& position);
  Vec2 Normalize(const Vec2& v);
  Point GetShiftedPoint(int idx) const;
  static float Distance(const Point& p1, const Point& p2);
};

} // namespace planning
} // namespace avt_341_nav

#endif // FASTMARCHING_H
