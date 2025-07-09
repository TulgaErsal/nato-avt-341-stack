#pragma once

#include <avt_341/planning/local/dwa/cells.hpp>
#include <avt_341/planning/local/dwa/obstacles.hpp>
#include <avt_341/planning/local/dwa/path.hpp>
#include <avt_341/planning/local/dwa/state.hpp>


namespace avt_341 {
namespace planning {
namespace dwa {

class Trajectory {
  public:
    Trajectory();

    void Add(State state);

    int GetNumberOfStates();

    State GetState(int i);

    const State& GetLastState() const;

    avt_341::msg::Path ToRosPath();

    void Reset();

    void EvaluateGoalCost(const double& weight,
                          const double& goal_x,
                          const double& goal_y);

    const double& GetGoalCost();

    void EvaluateObstacleCost(const double& weight,
                              const Obstacles& obstacles,
                              const double& collision_radius);

    const double& GetObstacleCost();

    void EvaluateSegmentationCost(const double& weight,
                                  const std::vector<signed char>& grid_data,
                                  const int& grid_width,
                                  const int& grid_height,
                                  const double& grid_origin_x,
                                  const double& grid_origin_y,
                                  const double& grid_resolution,
                                  const double& score_threshold);
    const double& GetSegmentationCost();

    void EvaluateHeadingCost(const double& weight,
                             const double& goal_x,
                             const double& goal_y);

    const double& GetHeadingCost();

    void EvaluateSpeedCost(const double& weight, const double& speed_threshold);

    const double& GetSpeedCost();

    void EvaluateGlobalPathCost(const double& weight, const Path& global_path);

    const double& GetGlobalPathCost();

    void EvaluateDeviationCost(const double& weight,
                               const Trajectory& current_trajectory);

    const double& GetDeviationCost();

    void EvaluateTotalCost();

    const double& GetTotalCost();

    avt_341::msg::DwaTrajectory GetROSTrajectoryMessage() const;

  private:
    std::vector<State> states_;

    double cost_ = 0.0;

    double goal_cost_ = 0.0;

    double heading_cost_ = 0.0;

    double obstacle_cost_ = 0.0;

    double speed_cost_ = 0.0;

    double global_path_cost_ = 0.0;

    double segmentation_cost_ = 0.0;

    double deviation_cost_ = 0.0;
};

} // namespace dwa
} // namespace planning
} // namespace avt_341
