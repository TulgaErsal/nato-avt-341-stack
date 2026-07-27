#pragma once

#include <iostream>

#include <boost/algorithm/clamp.hpp>

#include <avt_341_nav/avt_341_utils.h>
#include "nav_msgs/msg/path.hpp"
#include <avt_341_nav/planning/local/dwa/dynamic_window.hpp>
#include <avt_341_nav/planning/local/dwa/obstacles.hpp>
#include <avt_341_nav/planning/local/dwa/path.hpp>
#include <avt_341_nav/planning/local/dwa/state.hpp>
#include <avt_341_nav/planning/local/dwa/trajectory.hpp>

namespace avt_341_nav {
namespace planning {
namespace dwa {

class Planner {
  public:
    Planner();

    void Plan();

    double GetPlannedLinearSpeed();

    double GetPlannedAngularSpeed();

    const Trajectory& GetOptimalTrajectory() const;

    nav_msgs::msg::Path GetPlannedPathRos();

    void SetWindowLinearSpeedMin(double speed_lin_min);

    void SetWindowLinearSpeedMax(double speed_lin_max);

    void SetWindowLinearSpeedSteps(int speed_lin_steps);

    void SetWindowAccelerationMax(double accel_max);

    void SetWindowAngularSpeedMin(double speed_ang_min);

    void SetWindowAngularSpeedMax(double speed_ang_max);

    void SetWindowAngularSpeedSteps(int speed_ang_steps);

    void SetWindowAngularAccelerationMax(double ang_accel_max);

    void SetLateralAccelerationMax(double lat_accel_max);

    void SetTimeStep(double time_step);

    void SetWindowTimeSpanMin(double time_span_min);

    void SetWindowTimeSpanMax(double time_span_max);

    void SetWindowTimeSpanVariable(double time_span_var);

    void SetWindowTimeSpanGain(double time_span_gain);

    void SetCostGoalWeight(double w_cost_goal);

    void SetCostHeadingWeight(double w_cost_head);

    void SetCostSpeedWeight(double w_cost_speed);

    void SetCostObstacleWeight(double w_cost_obs);

    void SetCostGlobalPathWeight(double w_cost_path);

    void SetCostDeviationWeight(double w_cost_dev);

    void SetCostSegmentationWeight(double w_cost_seg);

    void SetObstacleThreshold(int thresh_obs);

    void SetCollisionRadius(double collision_radius);

    void SetObstacleSearch(std::string obs_search);

    void SetObstacleSearchRadius(double search_radius);

    void SetGoal(double x, double y);

    void SetOccupancyGridWidth(double grid_width);

    void SetOccupancyGridHeight(double grid_height);

    void SetOccupancyGridOriginX(double grid_origin_x);

    void SetOccupancyGridOriginY(double grid_origin_y);

    void SetOccupancyGridResolution(double grid_res);

    void SetOccupancyGridData(std::vector<signed char> grid_data);

    void SetSegmentationGridData(std::vector<signed char> grid_data);

    void SetSegmentationThreshold(int thresh_seg);

    void SetGlobalPath(Path path);

    void SetPrintSummary(bool print_summary);

    void
    SetState(double x, double y, double yaw, double speed, double speed_ang);

    void SetHorizon(std::string horizon);

    bool GetUseGlobalPath();

    void SetUseGlobalPath(bool use_global_path);

    bool GetUseSegmentation();

    void SetUseSegmentation(bool use_segmentation);

    void SetVehicleWheelbase(double wheelbase);

    void Reset();

    const std::vector<Trajectory>& GetTrajectories() const;

    const double& GetMaxCost() const;

    const double& GetMinCost() const;

  private:
    void GetObstacles();

    void EvaluateDynamicWindow();

    State
    PredictMotion(State state, double v, double thetadot, double time_step);

    Trajectory PredictTrajectory(double speed, double speed_ang);

    template <typename T>
    std::vector<double> GetInterval(T start_in, T end_in, int num_in) {
        std::vector<double> linspaced;

        double start = static_cast<double>(start_in);
        double end = static_cast<double>(end_in);
        double num = static_cast<double>(num_in);

        if(num == 0) { return linspaced; }
        if(num == 1) {
            linspaced.push_back(start);
            return linspaced;
        }

        double delta = (end - start) / (num - 1);

        for(int i = 0; i < num - 1; ++i) {
            linspaced.push_back(start + delta * i);
        }

        linspaced.push_back(end);

        return linspaced;
    }

    int FindClosest(std::vector<double> const& v, int value);

    double speed_lin_min_;
    double speed_lin_max_;
    int speed_lin_steps_;
    double accel_max_;
    double speed_ang_min_;
    double speed_ang_max_;
    int speed_ang_steps_;
    double ang_accel_max_;
    double lat_accel_max_;
    double time_step_;
    double time_span_;
    double time_span_min_;
    double time_span_max_;
    double time_span_var_;
    double time_span_gain_;
    int thresh_obs_;
    double collision_radius_;
    std::string obs_search_;
    double search_radius_;
    double goal_x_;
    double goal_y_;
    double grid_occ_width_;
    double grid_occ_height_;
    double grid_occ_origin_x_;
    double grid_occ_origin_y_;
    double grid_occ_res_;
    std::vector<signed char> grid_occ_data_;
    Obstacles obs_occ_;
    bool use_segmentation_;
    std::vector<signed char> grid_seg_data_;
    int thresh_seg_;
    double w_cost_goal_;
    double w_cost_head_;
    double w_cost_speed_;
    double w_cost_obs_;
    double w_cost_seg_;
    double w_cost_dev_;
    State state_;
    Trajectory traj_best_;
    double speed_lin_best_;
    double speed_ang_best_;
    std::string model_;
    double wheelbase_;
    std::string horizon_;
    Path global_path_;
    bool use_global_path_;
    double w_cost_global_path_;
    Trajectory traj_last_;
    bool has_plan_ = false;
    bool print_summary_;
    std::vector<Trajectory> trajectories_;
    double min_cost_ = std::numeric_limits<double>::infinity();
    double max_cost_ = -std::numeric_limits<double>::infinity();
    DynamicWindow dynamic_window_;
};

} // namespace dwa
} // namespace planning
} // namespace avt_341_nav