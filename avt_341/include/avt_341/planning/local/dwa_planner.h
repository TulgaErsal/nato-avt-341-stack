/**
 * \class DwaPlanner
 *
 * Class for the dynamic window approach planner.
 *
 * Original paper: "The Dynamic Window Approach to Collision Avoidance" by Dieter Fox, Wolfram Burgard, Sebastian Thrun; 1997
 * Ref:
 * https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf
 *
 * Modified for Ackermann-steered vehicles. Added cost terms to penalise deviations from global and current paths and to use segmentation grid data.
 *
 * \author Dario Sirangelo
 *
 * \date 1/23/2023
*/

#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include "avt_341/node/ros_types.h"
#include "avt_341/avt_341_utils.h"
#include <algorithm>
#include <vector>
#include <iomanip>
#include <iostream>
#include <functional>

namespace avt_341 {
namespace planning {

class DwaCost {
    public:
        DwaCost() {}

        DwaCost(int i) {
            cost_norm_.resize(i);
            cost_.resize(i);
        }

        void Add(int i, double cost) { cost_[i] = cost; }

        double GetCost(int i) const { return cost_[i]; }

    private:
        std::vector<double> cost_;
        std::vector<double> cost_norm_;
        double min_ = -std::numeric_limits<double>::quiet_NaN();
        double max_ = std::numeric_limits<double>::quiet_NaN();
};

/** @brief Prediction window in the dynamic window approach (DWA) planner. */
struct DwaWindow {
    DwaWindow() {}

    DwaWindow(double speed_min, double speed_max, double speed_ang_min, double speed_ang_max) :
    speed_min_(speed_min), speed_max_(speed_max), speed_ang_min_(speed_ang_min), speed_ang_max_(speed_ang_max) {}

    double speed_min_;
    double speed_max_;
    double speed_ang_min_;
    double speed_ang_max_;
};

class DwaState {
    public:
        DwaState() {}

        DwaState(double x, double y, double yaw, double speed, double speed_ang) :
        x_(x), y_(y), yaw_(yaw), speed_(speed), speed_ang_(speed_ang) {}

        double GetX() { return x_; }

        void SetX(double x) { x_ = x; }

        double GetY() { return y_; }

        void SetY(double y) { y_ = y; }

        double GetYaw() { return yaw_; }

        void SetYaw(double yaw) { yaw_ = yaw; }

        double GetSpeed() { return speed_; }

        void SetSpeed(double speed) { speed_ = speed; }

        double GetAngularSpeed() { return speed_ang_; }

        void SetAngularSpeed(double speed_ang) { speed_ang_ = speed_ang; }

        avt_341::msg::PoseStamped
        ToRosPoseStamped() {
            avt_341::msg::PoseStamped msg_posestamped;

            msg_posestamped.pose.position.x = x_;
            msg_posestamped.pose.position.y = y_;

            avt_341::msg_tf::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, yaw_);
            msg_posestamped.pose.orientation.x = quaternion.x();
            msg_posestamped.pose.orientation.y = quaternion.y();
            msg_posestamped.pose.orientation.z = quaternion.z();
            msg_posestamped.pose.orientation.w = quaternion.w();

            return msg_posestamped;
        }

    private:
        double x_;
        double y_;
        double yaw_;
        double speed_;
        double speed_ang_;
};

class DwaTrajectory {
    public:
        DwaTrajectory() {}

        void Add(DwaState state) { states_.push_back(state); }

        int GetNumberOfStates() { return (int)states_.size(); }

        DwaState GetState(int i) { return states_[i]; }

        DwaState GetLastState() { return states_.back(); }

        avt_341::msg::Path
        ToRosPath() {
            avt_341::msg::Path msg_path;

            // Fill the poses array with all states in the trajectory.
            for (DwaState& state : states_) {
                msg_path.poses.push_back(state.ToRosPoseStamped());
            }

            return msg_path;
        }

      void Reset(){ states_.clear(); }

      double GetCost() { return cost_; }

      void SetCost(double cost){ cost_ =  cost; }

    private:
        double cost_ = 0.0;
        std::vector<DwaState> states_;
};

class DwaPath {
    public:
        DwaPath() {}

        void
        Add(double x, double y) {
            x_.push_back(x);
            y_.push_back(y);
        }

        int
        FindClosestDistance(double x, double y) {
            double d_min = std::numeric_limits<double>::infinity();

            for (int i = 0; i < (int)x_.size(); ++i) {
                double d_curr = std::hypot(x - x_[i], y - y_[i]);

                if (d_curr < d_min) {
                    d_min = d_curr;
                }
            }

            return d_min;
        }

    private:
        std::vector<double> x_;
        std::vector<double> y_;
};

class DwaObstacles {

    public:
        DwaObstacles() {}

        void Add(double x, double y) {
            x_.push_back(x);
            y_.push_back(y);
        }

        int GetNumberOfObstacles() { return (int)x_.size(); }

        double GetDistance(int i, double x, double y) { return std::hypot(x - x_[i], y - y_[i]); }

        void
        Clear() {
            x_.clear();
            y_.clear();
        }

    private:
        std::vector<double> x_;
        std::vector<double> y_;
};

class DwaCells {
    public:
        DwaCells() {}

        void Add(double x, double y, double cost) {
            x_.push_back(x);
            y_.push_back(y);
            costs_.push_back(cost);
        }

        int GetNumberOfObstacles() { return (int)x_.size(); }

        void
        Clear() {
            x_.clear();
            y_.clear();
        }

    private:
        std::vector<double> x_;
        std::vector<double> y_;
        std::vector<int> costs_;
};

class DwaPlanner {

    public:
        DwaPlanner();

        void Plan();

        double GetPlannedLinearSpeed() { return speed_lin_best_; }

        double GetPlannedAngularSpeed() { return speed_ang_best_; }

        avt_341::msg::Path GetPlannedPathRos() { return traj_best_.ToRosPath(); }

        void SetWindowLinearSpeedMin(double speed_lin_min) { speed_lin_min_ = speed_lin_min; }

        void SetWindowLinearSpeedMax(double speed_lin_max) { speed_lin_max_ = speed_lin_max; }

        void SetWindowLinearSpeedSteps(int speed_lin_steps) { speed_lin_steps_ = speed_lin_steps; }

        void SetWindowAccelerationMax(double accel_max) { accel_max_ = accel_max; }

        void SetWindowAngularSpeedMin(double speed_ang_min) { speed_ang_min_ = speed_ang_min; }

        void SetWindowAngularSpeedMax(double speed_ang_max) { speed_ang_max_ = speed_ang_max; }

        void SetWindowAngularSpeedSteps(int speed_ang_steps) { speed_ang_steps_ = speed_ang_steps; }

        void SetWindowAngularAccelerationMax(double ang_accel_max) { ang_accel_max_ = ang_accel_max; }

        void SetLateralAccelerationMax(double lat_accel_max) { lat_accel_max_ = lat_accel_max; }

        void SetWindowTimeStepMin(double time_step_min) { time_step_min_ = time_step_min; }

        void SetWindowTimeSpanMin(double time_span_min) { time_span_min_ = time_span_min; }

        void SetWindowTimeSpanMax(double time_span_max) { time_span_max_ = time_span_max; }

        void SetWindowTimeSpanVariable(double time_span_var) { time_span_var_ = time_span_var; }

        void SetWindowTimeSpanGain(double time_span_gain) { time_span_gain_ = time_span_gain; }

        void SetCostGoalWeight(double w_cost_goal) { w_cost_goal_ = w_cost_goal; }

        void SetCostHeadingWeight(double w_cost_head) { w_cost_head_ = w_cost_head; }

        void SetCostSpeedWeight(double w_cost_speed) { w_cost_speed_ = w_cost_speed; }

        void SetCostObstacleWeight(double w_cost_obs) { w_cost_obs_ = w_cost_obs; }

        void SetCostGlobalPathWeight(double w_cost_path) { w_cost_global_path_ = w_cost_path; }

        void SetCostDeviationWeight(double w_cost_dev) { w_cost_dev_ = w_cost_dev; }

        void SetCostSegmentationWeight(double w_cost_seg) { w_cost_seg_ = w_cost_seg; }

        void SetObstacleThreshold(int thresh_obs) { thresh_obs_ = thresh_obs; }

        void SetCollisionRadius(double collision_radius) { collision_radius_ = collision_radius; }

        void SetObstacleSearch(std::string obs_search) {
            // Search modality defaults to adaptive when providing a wrong parameter.
            if ((obs_search != "adaptive") || (obs_search != "fixed")) {
                obs_search_ = "adaptive";
            } else {
                obs_search_ = obs_search;
            }
        }

        void SetObstacleSearchRadius(double search_radius) { search_radius_ = search_radius; }

        void
        SetGoal(double x, double y) {
            goal_x_ = x;
            goal_y_ = y;
        };

        void SetOccupancyGridWidth(double grid_width) { grid_occ_width_ = grid_width; }

        void SetOccupancyGridHeight(double grid_height) { grid_occ_height_ = grid_height; }

        void SetOccupancyGridOriginX(double grid_origin_x) { grid_occ_origin_x_ = grid_origin_x; }

        void SetOccupancyGridOriginY(double grid_origin_y) { grid_occ_origin_y_ = grid_origin_y; }

        void SetOccupancyGridResolution(double grid_res) { grid_occ_res_ = grid_res; }

        void SetOccupancyGridData(std::vector<signed char> grid_data) { grid_occ_data_ = grid_data; }

        void SetSegmentationGridData(std::vector<signed char> grid_data) { grid_seg_data_ = grid_data; }

        void SetSegmentationThreshold(int thresh_seg) { thresh_seg_ = thresh_seg; }

        void SetGlobalPath(DwaPath path) { global_path_ = path; }

        void SetPrintSummary(bool print_summary) { print_summary_ = print_summary; }

        void
        SetState(double x, double y, double yaw, double speed, double speed_ang) {
            state_.SetX(x);
            state_.SetY(y);
            state_.SetYaw(yaw);
            state_.SetSpeed(speed);
            state_.SetAngularSpeed(speed_ang);
        }

        void
        SetHorizon(std::string horizon) { horizon_ = horizon; }

        bool GetUseGlobalPath() { return use_global_path_; }

        void SetUseGlobalPath(bool use_global_path) { use_global_path_ = use_global_path; }

        bool GetUseSegmentation() { return use_segmentation_; }

        void SetUseSegmentation(bool use_segmentation) { use_segmentation_ = use_segmentation; }

        void SetVehicleWheelbase(double wheelbase) { wheelbase_ = wheelbase; }

        void Reset();

        std::vector<DwaTrajectory> GetTrajectories() { return trajectories_; }

        double GetMaxCost() { return max_cost_; }

    private:
        void GetObstacles();

        DwaWindow EvaluateDynamicWindow();

        DwaState PredictMotion(DwaState state, double v, double thetadot, double time_step);

        DwaTrajectory PredictTrajectory(double speed, double speed_ang);

        double EvaluateCostGoal(DwaTrajectory traj);

        double EvaluateCostHeading(DwaTrajectory traj);

        double EvaluateCostObstacle(DwaTrajectory traj);

        double EvaluateCostSegmentation(DwaTrajectory traj);

        double EvaluateCostSpeed(DwaTrajectory traj);

        double EvaluateCostGlobalPath(DwaTrajectory traj);

        double EvaluateCostDeviation(DwaTrajectory traj);

        template<typename T>
        std::vector<double>
        GetInterval(T start_in, T end_in, int num_in) {

            std::vector<double> linspaced;

            double start = static_cast<double>(start_in);
            double end = static_cast<double>(end_in);
            double num = static_cast<double>(num_in);

            if (num == 0) {
                return linspaced;
            }
            if (num == 1) {
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

        int
        FindClosest(std::vector<double> const& v, int value) {
            auto const it = std::lower_bound(v.begin(), v.end(), value);

            return it - v.begin();
        }

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
        double time_step_min_;
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
        DwaObstacles obs_occ_;
        bool use_segmentation_;
        std::vector<signed char> grid_seg_data_;
        int thresh_seg_;
        double w_cost_goal_;
        double w_cost_head_;
        double w_cost_speed_;
        double w_cost_obs_;
        double w_cost_seg_;
        double w_cost_dev_;
        DwaState state_;
        DwaTrajectory traj_best_;
        double speed_lin_best_;
        double speed_ang_best_;
        std::string model_;
        double wheelbase_;
        std::string horizon_;
        DwaPath global_path_;
        bool use_global_path_;
        double w_cost_global_path_;
        DwaTrajectory traj_last_;
        bool has_plan_ = false;
        bool print_summary_;
        std::vector<DwaTrajectory> trajectories_;
        double max_cost_ = std::numeric_limits<double>::min();
};

} // end namespace planning
} // end namespace avt_341

#endif // define DWA_PLANNER_H
