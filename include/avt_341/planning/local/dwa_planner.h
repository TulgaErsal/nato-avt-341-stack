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

        ~DwaCost() {}

        void Add(int i, float cost) { cost_[i] = cost; }

        float GetCost(int i) const { return cost_[i]; }

        float GetNormalisedCost(int i) const { return cost_norm_[i]; }

        void
        Normalise() {
            max_ = *std::max_element(cost_.begin(), cost_.end());
            min_ = *std::min_element(cost_.begin(), cost_.end());

            for (int i = 0; i < (int)cost_.size(); ++i) {
                if (cost_[i] == std::numeric_limits<float>::infinity()) {
                    cost_norm_[i] = std::numeric_limits<float>::infinity();
                } else if (std::abs(max_ - min_) < 100.0f * std::numeric_limits<float>::epsilon()) {
                    //
                    cost_norm_[i] = 0.0f;
                } else {
                    // Compute the normalised cost
                    cost_norm_[i] = 0.0f + (1.0 * (1.0f - 0.0f) / (max_ - min_)) * (cost_[i] - min_);
                }

            }

        }

    private:
        std::vector<float> cost_;
        std::vector<float> cost_norm_;
        float min_ = -std::numeric_limits<float>::quiet_NaN();
        float max_ = std::numeric_limits<float>::quiet_NaN();
};

/** @brief Prediction window in the dynamic window approach (DWA) planner. */
struct DwaWindow {
    DwaWindow() {}

    DwaWindow(float speed_min, float speed_max, float speed_ang_min, float speed_ang_max) :
    speed_min_(speed_min), speed_max_(speed_max), speed_ang_min_(speed_ang_min), speed_ang_max_(speed_ang_max) {}

    float speed_min_;
    float speed_max_;
    float speed_ang_min_;
    float speed_ang_max_;
};

class DwaState {
    public:
        DwaState() {}

        DwaState(float x, float y, float yaw, float speed, float speed_ang) :
        x_(x), y_(y), yaw_(yaw), speed_(speed), speed_ang_(speed_ang) {}

        ~DwaState() {}

        float GetX() { return x_; }

        void SetX(float x) { x_ = x; }

        float GetY() { return y_; }

        void SetY(float y) { y_ = y; }

        float GetYaw() { return yaw_; }

        void SetYaw(float yaw) { yaw_ = yaw; }

        float GetSpeed() { return speed_; }

        void SetSpeed(float speed) { speed_ = speed; }

        float GetAngularSpeed() { return speed_ang_; }

        void SetAngularSpeed(float speed_ang) { speed_ang_ = speed_ang; }

        avt_341::msg::PoseStamped
        ToRosPoseStamped() {
            avt_341::msg::PoseStamped msg_posestamped;

            msg_posestamped.pose.position.x = x_;
            msg_posestamped.pose.position.y = y_;

            return msg_posestamped;
        }

    private:
        float x_;
        float y_;
        float yaw_;
        float speed_;
        float speed_ang_;
};

class DwaTrajectory {
    public:
        DwaTrajectory() {}

        ~DwaTrajectory() {}

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

    private:

        std::vector<DwaState> states_;
};

class DwaPath {
    public:
        DwaPath() {}

        ~DwaPath() {}

        void
        Add(float x, float y) {
            x_.push_back(x);
            y_.push_back(y);
        }

        int
        FindClosestDistance(float x, float y) {
            float d_min = std::numeric_limits<float>::infinity();

            for (int i = 0; i < (int)x_.size(); ++i) {
                float d_curr = std::hypot(x - x_[i], y - y_[i]);

                if (d_curr < d_min) {
                    d_min = d_curr;
                }
            }

            return d_min;
        }

    private:
        std::vector<float> x_;
        std::vector<float> y_;
};

class DwaObstacles {

    public:
        DwaObstacles() {}

        ~DwaObstacles() {}

        void Add(float x, float y) {
            x_.push_back(x);
            y_.push_back(y);
        }

        int GetNumberOfObstacles() { return (int)x_.size(); }

        float GetDistance(int i, float x, float y) { return std::hypot(x - x_[i], y - y_[i]); }

        void
        Clear() {
            x_.clear();
            y_.clear();
        }

    private:
        std::vector<float> x_;
        std::vector<float> y_;
};

class DwaCells {
    public:
        DwaCells() {}

        ~DwaCells() {}

        void Add(float x, float y, float cost) {
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
        std::vector<float> x_;
        std::vector<float> y_;
        std::vector<int> costs_;
};

class DwaPlanner {

    public:
        DwaPlanner();

        ~DwaPlanner();

        void Plan();

        float GetPlannedLinearSpeed() { return speed_lin_best_; }

        float GetPlannedAngularSpeed() { return speed_ang_best_; }

        avt_341::msg::Path GetPlannedPathRos() { return traj_best_.ToRosPath(); }

        void SetWindowLinearSpeedMin(float speed_lin_min) { speed_lin_min_ = speed_lin_min; }

        void SetWindowLinearSpeedMax(float speed_lin_max) { speed_lin_max_ = speed_lin_max; }

        void SetWindowLinearSpeedSteps(int speed_lin_steps) { speed_lin_steps_ = speed_lin_steps; }

        void SetWindowAccelerationMax(float accel_max) { accel_max_ = accel_max; }

        void SetWindowAngularSpeedMin(float speed_ang_min) { speed_ang_min_ = speed_ang_min; }

        void SetWindowAngularSpeedMax(float speed_ang_max) { speed_ang_max_ = speed_ang_max; }

        void SetWindowAngularSpeedSteps(int speed_ang_steps) { speed_ang_steps_ = speed_ang_steps; }

        void SetWindowAngularAccelerationMax(float ang_accel_max) { ang_accel_max_ = ang_accel_max; }

        void SetWindowTimeStepMin(float time_step_min) { time_step_min_ = time_step_min; }

        void SetWindowTimeSpanMin(float time_span_min) { time_span_min_ = time_span_min; }

        void SetWindowTimeSpanMax(float time_span_max) { time_span_max_ = time_span_max; }

        void SetWindowTimeSpanVariable(float time_span_var) { time_span_var_ = time_span_var; }

        void SetWindowTimeSpanGain(float time_span_gain) { time_span_gain_ = time_span_gain; }

        void SetCostGoalWeight(float w_cost_goal) { w_cost_goal_ = w_cost_goal; }

        void SetCostHeadingWeight(float w_cost_head) { w_cost_head_ = w_cost_head; }

        void SetCostSpeedWeight(float w_cost_speed) { w_cost_speed_ = w_cost_speed; }

        void SetCostObstacleWeight(float w_cost_obs) { w_cost_obs_ = w_cost_obs; }

        void SetCostGlobalPathWeight(float w_cost_path) { w_cost_global_path_ = w_cost_path; }

        void SetCostDeviationWeight(float w_cost_dev) { w_cost_dev_ = w_cost_dev; }

        void SetCostSegmentationWeight(float w_cost_seg) { w_cost_seg_ = w_cost_seg; }

        void SetObstacleThreshold(int thresh_obs) { thresh_obs_ = thresh_obs; }

        void SetCollisionRadius(float collision_radius) { collision_radius_ = collision_radius; }

        void
        SetGoal(float x, float y) {
            goal_x_ = x;
            goal_y_ = y;
        };

        void SetOccupancyGridWidth(float grid_width) { grid_occ_width_ = grid_width; }

        void SetOccupancyGridHeight(float grid_height) { grid_occ_height_ = grid_height; }

        void SetOccupancyGridOriginX(float grid_origin_x) { grid_occ_origin_x_ = grid_origin_x; }

        void SetOccupancyGridOriginY(float grid_origin_y) { grid_occ_origin_y_ = grid_origin_y; }

        void SetOccupancyGridResolution(float grid_res) { grid_occ_res_ = grid_res; }

        void SetOccupancyGridData(std::vector<signed char> grid_data) { grid_occ_data_ = grid_data; }

        void SetSegmentationGridData(std::vector<signed char> grid_data) { grid_occ_data_ = grid_data; }

        void SetGlobalPath(DwaPath path) { global_path_ = path; }

        void SetPrintSummary(bool print_summary) { print_summary_ = print_summary; }

        void
        SetState(float x, float y, float yaw, float speed, float speed_ang) {
            state_.SetX(x);
            state_.SetY(y);
            state_.SetYaw(yaw);
            state_.SetSpeed(speed);
            state_.SetAngularSpeed(speed_ang);
        }

        void
        SetMotionModel(std::string model) { model_ = model; }

        void
        SetHorizon(std::string horizon) { horizon_ = horizon; }

        bool GetUseGlobalPath() { return use_global_path_; }

        void SetUseGlobalPath(bool use_global_path) { use_global_path_ = use_global_path; }

        bool GetUseSegmentation() { return use_segmentation_; }

        void SetUseSegmentation(bool use_segmentation) { use_segmentation_ = use_segmentation; }

        void SetVehicleWheelbase(float wheelbase) { wheelbase_ = wheelbase; }

    private:
        void GetObstacles();

        DwaWindow EvaluateDynamicWindow();

        DwaState PredictMotion(DwaState state, float v, float thetadot, float time_step);

        DwaState PredictMotionSynchro(DwaState state, float speed_lin, float speed_ang, float time_step);

        DwaState PredictMotionAckermann(DwaState state, float speed, float steer, float time_step);

        DwaTrajectory PredictTrajectory(float speed, float speed_ang);

        float EvaluateCostGoal(DwaTrajectory traj);

        float EvaluateCostHeading(DwaTrajectory traj);

        float EvaluateCostObstacle(DwaTrajectory traj);

        float EvaluateCostSpeed(DwaTrajectory traj);

        float EvaluateCostGlobalPath(DwaTrajectory traj);

        float EvaluateCostDeviation(DwaTrajectory traj);

        template<typename T>
        std::vector<float>
        GetInterval(T start_in, T end_in, int num_in) {

            std::vector<float> linspaced;

            float start = static_cast<float>(start_in);
            float end = static_cast<float>(end_in);
            float num = static_cast<float>(num_in);

            if (num == 0) {
                return linspaced;
            }
            if (num == 1) {
                linspaced.push_back(start);
                return linspaced;
            }

            float delta = (end - start) / (num - 1);

            for(int i = 0; i < num - 1; ++i) {
                linspaced.push_back(start + delta * i);
            }

            linspaced.push_back(end);

            return linspaced;
        }

        int
        FindClosest(std::vector<float> const& v, int value) {
            auto const it = std::lower_bound(v.begin(), v.end(), value);

            return it - v.begin();
        }

        float speed_lin_min_;
        float speed_lin_max_;
        int speed_lin_steps_;
        float accel_max_;
        float speed_ang_min_;
        float speed_ang_max_;
        int speed_ang_steps_;
        float ang_accel_max_;
        float time_step_;
        float time_step_min_;
        float time_span_;
        float time_span_min_;
        float time_span_max_;
        float time_span_var_;
        float time_span_gain_;
        int thresh_obs_;
        float collision_radius_;
        float goal_x_;
        float goal_y_;
        float grid_occ_width_;
        float grid_occ_height_;
        float grid_occ_origin_x_;
        float grid_occ_origin_y_;
        float grid_occ_res_;
        std::vector<signed char> grid_occ_data_;
        DwaObstacles obs_occ_;
        bool use_segmentation_;
        std::vector<signed char> grid_seg_data_;
        float w_cost_goal_;
        float w_cost_head_;
        float w_cost_speed_;
        float w_cost_obs_;
        float w_cost_seg_;
        float w_cost_dev_;
        DwaState state_;
        DwaTrajectory traj_best_;
        float speed_lin_best_;
        float speed_ang_best_;
        std::string model_;
        float wheelbase_;
        std::string horizon_;
        DwaPath global_path_;
        bool use_global_path_;
        float w_cost_global_path_;
        DwaTrajectory traj_last_;
        bool has_plan_ = false;
        bool print_summary_;
};

} // end namespace planning
} // end namespace avt_341

#endif // define DWA_PLANNER_H
