#include <avt_341_nav/planning/local/dwa/planner.hpp>
#include "nav_msgs/msg/path.hpp"
#include "avt_341_nav/core/math_dto.hpp"

namespace avt_341_nav {
namespace planning {
namespace dwa {

Planner::Planner() {}

void Planner::Plan() {
    // Update the obstacles collection based on the latest occupancy data.
    GetObstacles();

    // Update prediction time span
    double speed_norm = 1.0f / (speed_lin_max_ - speed_lin_min_) *
            (state_.GetSpeed() - speed_lin_max_) +
        1.0f;
    if(speed_norm > speed_lin_max_) {
        speed_norm = 1.0f;
    } else if(speed_norm <= speed_lin_min_) {
        speed_norm = 0.0f;
    }

    time_span_ = (horizon_ == "adaptive")
        ? time_span_min_ + time_span_gain_ * speed_norm * time_span_var_
        : time_span_min_;
    if(time_span_ > time_span_max_) { time_span_ = time_span_max_; }

    boost::algorithm::clamp(time_span_, time_span_min_, time_span_max_);

    // Update the dynamic window based on the current vehicle state and collect
    // the candidate speeds and steering rates based on the updated thresholds.
    EvaluateDynamicWindow();
    auto candidate_speeds = GetInterval(dynamic_window_.GetMinimumSpeed(),
                                        dynamic_window_.GetMaximumSpeed(),
                                        speed_lin_steps_);
    auto candidate_steering_rates =
        GetInterval(dynamic_window_.GetMinimumSteeringRate(),
                    dynamic_window_.GetMaximumSteeringRate(),
                    speed_ang_steps_);

    // Add a zero angular speed state to preserve the current heading.
    // Keep the angular speed vector sorted to allow for easier debugging.
    if(!(std::find(candidate_steering_rates.begin(),
                   candidate_steering_rates.end(),
                   0.0f) != candidate_steering_rates.end())) {
        candidate_steering_rates.insert(
            candidate_steering_rates.begin() +
                FindClosest(candidate_steering_rates, 0.0f),
            0.0f);
    }

    // Filter out trajectories which exceed the maximum lateral acceleration.
    bool not_ackermann = model_ != "ackermann";
    std::vector<core::ivec2> search_actions;
    for(int i = 0; i < (int)candidate_speeds.size(); ++i) {
        for(int j = 0; j < (int)candidate_steering_rates.size(); ++j) {
            // Compute the lateral acceleration for a kinematic bicycle
            // model.
            // NOTE: Recall the angular speed variable does not hold
            // an angular speed, but rather a steering angle in the
            // Ackermann motion model for this planner.
            if(not_ackermann ||
               !(std::abs(
                     (candidate_speeds[i] * candidate_speeds[i]) /
                     (wheelbase_ / std::tan(candidate_steering_rates[j]))) >
                 lat_accel_max_)) {
                search_actions.push_back(core::ivec2(i, j));
            }
        }
    }

    int obs_size = obs_occ_.GetNumberOfObstacles();

    trajectories_.clear();
    trajectories_.resize(search_actions.size());

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    // Iterate through the speed/yaw rate pairs in the dynamic window
    // NOTE: Old OpenMP version on windows does not support size_t parallel for
    for(int k = 0; k < search_actions.size(); k++) {
        int i = search_actions[k].x;
        int j = search_actions[k].y;

        // Compute the predicted trajectory for this speed/yaw rate pair by
        // integrating the motion model.
        trajectories_[k] =
            PredictTrajectory(candidate_speeds[i], candidate_steering_rates[j]);

        // Evaluate the individual weighted cost terms of the objective
        // function and compute the cumulative objective function value.
        trajectories_[k].EvaluateGoalCost(w_cost_goal_, goal_x_, goal_y_);

        if(obs_size > 0)
            trajectories_[k].EvaluateObstacleCost(w_cost_obs_,
                                                  obs_occ_,
                                                  collision_radius_);

        if(use_segmentation_)
            trajectories_[k].EvaluateSegmentationCost(w_cost_seg_,
                                                      grid_seg_data_,
                                                      grid_occ_width_,
                                                      grid_occ_height_,
                                                      grid_occ_origin_x_,
                                                      grid_occ_origin_y_,
                                                      grid_occ_res_,
                                                      thresh_seg_);

        trajectories_[k].EvaluateHeadingCost(w_cost_head_, goal_x_, goal_y_);

        trajectories_[k].EvaluateSpeedCost(w_cost_speed_, speed_lin_min_);

        if(use_global_path_)
            trajectories_[k].EvaluateGlobalPathCost(w_cost_global_path_,
                                                    global_path_);

        if(has_plan_)
            trajectories_[k].EvaluateDeviationCost(w_cost_dev_, traj_last_);

        trajectories_[k].EvaluateTotalCost();
    }

    // Find the minimum of the objective function.
    int k_min = 0;
    min_cost_ = std::numeric_limits<double>::infinity();
    max_cost_ = -std::numeric_limits<double>::infinity();
    for(size_t k = 0; k < search_actions.size(); ++k) {
        auto cost = trajectories_[k].GetTotalCost();

        if(cost > max_cost_) { max_cost_ = cost; }

        if(cost <= min_cost_) {
            min_cost_ = cost;
            k_min = k;
        }
    }

    // Compute and store the optimal trajectory and longitudinal speed.
    // NOTE: While we already computed this trajectory before, it is more
    // efficient to compute one more trajectory than to manage all candidate
    // trajectories in memory.
    int i_min = search_actions[k_min].x;
    int j_min = search_actions[k_min].y;
    traj_best_ = trajectories_[k_min];
    speed_lin_best_ = candidate_speeds[i_min];
    speed_ang_best_ = candidate_steering_rates[j_min];

    // Mark the planner as executed at least once and save the optimal path to
    // penalise deviations in future planning steps.
    has_plan_ = true;
    traj_last_ = traj_best_;
}

double Planner::GetPlannedLinearSpeed() { return speed_lin_best_; }

double Planner::GetPlannedAngularSpeed() { return speed_ang_best_; }

const Trajectory& Planner::GetOptimalTrajectory() const { return traj_best_; }

nav_msgs::msg::Path Planner::GetPlannedPathRos() {
    return traj_best_.ToRosPath();
}

void Planner::SetWindowLinearSpeedMin(double speed_lin_min) {
    speed_lin_min_ = speed_lin_min;
}

void Planner::SetWindowLinearSpeedMax(double speed_lin_max) {
    speed_lin_max_ = speed_lin_max;
}

void Planner::SetWindowLinearSpeedSteps(int speed_lin_steps) {
    speed_lin_steps_ = speed_lin_steps;
}

void Planner::SetWindowAccelerationMax(double accel_max) {
    accel_max_ = accel_max;
}

void Planner::SetWindowAngularSpeedMin(double speed_ang_min) {
    speed_ang_min_ = speed_ang_min;
}

void Planner::SetWindowAngularSpeedMax(double speed_ang_max) {
    speed_ang_max_ = speed_ang_max;
}

void Planner::SetWindowAngularSpeedSteps(int speed_ang_steps) {
    speed_ang_steps_ = speed_ang_steps;
}

void Planner::SetWindowAngularAccelerationMax(double ang_accel_max) {
    ang_accel_max_ = ang_accel_max;
}

void Planner::SetLateralAccelerationMax(double lat_accel_max) {
    lat_accel_max_ = lat_accel_max;
}

void Planner::SetTimeStep(double time_step) { time_step_ = time_step; }

void Planner::SetWindowTimeSpanMin(double time_span_min) {
    time_span_min_ = time_span_min;
}

void Planner::SetWindowTimeSpanMax(double time_span_max) {
    time_span_max_ = time_span_max;
}

void Planner::SetWindowTimeSpanVariable(double time_span_var) {
    time_span_var_ = time_span_var;
}

void Planner::SetWindowTimeSpanGain(double time_span_gain) {
    time_span_gain_ = time_span_gain;
}

void Planner::SetCostGoalWeight(double w_cost_goal) {
    w_cost_goal_ = w_cost_goal;
}

void Planner::SetCostHeadingWeight(double w_cost_head) {
    w_cost_head_ = w_cost_head;
}

void Planner::SetCostSpeedWeight(double w_cost_speed) {
    w_cost_speed_ = w_cost_speed;
}

void Planner::SetCostObstacleWeight(double w_cost_obs) {
    w_cost_obs_ = w_cost_obs;
}

void Planner::SetCostGlobalPathWeight(double w_cost_path) {
    w_cost_global_path_ = w_cost_path;
}

void Planner::SetCostDeviationWeight(double w_cost_dev) {
    w_cost_dev_ = w_cost_dev;
}

void Planner::SetCostSegmentationWeight(double w_cost_seg) {
    w_cost_seg_ = w_cost_seg;
}

void Planner::SetObstacleThreshold(int thresh_obs) { thresh_obs_ = thresh_obs; }

void Planner::SetCollisionRadius(double collision_radius) {
    collision_radius_ = collision_radius;
}

void Planner::SetObstacleSearch(std::string obs_search) {
    // Search modality defaults to adaptive when providing a wrong parameter.
    if((obs_search != "adaptive") || (obs_search != "fixed")) {
        obs_search_ = "adaptive";
    } else {
        obs_search_ = obs_search;
    }
}

void Planner::SetObstacleSearchRadius(double search_radius) {
    search_radius_ = search_radius;
}

void Planner::SetGoal(double x, double y) {
    goal_x_ = x;
    goal_y_ = y;
};

void Planner::SetOccupancyGridWidth(double grid_width) {
    grid_occ_width_ = grid_width;
}

void Planner::SetOccupancyGridHeight(double grid_height) {
    grid_occ_height_ = grid_height;
}

void Planner::SetOccupancyGridOriginX(double grid_origin_x) {
    grid_occ_origin_x_ = grid_origin_x;
}

void Planner::SetOccupancyGridOriginY(double grid_origin_y) {
    grid_occ_origin_y_ = grid_origin_y;
}

void Planner::SetOccupancyGridResolution(double grid_res) {
    grid_occ_res_ = grid_res;
}

void Planner::SetOccupancyGridData(std::vector<signed char> grid_data) {
    grid_occ_data_ = grid_data;
}

void Planner::SetSegmentationGridData(std::vector<signed char> grid_data) {
    grid_seg_data_ = grid_data;
}

void Planner::SetSegmentationThreshold(int thresh_seg) {
    thresh_seg_ = thresh_seg;
}

void Planner::SetGlobalPath(Path path) { global_path_ = path; }

void Planner::SetPrintSummary(bool print_summary) {
    print_summary_ = print_summary;
}

void Planner::SetState(double x,
                       double y,
                       double yaw,
                       double speed,
                       double speed_ang) {
    state_.SetX(x);
    state_.SetY(y);
    state_.SetYaw(yaw);
    state_.SetSpeed(speed);
    state_.SetAngularSpeed(speed_ang);
}

void Planner::SetHorizon(std::string horizon) { horizon_ = horizon; }

bool Planner::GetUseGlobalPath() { return use_global_path_; }

void Planner::SetUseGlobalPath(bool use_global_path) {
    use_global_path_ = use_global_path;
}

bool Planner::GetUseSegmentation() { return use_segmentation_; }

void Planner::SetUseSegmentation(bool use_segmentation) {
    use_segmentation_ = use_segmentation;
}

void Planner::SetVehicleWheelbase(double wheelbase) { wheelbase_ = wheelbase; }

void Planner::Reset() { traj_best_.Reset(); }

const std::vector<Trajectory>& Planner::GetTrajectories() const {
    return trajectories_;
}

const double& Planner::GetMaxCost() const { return max_cost_; }

const double& Planner::GetMinCost() const { return min_cost_; }

void Planner::GetObstacles() {
    // Clear the previous obstacle list.
    obs_occ_.Clear();

    double dis_cutoff = search_radius_;
    if(obs_search_ == "adaptive") {
        // The cutoff distance is selected based on the maximum travel distance
        // throughout the window (with a constant bias term equal to the
        // collision radius).
        dis_cutoff = collision_radius_ +
            0.75f *
                (speed_lin_max_ * time_span_ +
                 0.5 * accel_max_ * time_span_ * time_span_);
    }

    for(int i = 0; i < grid_occ_width_; ++i) {
        for(int j = 0; j < grid_occ_height_; ++j) {
            double x = grid_occ_origin_x_ + (i + 0.5f) * grid_occ_res_;
            double y = grid_occ_origin_y_ + (j + 0.5f) * grid_occ_res_;

            unsigned int ndx = j * grid_occ_width_ + i;

            int cost = (int)grid_occ_data_[ndx];

            if(cost > thresh_obs_) {
                double d = std::hypot(state_.GetX() - x, state_.GetY() - y);

                if(d < dis_cutoff) { obs_occ_.Add(x, y); }
            }
        }
    }
}

void Planner::EvaluateDynamicWindow() {
    dynamic_window_.Update(
        std::max(speed_lin_min_, state_.GetSpeed() - accel_max_ * time_span_),
        std::min(speed_lin_max_, state_.GetSpeed() + accel_max_ * time_span_),
        std::max(speed_ang_min_,
                 state_.GetAngularSpeed() - ang_accel_max_ * time_span_),
        std::min(speed_ang_max_,
                 state_.GetAngularSpeed() + ang_accel_max_ * time_span_));
}

State Planner::PredictMotion(State state,
                             double v,
                             double steer,
                             double time_step) {
    return State(state.GetX() + v * std::cos(state.GetYaw()) * time_step,
                 state.GetY() + v * std::sin(state.GetYaw()) * time_step,
                 state.GetYaw() +
                     state.GetSpeed() * std::atan(steer) / wheelbase_ *
                         time_step,
                 v,
                 steer);
}

Trajectory Planner::PredictTrajectory(double speed, double speed_ang) {
    // Initialise an empty trajectory.
    Trajectory trajectory;

    // Add current pose to the trajectory.
    trajectory.Add(state_);

    // Predict the AGV motion through the current horizon starting from the
    // current pose.
    double time = 0.0;
    State state_curr = state_;
    while(time < time_span_) {
        state_curr = PredictMotion(state_curr, speed, speed_ang, time_step_);
        time += time_step_;
        trajectory.Add(state_curr);
    }

    return trajectory;
}

int Planner::FindClosest(std::vector<double> const& v, int value) {
    auto const it = std::lower_bound(v.begin(), v.end(), value);

    return it - v.begin();
}

} // namespace dwa
} // namespace planning
} // namespace avt_341_nav