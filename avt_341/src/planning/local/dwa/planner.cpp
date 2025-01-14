#include <avt_341/planning/local/dwa/planner.hpp>

namespace avt_341 {
namespace planning {
namespace dwa {

Planner::Planner() {}

void Planner::Plan() {
    // Update the obstacles collection based on the latest occupancy data.
    GetObstacles();

    // Initialise a path/control output pair (defaults to an empty trajectory
    // and zero speed).
    Trajectory traj_best;
    double speed_best = 0.0;

    // Initialise the cost to infinity (any planned path not intersecting with
    // an obstacle will have a lower cost than this).
    double cost_min = std::numeric_limits<double>::infinity();

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

    time_step_ = time_step_min_;

    // Evaluate the dynamic window based on the current AGV state.
    DynamicWindow window = EvaluateDynamicWindow();

    // Collect the speeds_lin/yaw rates.
    std::vector<double> speeds_win_lin =
        GetInterval(window.speed_min_, window.speed_max_, speed_lin_steps_);
    std::vector<double> speeds_win_ang = GetInterval(window.speed_ang_min_,
                                                     window.speed_ang_max_,
                                                     speed_ang_steps_);

    // Add a zero angular speed state to preserve the current heading.
    // Keep the angular speed vector sorted to allow for easier debugging.
    if(!(std::find(speeds_win_ang.begin(), speeds_win_ang.end(), 0.0f) !=
         speeds_win_ang.end())) {
        speeds_win_ang.insert(speeds_win_ang.begin() +
                                  FindClosest(speeds_win_ang, 0.0f),
                              0.0f);
    }

    // Filter out trajectories which exceed the maximum lateral acceleration.
    bool not_ackermann = model_ != "ackermann";
    std::vector<utils::ivec2> search_actions;
    for(int i = 0; i < (int)speeds_win_lin.size(); ++i) {
        for(int j = 0; j < (int)speeds_win_ang.size(); ++j) {
            // Compute the lateral acceleration for a kinematic bicycle
            // model.
            // NOTE: Recall the angular speed variable does not hold
            // an angular speed, but rather a steering angle in the
            // Ackermann motion model for this planner.
            if(not_ackermann ||
               !(std::abs((speeds_win_lin[i] * speeds_win_lin[i]) /
                          (wheelbase_ / std::tan(speeds_win_ang[j]))) >
                 lat_accel_max_)) {
                search_actions.push_back(utils::ivec2(i, j));
            }
        }
    }

    int obs_size = obs_occ_.GetNumberOfObstacles();
    int window_size = (int)search_actions.size();

    auto cost_speed = Cost(window_size);
    auto cost_obs = Cost(window_size);
    auto cost_goal = Cost(window_size);
    auto cost_head = Cost(window_size);
    auto cost_seg = Cost(window_size);
    auto cost_path = Cost(window_size);
    auto cost_dev = Cost(window_size);

    trajectories_.clear();

    // Iterate through the speed/yaw rate pairs in the dynamic window
    for(int k = 0; k < (int)search_actions.size(); k++) {
        int i = search_actions[k].x;
        int j = search_actions[k].y;

        // Compute the predicted trajectory for this speed/yaw rate pair by
        // integrating the motion model.
        Trajectory traj =
            PredictTrajectory(speeds_win_lin[i], speeds_win_ang[j]);

        trajectories_.push_back(traj);

        // Evaluate the cost terms
        cost_goal.Add(k, EvaluateCostGoal(traj));
        cost_head.Add(k, EvaluateCostHeading(traj));
        if(obs_size > 0) cost_obs.Add(k, EvaluateCostObstacle(traj));
        cost_speed.Add(k, EvaluateCostSpeed(traj));
        if(use_segmentation_) cost_seg.Add(k, EvaluateCostSegmentation(traj));

        // Evaluate the optional cost terms
        // If at least one planning step was performed, use it to penalise
        // deviations from the current path.
        if(use_global_path_) cost_path.Add(k, EvaluateCostGlobalPath(traj));
        if(has_plan_) cost_dev.Add(k, EvaluateCostDeviation(traj));
    }

    // Find the minimum of the objective function.
    // Assume the initial speed/yaw pair is optimal
    int k_min = 0;
    for(int k = 0; k < window_size; ++k) {
        double cost = w_cost_goal_ * cost_goal.GetCost(k) +
            w_cost_head_ * cost_head.GetCost(k) +
            w_cost_speed_ * cost_speed.GetCost(k);

        if(obs_size > 0) cost += w_cost_obs_ * cost_obs.GetCost(k);
        if(use_segmentation_) cost += w_cost_seg_ * cost_seg.GetCost(k);
        if(use_global_path_ > 0)
            cost += w_cost_global_path_ * cost_path.GetCost(k);
        if(has_plan_) cost += w_cost_dev_ * cost_dev.GetCost(k);

        trajectories_[k].SetCost(cost);

        if(cost > max_cost_) { max_cost_ = cost; }

        if(cost_min >= cost) {
            cost_min = cost;
            k_min = k;
        }
    }

    if(cost_min == std::numeric_limits<double>::infinity()) {
        std::cout << "WARNING: ALL PLANNED PATHS INTERSECT WITH AT LEAST ONE "
                     "OBSTACLE!\n";
    }

    // Compute and store the optimal trajectory and longitudinal speed.
    // NOTE: While we already computed this trajectory before, it is more
    // efficient to compute one more trajectory than to manage all candidate
    // trajectories in memory.
    int i_min = search_actions[k_min].x;
    int j_min = search_actions[k_min].y;
    traj_best_ =
        PredictTrajectory(speeds_win_lin[i_min], speeds_win_ang[j_min]);
    speed_lin_best_ = speeds_win_lin[i_min];
    speed_ang_best_ = speeds_win_ang[j_min];

    // Mark the planner as executed at least once and save the optimal path to
    // penalise deviations in future planning steps.
    has_plan_ = true;
    traj_last_ = traj_best_;

    // Print summary of the planning step
    if(print_summary_) {
        std::cout << std::fixed << std::setfill('0')
                  << "\nDWA PLANNER RESULTS\n===================\nObjective "
                     "function >> OF: "
                  << cost_min << "\nDynamic window:\n\t"
                  << "\n\tLSMIN:\t" << window.speed_min_ << "\n\tLSMAX:\t"
                  << window.speed_max_ << "\n\tYRMIN:\t"
                  << window.speed_ang_min_ << "\n\tYRMAX:\t"
                  << window.speed_ang_max_
                  << "\nOptimal control >> LS: " << speeds_win_lin[i_min]
                  << " YR: " << speeds_win_ang[j_min]
                  << "\n\nNumber of obstacles: " << obs_size
                  << "\nCosts:\n\tName (Weight) "
                     "[Cost]\n\t=====================================\n\tGoal "
                     "cost:\t("
                  << std::setprecision(2) << w_cost_goal_ << ") ["
                  << std::setprecision(3) << cost_goal.GetCost(k_min) << "] {"
                  << "}\n\tHeading cost:\t(" << std::setprecision(2)
                  << w_cost_head_ << ") [" << std::setprecision(3)
                  << cost_head.GetCost(k_min) << "] {"
                  << "}\n\tObstacle cost:\t(" << std::setprecision(2)
                  << w_cost_obs_ << ") [" << std::setprecision(3)
                  << cost_obs.GetCost(k_min) << "] {"
                  << "}\n\tSegmentation cost:\t(" << std::setprecision(2)
                  << w_cost_seg_ << ") [" << std::setprecision(3)
                  << cost_seg.GetCost(k_min) << "] {"
                  << "}\n\tSpeed cost:\t(" << std::setprecision(2)
                  << w_cost_speed_ << ") [" << std::setprecision(3)
                  << cost_speed.GetCost(k_min) << "] {"
                  << "}\n\tGlobal path cost:\t(" << std::setprecision(2)
                  << w_cost_global_path_ << ") [" << std::setprecision(3)
                  << cost_path.GetCost(k_min) << "] {"
                  << "}\n\tDeviation cost:\t(" << std::setprecision(2)
                  << w_cost_dev_ << ") [" << std::setprecision(3)
                  << cost_dev.GetCost(k_min) << "] {"
                  << "}\n";
    }
}

double Planner::GetPlannedLinearSpeed() { return speed_lin_best_; }

double Planner::GetPlannedAngularSpeed() { return speed_ang_best_; }

avt_341::msg::Path Planner::GetPlannedPathRos() {
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

void Planner::SetWindowTimeStepMin(double time_step_min) {
    time_step_min_ = time_step_min;
}

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

std::vector<Trajectory> Planner::GetTrajectories() { return trajectories_; }

double Planner::GetMaxCost() { return max_cost_; }

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

DynamicWindow Planner::EvaluateDynamicWindow() {
    double model_speed_lin_min = state_.GetSpeed() - accel_max_ * time_span_;
    double model_speed_lin_max = state_.GetSpeed() + accel_max_ * time_span_;
    double model_speed_ang_min =
        state_.GetAngularSpeed() - ang_accel_max_ * time_span_;
    double model_speed_ang_max =
        state_.GetAngularSpeed() + ang_accel_max_ * time_span_;

    return DynamicWindow(std::max(speed_lin_min_, model_speed_lin_min),
                         std::min(speed_lin_max_, model_speed_lin_max),
                         std::max(speed_ang_min_, model_speed_ang_min),
                         std::min(speed_ang_max_, model_speed_ang_max));
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
    Trajectory traj;

    // Add current pose to the trajectory.
    traj.Add(state_);

    // Predict the AGV motion through the current horizon starting from the
    // current pose.
    double time = 0.0;
    State state_curr = state_;
    while(time < time_span_) {
        state_curr = PredictMotion(state_curr, speed, speed_ang, time_step_);
        time += time_step_;
        traj.Add(state_curr);
    }

    return traj;
}

double Planner::EvaluateCostGoal(Trajectory traj) {
    double dx = goal_x_ - traj.GetLastState().GetX();
    double dy = goal_y_ - traj.GetLastState().GetY();

    return std::hypot(dx, dy);
}

double Planner::EvaluateCostHeading(Trajectory traj) {
    double dx = goal_x_ - traj.GetLastState().GetX();
    double dy = goal_y_ - traj.GetLastState().GetY();

    double error_angle = std::atan2(dy, dx);

    double cost_angle = error_angle - traj.GetLastState().GetYaw();

    return std::abs(std::atan2(std::sin(cost_angle), std::cos(cost_angle)));
}

double Planner::EvaluateCostObstacle(Trajectory traj) {
    // Initialise the minimum distance to an obstacle to a very large value.
    double d_min = std::numeric_limits<double>::infinity();

    for(int j = 0; j < traj.GetNumberOfStates(); j++) {
        for(int i = 0; i < obs_occ_.GetNumberOfObstacles(); ++i) {
            double d = obs_occ_.GetDistance(i,
                                            traj.GetState(j).GetX(),
                                            traj.GetState(j).GetY());

            if(d < d_min) { d_min = d; }
        }
    }

    // Trajectories intersecting collision radii are significantly higher cost
    // than others. Note that these must still yield a finite cost, as the
    // planner must rely on the other terms to discern between all
    // obstacles-intersecting trajectories.
    if(d_min <= collision_radius_) {
        return 1000.0f * (1 / (d_min * d_min * d_min));
    }

    return 1 / (d_min * d_min);
}

double Planner::EvaluateCostSegmentation(Trajectory traj) {
    double segmentation_cost = 0.0;
    for(int k = 0; k < traj.GetNumberOfStates(); k++) {
        int i = (traj.GetState(k).GetX() - grid_occ_origin_x_) / grid_occ_res_ -
            0.5f;
        int j = (traj.GetState(k).GetY() - grid_occ_origin_y_) / grid_occ_res_ -
            0.5f;
        if(i < 0) continue;
        if(j < 0) continue;
        if(i >= grid_occ_width_) continue;
        if(j >= grid_occ_height_) continue;
        unsigned int ndx = j * grid_occ_width_ + i;
        int cost = (int)grid_seg_data_[ndx];
        if(cost > thresh_seg_) {
            segmentation_cost = 1000000000.0;
            return segmentation_cost;
        }
        segmentation_cost += cost;
    }
    return segmentation_cost;
}

double Planner::EvaluateCostSpeed(Trajectory traj) {
    return std::max(speed_lin_max_ - traj.GetLastState().GetSpeed(), 0.0);
}

double Planner::EvaluateCostGlobalPath(Trajectory traj) {
    double cost_path = 0.0f;

    for(int i = 0; i < traj.GetNumberOfStates(); ++i) {
        cost_path += global_path_.FindClosestDistance(traj.GetState(i).GetX(),
                                                      traj.GetState(i).GetY());
    }

    return cost_path;
}

double Planner::EvaluateCostDeviation(Trajectory traj) {
    double dx = traj_last_.GetLastState().GetX() - traj.GetLastState().GetX();
    double dy = traj_last_.GetLastState().GetY() - traj.GetLastState().GetY();

    return std::hypot(dx, dy);
}

int Planner::FindClosest(std::vector<double> const& v, int value) {
    auto const it = std::lower_bound(v.begin(), v.end(), value);

    return it - v.begin();
}

} // namespace dwa
} // namespace planning
} // namespace avt_341