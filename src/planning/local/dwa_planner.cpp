#include "avt_341/planning/local/dwa_planner.h"

namespace avt_341 {
namespace planning {

DwaPlanner::DwaPlanner() {}

DwaPlanner::~DwaPlanner() {}

DwaWindow
DwaPlanner::EvaluateDynamicWindow() {
    float model_speed_lin_min = state_.GetSpeed() - accel_max_ * time_span_;
    float model_speed_lin_max = state_.GetSpeed() + accel_max_ * time_span_;
    float model_speed_ang_min = state_.GetAngularSpeed() - ang_accel_max_ * time_span_;
    float model_speed_ang_max = state_.GetAngularSpeed() + ang_accel_max_ * time_span_;

    return DwaWindow(
        std::max(speed_lin_min_, model_speed_lin_min),
        std::min(speed_lin_max_, model_speed_lin_max),
        std::max(speed_ang_min_, model_speed_ang_min),
        std::min(speed_ang_max_, model_speed_ang_max)
    );
}

void
DwaPlanner::Plan() {
    // Update the obstacles collection based on the latest occupancy data.
    GetObstacles();

    // Initialise a path/control output pair (defaults to an empty trajectory and zero speed).
    DwaTrajectory traj_best;
    float speed_best = 0.0;

    // Initialise the cost to infinity (any planned path not intersecting with an obstacle will have a lower cost than this).
    float cost_min = std::numeric_limits<float>::infinity();

    // Update prediction time span
    float speed_norm = 1.0f / (speed_lin_max_ - speed_lin_min_) * (state_.GetSpeed() - speed_lin_max_ ) + 1.0f;
    if (speed_norm > speed_lin_max_) {
        speed_norm = 1.0f;
    } else if (speed_norm <= speed_lin_min_) {
        speed_norm = 0.0f;
    }

    time_span_ = (horizon_ == "adaptive") ? time_span_min_ + time_span_gain_ * speed_norm * time_span_var_ : time_span_min_;
    if (time_span_ > time_span_max_) {
        time_span_ = time_span_max_;
    }

    time_step_ = time_step_min_;

    // Evaluate the dynamic window based on the current AGV state.
    DwaWindow window = EvaluateDynamicWindow();

    // Collect the speeds_lin/yaw rates.
    std::vector<float> speeds_win_lin = GetInterval(window.speed_min_, window.speed_max_, speed_lin_steps_);
    std::vector<float> speeds_win_ang = GetInterval(window.speed_ang_min_, window.speed_ang_max_, speed_ang_steps_);

    // Add a zero angular speed state to preserve the current heading.
    // Keep the angular speed vector sorted to allow for easier debugging.
    if (!(std::find(speeds_win_ang.begin(), speeds_win_ang.end(), 0.0f) != speeds_win_ang.end())) {
        speeds_win_ang.insert(speeds_win_ang.begin() + FindClosest(speeds_win_ang, 0.0f), 0.0f);
    }

    std::vector<float> speeds_lin, speeds_ang;

    // Filter out trajectories which exceed the maximum lateral acceleration.
    if (model_ == "ackermann") {
        for (int i = 0; i < (int)speeds_win_lin.size(); ++i) {
            for (int j = 0; j < (int)speeds_win_ang.size(); ++j) {
                // Compute the lateral acceleration for a kinematic bicycle
                // model.
                // NOTE: Recall the angular speed variable does not hold
                // an angular speed, but rather a steering angle in the
                // Ackermann motion model for this planner.
                if (!(std::abs((speeds_win_lin[i] * speeds_win_lin[i]) / (wheelbase_ / std::tan(speeds_win_ang[j]))) > lat_accel_max_)) {
                    speeds_lin.push_back(speeds_win_lin[i]);
                    speeds_ang.push_back(speeds_win_ang[j]);
                }
            }
        }
    } else {
        speeds_lin = speeds_win_lin;
        speeds_ang = speeds_win_ang;
    }


    int obs_size = obs_occ_.GetNumberOfObstacles();

    int speeds_lin_size = (int)speeds_lin.size();
    int speeds_ang_size = (int)speeds_ang.size();
    int window_size = speeds_lin_size * speeds_ang_size;

    auto cost_speed = DwaCost(window_size);
    auto cost_obs = DwaCost(window_size);
    auto cost_goal = DwaCost(window_size);
    auto cost_head = DwaCost(window_size);
    auto cost_seg = DwaCost(window_size);
    auto cost_path = DwaCost(window_size);
    auto cost_dev = DwaCost(window_size);

    // Iterate through the speed/yaw rate pairs in the dynamic window
    #pragma omp parallel for schedule(static) collapse(2)
    for (int i = 0; i < speeds_lin_size; ++i) {
        for (int j = 0; j < speeds_ang_size; ++j) {
            // Compute flattened index for the window
            int k = i * speeds_ang_size + j;

            // Compute the predicted trajectory for this speed/yaw rate pair by integrating the motion model.
            DwaTrajectory traj = PredictTrajectory(speeds_lin[i], speeds_ang[j]);

            // Evaluate the cost terms
            cost_goal.Add(k, EvaluateCostGoal(traj));
            cost_head.Add(k, EvaluateCostHeading(traj));
            if (obs_size > 0) cost_obs.Add(k, EvaluateCostObstacle(traj));
            cost_speed.Add(k, EvaluateCostSpeed(traj));

            // Evaluate the optional cost terms
            // If at least one planning step was performed, use it to penalise deviations from the current path.
            if (use_global_path_) cost_path.Add(k, EvaluateCostGlobalPath(traj));
            if (has_plan_) cost_dev.Add(k, EvaluateCostDeviation(traj));
        }
    }

    // Normalise the cost terms.
    cost_goal.Normalise();
    cost_speed.Normalise();

    // Normalise the optional cost terms.
    if (use_segmentation_) cost_seg.Normalise();
    if (use_global_path_) cost_path.Normalise();
    if (has_plan_) cost_dev.Normalise();

    // Find the minimum of the objective function.
    // Assume the initial speed/yaw pair is optimal
    int k_min = 0;
    for (int k = 0; k < window_size; ++k) {
        float cost =
            w_cost_goal_ * cost_goal.GetNormalisedCost(k) +
            w_cost_head_ * cost_head.GetCost(k) +
            w_cost_speed_ * cost_speed.GetNormalisedCost(k);

        if (obs_size > 0) cost += w_cost_obs_ * cost_obs.GetCost(k);
        if (use_global_path_ > 0) cost += w_cost_global_path_ * cost_path.GetNormalisedCost(k);
        if (has_plan_) cost += w_cost_dev_ * cost_dev.GetNormalisedCost(k);

        if (cost_min >= cost) {
            cost_min = cost;
            k_min = k;
        }
    }

    if (cost_min == std::numeric_limits<float>::infinity()) {
        std::cout << "WARNING: ALL PLANNED PATHS INTERSECT WITH AT LEAST ONE OBSTACLE!\n";
    }

    // Compute and store the optimal trajectory and longitudinal speed.
    // NOTE: While we already computed this trajectory before, it is more efficient to compute one more trajectory than to manage all candidate trajectories in memory.
    int i_min = std::round(k_min / speeds_ang_size);
    int j_min = k_min % speeds_ang_size;
    traj_best_ = PredictTrajectory(speeds_lin[i_min], speeds_ang[j_min]);
    speed_lin_best_ = speeds_lin[i_min];
    speed_ang_best_ = speeds_ang[j_min];

    // Mark the planner as executed at least once and save the optimal path to penalise deviations in future planning steps.
    has_plan_ = true;
    traj_last_ = traj_best_;

    // Print summary of the planning step
    if (print_summary_) {
        std::cout << std::fixed << std::setfill('0')
            << "\nDWA PLANNER RESULTS\n===================\nObjective function >> OF: " << cost_min
            << "\nDynamic window:\n\t"
                << "\n\tLSMIN:\t"<< window.speed_min_
                << "\n\tLSMAX:\t"<< window.speed_max_
                << "\n\tYRMIN:\t"<< window.speed_ang_min_
                << "\n\tYRMAX:\t"<< window.speed_ang_max_
            << "\nOptimal control >> LS: " << speeds_lin[i_min] << " YR: " << speeds_ang[j_min]
            << "\n\nNumber of obstacles: " << obs_size
            << "\nCosts:\n\tName (Weight) [Cost] {Normalised cost}\n\t=====================================\n\tGoal cost:\t("
            << std::setprecision(2) << w_cost_goal_ << ") ["
            << std::setprecision(3) << cost_goal.GetCost(k_min) << "] {"
            << std::setprecision(2) << cost_goal.GetNormalisedCost(k_min)
            << "}\n\tHeading cost:\t("
            << std::setprecision(2) << w_cost_head_ << ") ["
            << std::setprecision(3) << cost_head.GetCost(k_min) << "] {"
            << std::setprecision(2) << cost_head.GetNormalisedCost(k_min)
            << "}\n\tObstacle cost:\t("
            << std::setprecision(2) << w_cost_obs_ << ") ["
            << std::setprecision(3) << cost_obs.GetCost(k_min) << "] {"
            << std::setprecision(2) << cost_obs.GetNormalisedCost(k_min)
            << "}\n\tSpeed cost:\t("
            << std::setprecision(2) << w_cost_speed_ << ") ["
            << std::setprecision(3) << cost_speed.GetCost(k_min) << "] {"
            << std::setprecision(2) << cost_speed.GetNormalisedCost(k_min)
            << "}\n\tGlobal path cost:\t("
            << std::setprecision(2) << w_cost_global_path_ << ") ["
            << std::setprecision(3) << cost_path.GetCost(k_min) << "] {"
            << std::setprecision(2) << cost_path.GetNormalisedCost(k_min)
            << "}\n\tDeviation cost:\t("
            << std::setprecision(2) << w_cost_dev_ << ") ["
            << std::setprecision(3) << cost_dev.GetCost(k_min) << "] {"
            << std::setprecision(2) << cost_dev.GetNormalisedCost(k_min)
            << "}\n";
    }
}

DwaState
DwaPlanner::PredictMotion(DwaState state, float v, float thetadot, float time_step) {
    if (model_ == "ackermann") {
        return PredictMotionAckermann(state, v, thetadot, time_step);
    }

    return PredictMotionSynchro(state, v, thetadot, time_step);
}

DwaState
DwaPlanner::PredictMotionSynchro(DwaState state, float v, float thetadot, float time_step) {
    return DwaState(
        state.GetX() + v * std::cos(state.GetYaw()) * time_step,
        state.GetY() + v * std::sin(state.GetYaw()) * time_step,
        state.GetYaw() + thetadot * time_step,
        v,
        thetadot
    );
}

DwaState
DwaPlanner::PredictMotionAckermann(DwaState state, float v, float steer, float time_step) {
    return DwaState(
        state.GetX() + v * std::cos(state.GetYaw()) * time_step,
        state.GetY() + v * std::sin(state.GetYaw()) * time_step,
        state.GetYaw() + state.GetSpeed() * std::atan(steer) / wheelbase_ * time_step,
        v,
        steer
    );
}

DwaTrajectory
DwaPlanner::PredictTrajectory(float speed, float speed_ang) {
    // Initialise an empty trajectory.
    DwaTrajectory traj;

    // Add current pose to the trajectory.
    traj.Add(state_);

    // Predict the AGV motion through the current horizon starting from the current pose.
    float time = 0.0;
    DwaState state_curr = state_;
    while (time < time_span_) {
        state_curr = PredictMotion(state_curr, speed, speed_ang, time_step_);
        time += time_step_;
        traj.Add(state_curr);
    }

    return traj;
}

float
DwaPlanner::EvaluateCostObstacle(DwaTrajectory traj) {
    // Initialise the minimum distance to an obstacle to a very large value.
    float d_min = std::numeric_limits<float>::infinity();

    #pragma omp parallel for schedule(static) collapse(2)
    for (int j = 0; j < traj.GetNumberOfStates(); j++) {
        for (int i = 0; i < obs_occ_.GetNumberOfObstacles(); ++i) {
            float d = obs_occ_.GetDistance(i, traj.GetState(j).GetX(), traj.GetState(j).GetY());

            if (d < d_min) {
                d_min = d;
            }
        }
    }

    // Trajectories intersecting collision radii are significantly higher cost than others.
    // Note that these must still yield a finite cost, as the planner must rely on the other terms to discern between all obstacles-intersecting trajectories.
    if (d_min <= collision_radius_) {
        return 1000.0f * (1 / (d_min * d_min * d_min));
    }

    return 1 / (d_min * d_min);
}

float
DwaPlanner::EvaluateCostSpeed(DwaTrajectory traj) { return std::max(speed_lin_max_ - traj.GetLastState().GetSpeed(), 0.0f); }

float
DwaPlanner::EvaluateCostHeading(DwaTrajectory traj) {
    float dx = goal_x_ - traj.GetLastState().GetX();
    float dy = goal_y_ - traj.GetLastState().GetY();

    double error_angle = std::atan2(dy, dx);

    double cost_angle = error_angle - traj.GetLastState().GetYaw();

    return std::abs(std::atan2(std::sin(cost_angle), std::cos(cost_angle)));
}

float
DwaPlanner::EvaluateCostGoal(DwaTrajectory traj) {
    float dx = goal_x_ - traj.GetLastState().GetX();
    float dy = goal_y_ - traj.GetLastState().GetY();

    return std::hypot(dx, dy);
}

float
DwaPlanner::EvaluateCostDeviation(DwaTrajectory traj) {
    float dx = traj_last_.GetLastState().GetX() - traj.GetLastState().GetX();
    float dy = traj_last_.GetLastState().GetY() - traj.GetLastState().GetY();

    return std::hypot(dx, dy);
}

float
DwaPlanner::EvaluateCostGlobalPath(DwaTrajectory traj) {
    float cost_path = 0.0f;

    for (int i = 0; i < traj.GetNumberOfStates(); ++i) {
        cost_path += global_path_.FindClosestDistance(
            traj.GetState(i).GetX(),
            traj.GetState(i).GetY()
        );
    }

    return cost_path;
}

void
DwaPlanner::GetObstacles() {
    // Clear the previous obstacle list.
    obs_occ_.Clear();

    float dis_cutoff = search_radius_;
    if (obs_search_ == "adaptive")
    {
        // The cutoff distance is selected based on the maximum travel distance throughout the window (with a constant bias term equal to the collision radius).
        dis_cutoff = collision_radius_ + 0.75f * (speed_lin_max_ * time_span_ + 0.5 * accel_max_ * time_span_ * time_span_);
    }

    #pragma omp parallel for schedule(static) collapse(2)
	for (int i = 0; i < grid_occ_width_; ++i) {
        for (int j = 0; j < grid_occ_height_; ++j) {
            float x = grid_occ_origin_x_ + (i + 0.5f) * grid_occ_res_;
            float y = grid_occ_origin_y_ + (j + 0.5f) * grid_occ_res_;

			unsigned int ndx = j * grid_occ_width_ + i;

			int cost = (int)grid_occ_data_[ndx];

            if (use_segmentation_) {
                cost += grid_seg_data_[ndx];
            }

			if (cost > thresh_obs_) {
				float d = std::hypot(
                    state_.GetX() - x,
                    state_.GetY() - y);

                if (d < dis_cutoff) {
					obs_occ_.Add(x, y);
				}
			}
		}
	}
}

} // end namespace planning
} // end namespace avt_341
