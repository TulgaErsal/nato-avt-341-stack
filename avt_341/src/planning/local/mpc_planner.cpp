#include <utility>

#include "avt_341/planning/local/mpc_planner.h"

namespace avt_341 {
namespace planning {

void MpcPlanner::plan() {
    if (!ready()) {
        std::cout << "Not ready to plan" << std::endl;
        return;
    }
    mpc_->Setup();
    mpc_->Plan();
}

msg::Path MpcPlanner::getPlannedPathRos() {
    msg::Path path;
    path.header.frame_id = "map";
    if (mpc_->HasPlan()) {
        auto x = mpc_->GetVar(0);
        auto y = mpc_->GetVar(1);
        auto psi = mpc_->GetVar(4);
        msg::PoseStamped pose;
        pose.header.frame_id = "map";
        for (size_t i = 0; i < x.size(); i++) {
            pose.pose.position.x = x[i];
            pose.pose.position.y = y[i];
            pose.pose.orientation = MpcUtils::yaw2quaternionMsg(psi[i]);
            path.poses.push_back(pose);
        }
    }
    return path;
}

double MpcPlanner::getControlJerk() {
    auto jerk = mpc_->GetCtrl(0);
    return jerk[0];
}

// TODO fix
double MpcPlanner::getControlSteeringRate() {
    auto steering_rate = mpc_->GetCtrl(0);
    return steering_rate[0];
}

Control MpcPlanner::getControls() {
    auto speed_longitudinal = mpc_->GetVar(6);
    auto steering_angle= mpc_->GetVar(5);
    return {speed_longitudinal[1], steering_angle[1]};
}

std::vector<Control> MpcPlanner::getControlsVector() {
    auto speed_longitudinal = mpc_->GetVar(6);
    auto steering_angle= mpc_->GetVar(5);
    std::vector<Control> controls;
    for (size_t i = 0; i < speed_longitudinal.size(); i++) {
        controls.push_back({speed_longitudinal[i], steering_angle[i]});
    }
    return controls;
}


void MpcPlanner::initialize() {
    mpc_ = std::make_unique<avt_341::MPC>();
    mpc_->SetMass(m_);
    mpc_->SetInertia(i_zz_);
    mpc_->SetCornStiffnessFront(c_alpha_f_);
    mpc_->SetCornStiffnessRear(c_alpha_r_);
    mpc_->SetRelaxLength(b_);
    mpc_->SetDisAxleFront(d_a_);
    mpc_->SetDisAxleRear(d_b_);
    setSolverOptions();
    setBounds();
    setMpcWeights();
    mpc_->Initialise();
    initialized_ = true;
}

void MpcPlanner::setState(double x,
                          double y,
                          double yaw,
                          double long_speed,
                          double lateral_speed,
                          double yaw_rate,
                          double steering_angle,
                          double long_acceleration,
                          double tan_alpha_f,
                          double tan_alpha_r) {
    mpc_->SetInitX(x);
    mpc_->SetInitY(y);
    mpc_->SetInitLatVel(lateral_speed);
    mpc_->SetInitYawRate(yaw_rate);
    mpc_->SetInitYaw(yaw);
    mpc_->SetInitSteer(steering_angle);
    mpc_->SetInitSpeed(long_speed);
    mpc_->SetInitAcc(long_acceleration);
    mpc_->SetInitTanSlipFront(tan_alpha_f);
    mpc_->SetInitTanSlipRear(tan_alpha_r);
    has_state_ = true;
}

void MpcPlanner::setBounds() {
    // states
    mpc_->SetMinLatVel(lateral_speed_min_);
    mpc_->SetMaxLatVel(lateral_speed_max_);
    mpc_->SetMinYawRate(yaw_rate_min_);
    mpc_->SetMaxYawRate(yaw_rate_max_);
    mpc_->SetMinSteer(steering_angle_min_);
    mpc_->SetMaxSteer(steering_angle_max_);
    mpc_->SetMinSpeed(long_speed_min_);
    mpc_->SetMaxSpeed(long_speed_max_);
    mpc_->SetMinAcc(long_acceleration_min_);
    mpc_->SetMaxAcc(long_acceleration_max_);
    // controls
    mpc_->SetMinJerk(long_jerk_min_);
    mpc_->SetMaxJerk(long_jerk_max_);
    mpc_->SetMinSteerRate(steering_rate_min_);
    mpc_->SetMaxSteerRate(steering_rate_max_);
}

void MpcPlanner::setSolverOptions() {
    mpc_->SetSolver(solver_name_);
    mpc_->SetMaxIters(solver_max_iterations_);
    mpc_->SetMaxCPUTime(solver_max_wall_time_);
    mpc_->SetHorizon(solver_time_span_);
    mpc_->SetTimeStep(solver_time_step_);
    mpc_->SetTolerance(solver_tolerance_);
    mpc_->SetPrintLevel(solver_print_level_);
}

void MpcPlanner::setGoal(double goal_x, double goal_y) {
    goal_x_ = goal_x;
    goal_y_ = goal_y;
    mpc_->SetGoalX(goal_x);
    mpc_->SetGoalY(goal_y);
    has_goal_ = true;
}

void MpcPlanner::setGoal(double goal_x, double goal_y, double goal_yaw) {
    goal_x_ = goal_x;
    goal_y_ = goal_y;
    goal_yaw_ = goal_yaw;
    mpc_->SetGoal(goal_x, goal_y, goal_yaw);
    has_goal_ = true;
}

void MpcPlanner::setWeights(double w_goal, double w_steering_rate, double w_obstacle_term, double w_long_jerk) {
    w_obstacle_term_ = w_obstacle_term;
    w_goal_ = w_goal;
    w_long_jerk_ = w_long_jerk;
    w_steering_rate_ = w_steering_rate;
}

void MpcPlanner::setMpcWeights() {
    mpc_->SetWeightObstacle(w_obstacle_term_);
    mpc_->SetWeightGoal(w_goal_);
    mpc_->SetWeightJerk(w_long_jerk_);
    mpc_->SetWeightSteerRate(w_steering_rate_);
}


void MpcPlanner::setObstacles(std::vector<float> x,
                              std::vector<float> y,
                              std::vector<float> r) {
    mpc_->ClearObstacles();
    mpc_->AddObstacles(x, y, r);
}

} // end namespace planning
} // end namespace avt_341
