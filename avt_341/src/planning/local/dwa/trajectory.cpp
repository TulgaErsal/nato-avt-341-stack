#include <avt_341/planning/local/dwa/trajectory.hpp>
#include "avt_341_msgs/msg/dwa_objective.hpp"
#include "avt_341_msgs/msg/dwa_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace avt_341 {
namespace planning {
namespace dwa {

Trajectory::Trajectory() {}

void Trajectory::Add(State state) { states_.push_back(state); }

int Trajectory::GetNumberOfStates() { return (int)states_.size(); }

State Trajectory::GetState(int i) { return states_[i]; }

const State& Trajectory::GetLastState() const { return states_.back(); }

nav_msgs::msg::Path Trajectory::ToRosPath() {
    nav_msgs::msg::Path msg_path;

    // Fill the poses array with all states in the trajectory.
    for(State& state : states_) {
        msg_path.poses.push_back(state.ToRosPoseStamped());
    }

    return msg_path;
}

void Trajectory::Reset() { states_.clear(); }

void Trajectory::EvaluateGoalCost(const double& weight,
                                  const double& goal_x,
                                  const double& goal_y) {
    goal_cost_ = weight *
        std::hypot(goal_x - states_.back().GetX(),
                   goal_y - states_.back().GetY());
}

const double& Trajectory::GetGoalCost() { return goal_cost_; }

void Trajectory::EvaluateObstacleCost(const double& weight,
                                      const Obstacles& obstacles,
                                      const double& collision_radius) {
    // Initialise the minimum distance to an obstacle to a very large value.
    double minimum_distance = std::numeric_limits<double>::infinity();

    for(int j = 0; j < GetNumberOfStates(); j++) {
        for(int obstacle_index = 0;
            obstacle_index < obstacles.GetNumberOfObstacles();
            ++obstacle_index) {
            double distance = obstacles.GetDistance(obstacle_index,
                                                    GetState(j).GetX(),
                                                    GetState(j).GetY());

            if(distance < minimum_distance) { minimum_distance = distance; }
        }
    }

    // Trajectories intersecting collision radii are significantly higher cost
    // than others. Note that these must still yield a finite cost, as the
    // planner must rely on the other terms to discern between all
    // obstacles-intersecting trajectories.
    if(minimum_distance <= collision_radius) {
        obstacle_cost_ =
            weight * 1000.0 * (1.0 / std::pow(minimum_distance, 3.0));
    }

    obstacle_cost_ = weight * 1.0 / std::pow(minimum_distance, 2.0);
}

const double& Trajectory::GetObstacleCost() { return obstacle_cost_; }

void Trajectory::EvaluateSegmentationCost(
    const double& weight,
    const std::vector<signed char>& grid_data,
    const int& grid_width,
    const int& grid_height,
    const double& grid_origin_x,
    const double& grid_origin_y,
    const double& grid_resolution,
    const double& score_threshold) {
    segmentation_cost_ = 0.0;
    for(size_t state_index = 0; state_index < states_.size(); state_index++) {
        int i =
            (states_[state_index].GetX() - grid_origin_x) / grid_resolution -
            0.5;
        int j =
            (states_[state_index].GetY() - grid_origin_y) / grid_resolution -
            0.5;
        if(i < 0 || j < 0 || i >= grid_width || j >= grid_height) continue;

        int cost = (int)grid_data[j * grid_width + i];
        if(cost > score_threshold) {
            segmentation_cost_ = 10000000.0;
            break;
        } else {
            segmentation_cost_ += cost;
        }
    }
    segmentation_cost_ *= weight;
}

const double& Trajectory::GetSegmentationCost() { return segmentation_cost_; }

void Trajectory::EvaluateHeadingCost(const double& weight,
                                     const double& goal_x,
                                     const double& goal_y) {
    double error_angle = std::atan2(goal_y - states_.back().GetY(),
                                    goal_x - states_.back().GetX());

    double cost_angle = error_angle - states_.back().GetYaw();

    heading_cost_ = weight *
        std::abs(std::atan2(std::sin(cost_angle), std::cos(cost_angle)));
}

const double& Trajectory::GetHeadingCost() { return heading_cost_; }

void Trajectory::EvaluateSpeedCost(const double& weight,
                                   const double& speed_threshold) {
    speed_cost_ =
        weight * std::max(speed_threshold - states_.back().GetSpeed(), 0.0);
}

const double& Trajectory::GetSpeedCost() { return speed_cost_; }

void Trajectory::EvaluateGlobalPathCost(const double& weight,
                                        const Path& global_path) {
    global_path_cost_ = 0.0;
    for(int state_index = 0; state_index < GetNumberOfStates(); ++state_index) {
        global_path_cost_ +=
            global_path.FindClosestDistance(states_[state_index].GetX(),
                                            states_[state_index].GetY());
    }
    global_path_cost_ *= weight;
}

const double& Trajectory::GetGlobalPathCost() { return global_path_cost_; }

void Trajectory::EvaluateDeviationCost(const double& weight,
                                       const Trajectory& current_trajectory) {
    deviation_cost_ =
        weight *
        std::hypot(
            current_trajectory.GetLastState().GetX() - states_.back().GetX(),
            current_trajectory.GetLastState().GetY() - states_.back().GetY());
}

const double& Trajectory::GetDeviationCost() { return deviation_cost_; }

void Trajectory::EvaluateTotalCost() {
    cost_ = goal_cost_ + obstacle_cost_ + segmentation_cost_ + heading_cost_ +
        speed_cost_ + global_path_cost_ + deviation_cost_;
}

const double& Trajectory::GetTotalCost() { return cost_; }

avt_341_msgs::msg::DwaTrajectory Trajectory::GetROSTrajectoryMessage() const {
    avt_341_msgs::msg::DwaTrajectory trajectory_message;

    nav_msgs::msg::Path path_message;
    path_message.header.frame_id = "map";
    for (auto& state : states_)  {
        geometry_msgs::msg::PoseStamped pose_stamped_message;
        pose_stamped_message.header.frame_id = "map";
        pose_stamped_message.pose.position.x = state.GetX();
        pose_stamped_message.pose.position.y = state.GetY();
        path_message.poses.push_back(pose_stamped_message);
    }
    trajectory_message.path = path_message;


    avt_341_msgs::msg::DwaObjective objective_message;
    objective_message.goal_cost = goal_cost_;
    objective_message.obstacle_cost = obstacle_cost_;
    objective_message.segmentation_cost = segmentation_cost_;
    objective_message.heading_cost = heading_cost_;
    objective_message.speed_cost = speed_cost_;
    objective_message.deviation_cost = deviation_cost_;
    trajectory_message.objective = objective_message;
    trajectory_message.cost = cost_;
    return trajectory_message;
}

} // namespace dwa
} // namespace planning
} // namespace avt_341