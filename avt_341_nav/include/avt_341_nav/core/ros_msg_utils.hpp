/**
* @file      ros_msg_utils.hpp
* @brief     Helpers operating on ROS message types: nav stack state, poses and goals.
*/

#ifndef AVT_341_CORE_ROS_MSG_UTILS_H
#define AVT_341_CORE_ROS_MSG_UTILS_H

#include <cmath>

#include "avt_341_msgs/msg/nav_goal.hpp"
#include "avt_341_msgs/msg/nav_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "avt_341_nav/core/math_utils.hpp"

namespace avt_341_nav::core
{

enum NavStackState : int {
    NotInit = -1,
    Active = 0,
    InactiveCoast = 1,
    InactiveGradualStop = 2,
    InactiveHardStop = 3
};

enum NavStateCmd : int {
    GoInactive = 0,
    GoActive = 1
};

inline bool IsValidShutdownBehavior(const int shutdown_behavior)
{
    return shutdown_behavior >= static_cast<int>(InactiveCoast) && shutdown_behavior <= static_cast<int>(InactiveHardStop);
}

inline double GetDistance(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
    const double dx = p1.x - p2.x;
    const double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

inline float GetHeadingFromOrientation(const geometry_msgs::msg::Quaternion& orientation){
    tf2::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
    const tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return static_cast<float>(yaw);
}

inline bool UseGoalOrientation(const avt_341_msgs::msg::NavGoal& msg)
{
    return msg.yaw_threshold < M_PI;
}

inline void GetGoalError(const geometry_msgs::msg::Pose& pose, const avt_341_msgs::msg::NavGoal& goal, double& dist_error, double& yaw_error)
{
    if (UseGoalOrientation(goal))
    {
        const double pose_yaw = GetHeadingFromOrientation(pose.orientation);
        const double goal_yaw = GetHeadingFromOrientation(goal.pose.orientation);
        yaw_error = std::abs(DiffAngle(pose_yaw, goal_yaw));
    }
    else
    {
        yaw_error = 0.0;
    }

    dist_error = GetDistance(pose.position, goal.pose.position);
}

inline bool IsGoalReached(const geometry_msgs::msg::Pose& pose, const avt_341_msgs::msg::NavGoal& goal)
{
    double dist_diff, yaw_diff;
    GetGoalError(pose, goal, dist_diff, yaw_diff);
    return yaw_diff < goal.yaw_threshold && dist_diff < goal.dist_threshold;
}

inline bool IsGoalReached(const avt_341_msgs::msg::NavState& state, const avt_341_msgs::msg::NavGoal& goal)
{
    return state.goal_distance < goal.dist_threshold && state.goal_yaw_difference < goal.yaw_threshold;
}

}

#endif  // AVT_341_CORE_ROS_MSG_UTILS_H
