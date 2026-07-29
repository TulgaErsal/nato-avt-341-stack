#ifndef AVT_341_DTO_CONVERSION_H
#define AVT_341_DTO_CONVERSION_H

#include "avt_341_msgs/msg/nav_goal.hpp"
#include "avt_341_msgs/msg/nav_goal_sequence.hpp"
#include "avt_341_msgs/msg/nav_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "nav_msgs/msg/path.hpp"
#include "avt_341_nav/core/math_dto.hpp"
#include "avt_341_nav/core/ros_msg_utils.hpp"
#include <algorithm>
#include <stdexcept>

namespace avt_341_nav::core
{
    inline auto ToVec2(const geometry_msgs::msg::Point & p)
    {
        return core::vec2{
            static_cast<float>(p.x),
            static_cast<float>(p.y)
        };
    }

    inline auto ToVec2(const geometry_msgs::msg::Pose & p)
    {
        return core::vec2{
            static_cast<float>(p.position.x),
            static_cast<float>(p.position.y)
        };
    }

    inline bool HasActiveGoal(const avt_341_msgs::msg::NavState::SharedPtr& msg)
    {
        return msg->run_state == core::NavStackState::Active;
    }

    inline geometry_msgs::msg::PoseStamped ToPoseStamped(const avt_341_msgs::msg::NavGoal & nav_goal)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = nav_goal.header;
        pose.pose = nav_goal.pose;
        return pose;
    }

    inline avt_341_msgs::msg::NavGoal ToNavGoal(
            const geometry_msgs::msg::PoseStamped & pose,
            const double dist_threshold = -1.0,
            const double yaw_threshold = -1.0) {
        avt_341_msgs::msg::NavGoal goal;
        goal.header = pose.header;
        goal.pose = pose.pose;
        goal.dist_threshold = dist_threshold;
        goal.yaw_threshold = yaw_threshold;
        return goal;
    }

    inline avt_341_msgs::msg::NavGoalSequence ToNavGoalSequence(
            const nav_msgs::msg::Path & path,
            const double dist_threshold = -1.0,
            const double yaw_threshold = -1.0
            ) {
        avt_341_msgs::msg::NavGoalSequence sequence;
        sequence.header = path.header;
        sequence.goals.reserve(path.poses.size());
        for (const auto & pose : path.poses) {
            sequence.goals.push_back(ToNavGoal(pose, dist_threshold, yaw_threshold));
        }
        return sequence;
    }

    inline nav_msgs::msg::Path ToPath(const avt_341_msgs::msg::NavGoalSequence & nav_goals)
    {
        nav_msgs::msg::Path path;
        path.header = nav_goals.header;
        path.poses.resize(nav_goals.goals.size());
        std::transform(nav_goals.goals.begin(), nav_goals.goals.end(), path.poses.begin(),
            [](const avt_341_msgs::msg::NavGoal & goal) { return ToPoseStamped(goal); });
        return path;
    }

    inline geometry_msgs::msg::Pose ToPose(float px,
                            float py,
                            float pz = 0.0f,
                            float ow = 1.0f,
                            float ox = 0.0f,
                            float oy = 0.0f,
                            float oz = 0.0f) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = px;
        pose.position.y = py;
        pose.position.z = pz;
        pose.orientation.w = ow;
        pose.orientation.x = ox;
        pose.orientation.y = oy;
        pose.orientation.z = oz;
        return pose;
    }

    inline geometry_msgs::msg::PoseStamped ToPoseStamped(std::string frame_id,
                                            float px,
                                            float py,
                                            float pz = 0.0f,
                                            float ow = 1.0f,
                                            float ox = 0.0f,
                                            float oy = 0.0f,
                                            float oz = 0.0f) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = std::move(frame_id);
        pose.pose = ToPose(px, py, pz, ow, ox, oy, oz);
        return pose;
    }

    inline avt_341_msgs::msg::NavGoal ToNavGoal(const double x,
        const double y,
        const double dist_threshold,
        const double yaw_threshold,
        const std::string& frame_id = "map"
        ) {
        return ToNavGoal(ToPoseStamped(frame_id, static_cast<float>(x), static_cast<float>(y)), dist_threshold, yaw_threshold);
    }

    inline geometry_msgs::msg::Pose ToPose(const geometry_msgs::msg::Transform & tx) {
        geometry_msgs::msg::Pose pose_msg;

        pose_msg.position.x = tx.translation.x;
        pose_msg.position.y = tx.translation.y;
        pose_msg.position.z = tx.translation.z;

        pose_msg.orientation = tx.rotation;

        return pose_msg;
    }

    inline nav_msgs::msg::Path ToPath(const std::vector<core::vec2>& path)
    {
        nav_msgs::msg::Path ros_path;
        ros_path.header.frame_id = "map";
        for (const auto& p: path) {
            ros_path.poses.push_back(ToPoseStamped(ros_path.header.frame_id, p.x, p.y));
        }
        return ros_path;
    }

    inline avt_341_msgs::msg::NavGoalSequence ToNavGoalSequence(
        const std::vector<double> & goals_x,
        const std::vector<double> & goals_y,
        const core::vec2 tx,
        const double dist_threshold,
        const double yaw_threshold,
        const std::string& frame_id = "map"
    ) {
        avt_341_msgs::msg::NavGoalSequence nav_goals;
        nav_goals.header.frame_id = frame_id;

        if (goals_x.size() != goals_y.size()) {
            throw std::runtime_error("goals_x and goals_y must have the same size");
        }

        for (size_t i = 0; i < goals_x.size(); i++) {
            const auto nav_goal = ToNavGoal(goals_x[i] - tx.x, goals_y[i] - tx.y, dist_threshold, yaw_threshold, frame_id);
            nav_goals.goals.push_back(nav_goal);
        }
        return nav_goals;
    }

}

#endif //AVT_341_DTO_CONVERSION_H