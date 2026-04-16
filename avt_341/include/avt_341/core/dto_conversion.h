#ifndef AVT_341_DTO_CONVERSION_H
#define AVT_341_DTO_CONVERSION_H

#include "avt_341/node/ros_types.h"
#include "avt_341/avt_341_utils.h"
#include <algorithm>
#include <stdexcept>

namespace avt_341::core
{
    inline utils::vec2 ToPoint(const msg::Point & p)
    {
        return utils::vec2(p.x, p.y);
    }

    inline msg::Int32 ToIntState(const msg::NavState& msg)
    {
        msg::Int32 state_msg;
        state_msg.data = msg.run_state;
        return state_msg;
    }

    inline msg::PoseStamped ToPoseStamped(const msg::NavGoal & nav_goal)
    {
        msg::PoseStamped pose;
        pose.header = nav_goal.header;
        pose.pose = nav_goal.pose;
        return pose;
    }

    inline msg::NavGoal ToNavGoal(const msg::Header & header, const msg::Pose& pose, const double arrival_threshold = -1.0) {
        msg::NavGoal goal;
        goal.header = header;
        goal.pose = pose;
        goal.threshold = arrival_threshold;
        goal.use_orientation = false;
        return goal;
    }

    inline msg::NavGoal ToNavGoal(const msg::PoseStamped & pose, const double arrival_threshold = -1.0) {
        return ToNavGoal(pose.header, pose.pose, arrival_threshold);
    }

    // inline msg::NavGoal ToNavGoal(const msg::Odometry& odom, const double arrival_threshold = -1.0) {
    //     return ToNavGoal(odom.header, odom.pose.pose, arrival_threshold);
    // }

    inline msg::NavGoalSequence ToNavGoalSequence(const msg::Path & path, const double arrival_threshold = -1.0) {
        msg::NavGoalSequence sequence;
        sequence.header = path.header;
        sequence.goals.reserve(path.poses.size());
        for (const auto & pose : path.poses) {
            sequence.goals.push_back(ToNavGoal(pose, arrival_threshold));
        }
        return sequence;
    }

    inline msg::Path ToPath(const msg::NavGoalSequence & nav_goals)
    {
        msg::Path path;
        path.header = nav_goals.header;
        path.poses.resize(nav_goals.goals.size());
        std::transform(nav_goals.goals.begin(), nav_goals.goals.end(), path.poses.begin(),
            [](const msg::NavGoal & goal) { return ToPoseStamped(goal); });
        return path;
    }

    inline msg::Pose ToPose(float px,
                            float py,
                            float pz = 0.0f,
                            float ow = 1.0f,
                            float ox = 0.0f,
                            float oy = 0.0f,
                            float oz = 0.0f) {
        msg::Pose pose;
        pose.position.x = px;
        pose.position.y = py;
        pose.position.z = pz;
        pose.orientation.w = ow;
        pose.orientation.x = ox;
        pose.orientation.y = oy;
        pose.orientation.z = oz;
        return pose;
    }

    inline msg::PoseStamped ToPoseStamped(std::string frame_id,
                                            float px,
                                            float py,
                                            float pz = 0.0f,
                                            float ow = 1.0f,
                                            float ox = 0.0f,
                                            float oy = 0.0f,
                                            float oz = 0.0f) {
        msg::PoseStamped pose;
        pose.header.frame_id = std::move(frame_id);
        pose.pose = ToPose(px, py, pz, ow, ox, oy, oz);
        return pose;
    }

    inline msg::NavGoal ToNavGoal(const double x, const double y, const double arrival_threshold, const std::string& frame_id = "map") {
        return ToNavGoal(ToPoseStamped(frame_id, static_cast<float>(x), static_cast<float>(y)), arrival_threshold);
    }

    inline msg::Pose ToPose(const msg::Transform & tx) {
        msg::Pose pose_msg;

        pose_msg.position.x = tx.translation.x;
        pose_msg.position.y = tx.translation.y;
        pose_msg.position.z = tx.translation.z;

        pose_msg.orientation = tx.rotation;

        return pose_msg;
    }

    inline msg::Path ToPath(const std::vector<utils::vec2>& path)
    {
        msg::Path ros_path;
        ros_path.header.frame_id = "map";
        for (const auto& p: path) {
            ros_path.poses.push_back(ToPoseStamped(ros_path.header.frame_id, p.x, p.y));
        }
        return ros_path;
    }

    inline msg::NavGoalSequence ToNavGoalSequence(
        const std::vector<double> & goals_x,
        const std::vector<double> & goals_y,
        const utils::vec2 tx,
        const double arrival_threshold,
        const std::string& frame_id = "map"
    ) {
        msg::NavGoalSequence nav_goals;
        nav_goals.header.frame_id = frame_id;

        if (goals_x.size() != goals_y.size()) {
            throw std::runtime_error("goals_x and goals_y must have the same size");
        }

        for (size_t i = 0; i < goals_x.size(); i++) {
            const auto nav_goal = ToNavGoal(goals_x[i] - tx.x, goals_y[i] - tx.y, arrival_threshold, frame_id);
            nav_goals.goals.push_back(nav_goal);
        }
        return nav_goals;
    }

}

#endif //AVT_341_DTO_CONVERSION_H