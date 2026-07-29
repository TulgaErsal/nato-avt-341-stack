#ifndef FORMATION_GOAL_FILTER_H
#define FORMATION_GOAL_FILTER_H
#include "geometry_msgs/msg/pose.hpp"

namespace avt_341_nav::mission {

class GoalFilter {

public:
    virtual ~GoalFilter() = default;
    virtual geometry_msgs::msg::Pose Filter(const geometry_msgs::msg::Pose& candidate_goal, const geometry_msgs::msg::Pose& leader_pose) = 0;
    virtual void Reset() = 0;
};


class NullGoalFilter : public GoalFilter {

public:
    geometry_msgs::msg::Pose Filter(const geometry_msgs::msg::Pose& candidate_goal, const geometry_msgs::msg::Pose& leader_pose) override { return candidate_goal; }
    void Reset() override {}
};


}


#endif //FORMATION_GOAL_FILTER_H
