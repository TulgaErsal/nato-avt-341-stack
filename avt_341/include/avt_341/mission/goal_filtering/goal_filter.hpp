#ifndef FORMATION_GOAL_FILTER_H
#define FORMATION_GOAL_FILTER_H
#include "avt_341/node/ros_types.h"

namespace avt_341::mission {

class GoalFilter {

public:
    virtual ~GoalFilter() = default;
    virtual msg::Pose Filter(const msg::Pose& candidate_goal, const msg::Pose& leader_pose) = 0;
    virtual void Reset() = 0;
};


class NullGoalFilter : public GoalFilter {

public:
    msg::Pose Filter(const msg::Pose& candidate_goal, const msg::Pose& leader_pose) override { return candidate_goal; }
    void Reset() override {}
};


}


#endif //FORMATION_GOAL_FILTER_H
