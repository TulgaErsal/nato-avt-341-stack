#ifndef FORMATION_GOAL_FILTER_H
#define FORMATION_GOAL_FILTER_H
#include "avt_341/node/ros_types.h"

namespace avt_341::mission {

struct GoalFilterMethod {
    static const std::string ObstacleAvoidance;
    static const std::string None;
    static bool IsValid(const std::string & selected_method);
    static std::string Default() { return None; }
};

class FormationGoalFilter {

public:
    virtual ~FormationGoalFilter() = default;
    virtual msg::Pose Filter(const msg::Pose& candidate_goal, const msg::Pose& leader_pose) = 0;
};


class NullGoalFilter : public FormationGoalFilter {
    virtual msg::Pose Filter(const msg::Pose& candidate_goal, const msg::Pose& leader_pose) override { return candidate_goal; }
};


}


#endif //FORMATION_GOAL_FILTER_H
