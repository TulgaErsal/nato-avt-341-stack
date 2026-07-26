#ifndef FORMATION_GOAL_FILTER_FACTORY_H
#define FORMATION_GOAL_FILTER_FACTORY_H

#include "avt_341/mission/goal_filtering/obs_avoid_goal_filter.hpp"

namespace avt_341::mission {

struct GoalFilterMethod {
    static const std::string ObstacleAvoidance;
    static const std::string None;
    static bool IsValid(const std::string & selected_method);
    static std::string Default() { return None; }
};

std::shared_ptr<GoalFilter> create_goal_filter(
    const std::string & vehicle_id,
    const std::string & method_id,
    const rclcpp::Node::SharedPtr & node,
    const avt_341::params::mission_manager::Params::FgfObsAvoid& filter_params,
    const std::string & publish_method = std::string()
    );

}

#endif //FORMATION_GOAL_FILTER_FACTORY_H
