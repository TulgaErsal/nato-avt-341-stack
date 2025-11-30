#include "avt_341/mission/goal_filtering/formation_goal_filter.hpp"

namespace avt_341::mission {

const std::string GoalFilterMethod::ObstacleAvoidance = "obs_avoid";
const std::string GoalFilterMethod::None = "none";

bool GoalFilterMethod::IsValid(const std::string & selected_method){
    std::vector<std::string> valid_methods = {ObstacleAvoidance, None};
    return std::find(valid_methods.begin(), valid_methods.end(), selected_method) != valid_methods.end();
}

}
