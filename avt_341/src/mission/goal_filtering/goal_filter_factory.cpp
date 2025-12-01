#include "avt_341/mission/goal_filtering/goal_filter_factory.hpp"

namespace avt_341::mission {

const std::string GoalFilterMethod::ObstacleAvoidance = "obs_avoid";
const std::string GoalFilterMethod::None = "none";

bool GoalFilterMethod::IsValid(const std::string & selected_method){
    std::vector<std::string> valid_methods = {ObstacleAvoidance, None};
    return std::find(valid_methods.begin(), valid_methods.end(), selected_method) != valid_methods.end();
}

std::shared_ptr<GoalFilter> create_goal_filter(
    const std::string & vehicle_id,
    const std::string & method_id,
    const std::shared_ptr<node::NodeProxy> & node
    ) {

    std::string candidate_method = method_id;
    if (candidate_method.empty() || !GoalFilterMethod::IsValid(candidate_method)) {
        node->log_warning(
            "Formation goal filter %s for follow vehicles is invalid. Using default instead.",
            candidate_method.c_str());
        candidate_method = GoalFilterMethod::Default();
    }

    node->log_info("Using goal filter method: %s", candidate_method.c_str());

    if (candidate_method == GoalFilterMethod::ObstacleAvoidance) {
        return std::make_shared<ObsAvoidGoalFilter>(node, vehicle_id);
    }
    return std::make_shared<NullGoalFilter>();
}

}
