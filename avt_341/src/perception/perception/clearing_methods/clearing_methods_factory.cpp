#include "avt_341/perception/clearing_methods/clearing_methods_factory.h"

#include <avt_341/perception/clearing_methods/channel_clearing_method.h>
#include <avt_341/perception/clearing_methods/raytrace_clearing_method.h>
#include <avt_341/perception/clearing_methods/time_clearing_method.h>

namespace avt_341::perception {


std::shared_ptr<OccupancyClearingMethod> ClearingMethodFactory::CreateClearingMethod(
        const std::string& clear_method_type,
        const std::shared_ptr<node::NodeProxy>& node_ref,
        std::vector<std::vector<Cell>> & cells,
        const ClearMethodSettings & params,
        const PerceptionSettings& settings,
        CellObstacleCalculator *obs_calculator
    ) {

    std::string cm_type = clear_method_type;
    cm_type.erase(std::remove_if(cm_type.begin(), cm_type.end(), ::isspace), cm_type.end());

    // TIME-BASED CLEARING
    // ----------------------------------------------------------------------------------------------------------------
    if (cm_type == CostmapClearMethodType::Time) {
        return std::make_shared<TimedClearingMethod>(
            params.max_point_age, cells, settings, obs_calculator);
    }

    // TIME-BASED CLEARING CHECKING FOR NO REMARKED OBSTACLES
    // ----------------------------------------------------------------------------------------------------------------
    if (cm_type == CostmapClearMethodType::NoObsTime) {

        return std::make_shared<TimedNoObsClearingMethod>(
            cells, settings, params, obs_calculator);
    }

    // RAYTRACE CLEARING
    // ----------------------------------------------------------------------------------------------------------------
    if (cm_type == CostmapClearMethodType::Raytrace) {
        return std::make_shared<RaytraceClearingMethod>(
            node_ref, cells, settings, params, obs_calculator);
    }

    // RAYTRACE CLEARING WITH OBS DISTANCE FILTER
    // ----------------------------------------------------------------------------------------------------------------
    if (cm_type == CostmapClearMethodType::RaytraceWithFiltering) {

        if (settings.dilation_x_cells() <= 0 ||
            settings.dilation_y_cells() <= 0) {
            node_ref->log_warning("Raytrace Clearing: Dilation should be enabled to reduce intermittent obstacle.");
        }

        return std::make_shared<RaytraceWithFilteringClearingMethod>(
            node_ref, cells, settings, params, obs_calculator);
    }

    // CLEAR BY CHANNEL THRESHOLD
    // ----------------------------------------------------------------------------------------------------------------
    if (cm_type == CostmapClearMethodType::ChannelThreshold) {

        if (params.channel_to_clear.empty()) {
            node_ref->log_error("Channel threshold clearing method channel name empty.");
        }

        if (params.channel_threshold < 1e-3) {
            node_ref->log_error("Channel threshold clearing method should have > 0 threshold configured.");
        }

        return std::make_shared<ChannelThresholdClearingMethod>(
            cells, settings, params, obs_calculator);
    }

    // NULL / NO CLEARING METHOD
    // ----------------------------------------------------------------------------------------------------------------
    if (cm_type != CostmapClearMethodType::None) {
        node_ref->log_warning(
            "Unknown costmap clearing method: %s. Reverting to default no clearing behavior.",
            clear_method_type.c_str()
            );
    }

    return std::make_shared<NullClearingMethod>(
        cells, settings, obs_calculator);
}

std::vector<std::string> ClearingMethodFactory::ParseClearMethodsString(std::string cm_methods_str) {

    size_t pos = 0;
    std::vector<std::string> cm_methods_list;
    while ((pos = cm_methods_str.find(",")) != std::string::npos) {
        cm_methods_list.push_back(cm_methods_str.substr(0, pos));
        cm_methods_str.erase(0, pos + 1);
    }
    cm_methods_list.push_back(cm_methods_str.substr(0, pos));

    return cm_methods_list;
}


std::vector<std::shared_ptr<OccupancyClearingMethod>> ClearingMethodFactory::CreateClearingMethods(
        const std::shared_ptr<node::NodeProxy>& node_ref,
        std::vector<std::vector<Cell>> & cells,
        const ClearMethodSettings & params,
        const PerceptionSettings& settings,
        CellObstacleCalculator *obs_calculator
        ) {

    std::vector<std::shared_ptr<OccupancyClearingMethod>> cm_methods;
    std::vector<std::string> cm_types = ParseClearMethodsString(params.type);

    // Create clearing methods
    for (const auto& cm_type: cm_types) {
        cm_methods.push_back(CreateClearingMethod(
            cm_type, node_ref, cells, params, settings, obs_calculator));
    }

    // Set sibling clearing methods for each method
    for (auto& cm: cm_methods) {
        std::vector<std::shared_ptr<OccupancyClearingMethod>> sibling_cms;
        std::copy_if(cm_methods.begin(), cm_methods.end(), std::back_inserter(sibling_cms),
                     [&cm](const std::shared_ptr<OccupancyClearingMethod>& other_cm) { return other_cm != cm; });
        cm->SetSiblingClearingMethods(sibling_cms);
    }

	node_ref->log_info(
        "Costmap clearing methods: %s (%d)",
        params.type.c_str(), cm_methods.size());
    for (const auto & cm: cm_methods) {
        node_ref->log_info(" - %s", cm->GetDescription().c_str());
    }


    return cm_methods;
}

}
