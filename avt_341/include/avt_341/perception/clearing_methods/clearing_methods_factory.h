#ifndef AVT_341_CLEARING_METHOD_FACTORY_H
#define AVT_341_CLEARING_METHOD_FACTORY_H

#include "avt_341/perception/clearing_methods/costmap_clearing_method.h"

namespace avt_341::perception {

class ClearingMethodFactory {

public:

    static std::shared_ptr<OccupancyClearingMethod> CreateClearingMethod(
        const std::string& clear_method_type,
        const std::shared_ptr<node::NodeProxy>& node_ref,
        std::vector<std::vector<Cell>> & cells,
        const ClearMethodSettings & params,
        const PerceptionSettings& settings,
        CellObstacleCalculator *obs_calculator
    );

    static std::vector<std::shared_ptr<OccupancyClearingMethod>> CreateClearingMethods(
        const std::shared_ptr<node::NodeProxy>& node_ref,
        std::vector<std::vector<Cell>> & cells,
        const ClearMethodSettings & params,
        const PerceptionSettings& settings,
        CellObstacleCalculator *obs_calculator
    );

private:

    static std::vector<std::string> ParseClearMethodsString(std::string cm_methods_str);

};

}

#endif // AVT_341_CLEARING_METHOD_FACTORY_H
