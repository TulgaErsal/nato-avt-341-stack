#ifndef AVT_341_CHANNEL_CLEARING_METHOD_H
#define AVT_341_CHANNEL_CLEARING_METHOD_H

#include "costmap_clearing_method.h"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace avt_341_nav::perception {

using ChannelClearingSettings = ClearMethodSettings;

class ChannelThresholdClearingMethod : public OccupancyClearingMethod {
public:
    ChannelThresholdClearingMethod(
        std::vector<std::vector<Cell>> &cells,
        const PerceptionSettings &settings,
        const ChannelClearingSettings &ch_config,
        CellObstacleCalculator *obs_calculator
        );

    void ClearOccupancy(const sensor_msgs::msg::PointCloud &point_cloud) override;
    std::string GetDescription() const override;

private:
    ChannelClearingSettings ch_config_;
};


}

#endif //AVT_341_CHANNEL_CLEARING_METHOD_H
