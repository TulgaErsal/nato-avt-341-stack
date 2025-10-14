#ifndef AVT_341_CHANNEL_CLEARING_METHOD_H
#define AVT_341_CHANNEL_CLEARING_METHOD_H

#include "costmap_clearing_method.h"

namespace avt_341::perception {

struct ChannelClearingSettings {
    std::string channel_to_clear;
    float threshold;

    ChannelClearingSettings() = default;

    ChannelClearingSettings(const std::string &ch_to_clear, const float thresh)
        : channel_to_clear(ch_to_clear), threshold(thresh) {}
};

class ChannelThresholdClearingMethod : public OccupancyClearingMethod {
public:
    ChannelThresholdClearingMethod(
        std::vector<std::vector<Cell>> &cells,
        const BaseClearingSettings &base_config,
        const ChannelClearingSettings &ch_config,
        CellObstacleCalculator *obs_calculator
        );

    virtual void ClearOccupancy(const msg::PointCloud &point_cloud) override;

private:
    ChannelClearingSettings ch_config_;
};


}

#endif //AVT_341_CHANNEL_CLEARING_METHOD_H