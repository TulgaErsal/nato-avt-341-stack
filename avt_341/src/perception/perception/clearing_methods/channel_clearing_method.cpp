#include "avt_341/perception/clearing_methods/channel_clearing_method.h"

namespace avt_341::perception {

ChannelThresholdClearingMethod::ChannelThresholdClearingMethod(
    std::vector<std::vector<Cell>> &cells,
    const BaseClearingSettings &base_config,
    const ChannelClearingSettings &ch_config,
    CellObstacleCalculator *obs_calculator)
        : OccupancyClearingMethod(cells, base_config, obs_calculator), ch_config_(ch_config) {
}

void ChannelThresholdClearingMethod::ClearOccupancy(const msg::PointCloud &point_cloud) {
    if (point_cloud.channels.empty()) {
        return;
    }

    int channel_idx = -1;
    for (size_t i = 0; i < point_cloud.channels.size(); i++) {
        if (point_cloud.channels[i].name == ch_config_.channel_to_clear) {
            channel_idx = static_cast<int>(i);
            break;
        }
    }

    if (channel_idx < 0) {
        return;
    }

    for (size_t i = 0; i < point_cloud.points.size(); i++) {
        const float channel_val = point_cloud.channels[channel_idx].values[i];

        if (channel_val < ch_config_.threshold) {

            const auto &pt = point_cloud.points[i];
            int xi = static_cast<int>((pt.x - config_.llx) / config_.res);
            int yi = static_cast<int>((pt.y - config_.lly) / config_.res);

            if (xi >= 0 && xi < Nx_ && yi >= 0 && yi < Ny_) {
                cells_[yi][xi].ResetHeight();
                RemoveDilationAtCell(xi, yi);
                BroadcastClearToSiblings(xi, yi);
            }

        }
    }

}

std::string ChannelThresholdClearingMethod::GetDescription() const {
    return "ChannelThresholdClearingMethod: channel=" + ch_config_.channel_to_clear +
           ", threshold=" + std::to_string(ch_config_.threshold) + "m";
}

}

