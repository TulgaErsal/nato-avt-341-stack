#include "avt_341/perception/layers/static_grid_layer.h"

namespace avt_341::perception
{
    StaticGridLayer::StaticGridLayer(const std::shared_ptr<node::NodeProxy>& node_ref,
    const CostmapSizeInfo& size_info, const ThresholdSettings& thresholds, const DilationSettings& dilation)
        : CostmapLayer(node_ref, size_info, thresholds, dilation)
    {
    }
}

