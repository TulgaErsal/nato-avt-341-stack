#include "avt_341/perception/layers/polygon_layer.h"

namespace avt_341::perception
{
    PolygonLayer::PolygonLayer(
        const std::shared_ptr<node::NodeProxy>& node_ref,
        const CostmapSettings& cm_settings,
        const std::string& label)
        : CostmapLayer(node_ref, cm_settings, label)
    {
        is_valid_ = false;
    }
}
