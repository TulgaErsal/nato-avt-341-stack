#include "avt_341/perception/layers/polygon_layer.h"

namespace avt_341::perception
{
    PolygonLayer::PolygonLayer(
        const std::shared_ptr<node::NodeProxy>& node_ref,
        const CostmapSettings& cm_settings,
        const std::string& label,
        const std::string& input_file
        )
        : CostmapLayer(node_ref, cm_settings, label), input_file_(input_file)
    {
        is_valid_ = false;
    }

    std::string PolygonLayer::ToString() const
    {
        return "[PolygonLayer] id: " + label_
            + ", file: " + input_file_;
    }
}
