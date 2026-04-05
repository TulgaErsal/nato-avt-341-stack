#include "avt_341/perception/layers/static_grid_layer.h"

namespace avt_341::perception
{
    StaticGridLayer::StaticGridLayer(
        const std::shared_ptr<node::NodeProxy>& node_ref,
        const CostmapSettings& cm_settings,
        const std::string& label,
        const std::string& input_file
        )
        : CostmapLayer(node_ref, cm_settings, label), input_file_(input_file)
    {
        is_valid_ = false;
    }

    std::string StaticGridLayer::ToString() const
    {
        return "[StaticGridLayer] id: " + label_
            + ", file: " + input_file_;
    }
}

