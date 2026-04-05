
#ifndef AVT_341_STATIC_GRID_LAYER_H
#define AVT_341_STATIC_GRID_LAYER_H
#include "costmap_layer.h"

namespace avt_341::perception
{
    class StaticGridLayer: public CostmapLayer
    {
    public:
        StaticGridLayer(
            const std::shared_ptr<node::NodeProxy>& node_ref,
            const CostmapSettings& cm_settings,
            const std::string& label);
    };
}


#endif //AVT_341_STATIC_GRID_LAYER_H
