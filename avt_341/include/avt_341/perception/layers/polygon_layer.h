#ifndef AVT_341_POLYGON_LAYER_H
#define AVT_341_POLYGON_LAYER_H

#include "costmap_layer.h"
#include <avt_341/perception_params_dto.hpp>

namespace avt_341::perception
{
    class PolygonLayer: public CostmapLayer
    {
    public:
        PolygonLayer(
            const std::shared_ptr<node::NodeProxy>& node_ref,
            const PerceptionSettings& settings,
            const std::string& label,
            const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
            const avt_341::params::perception::Params::PolygonLayer& params
            );

        void PublishMarkers() const;
        std::string ToString() const override;
        void Visualize() override;
        void Clear() override;

    private:
        void LoadZones();
        void RebuildCellCache(bool clear_existing = true);

        std::string input_file_;
        bool visualize_;
        std::vector<PolygonZone> zones_;
        std::shared_ptr<node::Publisher<msg::MarkerArray>> marker_pub_;

    };
}


#endif //AVT_341_POLYGON_LAYER_H
