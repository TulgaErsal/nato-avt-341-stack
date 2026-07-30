#ifndef AVT_341_POLYGON_LAYER_H
#define AVT_341_POLYGON_LAYER_H

#include "costmap_layer.h"
#include <avt_341_nav/perception_params_dto.hpp>
#include "avt_341_nav/core/geometry/geometry_dto.hpp"
#include "avt_341_nav/node/tf_interface.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include <rclcpp/rclcpp.hpp>

namespace avt_341_nav::perception
{
    class PolygonLayer: public CostmapLayer
    {
    public:
        PolygonLayer(
            const rclcpp::Node::SharedPtr& node_ref,
            const std::shared_ptr<node::TfInterface>& tf,
            const PerceptionSettings& settings,
            const std::string& label,
            const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
            const avt_341_nav::params::perception::Params::PolygonLayer& params
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
        bool markers_published_ = false;
        core::PolygonZoneCollection zone_collection_;
        std::shared_ptr<node::TfInterface> tf_;
        std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> marker_pub_;

    };
}


#endif //AVT_341_POLYGON_LAYER_H
