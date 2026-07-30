#include "avt_341_nav/perception/layers/polygon_layer.h"
#include "avt_341_nav/core/geometry/polygon_zone_parser.hpp"
#include "avt_341_nav/core/coord_transform.hpp"
#include "avt_341_nav/core/eigen_utils.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace avt_341_nav::perception
{
    constexpr const char* MAP_FRAME = "map";

    PolygonLayer::PolygonLayer(
        const rclcpp::Node::SharedPtr& node_ref,
        const std::shared_ptr<node::TfInterface>& tf,
        const PerceptionSettings& settings,
        const std::string& label,
        const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
        const avt_341_nav::params::perception::Params::PolygonLayer& params
        )
        : CostmapLayer(
            node_ref, settings, label, compute_time_recorder,
            params.contribute_occupancy, params.contribute_segmentation),
        tf_(tf)
    {
        input_file_ = params.data_file;
        visualize_ = params.visualize;
        marker_pub_ = node_ref_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "avt_341/" + label + "/markers", rclcpp::QoS(1).transient_local());

        LoadZones();
        RebuildCellCache(false);
        settings_.costmap.thresholds.use_elevation = true;
    }

    void PolygonLayer::LoadZones()
    {
        zone_collection_ = core::PolygonZoneCollection();

        if (input_file_.empty())
        {
            is_enabled_ = false;
            return;
        }

        try {
            RCLCPP_INFO(node_ref_->get_logger(), "Attempting to read polygon zones file: %hs", input_file_.c_str());

            core::PolygonZoneCollection collection = core::PolygonZoneParser::ParseFile(input_file_);
            const std::string source_frame = collection.frame.empty() ? MAP_FRAME : collection.frame;

            const core::CoordTransformer coord_transformer(tf_->get_buffer(), node_ref_->get_logger());
            coord_transformer.TransformZones(collection, MAP_FRAME);
            zone_collection_ = std::move(collection);

            RCLCPP_INFO(node_ref_->get_logger(), "Loaded %zu zone(s) from %s (frame: %s)",
                zone_collection_.zones.size(), input_file_.c_str(), source_frame.c_str());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_ref_->get_logger(), "Failed to load polygon zones: %s", e.what());
            is_enabled_ = false;
        }
    }

    /// Iterate every grid cell and record those whose center falls inside a zone.
    void PolygonLayer::RebuildCellCache(const bool clear_existing)
    {
        if (clear_existing){
            Clear();
        }

        if (zone_collection_.zones.empty()) {
            return;
        }

        const int    w   = settings_.nx();
        const int    h   = settings_.ny();
        int marked_cells = 0;
        has_segmentation_ = std::any_of(zone_collection_.zones.begin(), zone_collection_.zones.end(),
            [](const auto& zone) { return zone.seg_value >= 0; });

        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                const core::vec2 p = settings_.to_world(j, i);
                for (const auto& zone : zone_collection_.zones) {
                    if (core::IsInsidePolygon(zone.vertices, p.x, p.y)) {
                        cells_[i][j].high.val = zone.occ_value;
                        // Even though only high.val used when use_elevation = true. Cell thinks it is in unfilled stae when low.val has default value
                        cells_[i][j].low.val = zone.occ_value - 1.0;
                        cells_[i][j].terrain_seg = zone.seg_value;
                        marked_cells += 1;
                        break; // No need to test remaining zones for this cell.
                    }
                }
            }
        }

        RCLCPP_INFO(node_ref_->get_logger(), "Polygon layer cells cache rebuilt: %d cell(s) marked occupied.", marked_cells);
    }

    void PolygonLayer::Visualize()
    {
        if (!visualize_ || markers_published_)
        {
            return;
        }
        PublishMarkers();
        markers_published_ = true;
    }

    void PolygonLayer::PublishMarkers() const
    {
        if (zone_collection_.zones.empty()) {
            return;
        }

        marker_pub_->publish(core::CreateVisualization(
            zone_collection_, node_ref_->now(), MAP_FRAME, "no_go_zones"));
    }

    std::string PolygonLayer::ToString() const
    {
        return "[PolygonLayer] id: " + label_
            + ", file: " + input_file_
            + ", zones: " + std::to_string(zone_collection_.zones.size());
    }

    void PolygonLayer::Clear()
    {
        // No need to clear layer since has no internal changing state
    }
}
