#include "avt_341/perception/layers/polygon_layer.h"
#include "avt_341/perception/layers/polygon_zone_parser.h"
#include <fstream>

namespace avt_341::perception
{
    PolygonLayer::PolygonLayer(
        const std::shared_ptr<node::NodeProxy>& node_ref,
        const CostmapSettings& cm_settings,
        const std::string& label,
        const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder
        )
        : CostmapLayer(node_ref, cm_settings, label, compute_time_recorder)
    {
        node_ref_->get_parameter("~polygon_layer_data_file", input_file_, std::string(""));
        node_ref_->get_parameter("~polygon_layer_visualize", visualize_, false);
        marker_pub_ = node_ref_->create_publisher<msg::MarkerArray>("avt_341/" + label + "/markers", 1);

        LoadZones();
        RebuildCellCache(false);
        thresholds_.use_elevation = true; // For polygon layer, only use simple height value > threshold check
    }

    void PolygonLayer::LoadZones()
    {
        zones_.clear();

        if (input_file_.empty())
        {
            is_enabled_ = false;
            return;
        }

        try {

            node_ref_->log_info("Attempting to read polygon zones file: %hs", input_file_.c_str());

            std::ifstream file(input_file_);
            if (!file.is_open()) {
                throw std::invalid_argument("Cannot open input file " + input_file_);
            }

            const std::string text(
                (std::istreambuf_iterator<char>(file)),
                std::istreambuf_iterator<char>());

            PolygonZoneParser parser(text);
            for (const PolygonZone& zone : parser.Parse()) {
                if (zone.vertices.size() >= 3) {
                    zones_.push_back(zone);
                } else {
                    node_ref_->log_warning("Zone '%s' has fewer than 3 vertices — skipping.", zone.label.c_str());
                }
            }

            node_ref_->log_info("Loaded %zu zone(s) from %s", zones_.size(), input_file_.c_str());

        } catch (const std::exception& e) {
            node_ref_->log_error("Failed to polygon zones: %s", e.what());
            is_enabled_ = false;
        }
    }

    /// Iterate every grid cell and record those whose center falls inside a zone.
    void PolygonLayer::RebuildCellCache(const bool clear_existing)
    {
        if (clear_existing){
            Clear();
        }

        if (zones_.empty()) {
            return;
        }

        const int    w   = size_info_.nx();
        const int    h   = size_info_.ny();
        int marked_cells = 0;
        has_segmentation_ = std::any_of(zones_.begin(), zones_.end(), [](const auto& zone) { return zone.seg_value >= 0; });

        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                const utils::vec2 p = size_info_.ToPosWorld(j, i);
                for (const auto& zone : zones_) {
                    if (IsInsidePolygon(zone.vertices, p.x, p.y)) {
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

        node_ref_->log_info("Polygon layer cells cache rebuilt: %d cell(s) marked occupied.", marked_cells);
    }

    void PolygonLayer::Visualize()
    {
        if (visualize_)
        {
            PublishMarkers();
        }
    }

    void PolygonLayer::PublishMarkers() const
    {
        if (zones_.empty()) {
            return;
        }

        msg::MarkerArray ma;
        int marker_id = 0;

        for (const auto& zone : zones_) {
            // Polygon outline.
            msg::Marker outline;
            outline.header.frame_id = "map";
            outline.header.stamp    = node_ref_->get_stamp();
            outline.ns              = "no_go_zones";
            outline.id              = marker_id++;
            outline.type            = msg::Marker::LINE_STRIP;
            outline.action          = msg::Marker::ADD;
            outline.scale.x         = 0.3; // line width in metres
            outline.color.r         = 1.0f;
            outline.color.g         = 0.0f;
            outline.color.b         = 0.0f;
            outline.color.a         = 1.0f;

            for (const auto& v : zone.vertices) {
                geometry_msgs::msg::Point p;
                p.x = v.x;
                p.y = v.y;
                p.z = 0.0;
                outline.points.push_back(p);
            }
            // Close the ring.
            if (!zone.vertices.empty()) {
                geometry_msgs::msg::Point p;
                p.x = zone.vertices.front().x;
                p.y = zone.vertices.front().y;
                p.z = 0.0;
                outline.points.push_back(p);
            }

            ma.markers.push_back(outline);

            // Text label at the polygon centroid.
            double cx = 0.0, cy = 0.0;
            for (const auto& v : zone.vertices) { cx += v.x; cy += v.y; }
            cx /= static_cast<double>(zone.vertices.size());
            cy /= static_cast<double>(zone.vertices.size());

            msg::Marker label;
            label.header          = outline.header;
            label.ns              = "no_go_zones_labels";
            label.id              = marker_id++;
            label.type            = msg::Marker::TEXT_VIEW_FACING;
            label.action          = msg::Marker::ADD;
            label.scale.z         = 1.5; // text height in metres
            label.color.r         = 1.0f;
            label.color.g         = 1.0f;
            label.color.b         = 0.0f;
            label.color.a         = 1.0f;
            label.pose.position.x = cx;
            label.pose.position.y = cy;
            label.pose.position.z = 1.0;
            label.text            = zone.label;

            ma.markers.push_back(label);
        }

        marker_pub_->publish(ma);
    }

    std::string PolygonLayer::ToString() const
    {
        return "[PolygonLayer] id: " + label_
            + ", file: " + input_file_
            + ", zones: " + std::to_string(zones_.size());
    }

    void PolygonLayer::Clear()
    {
        // No need to clear layer since has no internal changing state
    }
}
