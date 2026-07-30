#include "avt_341_nav/mission/speed_zone_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "avt_341_nav/core/coord_transform.hpp"
#include "avt_341_nav/core/eigen_utils.hpp"
#include "avt_341_nav/core/geometry/polygon_zone_parser.hpp"

namespace avt_341_nav::mission
{
    constexpr const char* MAP_FRAME = "map";
    constexpr double ZONE_TRANSFORM_TIMEOUT_S = 5.0;

    SpeedZoneMonitor::SpeedZoneMonitor(
        const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<avt_341_nav::node::TfInterface>& tf,
        const std::shared_ptr<MissionManager>& mission_manager,
        const std::string& speed_zones_file)
        : node_(node), tf_(tf), mission_manager_(mission_manager)
    {
        LoadZones(speed_zones_file);

        if (zone_collection_.zones.empty()) {
            return;
        }

        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "avt_341/speed_zones/markers", rclcpp::QoS(1).transient_local());
        PublishMarkers();
    }

    void SpeedZoneMonitor::LoadZones(const std::string& speed_zones_file)
    {
        if (speed_zones_file.empty()) {
            RCLCPP_INFO(node_->get_logger(), "No speed zones file configured, speed zone monitoring disabled.");
            return;
        }

        try {
            RCLCPP_INFO(node_->get_logger(), "Attempting to read speed zones file: %s", speed_zones_file.c_str());

            core::PolygonZoneCollection collection = core::PolygonZoneParser::ParseFile(speed_zones_file);
            const std::string source_frame = collection.frame.empty() ? MAP_FRAME : collection.frame;

            const core::CoordTransformer coord_transformer(tf_->get_buffer(), node_->get_logger());
            coord_transformer.TransformZones(
                collection, MAP_FRAME, tf2::durationFromSec(ZONE_TRANSFORM_TIMEOUT_S));
            zone_collection_ = std::move(collection);

            RCLCPP_INFO(node_->get_logger(), "Loaded %zu speed zone(s) from %s (frame: %s)",
                zone_collection_.zones.size(), speed_zones_file.c_str(), source_frame.c_str());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load speed zones: %s", e.what());
        }
    }

    void SpeedZoneMonitor::UpdateOdometry(const nav_msgs::msg::Odometry& odom)
    {
        if (zone_collection_.zones.empty()) {
            return;
        }

        Eigen::Vector3d position(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0);
        const std::string& odom_frame = odom.header.frame_id;
        if (!odom_frame.empty() && odom_frame != MAP_FRAME) {
            const core::CoordTransformer coord_transformer(tf_->get_buffer(), node_->get_logger());
            position = coord_transformer.Transform(odom_frame, MAP_FRAME, position);
        }

        int current_zone = -1;
        for (std::size_t i = 0; i < zone_collection_.zones.size(); ++i) {
            if (core::IsInsidePolygon(zone_collection_.zones[i].vertices, position.x(), position.y())) {
                current_zone = static_cast<int>(i);
            }
        }

        if (current_zone == last_zone_) {
            return;
        }
        last_zone_ = current_zone;

        if (current_zone < 0) {
            RCLCPP_INFO(node_->get_logger(), "Vehicle left all speed zones, keeping last speed setpoint.");
            return;
        }

        const core::PolygonZone& zone = zone_collection_.zones[current_zone];
        RCLCPP_INFO(node_->get_logger(), "Entered speed zone '%s', setting speed to %.2f m/s.",
            zone.label.c_str(), zone.max_speed);
        mission_manager_->handleSetSpeedMsg(SetSpeedMsg(
            mission_manager_->my_name, 0, mission_manager_->my_name,
            zone.max_speed, PriorityType::PREEMPT));
    }

    bool SpeedZoneMonitor::SetZoneMaxSpeeds(
        const std::vector<std::string>& zone_ids,
        const std::vector<double>& max_speeds,
        std::string& error_message)
    {
        if (zone_ids.size() != max_speeds.size()) {
            error_message = "zone_ids size (" + std::to_string(zone_ids.size())
                + ") does not match max_speeds size (" + std::to_string(max_speeds.size()) + ").";
            return false;
        }

        // Validate the full request before applying so the update is all-or-nothing.
        std::vector<std::pair<std::size_t, double>> updates;
        for (std::size_t i = 0; i < zone_ids.size(); ++i) {
            const std::string& zone_id = zone_ids[i];
            const double max_speed = max_speeds[i];

            if (!std::isfinite(max_speed) || max_speed < 0.0) {
                error_message = "max speed " + std::to_string(max_speed) + " for zone '"
                    + zone_id + "' must be a finite non-negative value.";
                return false;
            }

            bool found = false;
            for (std::size_t z = 0; z < zone_collection_.zones.size(); ++z) {
                if (zone_collection_.zones[z].label == zone_id) {
                    updates.emplace_back(z, max_speed);
                    found = true;
                }
            }
            if (!found) {
                error_message = "no speed zone with id '" + zone_id + "'.";
                return false;
            }
        }

        for (const auto& [zone_index, max_speed] : updates) {
            core::PolygonZone& zone = zone_collection_.zones[zone_index];
            RCLCPP_INFO(node_->get_logger(), "Speed zone '%s' max speed updated from %.2f to %.2f m/s.",
                zone.label.c_str(), zone.max_speed, max_speed);
            zone.max_speed = max_speed;
        }

        // If the vehicle is inside an updated zone, force re-evaluation so the
        // new max speed is commanded on the next odometry update.
        if (last_zone_ >= 0 && std::any_of(updates.begin(), updates.end(),
                [this](const auto& update) { return static_cast<int>(update.first) == last_zone_; })) {
            last_zone_ = -1;
        }

        return true;
    }

    void SpeedZoneMonitor::PublishMarkers() const
    {
        marker_pub_->publish(core::CreateVisualization(
            zone_collection_, node_->now(), MAP_FRAME, "speed_zones"));
    }
}
