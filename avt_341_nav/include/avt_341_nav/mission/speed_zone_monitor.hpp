#ifndef AVT_341_MISSION_SPEED_ZONE_MONITOR_HPP
#define AVT_341_MISSION_SPEED_ZONE_MONITOR_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "avt_341_nav/core/geometry/geometry_dto.hpp"
#include "avt_341_nav/mission/mission_manager.h"
#include "avt_341_nav/node/tf_interface.h"

namespace avt_341_nav::mission
{

/**
 * @brief Monitors the ego-vehicle odometry against polygon speed zones and
 * commands a mission manager speed change when a new zone is entered.
 */
class SpeedZoneMonitor
{
public:
    SpeedZoneMonitor(
        const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<avt_341_nav::node::TfInterface>& tf,
        const std::shared_ptr<MissionManager>& mission_manager,
        const std::string& speed_zones_file);

    /// Checks which speed zone the odometry position falls in and commands a
    /// mission manager speed change when a new zone is entered.
    void UpdateOdometry(const nav_msgs::msg::Odometry& odom);

private:
    void LoadZones(const std::string& speed_zones_file);
    void PublishMarkers() const;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<avt_341_nav::node::TfInterface> tf_;
    std::shared_ptr<MissionManager> mission_manager_;
    core::PolygonZoneCollection zone_collection_;
    int last_zone_ = -1;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> marker_pub_;
};

}

#endif // AVT_341_MISSION_SPEED_ZONE_MONITOR_HPP
