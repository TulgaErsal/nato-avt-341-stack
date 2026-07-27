#ifndef OCCUPANCY_GRID_SUBSCRIBER_H
#define OCCUPANCY_GRID_SUBSCRIBER_H

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341/core/grid_components.h"

namespace avt_341::node {

    class OccupancyGridSubscriber{

    public:

        OccupancyGridSubscriber(
            const rclcpp::Node::SharedPtr &node,
            const std::string & topic_name,
            int qos,
            const std::string & publish_method
            );

        OccupancyGridSubscriber(
            const rclcpp::Node::SharedPtr &node,
            const std::string & topic_name,
            int qos,
            const std::string & publish_method,
            const std::function<void(const nav_msgs::msg::OccupancyGrid::SharedPtr &)> &callback
            );

        nav_msgs::msg::OccupancyGrid::SharedPtr GetGrid() const;
        nav_msgs::msg::OccupancyGrid GetGridCopy() const;
        bool HasData() const;
        void SetOnGridUpdated(std::function<void(const nav_msgs::msg::OccupancyGrid::SharedPtr &)> callback);

        inline core::GridRegion GetLastUpdateBounds() const { return last_update_bounds_; }
        core::GridRegion GetAndResetLastUpdateBounds();

    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
        rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr grid_sub_updates_;
        nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg_;
        core::GridRegion last_update_bounds_;

        void OccupancyGridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg);
        void OccupancyGridUpdateCallback(map_msgs::msg::OccupancyGridUpdate::SharedPtr update);

        std::function<void(const nav_msgs::msg::OccupancyGrid::SharedPtr &)> external_callback_;
    };

}
#endif //OCCUPANCY_GRID_SUBSCRIBER_H
