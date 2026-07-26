#include "avt_341/node/occupancy_grid_subscriber.h"

#include "avt_341/perception/costmap_dtos.h"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace avt_341::node {

OccupancyGridSubscriber::OccupancyGridSubscriber(
    const rclcpp::Node::SharedPtr &node,
    const std::string & topic_name,
    int qos,
    const std::string & publish_method
    )
    : OccupancyGridSubscriber(node, topic_name, qos, publish_method, nullptr){
}

OccupancyGridSubscriber::OccupancyGridSubscriber(
    const rclcpp::Node::SharedPtr &node,
    const std::string & topic_name,
    int qos,
    const std::string & publish_method,
    const std::function<void(const nav_msgs::msg::OccupancyGrid::SharedPtr &)> &callback
    )
    : grid_msg_(nullptr), external_callback_(callback){

    // In incremental update mode the full grid is published rarely, so a
    // late-joining subscriber must latch the last full grid.
    rclcpp::QoS grid_qos{rclcpp::KeepLast(static_cast<size_t>(qos))};
    if (publish_method == perception::GridPubMethod::Updates) {
        grid_qos.transient_local();
    }

    grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_name,
        grid_qos,
        std::bind(&OccupancyGridSubscriber::OccupancyGridCallback, this, std::placeholders::_1));

    grid_sub_updates_ = node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        topic_name + "_updates",
        qos,
        std::bind(&OccupancyGridSubscriber::OccupancyGridUpdateCallback, this, std::placeholders::_1));
}

nav_msgs::msg::OccupancyGrid::SharedPtr OccupancyGridSubscriber::GetGrid() const { return grid_msg_; }

nav_msgs::msg::OccupancyGrid OccupancyGridSubscriber::GetGridCopy() const { return grid_msg_ == nullptr ? nav_msgs::msg::OccupancyGrid() : *grid_msg_; }

bool OccupancyGridSubscriber::HasData() const { return grid_msg_ != nullptr; }

void OccupancyGridSubscriber::OccupancyGridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg) {

    grid_msg_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*grid_msg);

    last_update_bounds_.UpdateBounds(0, 0, grid_msg_->info.width, grid_msg_->info.height);

    if (external_callback_ != nullptr) {
        external_callback_(grid_msg);
    }
}

core::GridRegion OccupancyGridSubscriber::GetAndResetLastUpdateBounds() {
    const auto region_return = last_update_bounds_;
    last_update_bounds_.Reset();
    return region_return;
}


void OccupancyGridSubscriber::OccupancyGridUpdateCallback(map_msgs::msg::OccupancyGridUpdate::SharedPtr update) {

    if (!HasData()) {
        return;
    }

    unsigned int nx = grid_msg_->info.width;
    unsigned int di = 0;

    for (unsigned int y = 0; y < update->height ; y++)
    {
        unsigned int y_i = (update->y + y) * nx;
        for (unsigned int x = 0; x < update->width ; x++)
        {
            grid_msg_->data[y_i + x + update->x] = update->data[di++];
        }
    }

    last_update_bounds_.UpdateBounds(update->x, update->y, update->width, update->height);

    if (external_callback_ != nullptr) {
        external_callback_(grid_msg_);
    }
}

void OccupancyGridSubscriber::SetOnGridUpdated(std::function<void(const nav_msgs::msg::OccupancyGrid::SharedPtr &)> callback) {
    external_callback_ = std::move(callback);
}

}