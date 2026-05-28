#include "avt_341/node/occupancy_grid_subscriber.h"

namespace avt_341::node {

OccupancyGridSubscriber::OccupancyGridSubscriber(
    const std::shared_ptr<NodeProxy> &node,
    const std::string & topic_name,
    int qos
    )
    : OccupancyGridSubscriber(node, topic_name, qos, nullptr){
}

OccupancyGridSubscriber::OccupancyGridSubscriber(
    const std::shared_ptr<NodeProxy> &node,
    const std::string & topic_name,
    int qos,
    const std::function<void(const msg::OccupancyGridPtr &)> &callback
    )
    : grid_msg_(nullptr), external_callback_(callback){

    grid_sub_ = node->create_subscription<msg::OccupancyGrid>(
    topic_name,
    qos,
    std::bind(&OccupancyGridSubscriber::OccupancyGridCallback, this, std::placeholders::_1));

    grid_sub_updates_ = node->create_subscription<msg::OccupancyGridUpdate>(
        topic_name + "_updates",
        qos,
        std::bind(&OccupancyGridSubscriber::OccupancyGridUpdateCallback, this, std::placeholders::_1));
}

msg::OccupancyGridPtr OccupancyGridSubscriber::GetGrid() const { return grid_msg_; }

msg::OccupancyGrid OccupancyGridSubscriber::GetGridCopy() const { return grid_msg_ == nullptr ? msg::OccupancyGrid() : *grid_msg_; }

bool OccupancyGridSubscriber::HasData() const { return grid_msg_ != nullptr; }

void OccupancyGridSubscriber::OccupancyGridCallback(msg::OccupancyGridPtr grid_msg) {

    grid_msg_ = avt_341::node::make_msg_shared<msg::OccupancyGrid>(*grid_msg);

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


void OccupancyGridSubscriber::OccupancyGridUpdateCallback(msg::OccupancyGridUpdatePtr update) {

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

void OccupancyGridSubscriber::SetOnGridUpdated(std::function<void(const msg::OccupancyGridPtr &)> callback) {
    external_callback_ = std::move(callback);
}

}