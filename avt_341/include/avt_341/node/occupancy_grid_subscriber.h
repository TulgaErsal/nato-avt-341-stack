#ifndef OCCUPANCY_GRID_SUBSCRIBER_H
#define OCCUPANCY_GRID_SUBSCRIBER_H

#include <avt_341/node/ros_types.h>
#include "avt_341/node/node_proxy.h"

namespace avt_341 {
namespace node {

    class OccupancyGridSubscriber{

    public:
        OccupancyGridSubscriber(const std::shared_ptr<NodeProxy> &node, const std::string & topic_name, int qos);
        OccupancyGridSubscriber(const std::shared_ptr<NodeProxy> &node, const std::string & topic_name, int qos,
            const std::function<void(const msg::OccupancyGridPtr &)> &callback);

        msg::OccupancyGridPtr GetGrid() const;
        msg::OccupancyGrid GetGridCopy() const;
        bool HasData() const;
        void SetOnGridUpdated(std::function<void(const msg::OccupancyGridPtr &)> callback);

    private:
        node::Subscriber<msg::OccupancyGrid>::SharedPtr grid_sub_;
        node::Subscriber<msg::OccupancyGridUpdate>::SharedPtr grid_sub_updates_;
        avt_341::msg::OccupancyGridSharedPtr grid_msg_;
        void OccupancyGridCallback(msg::OccupancyGridPtr grid_msg);
        void OccupancyGridUpdateCallback(msg::OccupancyGridUpdatePtr update);

        std::function<void(const msg::OccupancyGridPtr &)> external_callback_;
    };

}
}
#endif //OCCUPANCY_GRID_SUBSCRIBER_H
