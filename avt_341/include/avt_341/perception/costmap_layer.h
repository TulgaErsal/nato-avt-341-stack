#include "avt_341/node/ros_types.h"
#include <string>

namespace avt_341{
namespace perception{

class CostmapLayer {
public:
    CostmapLayer() {
        name = "";
        is_enabled = true;
        topic_name = "/map";
    }
    CostmapLayer(std::string layer_name, bool enabled, std::string topic) : name(layer_name), is_enabled(enabled), topic_name(topic) {}

    std::string name;
    bool is_enabled;
    std::string topic_name;
    avt_341::node::SubscriberPtr<avt_341::msg::OccupancyGrid, std::function<void(avt_341::msg::OccupancyGridPtr msg)>&> grid_sub;
};

} // namespace perception
} // namespace avt_341