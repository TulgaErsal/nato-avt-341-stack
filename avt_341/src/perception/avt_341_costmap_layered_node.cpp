/**
 * Layered costmap implementation.
 * 
 * Evan Vandermate (evanderm@mtu.edu)
 * Last Modified: 04/02/2024
*/
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/costmap_layer.h"
#include <vector>
#include <iostream>


std::vector<avt_341::perception::CostmapLayer> layers;
std::shared_ptr<avt_341::node::NodeProxy> node;

void gridCallback(avt_341::msg::OccupancyGridPtr msg, int layer_index) {
    node->log_info("Received grid for layer [%d]", layer_index);
}

int main(int argc, char *argv[]) {
    node = avt_341::node::init_node(argc, argv, "avt_341_costmap_layered_node");

    std::string global_frame, robot_base_frame;
    float update_frequency, publish_frequency;
    std::vector<std::string> layer_defs;
    node->get_parameter("~global_frame", global_frame, std::string("map"));
    node->get_parameter("~robot_base_frame", robot_base_frame, std::string("base_link"));
    node->get_parameter("~update_frequency", update_frequency, 1.0f);
    node->get_parameter("~publish_frequency", publish_frequency, 1.0f);
    node->get_parameter("~layers", layer_defs, std::vector<std::string>{});

    int ci = 0;
    for(auto& layer_name: layer_defs) {
        avt_341::perception::CostmapLayer layer;
        layer.name = layer_name;

        // Read layer params
        node->get_parameter("~" + layer_name + "_enabled", layer.is_enabled, true);
        node->get_parameter("~" + layer_name + "_map_topic", layer.topic_name, std::string("/map"));

        // Create subscriber for layer
        std::function<void(avt_341::msg::OccupancyGridPtr msg)> fcn = std::bind(gridCallback, std::placeholders::_1, ci);
        layer.grid_sub = node->create_subscription<avt_341::msg::OccupancyGrid>(layer.topic_name,1,fcn);

        layers.push_back(layer);
        ci++;
    }

    node->spin();

}