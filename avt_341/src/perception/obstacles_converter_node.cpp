#include "avt_341/node/node_proxy.h"
#include "avt_341/node/ros_types.h"
#include <iostream>


std::shared_ptr <avt_341::node::NodeProxy> node;
std::shared_ptr <avt_341::node::Publisher<avt_341::msg::GridCells>> pub_cells;

void callback_obs(avt_341::msg::ObstaclesPtr obs) {
    avt_341::msg::GridCells cells;
    cells.header.frame_id = "map";
    cells.header.stamp = node->get_stamp();
    cells.cell_width = obs->obstacle_size_meters;
    cells.cell_height = obs->obstacle_size_meters;
    avt_341::msg::Point pt;
    pt.x = obs->data[0];
    pt.y = obs->data[1];
    cells.cells.push_back(pt);
    pub_cells->publish(cells);
}

int main(int argc, char* argv[]) {
    // Initialize ROS node.
    node = avt_341::node::init_node(argc, argv, "obstacles_converter_node");

    // Create node subscribers.
    auto sub_obstacles = node->create_subscription<avt_341::msg::Obstacles>("/mrzr/avt_341/obstacles", 1, callback_obs);

    // Create node publishers.
    pub_cells = node->create_publisher<avt_341::msg::GridCells>("/mrzr/avt_341/obstacle_grid_cells", 1);

    avt_341::node::Rate rosrate(10.0f);
    while (avt_341::node::ok()) {
        node->spin_some();
        rosrate.sleep();
    }
}