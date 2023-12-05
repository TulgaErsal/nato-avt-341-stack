/**
* \file avt_341_grid_compression_node.cpp
*
* Subscribes to occupancy grid and publishes compressed version, containing only a list of occupied (non-zero) cells
*
* \author Martin Hirschkorn
*
* \date 2023-01-11
*/

// ros includes

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

avt_341::msg::OccupancyGrid grid;

void OccupancyGridCallback(avt_341::msg::OccupancyGridPtr rcv_grid)
{
    grid = *rcv_grid;
}

int main(int argc, char** argv) {
    auto n = avt_341::node::init_node(argc, argv, "avt_341_grid_compression_node");
    auto occupied_cells_pub = n->create_publisher<avt_341::msg::OccupiedCells>("avt_341/occupied_cells", 10);
    auto occupancy_grid_sub = n->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 10, OccupancyGridCallback);

    avt_341::node::Rate rate(1.0);

    while (avt_341::node::ok()) {

        avt_341::msg::OccupiedCells cellsMsg;
        cellsMsg.header = grid.header;
        cellsMsg.info = grid.info;

        int height = grid.info.height;
        int width = grid.info.width;

        avt_341::msg::OccupiedCell cell;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                int data = grid.data[j * width + i];
                if (data != 0) {
                    //std::cerr << "Push cell, i: " << i << "  j: " << j << "  data: " << data << std::endl;
                    cell.x_index = i;
                    cell.y_index = j;
                    cell.data = data;
                    cellsMsg.cells.push_back(cell);
                }
            }
        }

        occupied_cells_pub->publish(cellsMsg);

        n->spin_some();
        rate.sleep();
    }

    return 0;
}
