/* avt_341_lidar_occlusion_map_node.cpp

 * ROS Node for converting segmented image mask to an occupancy grid
 * Subscribers: _occ_mask (Image)
 * Publishers:  _occ_grid (OccupancyGrid)

**/

// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/voxel_grid.hpp"

avt_341::msg::Image occ_mask; 
bool occ_mask_rcvd = false;

double lidar_range = 100.0f;  // range of the lidar system in meters
double max_vertical_angle = 22.5f; 
double min_vertical_angle = -22.5f;
double min_horizontal_angle = 0.0f;   // on a circle, min should always be lower
double max_horizontal_angle = 360.0f;   // on a circle, max should always be higher
double grid_resolution = 0.5f;      // meters per voxel
double vehicle_height = 3.0f;       // total height of the vehicle in meters
double lidar_mount_height = 1.5f;   // mounting height of the lidar
double lidar_x = 0.0;
double lidar_y = 0.0;

void IncomingOcclusionMaskCallback(avt_341::msg::ImagePtr rcv_image) {
  occ_mask = *rcv_image;
  occ_mask_rcvd = true;
}

int main(int argc, char* argv[]) {
  // Initialize the node
  auto n = avt_341::node::init_node(argc, argv, "lidar_occlusion_map_node");

  // Subscriptions
  auto occ_mask_sub = n->create_subscription<avt_341::msg::Image>("avt_341/occ_mask", 1, IncomingOcclusionMaskCallback);

  // Publishers
  auto mask_grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occ_mask_grid", 1);

  // handle parameters
  n->get_parameter("~lidar_range", lidar_range, 100.0);
  n->get_parameter("~vehicle_height", vehicle_height, 3.0);
  n->get_parameter("~grid_resolution", grid_resolution, 0.5);
  avt_341::node::Rate rate(10.0);

  // setup the grid look up table
  // grid dimensions are:
  int grid_length = (lidar_range * 2) / grid_resolution;
  int grid_width = (lidar_range * 2) / grid_resolution;
  int grid_height = vehicle_height / grid_resolution;
  // lidar_position = length, width, lidar_mount_height
  lidar_x = static_cast<int>(std::round(grid_length / 2));
  lidar_y = static_cast<int>(std::round(grid_width / 2));
  double lidar_z = static_cast<int>(std::round(lidar_mount_height));
  VoxelGrid grid(grid_length, grid_width, grid_height, grid_resolution);

  std::vector<Voxel> voxels;
  voxels = grid.drawLine(lidar_x, lidar_y, lidar_z, 0, 0, 10);

  for(const auto& voxel : voxels) {
    int x, y, z;
    std::tie(x, y, z) = voxel;
    std::cout << "Voxel: (" << x << ", " << y << ", " << z << ")\n";
  }

  voxels = grid.drawLineFromSpherical(lidar_x, lidar_y, lidar_z, 0.0, 90.0, 10.0);
  std::cout << "LineFromSpherical - Start (" << lidar_x << ", " << lidar_y << ", " << lidar_z << ") ";
  std::cout << " Pitch: 0 Azimuth: 90 Range: 10" << std::endl;

  for(const auto& voxel : voxels) {
    int x, y, z;
    std::tie(x, y, z) = voxel;
    std::cout << "Voxel: (" << x << ", " << y << ", " << z << ")\n";
  }

  voxels = grid.drawLineFromSpherical(lidar_x, lidar_y, lidar_z, 0.0, 0.0, 10.0);
  std::cout << "LineFromSpherical - Start (" << lidar_x << ", " << lidar_y << ", " << lidar_z << ") ";
  std::cout << " Pitch: 0 Azimuth: 0 Range: 10" << std::endl;

  for(const auto& voxel : voxels) {
    int x, y, z;
    std::tie(x, y, z) = voxel;
    std::cout << "Voxel: (" << x << ", " << y << ", " << z << ")\n";
  }

  voxels = grid.drawLineFromSpherical(lidar_x, lidar_y, lidar_z, 0.0, 45.0, 10.0);
  std::cout << "LineFromSpherical - Start (" << lidar_x << ", " << lidar_y << ", " << lidar_z << ") ";
  std::cout << " Pitch: 0 Azimuth: 45 Range: 10" << std::endl;

  for(const auto& voxel : voxels) {
    int x, y, z;
    std::tie(x, y, z) = voxel;
    std::cout << "Voxel: (" << x << ", " << y << ", " << z << ")\n";
  }

  voxels = grid.drawLineFromSpherical(lidar_x, lidar_y, lidar_z, 0.0, -90.0, 10.0);
  std::cout << "LineFromSpherical - Start (" << lidar_x << ", " << lidar_y << ", " << lidar_z << ") ";
  std::cout << " Pitch: 0 Azimuth: -90 Range: 10" << std::endl;
  for(const auto& voxel : voxels) {
    int x, y, z;
    std::tie(x, y, z) = voxel;
    std::cout << "Voxel: (" << x << ", " << y << ", " << z << ")\n";
  }



  while(avt_341::node::ok()) {
    n->spin_some();
    rate.sleep();
  }

}




