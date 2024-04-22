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
#include <chrono>
// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/voxel_grid.hpp"

avt_341::msg::Image occ_mask; 
bool occ_mask_rcvd = false;

double lidar_range = 100.0f;  // range of the lidar system in meters
int lidar_beams = 64;
int lidar_horz_resolution = 1024;
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
  std::cout << "Received occlusion mask of width: " << rcv_image->width << " and height: " << rcv_image->height << std::endl;
  if(rcv_image->width != lidar_horz_resolution) {
    std::cout << "Received occlusion mask width does not match expected resolution (" << lidar_horz_resolution << ")" << std::endl;
  }
  if(rcv_image->height != lidar_beams) {
    std::cout << "Received occlusion mask height does not match expected resolution (" << lidar_beams << ")" << std::endl;
  }
  occ_mask_rcvd = true;
}

int main(int argc, char* argv[]) {
  // Initialize the node
  auto n = avt_341::node::init_node(argc, argv, "lidar_occlusion_map_node");

  // Subscriptions
  auto occ_mask_sub = n->create_subscription<avt_341::msg::Image>("avt_341/occ_mask", 1, IncomingOcclusionMaskCallback);

  // Publishers
  auto mask_grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occ_mask_grid", 1);
  avt_341::msg::OccupancyGrid mask_grid;

  // handle parameters
  n->get_parameter("~lidar_range", lidar_range, 100.0);
  n->get_parameter("~vehicle_height", vehicle_height, 3.0);
  n->get_parameter("~grid_resolution", grid_resolution, 0.5);
  n->get_parameter("~lidar_beams", lidar_beams, 64);
  n->get_parameter("~lidar_horz_resolution", lidar_horz_resolution, 1024);
  n->get_parameter("~lidar_min_vertical_angle", min_vertical_angle, -22.5);
  n->get_parameter("~lidar_max_vertical_angle", max_vertical_angle, 22.5);
  n->get_parameter("~lidar_min_horizontal_angle", min_horizontal_angle, 0.0);
  n->get_parameter("~lidar_max_horizontal_angle", max_horizontal_angle, 360.0);
  if(min_horizontal_angle > max_horizontal_angle) {
    std::cout << "LIDAR OCCLUSION MAP NODE: parameter min_horizontal_angle MUST be less than max_horizontal_angle" << std::endl;
  }
  if(min_vertical_angle > max_vertical_angle) {
    std::cout << "LIDAR OCCLUSION MAP NODE: parameter min_vertical_angle MUST be less than max_vertical_angle" << std::endl;
  }
  avt_341::node::Rate rate(10.0);

  // setup the grid
  // grid dimensions are 2x range and vehicle height
  int grid_length = (lidar_range * 2) / grid_resolution;
  int grid_width = (lidar_range * 2) / grid_resolution;
  int grid_height = vehicle_height / grid_resolution;
  VoxelGrid grid(grid_length, grid_width, grid_height, grid_resolution);

  // lidar_position = center of grid (lidar_range), lidar_mount_height
  lidar_x = grid.toVoxelCoord(lidar_range);   // center the lidar
  lidar_y = grid.toVoxelCoord(lidar_range); 
  double lidar_z = grid.toVoxelCoord(lidar_mount_height);
  std::cout << "Map Node: setting up LUT" << std::endl;
    
  // Set up the grid look up table
  std::vector<std::vector<std::vector<Voxel>>> gridLUT(
    lidar_beams,
    std::vector<std::vector<Voxel>>(lidar_horz_resolution)
  );
  
  double pitch_range = max_vertical_angle - min_vertical_angle;
  double pitch_step = pitch_range / lidar_beams;
  double horz_range = max_horizontal_angle - min_horizontal_angle;
  double horz_step = horz_range / lidar_horz_resolution;
  double pitch = 0.0;
  double azimuth = 0.0;
  auto start = std::chrono::high_resolution_clock::now();
  std::cout << "Map Node: P, " << min_vertical_angle << ", " << max_vertical_angle << ", " << pitch_range << ", " << pitch_step << std::endl;
  std::cout << "Map Node: A, " << min_horizontal_angle << ", " << max_horizontal_angle << ", " << horz_range << ", " << horz_step << std::endl;
  for(int i = 0; i < lidar_beams; ++i) {
    pitch = min_vertical_angle + (pitch_step * i);
    for(int j = 0; j < lidar_horz_resolution; ++j) {
      azimuth = min_horizontal_angle + (horz_step * j);
      gridLUT[i][j] = grid.drawLineFromSpherical(lidar_x, lidar_y, lidar_z,
                                  pitch, azimuth, lidar_range);
      /*std::cout << "Ray(" << i << ", " << j << " (" << pitch << ", " << azimuth << ") " << std::endl;
      for(const auto& voxel : gridLUT[i][j]) {
        int x, y, z;
        std::tie(x, y, z) = voxel;
        std::cout << "   Voxel: (" << x << ", " << y << ", " << z << ")\n";
      }*/
    }
  }
  std::cout << "Map Node: LUT initialized" << std::endl;
    
  // build default voxel grid
  for(int i = 0; i < lidar_beams; ++i) {
    for(int j = 0; j < lidar_horz_resolution; ++j) {
      int index = (i * lidar_horz_resolution) + j;
      //std::cout << "Increment, Index, " << index << ", i, " << i << ", j, " << j << std::endl;
      for(const auto& voxel : gridLUT[i][j]) {
        // true argument indicates the 'clean' grid
        grid.incrementVoxel(i, j, std::get<0>(voxel), std::get<1>(voxel), std::get<2>(voxel), true);
      }
    }
  }
  std::cout << "Map Node: clean grid initialized" << std::endl;
    
  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
  std::cout << "Map Node: Elapsed time: " << elapsed.count() << " ms" << std::endl;
  
  while(avt_341::node::ok()) {
    //std::cout << "Map Node: starting loop" << std::endl;
    if(occ_mask_rcvd) {
      std::cout << "Map Node: received updated occlusion mask" << std::endl;
      occ_mask_rcvd = false;

      // handle the updated mask
      int count = 0;
      start = std::chrono::high_resolution_clock::now();
      //grid.reset(false);
      grid.copyCleanToDirty();
      
      for(int i = 0; i < lidar_beams; ++i) {
        for(int j = 0; j < lidar_horz_resolution; ++j) {
          int index = (i * lidar_horz_resolution) + j;
          if(occ_mask.data[index] == 255) {
            //std::cout << "Decrement, Index," << index << ", i, " << i << ", j, " << j << std::endl;
            count++;
            for(const auto& voxel : gridLUT[i][j]) {
              grid.decrementVoxel(i, j, std::get<0>(voxel), std::get<1>(voxel), std::get<2>(voxel));
            }
          }
        }
      }
      
      end = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
      std::cout << "Count: " << count << " Elapsed time: " << elapsed.count() << " ms" << std::endl;

      // publish the projection
      mask_grid.header.stamp = n->get_stamp();
      mask_grid.header.frame_id = "lidar";
      mask_grid.info.resolution = grid_resolution;
      mask_grid.info.width = grid_width;
      mask_grid.info.height = grid_length;
      mask_grid.info.origin.position.x = -100; // should these be cells or meters?
      mask_grid.info.origin.position.y = -100;
      mask_grid.info.origin.position.z = 0;
      mask_grid.info.origin.orientation.w = 1.0;
      mask_grid.data.resize(mask_grid.info.width * mask_grid.info.height);
      std::fill(mask_grid.data.begin(), mask_grid.data.end(), 0); 

      std::cout << "Grid: " << grid_width << " x " << grid_length << std::endl;
      for(int y = 0; y < grid_width; ++y) {
        for(int x = 0; x < grid_length; ++x) {
          //mask_grid.data[x + (y * grid_width)] = grid.dirtyPlane[x][y];
          float scaled_value = grid.differencePlane[x][y] * 100.0;
          mask_grid.data[x + y * grid_width] = static_cast<int>(scaled_value);
          /*
          if(grid.dirtyPlane[x][y] < 0 || grid.dirtyPlane[x][y] > 255) {
            std::cout << "What?" << x << ", " << y << ": " << grid.dirtyPlane[x][y] << std::endl;
          }*/
        }
      }

      //for(size_t y=0; y < grid_width; ++y) {
      //  for(size_t x=0; x < grid_length; ++x) {
          //mask_grid.data[x + y * grid_width] = grid.dirtyPlane[y][x];
          //mask_grid.data[x + y * grid_width] = grid.differencePlane[y][x]*255;
      //  }
      //}
      mask_grid_pub->publish(mask_grid);
    }

    n->spin_some();
    rate.sleep();
  }

}




/* Simple tests of the drawLine functions
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
  */