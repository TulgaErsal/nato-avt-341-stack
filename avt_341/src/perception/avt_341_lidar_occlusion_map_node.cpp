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
bool proportional = false;
std::string frame = "front/os_lidar";

  

void IncomingOcclusionMaskCallback(avt_341::msg::ImagePtr rcv_image) {
  avt_341::msg::Image flipped_img = *rcv_image;

  if(rcv_image->encoding == "32FC1") {
    int pixel_size = sizeof(float);

    // Accessing each row from the bottom to the top
    for (int i = 0; i < rcv_image->height; ++i) {
        int index1 = i * rcv_image->width * pixel_size;
        int index2 = (rcv_image->height - 1 - i) * rcv_image->width * pixel_size;
        for (int j = 0; j < rcv_image->width * pixel_size; ++j) {
            // Swapping the pixels vertically
            flipped_img.data[index1 + j] = rcv_image->data[index2 + j];
        }
    }

    occ_mask = flipped_img;
  } else {
    std::cout << "NOT VERTICALLY FLIPPED" << std::endl;
    occ_mask = *rcv_image;
  }
  
  std::cout << "Received occlusion mask of format: " << rcv_image->encoding << " with width: " << rcv_image->width << " and height: " << rcv_image->height << std::endl;
  if(rcv_image->width != lidar_horz_resolution) {
    std::cout << "Received occlusion mask width does not match expected resolution (" << lidar_horz_resolution << ")" << std::endl;
  }
  if(rcv_image->height != lidar_beams) {
    std::cout << "Received occlusion mask height does not match expected resolution (" << lidar_beams << ")" << std::endl;
  }
  occ_mask_rcvd = true;
}

bool getMaskValue(int row, int col) {
  //std::cout << "  Getting Mask Value for " << row << ", " << col << std::endl;
  if(occ_mask.encoding == "32FC1") {
    //std::cout << "  Mask Encoding 32FC1" << std::endl;
    // 32FC1 uses 4 bytes to store a float
    const float* imageData = reinterpret_cast<const float *>(&occ_mask.data[0]);
    int index = (occ_mask.width * row) + col;
    //std::cout << "  Index: " << index << " (Width: " << occ_mask.width << " * row: " << row << " + col: " << col << ")" << std::endl;
    float pixelValue = imageData[index];
    //std::cout << "  Value: " << pixelValue << std::endl;
    if(pixelValue > 0.9) {
      return true;
    } else {
      return false;
    }
  } else {
    std::cout << "Encoding not 32FC1" << std::endl;
  }
}

// Function to reshape the 1D occupancy grid data into a 2D grid
std::vector<std::vector<signed char>> reshapeGrid(const avt_341::msg::OccupancyGrid& occupancy_grid) {
    std::vector<std::vector<signed char>> grid;
    grid.reserve(occupancy_grid.info.height);
    for (int i = 0; i < occupancy_grid.info.height; ++i) {
        std::vector<signed char> row(occupancy_grid.info.width);
        for (int j = 0; j < occupancy_grid.info.width; ++j) {
            int index = i * occupancy_grid.info.width + j;
            row[j] = occupancy_grid.data[index];
        }
        grid.push_back(row);
    }
    return grid;
}

// Function to flatten the 2D grid back into a 1D array
std::vector<signed char> flattenGrid(const std::vector<std::vector<signed char>>& grid) {
    std::vector<signed char> data;
    for (const auto& row : grid) {
        data.insert(data.end(), row.begin(), row.end());
    }
    return data;
}

// Function to flip the grid data horizontally
void flipGridHorizontally(std::vector<std::vector<signed char>>& grid) {
    for (auto& row : grid) {
        std::reverse(row.begin(), row.end());
    }
}

// Function to rotate the grid data 90 degrees
void rotateGrid90Degrees(std::vector<std::vector<signed char>>& grid) {
    // Transpose the grid data
    std::vector<std::vector<signed char>> transposedGrid(grid[0].size(), std::vector<signed char>(grid.size()));
    for (size_t i = 0; i < grid.size(); ++i) {
        for (size_t j = 0; j < grid[0].size(); ++j) {
            transposedGrid[j][i] = grid[i][j];
        }
    }
    // Flip the transposed grid data horizontally to rotate it 90 degrees
    //flipGridHorizontally(transposedGrid);
    // Copy the rotated grid data back to the original grid
    grid = transposedGrid;
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
  n->get_parameter("~proportional", proportional, false);
  n->get_parameter("~frame", frame, std::string("front/os_lidar"));

  if(min_horizontal_angle > max_horizontal_angle) {
    std::cout << "LIDAR OCCLUSION MAP NODE: parameter min_horizontal_angle MUST be less than max_horizontal_angle" << std::endl;
  }
  if(min_vertical_angle > max_vertical_angle) {
    std::cout << "LIDAR OCCLUSION MAP NODE: parameter min_vertical_angle MUST be less than max_vertical_angle" << std::endl;
  }
  avt_341::node::Rate rate(10.0);

  // Create the grid
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
  // Create the Look up table
  std::vector<std::vector<std::vector<Voxel>>> gridLUT(
    lidar_beams,
    std::vector<std::vector<Voxel>>(lidar_horz_resolution)
  );
  
  // set up the pitch and azimuth according to the 
  double pitch_range = max_vertical_angle - min_vertical_angle;
  double pitch_step = pitch_range / lidar_beams;
  double horz_range = max_horizontal_angle - min_horizontal_angle;
  double horz_step = horz_range / lidar_horz_resolution;
  double pitch = 0.0;
  double azimuth = 0.0;
  auto start = std::chrono::high_resolution_clock::now();

  std::cout << "Map Node: P, " << min_vertical_angle << ", " << max_vertical_angle << ", " << pitch_range << ", " << pitch_step << std::endl;
  std::cout << "Map Node: A, " << min_horizontal_angle << ", " << max_horizontal_angle << ", " << horz_range << ", " << horz_step << std::endl;

  // Set up the Look Up Table 
  for(int i = 0; i < lidar_beams; ++i) {
    pitch = min_vertical_angle + (pitch_step * i);
    for(int j = 0; j < lidar_horz_resolution; ++j) {
      azimuth = min_horizontal_angle + (horz_step * j);
      gridLUT[i][j] = grid.drawLineFromSpherical(lidar_x, lidar_y, lidar_z,
                                  pitch, azimuth, lidar_range);
    }
  }
  std::cout << "Map Node: LUT initialized" << std::endl;
    
  // build a clean version of the grid using the LUT
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

    // Only do something if we have an updated occlusion mask
    if(occ_mask_rcvd) {
      std::cout << "Map Node: received updated occlusion mask" << std::endl;
      occ_mask_rcvd = false;

      // collect some statistics
      int count = 0;
      start = std::chrono::high_resolution_clock::now();
      
      // create a copy of the clean grid
      grid.copyCleanToDirty();
      
      // Decrement marks for voxels blocked by the occlusions
      for(int i = 0; i < lidar_beams; ++i) {
        for(int j = 0; j < lidar_horz_resolution; ++j) {
          int index = (i * lidar_horz_resolution) + j;
          //if(occ_mask.data[index] == 255) {
          if(getMaskValue(i, j)) {
            //std::cout << "Decrement, Index," << index << ", i, " << i << ", j, " << j << std::endl;
            count++;
            for(const auto& voxel : gridLUT[i][j]) {
              grid.decrementVoxel(i, j, std::get<0>(voxel), std::get<1>(voxel), std::get<2>(voxel));
            }
          }
        }
      }
      
      // collect and report the statistics
      end = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
      std::cout << "Blocked Rays: " << count << " Elapsed time: " << elapsed.count() << " ms" << std::endl;

      // publish the projection
      // Set up the header and other info
      mask_grid.header.stamp = n->get_stamp();
      mask_grid.header.frame_id = frame.c_str();
      mask_grid.info.resolution = grid_resolution;
      mask_grid.info.width = grid_width;
      mask_grid.info.height = grid_length;
      mask_grid.info.origin.position.x = 0.0 - ((grid_width * grid_resolution) / 2); // meters?
      mask_grid.info.origin.position.y = 0.0 - ((grid_length * grid_resolution) / 2);
      mask_grid.info.origin.position.z = -1;
      
      tf2::Quaternion rotation_quat;
      rotation_quat.setRPY(0, 0, -M_PI / 2.0);
      mask_grid.info.origin.orientation.x = rotation_quat.x();
      mask_grid.info.origin.orientation.y = rotation_quat.y();
      mask_grid.info.origin.orientation.z = rotation_quat.z(); 
      mask_grid.info.origin.orientation.w = rotation_quat.w();
      mask_grid.info.origin.orientation.x = mask_grid.info.origin.orientation.y = mask_grid.info.origin.orientation.z = 0.0;
      mask_grid.info.origin.orientation.w = 1.0;
      
      mask_grid.data.resize(mask_grid.info.width * mask_grid.info.height);
      std::fill(mask_grid.data.begin(), mask_grid.data.end(), 0); 


      // Update the occupancy grid with grid data
      for(int y = 0; y < grid_width; ++y) {
        for(int x = 0; x < grid_length; ++x) {
          if(grid.dirtyPlane[x][y] == 0) {      // if NOT seen, Mark as NOT seen
            mask_grid.data[x + y * grid_width] = 0.0;   
          } else {    // seen, at least somewhat
            // scale the difference between the dirty and clean by 100 to a percentage
            float scaled_value = grid.differencePlane[x][y] * 100.0;
            // if we're coloring the map by proportion - MARK with the proportion - as dirty goes to 0, difference will go to 0 (less seen)
            if(proportional) {
              mask_grid.data[x + y * grid_width] = static_cast<int>(scaled_value);
            } else {    
              // Otherwise - if the scaled value is less than 100 (dirty is different, set it as UNSEEN and we can play with this threshold)
              if(scaled_value < 95) {
                mask_grid.data[x + y * grid_width] = 0.0;
              } else {
                mask_grid.data[x + y * grid_width] = 100.0;
              }
            }  
          }
        }
      }

      std::vector<std::vector<signed char>> fix_grid = reshapeGrid(mask_grid);
      // Flip the grid data horizontally
      //flipGridHorizontally(fix_grid);
      // Rotate the grid data 90 degrees
      rotateGrid90Degrees(fix_grid);
      //flipGridHorizontally(fix_grid);
      // Flatten the data
      mask_grid.data = flattenGrid(fix_grid);

      // Publish the occupancy grid
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