/* avt_341_lidar_occlusion_node.cpp

 * ROS Node for simulating occlusion of 3D LiDAR 

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

avt_341::msg::PointCloud2 in_points, occ_mask;
bool points_rcvd = false;
bool mask_rcvd = false;
bool using_mask = false;
bool print_contents = true;

std::vector<uint8_t> mask_vector;
avt_341::msg::Image mask_image;

// Utility function to create a vector of spaced values
std::vector<double> linspace(double start, double end, int num) {
    std::vector<double> linspaced;

    if (num == 0) { 
        return linspaced; 
    }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for(int i=0; i < num-1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // Ensure that end is exactly end

    return linspaced;
}

// Utility function to convert degrees to radians
double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

void createSquareMask(int row, int height, int col, int width, int cloud_height, int cloud_width) {
    /// row is start position for rows (height); 0 is bottom beam, height is top beam
    /// col is start position for columns (width); 0 is directly in rear of the vehicle; 1 is to the front and left by 1 step; width is to the front and right by 1 step
    /// height is how many rows to mask
    /// width is how many columns to mask
    /// row + height cannot exceed cloud_height
    /// if col + width exceeds cloud_width - we should wrap around to 0 + ((col + width) - cloud_width)
    /// for every increment in index, the _row_ increases
    
    // Initialize the mask vector with zeros if not done already.
    if (mask_vector.empty()) {
        mask_vector.resize(cloud_height * cloud_width, 0);
    }

    // start position - column and row
    // don't allow negative numbers and don't allow a start position greater than the width of the cloud
    if(col > cloud_width) {
      std::cout << "Warning: Start Column " << col << " greater than point cloud width " << cloud_width << ". Setting col to " << cloud_width - 1 << std::endl;
    }
    if(col < 0) {
      std::cout << "Warning: Negative column index is not allowed. Setting col to 0." << std::endl;
    }
    col = std::max(0, std::min(col, cloud_width - 1));
    if(row < 0) {
      std::cout << "Warning: Negative row index is not allowed. Setting row to 0." << std::endl;
    }
    row = std::max(0, row);   // assume y starts from 0 and goes up
    
    // Clamp the mask region to the bounds of the point cloud
    //int max_x = std::min(x + vert, cloud_height);
    //int max_y = std::min(y + horz, cloud_width);
    int col_end = col + width;
    int row_end = row + height;
    if (row_end > cloud_height) {
      std::cout << "Warning: End row position (row: " << row << " + " << " height: " << height << " = row_end: " << row_end << ") is greater than point cloud height. Clamping to " << cloud_height << "." << std::endl;
    }
    row_end = std::min(cloud_height, row_end);
    if(col_end > cloud_width) {
      std::cout << "End column position (col: " << col << " + " << " width: " << width << " = col_end: " << col_end << ") is greater than point cloud width. Wrapping and applying to columns 0 - " << col_end - cloud_width << "." << std::endl;
    }

    // Populate the mask vector with 1s in the specified region
    for (int curr_row = row; curr_row < row_end; ++curr_row) {
        for (int curr_col = col; curr_col < col_end; ++curr_col) {
            int wrapped_col = curr_col % cloud_width;
            int index = curr_row * cloud_width + wrapped_col;

            // This does not seem to match documentation for PointCloud2 (should be row major not col major)
            index = wrapped_col * cloud_height + curr_row;
            //std::cout << "Masking Index: " << index << " Col: " << wrapped_col << ", Row: " << curr_row << std::endl;
            
            mask_vector[index] = 1;
        }
    }
}

void applyMaskToPointCloud(sensor_msgs::PointCloud2 &cloud) {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
    int count = 0;
    std::cout << "Point Cloud Height: " << cloud.height << " Width: " << cloud.width << " Size: " << cloud.height * cloud.width << std::endl;
    std::cout << "Mask Size: " << mask_vector.size() << std::endl;

    int row = 0;
    int col = 0;
    int width = 1024;
    int height = 64;
    int span = 2;
    int index = row * width + col; 
    index = col * height + row;

    
    for (size_t i = 0; i < mask_vector.size(); ++i) {
        if (mask_vector[i] == 1) {
          count++;
            //std::cout << "Masked point coordinates (before): ("  << *(iter_x + i) << ", " << *(iter_y + i) << ", " << *(iter_z + i) << ")" << std::endl;
            // Setting x, y, z to 0 to represent occlusion.
            *(iter_x + i) = 0.0f;
            *(iter_y + i) = 0.0f;
            *(iter_z + i) = 0.0f;
            *(iter_intensity + i) = 128.0f;
            //std::cout << "Masked point coordinates (after): (" << *(iter_x + i) << ", " << *(iter_y + i) << ", " << *(iter_z + i) << ")" << std::endl;
            
        }
    }

    for(int i = index; i < index + span; ++i) {
      *(iter_intensity + i) = 255.0f;
    }
    
    //std::cout << "Set " << count << " points x,y,z to 0" << std::endl;
}

std::vector<std::vector<int>> maskToGrid(const std::vector<uint8_t>& occlusionVector,
                                                  int numBeams = 64,
                                                  int horzResolution = 1024,
                                                  float lidarFOV = 45.0, 
                                                  float maxRange = 100.0,   // Max range of LiDAR
                                                  float lidarHeight = 1.8,
                                                  int gridSize = 200, 
                                                  int ceilingHeight = 4,
                                                  float cellSize = 0.5) { 
    // create voxel grid
    int gridResolutionXY = gridSize / cellSize;
    int gridResolutionZ = ceilingHeight / cellSize;
    std::cout << "Creating voxel grid (" << gridResolutionXY << "x" << gridResolutionXY << "x" << gridResolutionZ << ")"<< std::endl;
    std::vector<std::vector<std::vector<int>>> voxelGrid(gridResolutionXY, std::vector<std::vector<int>>(gridResolutionXY, std::vector<int>(gridResolutionZ, 0)));
    
    // create map
    std::cout << "Creating map (" << gridResolutionXY << "x" << gridResolutionXY << ")" << std::endl;
    std::vector<std::vector<int>> occlusionMap(gridResolutionXY, std::vector<int>(gridResolutionXY, -1));

    float lidarX = (gridSize / 2) / cellSize;   // lidar is in the center of the grid
    float lidarY = (gridSize / 2) / cellSize;
    float lidarZ = lidarHeight;

    std::vector<double> verticalAngles = linspace(-lidarFOV/2, lidarFOV/2, numBeams);   // assumes lidar FOV centered on horizon and beams evenly spaced
    std::vector<double> horizontalAngles = linspace(0, 360, horzResolution);
    horizontalAngles.pop_back();  /// remove the last element to avoid duplicating 0 and 360
    
    std::cout << "Processing " << numBeams << " beams with " << horzResolution << " horizontal resolution (samples per 360 deg)" << std::endl;

    int blocked = 0;
    int blocked_beam = -1;
    int one_time = 0;
    int one_time_line = 0;
    for(int beamIndex=0; beamIndex < numBeams; beamIndex++) {
      for(int angleIndex=0; angleIndex < horzResolution; angleIndex++) {
        double verticalAngle = verticalAngles[beamIndex];
        double horizontalAngle = horizontalAngles[angleIndex];
        // Is the beam blocked by the mask? If so, skip
        if(occlusionVector[angleIndex * numBeams + beamIndex] == 1) {
          // Report blocked beams (only once per beam index - not really useful)
          if(blocked_beam != beamIndex) { 
            std::cout << beamIndex << " is blocked. Vert: " << verticalAngle << " Horz: " << horizontalAngle << std::endl;
            blocked_beam = beamIndex;
          }
          blocked++;
        } else {          
          // calculate which cells the ray intersects 
          // calculate ray direction and dx, dy, dz
          double angleRad = deg2rad(horizontalAngle);
          double zAngleRad = deg2rad(verticalAngle);
          double dx = cos(angleRad) * cos(zAngleRad);
          double dy = sin(angleRad) * cos(zAngleRad);
          double dz = sin(zAngleRad);
          // Report the processing data for the first blocked beam we encounter - just gives us some sample output
          if(!one_time && blocked_beam != -1) {
            std::cout << "   Processing ray for beam " << beamIndex << "(" << verticalAngle << ") at " << angleIndex << "(" <<  horizontalAngle << ")" << std::endl;
            std::cout << "       From: " << lidarX << ", " << lidarY << ", " << lidarZ << " " << std::endl;
            std::cout << "       angleRad: " << angleRad << ", zAngleRad: " << zAngleRad << ", dx:" << dx << ", dy:" << dy << ", dz:" << dz << std::endl;
            std::cout << "       gridResolutionXY: " << gridResolutionXY << ", gridResolutionZ: " << gridResolutionZ << std::endl;
            one_time = 1;
          }
          // step along the ray in cell size steps - should probably step along the x in cell size steps and apply the same t 
          // to the other axes - this doesn't sample the center of each cell

          // rotate the dx, dy
          double tdx = dy;
          double tdy = -dx; 

          for(float t=0; t <= maxRange; t+= cellSize) {
            // calculate the ray position
            int x = static_cast<int>(lidarX + t * dx / cellSize);
            int y = static_cast<int>(lidarY + t * dy / cellSize);
            int z = static_cast<int>(lidarZ + t * dz / cellSize);

            // rotate x,y to match occupancy grid coordinate space
            int transformedX = static_cast<int>(lidarX + t * tdx / cellSize);
            int transformedY = static_cast<int>(lidarY + t * tdy / cellSize);
            //int gridX = transformedX + GRID_HEIGHT / 2; 
            //int gridY = transformedY + GRID_WIDTH / 2;

            // Report the step information
            if(!one_time_line && blocked_beam != -1) {
              std::cout << "           Step: " << t << " of " << maxRange << " at " << cellSize << " steps. Grid position: " << x << "," << y <<"," << z << ". Transformed position: " << transformedX << "," << transformedY <<"," << z <<  std::endl;
              //std::cout << "           Step: " << t << " of " << maxRange << " at " << cellSize << " steps. Grid position: " << x << "," << y <<"," << z << std::endl;
            }
            x = transformedX;
            y = transformedY;
            // check grid bounds
            // possible optimization? We can stop checking once the ray passes outside of the grid resolution
            if(x >= 0 && x < gridResolutionXY && y >= 0 && y < gridResolutionXY && z >= 0 && z < gridResolutionZ) {
              voxelGrid[x][y][z] = 1; // mark the voxel as 'seen'
              //occlusionMap[x][y]++; // increment beams 'seeing' this column
              occlusionMap[x][y] = 1; // set as 'seen'
              if(!one_time_line && blocked_beam != -1) { 
                std::cout << "             Marking voxel xyz (" << x << ", " << y << ", " << z << ") as seen." << std::endl;
              }
            }
          }
          if(!one_time_line && blocked_beam != -1) {
             one_time_line = 1;
          }
        }
      }      
      //std::cout << "Completed beam: " << beamIndex << std::endl;
    }  

    /*  DEBUG - direct write onto the occlusion_map
    for(int i = 50; i < 75; i++) {
        for(int j = 25; j < 50; j++) {
          occlusionMap[i][j] = 1;
        }
      }
    */

    
    std::cout << "Returning map. Blocked Rays:" << blocked << std::endl;
    return occlusionMap;
}

void IncomingPointCloud2Callback(avt_341::msg::PointCloud2Ptr rcv_points) {
  in_points = *rcv_points;
  points_rcvd = true;
}

void OcclusionMaskCallback(avt_341::msg::PointCloud2Ptr rcv_points) {
  occ_mask = *rcv_points;
  mask_rcvd = true;
  using_mask = true;
}

int main(int argc, char *argv[]) {
	// Initialize the node
	auto n = avt_341::node::init_node(argc, argv, "lidar_occlusion_node");

	// Subscriptions
  auto odom_sub = n->create_subscription<avt_341::msg::PointCloud2>("avt_341/points", 10, IncomingPointCloud2Callback);
  
	// Publishers
  auto points_pub = n->create_publisher<avt_341::msg::PointCloud2>("avt_341/occ_points", 10);
  auto mask_pub = n->create_publisher<avt_341::msg::Image>("avt_341/occ_mask", 1);
  auto mask_grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occ_mask_grid", 1); 

	// handle parameters
  bool occluded = true;
  double timer = -1.0;
  int start_row = 0;
  int start_col = 0;
  int mask_height = 64;
  int mask_width = 512; 
  
  n->get_parameter("~occluded", occluded, true);
  n->get_parameter("~timer", timer, -1.0);
	n->get_parameter("~start_row", start_row, 0);
  n->get_parameter("~start_col", start_col, 0);
  n->get_parameter("~mask_height", mask_height, 64);
  n->get_parameter("~mask_width", mask_width, 512);
  
  avt_341::msg::PointCloud2 out_points;
  avt_341::node::Rate rate(10.0);

  //createSquareMask(0, 512, 0, 64, 1024, 64);
  //createSquareMask(0, 64, 0, 512, 64, 1024);
  std::cout << "Creating mask" << std::endl;
  createSquareMask(start_row, mask_height, start_col, mask_width, 64, 1024); 
  std::vector<std::vector<int>> occ_mask = maskToGrid(mask_vector, 64, 1024, 45.0, 100.0, 1.8, 100, 4, 0.5); 
  std::cout << "Converting mask to occupancy grid (" << occ_mask[0].size() << "x" << occ_mask.size() << ")" << std::endl;
  avt_341::msg::OccupancyGrid mask_grid;

  std::cout << "Checking timer..." << std::endl;
  // check timer
  if(occluded && timer < 0.001) {
    std::cout << "Applying occlusion at startup (occluded = " << occluded << ", timer = " << timer << std::endl;
    using_mask = true;
  } else {
    using_mask = false;
  }

  double elapsed = 0;
  double start_time = -1.0;

  while(avt_341::node::ok()) {
    if(!points_rcvd) {
      std::cout << "No point cloud received." << std::endl;
    } else {
      // handle timer if necessary
      if(occluded && using_mask == false && timer > 0.0) 
      {
        // start timer on first points received
        if(start_time < 0.0) {
          start_time = n->get_now_seconds();
        }
        // check elapsed time
        elapsed = n->get_now_seconds() - start_time;
        if(elapsed > timer) {
          // start using the mask
          using_mask = true;
        }
      }

      // print the contents of the lidar data structure on first pass
      if (print_contents) {
        print_contents = false;     // print once
        std::cout << "Height: " << in_points.height << ", Width: " << in_points.width << std::endl;
        for(const auto &field : in_points.fields) {
          std::cout << "Field name: " << field.name << ", Datatype: " << static_cast<int>(field.datatype) << std::endl;
        }
      }

      // process points
      out_points = in_points;
      if(!using_mask) {
        std::cout << "Not using occlusion mask." << std::endl;
      } else {
        std::cout << "Applying occlusion mask." << std::endl;
        applyMaskToPointCloud(out_points);

        mask_image.header.stamp = n->get_stamp();
        mask_image.height = out_points.height;
        mask_image.width = out_points.width;
        mask_image.encoding = "mono8";
        mask_image.step = mask_image.height * mask_image.width;
        mask_image.data = mask_vector;
        mask_pub->publish(mask_image);

        mask_grid.header.stamp = n->get_stamp();
        mask_grid.header.frame_id = "lidar";
        mask_grid.info.resolution = 0.5;
        mask_grid.info.width = occ_mask[0].size();
        mask_grid.info.height = occ_mask.size();
        mask_grid.info.origin.position.x = -50;
        mask_grid.info.origin.position.y = -50;
        mask_grid.info.origin.position.z = 0.0;
        mask_grid.info.origin.orientation.w = 1.0; 
        mask_grid.data.resize(mask_grid.info.width * mask_grid.info.height);
        for(size_t y=0; y < occ_mask.size(); ++y) {
          for(size_t x=0; x < occ_mask[0].size(); ++x) {
            mask_grid.data[x + y * mask_grid.info.width] = occ_mask[y][x];
          }
        }
        mask_grid_pub->publish(mask_grid);
      }
      points_pub->publish(out_points);
    }
    n->spin_some();
    rate.sleep();
  }
  return 0;
}

