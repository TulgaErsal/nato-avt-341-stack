/* avt_341_lidar_occlusion_node.cpp

 * ROS Node for simulating occlusion of 3D LiDAR 

**/

// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

avt_341::msg::PointCloud2 in_points, occ_mask;
bool points_rcvd = false;
bool mask_rcvd = false;
bool using_mask = false;
bool print_contents = true;


std::vector<int> mask_vector;

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
  auto odom_sub = n->create_subscription<avt_341::msg::PointCloud2>("avt_341/points",10, IncomingPointCloud2Callback);
  auto mask_sub = n->create_subscription<avt_341::msg::PointCloud2>("avt_341/occ_mask", 10, OcclusionMaskCallback);

	// Publishers
  auto points_pub = n->create_publisher<avt_341::msg::PointCloud2>("avt_341/occ_points", 1);

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
  createSquareMask(start_row, mask_height, start_col, mask_width, 64, 1024);
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

      // print the contents of the lidar data structure
      if (print_contents) {
        print_contents = false;     // print once
        std::cout << "Height: " << in_points.height << ", Width: " << in_points.width << std::endl;
        for(const auto &field : in_points.fields) {
          std::cout << "Field name: " << field.name << ", Datatype: " << static_cast<int>(field.datatype) << std::endl;
        }
      }
      out_points = in_points;
      if(!using_mask) {
        std::cout << "Not using occlusion mask." << std::endl;
      } else {
        std::cout << "Applying occlusion mask." << std::endl;
        applyMaskToPointCloud(out_points);
      }
      points_pub->publish(out_points);

    }
    n->spin_some();
    rate.sleep();
  }
  return 0;
}

