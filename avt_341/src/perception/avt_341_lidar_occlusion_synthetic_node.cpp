/* avt_341_lidar_occlusion_synthetic_node.cpp

 * ROS Node for generating synthetic occlusions of lidar sensor data
 * Subscribers: gen_mask_request (int32), activate_mask (Int32), points (PointCloud2)
 * Publishers:  gt_occ_mask (Image), gt_occ_detected (int32), occ_points (PointCloud2)

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

avt_341::msg::PointCloud2 in_points, out_occ_points;
std::vector<uint8_t> mask_vector;
avt_341::msg::Int32 gt_occ_detected;
avt_341::msg::Image gt_occ_mask;

// status tracking 
bool points_rcvd = false;
bool activate_mask = false;

// debug statements
bool print_contents = true;

double toRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

double toDegrees(double radians) {
  return radians * 180.0 / M_PI;
}

// message handlers
void IncomingPointCloud2Callback(avt_341::msg::PointCloud2Ptr rcv_points) {
  std::cout << "Occlusion Synthetic Node: Received incoming point cloud.";
  in_points = *rcv_points;
  points_rcvd = true;
  if(print_contents) {
    std::cout << "  PC Height: " << in_points.height << ", Width: " << in_points.width << std::endl;
    for(const auto &field : in_points.fields) {
      std::cout << "Field name: " << field.name << ", Datatype: " << static_cast<int>(field.datatype) << std::endl;
    }
    print_contents = false;   // print once
  }
}

void ActivateMaskCallback(avt_341::msg::Int32Ptr rcv_cmd) {
  std::cout << "Occlusion Synthetic Node:  Received request to activate mask. ";
  if(*rcv_cmd == 1) {
    "Activating mask." << std::endl;
    activate_mask = true;
  } else {
    "De-activating mask." << std::endl;
    activate_mask = false;
  }
}

void GenerateMaskCallback(avt_341::msg::Int32Ptr gen_mask_cmd) {
  std::cout << "Occlusion Synthetic Node: Received request to generate a new mask.";
  
}

// Utility functions
void createSquareMask(int row, int height, int col, int width, int cloud_height, int cloud_width) {
  /// row is start position for rows (height); 0 is bottom beam, height is top beam
  /// col is start position for columns (width); 0 is directly in rear of the vehicle; 1 is to the front and left by 1 step; width is to the front and right by 1 step
  /// height is how many rows to mask
  /// width is how many columns to mask
  /// row + height cannot exceed cloud_height
  /// if col + width exceeds cloud_width - we should wrap around to 0 + ((col + width) - cloud_width)
  /// for every increment in index, the _row_ increases
  
  // check incoming values
  if(cloud_height < 1) {
    std::cout << "Warning: Invalid cloud height (number of beams) (" << cloud_height << "). Setting cloud height to 1." << std::endl;
    cloud_height = 1;
  }
  if(cloud_width < 1) {
    std::cout << "Warning: Invalid cloud width (horizontal resolution) (" << cloud_width << "). Setting cloud width to 1." << std::endl;
    cloud_width = 1;
  }
  // don't allow negative numbers and don't allow a start position greater than the dimensions of the cloud
  if(col > cloud_width) {
    std::cout << "Warning: Start column " << col << " is greater than the point cloud horizontal resolution " << cloud_width << ". Setting column to " << cloud_width - 1 << std::endl;
  }
  if(col < 0) {
    std::cout << "Warning: Negative column index is not allowed. Setting column to 0." << std::endl;
  }
  if(row > cloud_height) {
    std::cout << "Warning: Start row " << row << " is greater than the point cloud number of beams " << cloud_height << ". Setting row to " << cloud_height - 1 << std::endl;
  }
  if(row > 0 ) {
    std::cout << "Warning: Negative row index is not allowed. Setting row to 0." << std::endl;
  }
  col = std::max(0, std::min(col, cloud_width-1));
  row = std::max(0, std::min(row, cloud_height-1));

  // clamp the mask region to the bounds of the point cloud (no vertical wrapping)
  int col_end = col + width;
  int row_end = row + height;
  if(row_end > cloud_height) {
    std::cout << "Warning: End row (row: " << row << " height: " << height << " row_end: " << row_end << ") is greater than the number of beams. Clamping to " << cloud_height << std::endl;
  }
  if(col_end > cloud_width) {
    if(col_end - cloud_width < cloud_width) {
      std::cout << "Warning: End column (column: " << col << " height: " << height << " column_end: " << col_end << ") is greater than the horizontal resolution. Wrapping and applying to columns 0 - " << col_end - cloud_width << ". " << std::endl;
    } else {
      std::cout << "Warning: End column way out of bounds (column: " << col << " height: " << height << " column_end: " << col_end << ")." << std::endl;
    }
  }
  row_end = std::min(cloud_height, row_end);

  // initialize the mask vector
  if(mask_vector.empty()) {
    mask_vector.resize(cloud_height * cloud_height, 0);
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
  // Create iterators
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
      // Setting x, y, z to 0 to represent occlusion.
      *(iter_x + i) = 0.0f;
      *(iter_y + i) = 0.0f;
      *(iter_z + i) = 0.0f;
      *(iter_intensity + i) = 128.0f;
    }
  }

  // set intensity
  for(int i = index; i < index + span; ++i) {
    *(iter_intensity + i) = 255.0f;
  }
    
  //std::cout << "Set " << count << " points x,y,z to 0" << std::endl;
}

// Main Node Function
int main(int argc, char*argv[]) {
  // Initialize the node
  auto n = avt_341::node::init_node(argc, argv, "lidar_occlusion_synthetic_node");

  // Subscriptions
  auto points_sub = n->create_subscription<avt_341::msg::PointCloud2>("avt_341/points", 10, IncomingPointCloud2Callback);
  auto activate_mask_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/activate_mask", 1, ActivateMaskCallback);

  // Publishers: gt_occ_mask (Image), gt_occ_detected (int32), occ_points (PointCloud2)
  auto occ_points_pub  = n->create_publisher<avt_341::msg::PointCloud2>("avt_341/occ_points", 10);
  auto gt_occ_mask_pub = n->create_publisher<avt_341::msg::Image>("avt_341/gt_occ_mask", 1);
  auto gt_occ_detected = n->create_publisher<avt_341::msg::Int32>("avt_341/gt_occ_detected", 1);

  // parameters and default values
  bool occluded = true;     // is the mask applied to the incoming points to generate occluded points? 
  int start_row = 0;        // start position of the mask
  int start_col = 0;
  int mask_height = 64;     // dimensions of the mask
  int mask_width = 512;
  int cloud_height = 64;
  int cloud_width = 1024;
   
  n->get_parameter("~occluded", occluded, true);
	n->get_parameter("~start_row", start_row, 0);
  n->get_parameter("~start_col", start_col, 0);
  n->get_parameter("~mask_height", mask_height, 64);
  n->get_parameter("~mask_width", mask_width, 512);
  n->get_parameter("~cloud_height", cloud_height, 64);
  n->get_parameter("~cloud_width", cloud_width, 1024);
  
  avt_341::msg::PointCloud2 out_points;
  avt_341::node::Rate rate(10.0);

  std::cout << "Creating mask" << std::endl;
  createSquareMask(start_row, mask_height, start_col, mask_width, cloud_height, cloud_width); 

  bool using_mask = false;

  if(occluded) {
    std::cout << "Lidar Occlusion Synthetic Node occluding points on startup." << std::endl;
    using_mask = true;
  }
  
  // Enter the loop
  while(avt_341::node::ok()) {
    if(!points_rcvd) {
      std::cout << "Lidar Occlusion Synthetic Node: No points received." << std::endl;
    } else {
      std::cout << "Lidar Occlusion Synthetic Node: New point cloud received." << std::endl;
      // process the incoming points
      out_occ_points = in_points;
      if(!using_mask) {
        std::cout << "Lidar Occlusion Synthetic Node: Mask not in use. Points not modified." << std::endl;
      } else {
        applyMaskToPointCloud(out_occ_points);

        // prepare ground truth gt_occ_detected message
        gt_occ_detected.header.stamp = n->get_stamp();
        gt_occ_detected.data = 1;
        gt_occ_detected_pub = publish(gt_occ_detected);

        // prepare ground truth gt_occ_mask message
        gt_occ_mask.header.stamp = n->get_stamp();
        gt_occ_mask.height = out_points.height;
        gt_occ_mask.width = out_points.width;
        gt_occ_mask.encoding = "mono8";
        gt_occ_mask.step = mask_image.height * mask_image.width;
        gt_occ_mask.data = mask_vector;
        gt_occ_mask_pub->publish(gt_occ_mask);
      }
      occ_points_pub->publish(out_occ_points);
    }
    n->spin_some();
    rate.sleep();
  }
  return 0;
}