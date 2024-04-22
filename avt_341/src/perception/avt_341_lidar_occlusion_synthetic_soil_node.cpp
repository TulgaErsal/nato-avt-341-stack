/* avt_341_lidar_occlusion_synthetic_node.cpp

 * ROS Node for generating synthetic occlusions
 * Subscribers: gen_mask_request (int32)
 * Publishers:  gt_occ_mask (Image)

**/

// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <ctime>
// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

avt_341::msg::Image occ_mask;

int lidar_beams = 64;
int lidar_horz_resolution = 1024;
bool gen_mask= false;
std::string frame = "lidar";
int mask_type = 0;
int sq_row = 0;
int sq_height = 64;
int sq_col = 0;
int sq_width = 100;
double timer = 1.0;
 
void IncomingMaskRequest(avt_341::msg::Int32Ptr rcv_request) {
  if(rcv_request->data > 0) {
    // generate a new mask
    gen_mask = true; 
    mask_type = 1;  // TODO - support other mask types
  }
}

void createSquareMask(int row, int height, int col, int width, int lidar_beams, int lidar_horz_resolution) {
  // create a square mask 
  // row is start position for rows (height); 0 is bottom beam, height is top beam
  // col is start position for cols (width); 0 is rear of the vehicle in MAVS; 1 is to the front and left by 1 step; width is to the front and right by 1 step;
  // height is how many rows to mask
  // width is how many columns to mask
  // row + height cannot exceed lidar_beams
  // if col + width exceeds lidar_horz_resolution - we should wrap around to 0 + ((col + width) - lidar_horz_resolution)
  // for every increment in index, the _row_ increases (weird)
  
  // start position - column and row
  // don't allow negative numbers or greater than height or width
  if(col > lidar_horz_resolution) {
    std::cout << "Synthetic Node: Warning: Start Column " << col << " greater than the lidar horizontal resolution " << lidar_horz_resolution << ". Setting col to " << lidar_horz_resolution - 1 << std::endl;
  }
  if(col < 0) {
    std::cout << "Synthetic Node: Warning: Start column " << col << " is less than 0. Setting col to 0." << std::endl;
  }
  col = std::max(0, std::min(col, lidar_horz_resolution - 1));

  if(row < 0) {
    std::cout << "Synthetic Node: Warning: Negative row index is not allowed. Setting row to 0." << std::endl;
  }
  if(row > lidar_beams) {
    std::cout << "Synthetic Node: Warning: Start row " << row << " greater than the number of lidar beams. No mask will be created." << std::endl;
  }
  row = std::max(0, row);

  int col_end = col + width;
  int row_end = row + height; 
  if (row_end > lidar_beams) {
    std::cout << "Synthetic Node: Warning: End row position (row: " << row << " + " << "height: " << height << " is greater than the number of lidar beams. Clamping to " << lidar_beams << "." << std::endl;
  }
  row_end = std::min(lidar_beams, row_end);
  if(col_end > lidar_horz_resolution) {
    std::cout << "Synthetic Node: End column position (col: " << col << " + " << " width: " << width << ") is greater than point cloud width. Wrapping and applying to columns 0 - " << col_end - lidar_horz_resolution << "." << std::endl;
  }

  int count = 0;
  
  for(int curr_row = row; curr_row < row_end; ++curr_row) {
    for(int curr_col = col; curr_col < col_end; ++curr_col) {
      int wrapped_col = curr_col % lidar_horz_resolution;
      int index = (curr_row * lidar_horz_resolution) + wrapped_col;
      occ_mask.data[index] = 255;
      count++;
    }
  }
  
  std::cout << "Synthetic Node: Count: " << count << std::endl;
}

void generateMask() {
  std::cout << "Synthetic Node: Generating Mask Type: " << mask_type << std::endl;
  // initialize the mask vector with zeros
  if(occ_mask.data.empty()) {
    occ_mask.data.resize(lidar_beams * lidar_horz_resolution, 0);
  }
  if(mask_type == 1) {
    std::cout << "Synthetic Node: Generating Square Mask" << std::endl;
    createSquareMask(sq_row, sq_height, sq_col, sq_width, lidar_beams, lidar_horz_resolution);
  } else {
    // add other mask types
  }
}

int main(int argc, char* argv[]) {
  // Initialize the node
  auto n = avt_341::node::init_node(argc, argv, "lidar_occlusion_synthetic_soil_node");

  // Subscriptions
  auto mask_req_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/mask_request", 1, IncomingMaskRequest);

  // Publishers
  auto occ_mask_pub = n->create_publisher<avt_341::msg::Image>("avt_341/occ_mask", 1);

  // Handle parameters
  //n->get_parameter("~frame", frame);
  n->get_parameter("~lidar_beams", lidar_beams, 64);
  n->get_parameter("~lidar_horz_resolution", lidar_horz_resolution, 1024);
  n->get_parameter("~sq_row", sq_row, 1024);
  n->get_parameter("~sq_height", sq_height, 1024);
  n->get_parameter("~sq_col", sq_col, 1024);
  n->get_parameter("~sq_width", sq_width, 1024);
  n->get_parameter("~timer", timer, 2.0);
  
  avt_341::node::Rate rate(10.0);

  double start_time = n->get_now_seconds();
  double end_time = start_time + timer;
  while(avt_341::node::ok()) {
    if(n->get_now_seconds() > end_time) 
    {
      std::cout << "Synthetic Node: TIME!" << std::endl;
      gen_mask = 1;
      mask_type = 1;
      end_time = n->get_now_seconds() + 10000.0;
    }
    if(gen_mask) {
      std::cout << "Synthetic Node: Generating mask" << std::endl;
      // set up image message
      occ_mask.header.stamp = ros::Time::now();
      occ_mask.header.frame_id = frame;
      occ_mask.height = lidar_beams;
      occ_mask.width = lidar_horz_resolution;
      occ_mask.encoding = "mono8";
      occ_mask.is_bigendian = 0;
      occ_mask.step = occ_mask.width;
      occ_mask.data.resize(occ_mask.step * occ_mask.height, 0);

      generateMask();

      // publish mask
      occ_mask_pub->publish(occ_mask);
      gen_mask = false;

    }
    n->spin_some();
    rate.sleep();
  }
}