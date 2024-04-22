/* avt_341_lidar_occlusion_synthetic_node.cpp

 * ROS Node for generating synthetic occlusions of lidar sensor data
 * Subscribers: points (PointCloud2), occ_mask (Image)
 * Publishers:  occ_points (PointCloud2)

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

avt_341::msg::Image occ_mask;
bool occ_mask_rcvd = false;
avt_341::msg::PointCloud2 in_points;
bool points_rcvd = false;

void IncomingPointCloud2Callback(avt_341::msg::PointCloud2Ptr rcv_points) {
  in_points = *rcv_points;
  points_rcvd = true;
}

void IncomingOcclusionMaskCallback(avt_341::msg::ImagePtr rcv_image) {
  occ_mask = *rcv_image;
  occ_mask_rcvd = true;
}

void applyMaskToPointCloud(sensor_msgs::PointCloud2 &cloud) {
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
  
  //std::cout << "Point Cloud Height: " << cloud.height << " Width: " << cloud.width << " Size: " << cloud.height * cloud.width << std::endl;
  //std::cout << "Mask Size: " << mask_vector.size() << std::endl;

  int row = 0;
  int col = 0;
  int width = occ_mask.width;
  int height = occ_mask.height;
  int mask_index = 0;
  int point_index = 0;
  // mask is width x height
  // lidar data is height x width
  for(int row = 0; row < height; ++row) {
    for(int col = 0; col < width; ++col) {
      mask_index = (row * width) + col;
      point_index = (col * height) + row;
      if(occ_mask.data[mask_index] == 255) {
        *(iter_x + point_index) = 0.0f;
        *(iter_y + point_index) = 0.0f;
        *(iter_z + point_index) = 0.0f;
        *(iter_intensity + point_index) = 0.0f;
      }
    }
  }
  /*
  for (size_t i = 0; i < occ_mask.data.size(); ++i) {
    //if (occ_mask.data[i] == 255) {
      //std::cout << "Masked point coordinates (before): ("  << *(iter_x + i) << ", " << *(iter_y + i) << ", " << *(iter_z + i) << ")" << std::endl;
      // Setting x, y, z to 0 to represent occlusion.
      *(iter_x + i) = 0.0f;
      *(iter_y + i) = 0.0f;
      *(iter_z + i) = 0.0f;
      *(iter_intensity + i) = 0.0f;
      //std::cout << "Masked point coordinates (after): (" << *(iter_x + i) << ", " << *(iter_y + i) << ", " << *(iter_z + i) << ")" << std::endl;            
    } 
  }*/
/*
  int span = 2;
  int index = row * width + col; 
  index = col * height + row;
  for(int i = index; i < index + span; ++i) {
    *(iter_intensity + i) = 255.0f;
  }*/
}

// Main Node Function
int main(int argc, char*argv[]) {
  // Initialize the node
  auto n = avt_341::node::init_node(argc, argv, "lidar_occlusion_synthetic_node");

  // Subscriptions
  auto points_sub = n->create_subscription<avt_341::msg::PointCloud2>("avt_341/points", 10, IncomingPointCloud2Callback);
  auto occ_mask_sub = n->create_subscription<avt_341::msg::Image>("avt_341/occ_mask", 10, IncomingOcclusionMaskCallback);
  //auto activate_mask_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/activate_mask", 1, ActivateMaskCallback);

  // Publishers: gt_occ_mask (Image), gt_occ_detected (int32), occ_points (PointCloud2)
  auto occ_points_pub  = n->create_publisher<avt_341::msg::PointCloud2>("avt_341/occ_points", 10);
  //auto gt_occ_mask_pub = n->create_publisher<avt_341::msg::Image>("avt_341/gt_occ_mask", 1);
  //auto gt_occ_detected = n->create_publisher<avt_341::msg::Int32>("avt_341/gt_occ_detected", 1);

  // parameters and default values
  bool occluded = false;     // is the mask applied to the incoming points to generate occluded points? 
   
  avt_341::msg::PointCloud2 out_points;
  avt_341::node::Rate rate(10.0);

  // Enter the loop
  while(avt_341::node::ok()) {
    if(occ_mask_rcvd) {
      // start occluding the points, set rcvd false
      occluded = true;
      occ_mask_rcvd = false;
    }
    if(points_rcvd) {
      if(occluded) {
        std::cout << "Lidar Occlusion Synthetic Node: New point cloud received. Applying occlusion." << std::endl;
        // process the incoming points
        out_points = in_points;
        applyMaskToPointCloud(out_points);
        occ_points_pub->publish(out_points);
      }
    }
    n->spin_some();
    rate.sleep();
  }
  return 0;
}