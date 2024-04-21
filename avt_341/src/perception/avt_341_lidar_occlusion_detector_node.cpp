/* avt_341_lidar_occlusion_detector_node.cpp

 * ROS Node for detecting occlusions of lidar sensor data
 * Subscribers: points (PointCloud2)
 * Publishers:  _occ_detected (int32)

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

avt_341::msg::PointCloud2 in_points; 
avt_341::msg::Image occ_mask;
avt_341::msg::OccupancyGrid occ_grid;

// Callbacks
void IncomingPointCloud2Callback(avt_341::msg::PointCloud2Ptr rcv_points) {
  in_points = *rcv_points;
  points_rcvd = true;
}

// utility functions
bool CheckForOcclusions(avt_341::msg::PointCloud2 points) {
  return true;
}

void SegmentOcclusions(avt_341::msg::PointCloud2 points) {
  
}

void ConvertMaskToGrid(avt_341::msg::Image mask) {

}

int main(int argc, char *argv[]) {
  // Initialize the node
  auto n = avt_341::node::init_node(argc, argv, "lidar_occlusion_detection_node");

  // Subscriptions
  auto points_sub = n->create_subscription<avt_341::msg::PointCloud2>("avt_341/points", 10, IncomingPointCloud2Callback);

  // Publishers
  auto detection_pub = n->create_publisher<avt_341::msg::Int32>("avt_341/occ_detected", 10);
  auto mask_pub = n->create_publisher<avt_341::msg::Image>("avt_341/occ_mask", 10);
  auto grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occ_grid", 10);
  
  // declare parameters

  // read parameters

  avt_341::msg::Int32 occ_detected;
  avt_341::node::Rate rate(10.0);

  // Enter the loop
  while(avt_341::node::ok()) {
    if(!points_rcvd) {
      std::cout << "Occlusion Detection Node: no point cloud received." << std::endl;
    } else {
      // process the points
      bool detection = CheckForOcclusions(in_points); 
      occ_detected.header.stamp = n->get_stamp();
      occ_detected.data = detection;
      if(detection) {
        std::cout << "Occlusion Detection Node: occlusion detected." << std::endl;
        SegmentOcclusions(in_points);
        ConvertMaskToGrid(occ_mask);
      } else {
        std::cout << "Occlusion Detection Node: NO occlusion." << std::endl;
      }
      detection_pub->publish(occ_detected); 
    }
    n->spin_some();
    rate.sleep();
  }
  return 0;
}