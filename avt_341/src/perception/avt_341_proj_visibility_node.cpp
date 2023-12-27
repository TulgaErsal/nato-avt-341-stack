/* avt_341_proj_visibility_node.cpp

 * ROS Node for projecting visibility grid along the local path

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

avt_341::msg::OccupancyGrid in_grid;
bool grid_rcvd = false;
bool have_grid = false;
avt_341::msg::Path local_path;
bool local_path_rcvd = false;
bool have_path = false; 
bool generate_vis_grid = false; 

void IncomingOccupancyGridCallback(avt_341::msg::OccupancyGridPtr rcv_vis_grid) {
  in_grid = *rcv_vis_grid;
  grid_rcvd = true;
}

void IncomingPathCallback(avt_341::msg::PathPtr rcv_path){
  local_path.poses = rcv_path->poses;
  local_path.header = rcv_path->header;
  local_path_rcvd = true;
}

int main(int argc, char *argv[]) {
	// Initialize the node
	auto n = avt_341::node::init_node(argc, argv, "proj_visibility_node");
  
	// Subscriptions
  auto mask_grid_sub = n->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/occ_mask_grid", 1, IncomingOccupancyGridCallback); 
  auto path_sub = n->create_subscription<avt_341::msg::Path>("avt_341/local_path",1, IncomingPathCallback);

	// Publishers
  auto vis_grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/proj_vis_grid", 1); 

	// handle parameters
 
  //n->get_parameter("~mask_height", mask_height, 64);
  
  avt_341::node::Rate rate(10.0);

  while(avt_341::node::ok()) {
    if(grid_rcvd) {
      std::cout << "Proj Vis Node: New visibility mask grid received." << std::endl;
      grid_rcvd = false;
      have_grid = true;
      if(have_path) {
        generate_vis_grid = true;
      }
    } 
    if(local_path_rcvd) {
      std::cout << "Proj Vis Node: New local path received." << std::endl;
      local_path_rcvd = false;
      have_path = true;
      if(have_grid) {
        generate_vis_grid = true;
      }
    }

    // Generate a new projected visibility grid
    if(generate_vis_grid) {
      // Extract the poses
      for(size_t i = 0; i < local_path.poses.size(); ++i) {
        const avt_341::msg::Pose& currentPose = local_path.poses[i].pose;

        if(i + 1 < local_path.poses.size()) {
          const avt_341::msg::Pose& nextPose = local_path.poses[i + 1].pose;
          double dx = nextPose.position.x - currentPose.position.x;
          double dy = nextPose.position.y - currentPose.position.y;
          double distance = sqrt(dx * dx + dy * dy);
          double angle = atan2(dy, dx);

          std::cout << "Proj Vis Node: Current Position: " << currentPose.position.x << "," << currentPose.position.y << "," << currentPose.position.z;
          std::cout << " Next Position: " << nextPose.position.x << "," << nextPose.position.y << "," << nextPose.position.z; 
          std::cout << " DX: " << dx << " DY: " << dy << " Distance: " << distance << " Angle: " << angle << std::endl;

          // transform mask (move to pose and rotate)

          // marge into an accumulated grid
    
        }
      }

      // Generate orientation data - assume 

      /*mask_grid.header.stamp = n->get_stamp();
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
      }*/

      //vis_grid_pub->publish(mask_grid);
      generate_vis_grid = false;
    }
    n->spin_some();
    rate.sleep();
  }
  return 0;
}

