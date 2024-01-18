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
#include "tf/transform_datatypes.h"

avt_341::msg::OccupancyGrid in_grid;
bool grid_rcvd = false;
bool have_grid = false;
avt_341::msg::Path local_path;
bool local_path_rcvd = false;
bool have_path = false; 
bool generate_vis_grid = false; 
avt_341::msg::OccupancyGrid accum_grid;


void IncomingOccupancyGridCallback(avt_341::msg::OccupancyGridPtr rcv_vis_grid) {
  in_grid = *rcv_vis_grid;
  grid_rcvd = true;
}

void IncomingPathCallback(avt_341::msg::PathPtr rcv_path){
  local_path.poses = rcv_path->poses;
  local_path.header = rcv_path->header;
  local_path_rcvd = true;
}

// Print a path
void printPath(const nav_msgs::Path& path) {
    std::cout << "Path: " << std::endl;
    for (const auto& pose_stamped : path.poses) {
        const auto& pose = pose_stamped.pose;
        std::cout << std::fixed << std::setprecision(2); // setting precision for better readability
        std::cout << "Position,"
                  << pose.position.x << ", "
                  << pose.position.y << ", "
                  << pose.position.z << ", Orientation,"
                  << pose.orientation.x << ", "
                  << pose.orientation.y << ", "
                  << pose.orientation.z << ", "
                  << pose.orientation.w << std::endl;
    }
}

// Function to convert quaternion to 2D rotation matrix
Eigen::Matrix2f quaternionTo2DMatrix(const avt_341::msg::Quaternion& quat) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(quat, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    Eigen::Matrix2f rot;
    rot << cos(yaw), -sin(yaw),
           sin(yaw), cos(yaw);
    return rot;
}


// Function to apply transformation and update accum_grid
void updateGrid(avt_341::msg::OccupancyGrid& accum_grid, const avt_341::msg::OccupancyGrid& in_grid, const avt_341::msg::Pose& pose, int index) {
    auto rot = quaternionTo2DMatrix(pose.orientation);
    //Eigen::Vector2f translation(pose.position.x/accum_grid.info.resolution, pose.position.y/accum_grid.info.resolution);
    Eigen::Vector2f translation(pose.position.x, pose.position.y);
    std::cout << "Translation, " << pose.position.x << ", " << pose.position.y << std::endl;
    // offset between origins
    Eigen::Vector2f origin_offset((in_grid.info.origin.position.x - accum_grid.info.origin.position.x)/accum_grid.info.resolution,
                                  (in_grid.info.origin.position.y - accum_grid.info.origin.position.y)/accum_grid.info.resolution);
    
    std::cout << "Origin offset: " << origin_offset.x() << ", " << origin_offset.y() << std::endl;
    
    for (unsigned int y = 0; y < in_grid.info.height; ++y) {
        for (unsigned int x = 0; x < in_grid.info.width; ++x) {
            // Get the current value in in_grid
            int value = in_grid.data[y * in_grid.info.width + x];
            if(value == 1) {
              value = 20*index;
              // Calculate transformed position
              Eigen::Vector2f center(in_grid.info.width / 2.0f, in_grid.info.height / 2.0f);
              //Eigen::Vector2f pos_in(x - in_grid.info.origin.position.x, y - in_grid.info.origin.position.y);
              Eigen::Vector2f pos_in(x - center.x(), y - center.y());
              Eigen::Vector2f pos_trans = (rot * pos_in) + center + translation + origin_offset;
              //Eigen::Vector2f pos_trans = rot * pos_in + translation;
              
              // Update accum_grid if within bounds
              if (pos_trans.x() >= 0 && pos_trans.x() < accum_grid.info.width &&
                  pos_trans.y() >= 0 && pos_trans.y() < accum_grid.info.height) {
                  accum_grid.data[static_cast<int>(pos_trans.y()) * accum_grid.info.width + static_cast<int>(pos_trans.x())] = value;
              }
            }
        }
    }

    accum_grid.data[0] = 1;
    accum_grid.data[1] = 50;
    accum_grid.data[2] = 100;
    accum_grid.data[3] = 250;
    accum_grid.data[1 * accum_grid.info.width + 0] = 1;
    accum_grid.data[1 * accum_grid.info.width + 1] = 50;
    accum_grid.data[1 * accum_grid.info.width + 2] = 100;
    accum_grid.data[1 * accum_grid.info.width + 3] = 250;
    
    accum_grid.data[100 * accum_grid.info.width + 100] = 50;
    accum_grid.data[200 * accum_grid.info.width + 200] = 100;
    // DEBUG - accum_grid = in_grid;
}

// Function to calculate the distance between two points
double calculateDistance(const avt_341::msg::Point& p1, const avt_341::msg::Point& p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
}

// Function to calculate orientation from current position to target position
avt_341::msg::Quaternion calculateOrientation(const avt_341::msg::Point& current, const avt_341::msg::Point& target) {
    avt_341::msg::Quaternion orientation;
    double yaw = atan2(target.y - current.y, target.x - current.x);

    // Convert yaw to quaternion (assuming flat ground, so pitch and roll are 0)
    orientation = tf::createQuaternionMsgFromYaw(yaw);
    return orientation;
}

// Function to get the next pose
avt_341::msg::Pose getNextPose(const avt_341::msg::Path& path, double speed, double dt) {
    double distanceToMove = speed * dt; // Total distance to move along the path

    if (path.poses.empty()) {
        ROS_WARN("Path is empty.");
        return avt_341::msg::Pose();
    }

    double accumulatedDistance = 0.0;
    avt_341::msg::Point current_position = path.poses.front().pose.position;

    for (size_t i = 0; i < path.poses.size() - 1; ++i) {
        double segmentDistance = calculateDistance(path.poses[i].pose.position, path.poses[i + 1].pose.position);

        if (accumulatedDistance + segmentDistance >= distanceToMove) {
            double ratio = (distanceToMove - accumulatedDistance) / segmentDistance;
            current_position.x += ratio * (path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x);
            current_position.y += ratio * (path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y);
            current_position.z += ratio * (path.poses[i + 1].pose.position.z - path.poses[i].pose.position.z);

            avt_341::msg::Pose next_pose;
            next_pose.position = current_position;
            if (i + 1 < path.poses.size()) {
                next_pose.orientation = calculateOrientation(path.poses[i].pose.position, path.poses[i + 1].pose.position);
            }
            return next_pose;
        }
        accumulatedDistance += segmentDistance;
    }

    // Return the last pose if the end of the path is reached
    return path.poses.back().pose;
}

int main(int argc, char *argv[]) {
	// Initialize the node
	auto n = avt_341::node::init_node(argc, argv, "proj_visibility_node");
  
	// Subscriptions
  auto mask_grid_sub = n->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/occ_mask_grid", 10, IncomingOccupancyGridCallback); 
  auto path_sub = n->create_subscription<avt_341::msg::Path>("avt_341/local_path",10, IncomingPathCallback);

	// Publishers
  auto vis_grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/proj_vis_grid", 10); 

	// handle parameters
 
  //n->get_parameter("~mask_height", mask_height, 64);
  accum_grid.info.width = 400;
  accum_grid.info.height = 400;
  accum_grid.info.resolution = 0.5;
  accum_grid.header.frame_id = "lidar";
  accum_grid.info.origin.position.x = -100;
  accum_grid.info.origin.position.y = -100;
  accum_grid.data.resize(accum_grid.info.width * accum_grid.info.height, -1);
  
  avt_341::node::Rate rate(10.0);

  while(avt_341::node::ok()) {

    // Do we have new data?
    if(grid_rcvd) {
      std::cout << "Proj Vis Node: New visibility mask grid received. Frame: " << in_grid.header.frame_id << " Origin: " << in_grid.info.origin.position.x << ", " << in_grid.info.origin.position.y << " Size: " <<  in_grid.info.width << ", " << in_grid.info.height << " Res: " << in_grid.info.resolution << std::endl;
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
      // set parameters
      double speed = 4.0;
      double dt = 0.1;
      double elapsed_time = 0.0;
      bool reached_end = false;

      // get the local_path start position
      avt_341::msg::Pose startPose = local_path.poses[0].pose;
      printPath(local_path);

      /*
      while(!reached_end) {
        avt_341::msg::Pose nextPose = getNextPose(local_path, speed, elapsed_time);
        // Check if the next pose is the last pose in the path
        if (nextPose.position.x == local_path.poses.back().pose.position.x &&
          nextPose.position.y == local_path.poses.back().pose.position.y &&
          nextPose.position.z == local_path.poses.back().pose.position.z) {
          reached_end = true;
        } else {
          std::cout << "ProjVisNode, Position," << currentPose.position.x << "," << currentPose.position.y << "," << currentPose.position.z;
          std::cout << ", Time, " << elapsed_time << std::endl;
          
          elapsed_time += dt; // Increment elapsed time
          currentPose = nextPose; // Update current pose

          // merge in_grid nto the accum_grid at the position and orientation determined
          //updateGrid(accum_grid, in_grid, currentPose);
        }
      }*/
      
      for(size_t i = 0; i < local_path.poses.size() - 1; i++)
      {
        const auto& current_pose = local_path.poses[i].pose;
        const auto& next_pose = local_path.poses[i+1].pose;
        avt_341::msg::Pose grid_pose;
        grid_pose.position.x = current_pose.position.x - startPose.position.x;   
        grid_pose.position.y = current_pose.position.y - startPose.position.y;   
        grid_pose.position.z = current_pose.position.z - startPose.position.z;   
        grid_pose.orientation = calculateOrientation(current_pose.position, next_pose.position);
        std::cout << "ProjVisNode, Position," << grid_pose.position.x << "," << grid_pose.position.y << "," << grid_pose.position.z;
        std::cout << "," << grid_pose.orientation.x << ", "
                  << grid_pose.orientation.y << ", "
                  << grid_pose.orientation.z << ", "
                  << grid_pose.orientation.w << std::endl;
        updateGrid(accum_grid, in_grid, grid_pose, i); 
      }
    
      // publish the generated grid
      accum_grid.header.stamp = n->get_stamp();
           
      std::cout << "Proj Vis Node: Publishing the grid" << std::endl;
      vis_grid_pub->publish(accum_grid);
      generate_vis_grid = false;
    }
    n->spin_some();
    rate.sleep();
  }
  return 0;
}

