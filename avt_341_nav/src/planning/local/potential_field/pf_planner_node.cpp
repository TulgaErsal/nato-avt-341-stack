/**
 * \file avt_341_pf_planner_node.cpp
 * Plan a local trajectory using the potential field planner
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 1/19/2022
 */
// ROS includes
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <rclcpp/rclcpp.hpp>
#include <avt_341_nav/node/occupancy_grid_subscriber.h>
// avt_341 includes
#include "avt_341_nav/planning/local/pf_planner.h"
#include <avt_341_nav/pf_local_planner_params_service.hpp>

nav_msgs::msg::Odometry odom;
nav_msgs::msg::OccupancyGrid grid;
nav_msgs::msg::OccupancyGrid segmentation_grid;
nav_msgs::msg::Path global_path;
nav_msgs::msg::Path waypoints;
bool odom_rcvd = false;
bool new_grid_rcvd = false;
bool new_seg_grid_rcvd = false;

void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr rcv_odom){
  odom = *rcv_odom;
  odom_rcvd = true;
}

void GridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr rcv_grid){
  grid = *rcv_grid;
  new_grid_rcvd = true;
}

void SegmentationGridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr rcv_grid){
    segmentation_grid = *rcv_grid;
    new_seg_grid_rcvd = true;
}

void PathCallback(nav_msgs::msg::Path::SharedPtr rcv_path){
  global_path = *rcv_path;
}

void WaypointCallback(nav_msgs::msg::Path::SharedPtr wp_path){
  waypoints = *wp_path;
}

int main(int argc, char *argv[]){

  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("avt_341_pf_planner_node");
  avt_341_nav::params::pf_local_planner::ParamsListener param_listener(n);
  const auto params = param_listener.get_params();

  // Create publishers and subscribers
  auto path_pub = n->create_publisher<nav_msgs::msg::Path>("avt_341/local_path", 10);
  auto odometry_sub = n->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
  auto grid_sub = avt_341_nav::node::OccupancyGridSubscriber(
      n, params.grid_topic, 10, params.costmap.publish.method, GridCallback);
  auto segmentation_grid_sub = avt_341_nav::node::OccupancyGridSubscriber(
      n, "avt_341/segmentation_grid", 10, params.costmap.publish.method,
      SegmentationGridCallback);
  auto path_sub = n->create_subscription<nav_msgs::msg::Path>("avt_341/global_path", 10, PathCallback);
  auto wp_sub = n->create_subscription<nav_msgs::msg::Path>("avt_341/waypoints", 10, WaypointCallback);

  avt_341_nav::planning::PfPlanner planner;
  planner.SetEta(static_cast<float>(params.eta));
  planner.SetKp(static_cast<float>(params.kp));
  planner.SetCutoffDistance(static_cast<float>(params.cutoff_dist));
  planner.SetInnerCutoff(static_cast<float>(params.inner_cutoff_dist));
  planner.SetObstacleCostThreshold(
      static_cast<int>(params.obstacle_cost_thresh));
  planner.SetMotionModelRes(static_cast<float>(params.motion_model_res));

  unsigned int loop_count = 0;
  const float dt = 1.0f / static_cast<float>(params.rate);
  float elapsed_time = 0.0f;
  rclcpp::Rate rosrate(params.rate);
  while (rclcpp::ok()){
    double start_secs = n->now().seconds();
    if (global_path.poses.size() > 0 && odom_rcvd && grid.data.size() > 0){

      float gx, gy;
      if (params.use_global_path){
        gx = global_path.poses.back().pose.position.x;
        gy = global_path.poses.back().pose.position.y;
      }
      else{
        gx = waypoints.poses.back().pose.position.x;
        gy = waypoints.poses.back().pose.position.y;
      }

      planner.SetGoal(gx, gy);

      if (new_seg_grid_rcvd) planner.SetSegGrid(segmentation_grid);
      nav_msgs::msg::Path local_path = planner.Plan(grid, odom);

      local_path.header.frame_id = "map";
      local_path.header.stamp = n->now();
      path_pub->publish(local_path);
      /*}
      else {
        nav_msgs::msg::Path local_path;
        geometry_msgs::msg::PoseStamped pose;
        pose.pose = odom.pose.pose;
        local_path.poses.push_back(pose);
        local_path.header.frame_id = "map";
        local_path.header.stamp = n->now();
        path_pub->publish(local_path);
      }*/

      odom_rcvd = false;
    }
    else {
      if (global_path.poses.size() <= 0){
        //std::cout << "Local planner did not run because global path not recieved " << std::endl;
      }
      else if (!odom_rcvd){
        //std::cout << "Local planner did not run because vehicle odometry not recieved." << std::endl;
      }
      else if (grid.data.size() <= 0){
        //std::cout << "Local planner did not run because occupancy grid not recieved." << std::endl;
      }
    }
    new_grid_rcvd = false;
    new_seg_grid_rcvd = false;
    loop_count++;
    double end_secs = n->now().seconds();
    if ((end_secs - start_secs) > 2.5 * dt){
      std::cout << "WARNING: POTENTIAL FIELD PLANNER TOOK " << (end_secs - start_secs) << " TO COMPLETE. REQUESTED UPDATE SPEED IS " << dt << std::endl;
    }

    rclcpp::spin_some(n);
    rosrate.sleep();
  }

  return 0;
}
