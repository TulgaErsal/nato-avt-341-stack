/**
 * \file avt_341_planner_node.cpp
 * Plan a local trajectory using a global path.
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 8/31/2020
 */
#include <algorithm>
#include <math.h>
// ROS includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/node/occupancy_grid_subscriber.h"
// avt_341 includes
#include "avt_341/planning/local/spline_planner.h"
#include "avt_341/planning/local/rviz_spline_plotter.h"
#include <avt_341/rcc_local_planner_params_service.hpp>

avt_341::msg::Odometry odom;
avt_341::msg::OccupancyGrid grid;
avt_341::msg::OccupancyGrid segmentation_grid;
avt_341::msg::Path global_path;
avt_341::msg::Path waypoints;
double speedometer = 0.0;
bool odom_rcvd = false;
bool new_grid_rcvd = false;
bool new_seg_grid_rcvd = false;
bool speedometer_rcvd = false;


void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
  odom = *rcv_odom;
  odom_rcvd = true;
}

void GridCallback(avt_341::msg::OccupancyGridPtr rcv_grid){
  grid = *rcv_grid;
  new_grid_rcvd = true;
}

void SegmentationGridCallback(avt_341::msg::OccupancyGridPtr rcv_grid){
    segmentation_grid = *rcv_grid;
    new_seg_grid_rcvd = true;
}

void PathCallback(avt_341::msg::PathPtr rcv_path){
  global_path = *rcv_path;
}

void WaypointCallback(avt_341::msg::PathPtr wp_path){
  waypoints = *wp_path;
}

void SpeedCallback(avt_341::msg::Float64Ptr rcv_speed) {
	speedometer = rcv_speed->data;
  speedometer_rcvd = true; 
}

int main(int argc, char *argv[]){

  auto n = avt_341::node::init_node(argc, argv, "avt_341_planner_node");
  avt_341::params::rcc_local_planner::ParamsListener param_listener(n->get_raw_node());
  const auto params = param_listener.get_params();

  avt_341::planning::Planner planner;
  float path_look_ahead = static_cast<float>(params.path_look_ahead);
  float steer_angle_limit = static_cast<float>(params.steer_angle_limit);
  const int dilation_factor = static_cast<int>(params.dilation_factor);

    // Create publishers and subscribers
  auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/local_path", 10);
  auto odometry_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
  avt_341::node::OccupancyGridSubscriber grid_sub(
      n, params.map_topic, 10, params.costmap.publish.method, GridCallback);
  avt_341::node::OccupancyGridSubscriber segmentation_grid_sub(
      n, "avt_341/segmentation_grid", 10, params.costmap.publish.method,
      SegmentationGridCallback);
  auto path_sub = n->create_subscription<avt_341::msg::Path>("avt_341/global_path", 10, PathCallback);
  auto wp_sub = n->create_subscription<avt_341::msg::Path>("avt_341/waypoints", 10, WaypointCallback);
  auto speed_sub = n->create_subscription<avt_341::msg::Float64>("avt_341/forward_speed",10,SpeedCallback);

  planner.SetArcLengthIntegrationStep(
      static_cast<float>(params.path_integration_step));
  planner.SetComfortabilityWeight(static_cast<float>(params.w_c));
  planner.SetDynamicSafetyWeight(static_cast<float>(params.w_d));
  planner.SetStaticSafetyWeight(static_cast<float>(params.w_s));
  planner.SetPathAdherenceWeight(static_cast<float>(params.w_r));
  planner.SetSegmentationFactorWeight(static_cast<float>(params.w_t));
  planner.SetUseBlend(params.use_blend);
  planner.SetIgnoreCollBeforeDist(
      static_cast<float>(params.ignore_coll_before_dist));

  auto plotter = std::make_shared<avt_341::planning::RVIZPlotter>(params.cost_vis, n,
                                                                  planner.GetComfortabilityWeight(), planner.GetStaticSafetyWeight(),
                                                                  planner.GetPathAdherenceWeight(), planner.GetDynamicSafetyWeight(),
                                                                  planner.GetSegmentationWeight(), static_cast<float>(params.cost_vis_text_size));

  unsigned int loop_count = 0;
  const double dt = 1.0 / params.rate;
  float elapsed_time = 0.0f;
  avt_341::node::Rate rosrate(params.rate);
  while (avt_341::node::ok()){
    double start_secs = n->get_now_seconds();
    if (global_path.poses.size() > 0 && odom_rcvd && grid.data.size() > 0){
      //std::cout << ros::this_node::getName() << " Running Local planner " << global_path.poses.size() << std::endl;
      std::vector<avt_341::utils::vec2> path_points;
      if (params.use_global_path){
        for (int i = 0; i < global_path.poses.size(); i++){
          avt_341::utils::vec2 point(global_path.poses[i].pose.position.x, global_path.poses[i].pose.position.y);
          path_points.push_back(point);
        }
      }
      else{
        for (int i = 0; i < waypoints.poses.size(); i++){
          avt_341::utils::vec2 point(waypoints.poses[i].pose.position.x, waypoints.poses[i].pose.position.y);
          path_points.push_back(point);
        }
        
      }
      avt_341::planning::Path path;
      if (params.use_dynamic_window && speedometer_rcvd) {
        // Calulate path look ahead
        path_look_ahead =
            static_cast<float>(params.time_look_ahead * speedometer);
        path_look_ahead = std::min(
            path_look_ahead,
            static_cast<float>(params.max_path_look_ahead));
        path_look_ahead = std::max(
            path_look_ahead,
            static_cast<float>(params.min_path_look_ahead));

        // Calculate max steering angle
        steer_angle_limit = std::atan(
            params.vehicle_wheelbase * params.max_lateral_accel /
            (speedometer * speedometer));
        steer_angle_limit = std::min(
            steer_angle_limit,
            static_cast<float>(params.max_steer_angle_limit));
        steer_angle_limit = std::max(
            steer_angle_limit,
            static_cast<float>(params.min_steer_angle_limit));
      }
      if (params.trim_path && params.use_global_path ){
        avt_341::utils::vec2 current_pos(odom.pose.pose.position.x, odom.pose.pose.position.y);
        path.Init(path_points, current_pos, 1.5f * path_look_ahead);
      }
      else{
        path.Init(path_points);
      }

      path.FixBeginning(odom.pose.pose.position.x, odom.pose.pose.position.y);

      std::vector<avt_341::utils::vec2> culled_points = path.GetPoints();
      float s_max = path.GetTotalLength();
      avt_341::utils::vec2 srho = path.ToSRho(odom.pose.pose.position.x, odom.pose.pose.position.y);
      float s = srho.x;
      float rho_start = srho.y;
      avt_341::utils::vec2 pconv = path.ToCartesian(s,rho_start);
      float s_lookahead = std::min(path_look_ahead, s_max - s);
      float theta = avt_341::utils::GetHeadingFromOrientation(odom.pose.pose.orientation);
      avt_341::planning::CurveInfo ci = path.GetCurvatureAndAngle(s);

      // Fix to bug in curvature when heading west
      float d_theta = theta - ci.theta;
      d_theta += (d_theta>M_PI) ? -2.0*M_PI : (d_theta<-M_PI) ? 2.0*M_PI : 0.0;

      // Fix bug with paths not converging when d_theta ~= 90 degrees
      d_theta = std::max(
          std::min(d_theta, static_cast<float>(params.max_theta)),
          -static_cast<float>(params.max_theta));
      
      planner.GeneratePaths(
          static_cast<int>(params.num_paths), s, rho_start, d_theta,
          s_lookahead, steer_angle_limit,
          static_cast<float>(params.vehicle_width));
      planner.SetCenterline(path);

      // TODO: Issue #145 Add a better way to display this data instead of constant log (ex: rviz display or maybe just ros2 topic echo)
//      std::cout << "data: " << d_theta << "," << theta << "," << ci.theta << std::endl;
  
      // calculate bounds around the vehicle to limit grid dilation to space 10m behind and path_look_ahead distance in front of the vehicle
      float veh_heading_x = cos(theta);
      float veh_heading_y = sin(theta);
      float veh_left_offset_x = odom.pose.pose.position.x + (-veh_heading_y * (path_look_ahead/2));
      float veh_left_offset_y = odom.pose.pose.position.y + (veh_heading_x * (path_look_ahead/2));
      float veh_right_offset_x = odom.pose.pose.position.x + (-veh_heading_y * -(path_look_ahead/2));
      float veh_right_offset_y = odom.pose.pose.position.y + (veh_heading_x * -(path_look_ahead/2));
      float lf_bounds_x = veh_left_offset_x + (veh_heading_x * path_look_ahead);
      float lf_bounds_y = veh_left_offset_y + (veh_heading_y * path_look_ahead);
      float rf_bounds_x = veh_right_offset_x + (veh_heading_x * path_look_ahead);
      float rf_bounds_y = veh_right_offset_y + (veh_heading_y * path_look_ahead);
      float lr_bounds_x = veh_left_offset_x + (veh_heading_x * -10);
      float lr_bounds_y = veh_left_offset_y + (veh_heading_y * -10);
      float rr_bounds_x = veh_right_offset_x + (veh_heading_x * -10);
      float rr_bounds_y = veh_right_offset_y + (veh_heading_y * -10);
      float llx = std::min({lf_bounds_x, rf_bounds_x, lr_bounds_x, rr_bounds_x});
      float lly = std::min({lf_bounds_y, rf_bounds_y, lr_bounds_y, rr_bounds_y});
      float urx = std::max({lf_bounds_x, rf_bounds_x, lr_bounds_x, rr_bounds_x});
      float ury = std::max({lf_bounds_y, rf_bounds_y, lr_bounds_y, rr_bounds_y});

      if (new_grid_rcvd) planner.DilateGrid(grid, dilation_factor, llx, lly, urx, ury);
      if (new_seg_grid_rcvd) planner.DilateGrid(segmentation_grid, dilation_factor, llx, lly, urx, ury);
      // Note: if grid size gets large, DilateGrid can take a significant amount of time

      // most of the calculation time spent on this function call
      bool path_found = planner.CalculateCandidateCosts(grid, segmentation_grid, odom);
      plotter->AddMap(grid);
      plotter->SetPath(culled_points);
      std::vector<avt_341::planning::Candidate> paths = planner.GetCandidates();
      plotter->AddCurves(paths);
      plotter->Display();

      if (path_found){
        const float ds = static_cast<float>(params.output_path_step);
        avt_341::msg::Path local_path;
        avt_341::planning::Candidate best = planner.GetBestPath();
        float s0 = best.GetS0() + ds;
        float s_max = s0 + best.GetMaxLength() - ds;
        while (s0 < s_max){
          float rho0 = best.At(s0 - best.GetS0());
          avt_341::utils::vec2 point = path.ToCartesian(s0, rho0);
          avt_341::msg::PoseStamped pose;
          pose.pose.position.x = point.x;
          pose.pose.position.y = point.y;
          local_path.poses.push_back(pose);
          s0 += params.output_path_step;
        }
        //local_path.header.frame_id = "odom";
        local_path.header.frame_id = "map";
        local_path.header.stamp = n->get_stamp();
        avt_341::node::set_seq(local_path.header, loop_count);
        path_pub->publish(local_path);
      }
      else {
        avt_341::msg::Path local_path;
        avt_341::msg::PoseStamped pose;
        pose.pose = odom.pose.pose;
        local_path.poses.push_back(pose);
        //local_path.header.frame_id = "odom";
        local_path.header.frame_id = "map";
        local_path.header.stamp = n->get_stamp();
        avt_341::node::set_seq(local_path.header, loop_count);
        path_pub->publish(local_path);
      }
      odom_rcvd = false;
    }
    else {
      if (global_path.poses.size() <= 0){
        //std::cout << ros::this_node::getName() << " Local planner did not run because global path not recieved " << std::endl;
      }
      else if (!odom_rcvd){
        //std::cout << ros::this_node::getName() << " Local planner did not run because vehicle odometry not recieved." << std::endl;
      }
      else if (grid.data.size() <= 0){
        //std::cout << ros::this_node::getName() << " Local planner did not run because occupancy grid not recieved." << std::endl;
      }
    }
    new_grid_rcvd = false;
    new_seg_grid_rcvd = false;
    loop_count++;
    double end_secs = n->get_now_seconds();
    if ((end_secs - start_secs) > 2.5 * dt){
      //std::cout << "WARNING: TANG PLANNER TOOK " << (end_secs - start_secs) << " TO COMPLETE. REQUESTED UPDATE SPEED IS " << dt << std::endl;
    }

    n->spin_some();
    rosrate.sleep();
  }

  return 0;
}
