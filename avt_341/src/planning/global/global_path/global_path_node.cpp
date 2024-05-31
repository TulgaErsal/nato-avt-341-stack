/**
 * \file avt_341_global_path_node.cpp
 * 
 * ROS Node that publishes a user defined path as a global path.
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 9/1/2020
 */

// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/planning/global/astar.h"
#include "avt_341/visualization/visualization_factory.h"
avt_341::msg::Odometry odom;
bool odom_rcvd = false;
avt_341::msg::OccupancyGrid current_grid;
avt_341::msg::OccupancyGrid segmentation_grid;
avt_341::msg::Path current_waypoints;
bool waypoints_rcvd = false;
bool use_global_planner = true;
int nav_command = 0;
bool nav_command_rcvd = false;
bool verbose_gp_log = false;
bool shutdown_condition = false;

std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom)
{
  odom = *rcv_odom;
  odom_rcvd = true;
}

void MapCallback(avt_341::msg::OccupancyGridPtr rcv_grid)
{
  current_grid = *rcv_grid;
}

void SegmentationMapCallback(avt_341::msg::OccupancyGridPtr rcv_grid){
    segmentation_grid = *rcv_grid;
}

// From mission planner
void WaypointCallback(avt_341::msg::PathPtr rcv_waypoints)
{
  // Brute force - overwrite the current global waypoints
  current_waypoints = *rcv_waypoints;
  waypoints_rcvd = true;
  if(verbose_gp_log){
    n->log_info("%d waypoint(s) received! %.2f, %.2f", current_waypoints.poses.size(), current_waypoints.poses[0].pose.position.x, current_waypoints.poses[0].pose.position.y);
  }
}

void GlobalPlannerToggleCallback(avt_341::msg::Int32Ptr rcv_gptoggle) {
  bool set_val = (bool)rcv_gptoggle->data;
  if(use_global_planner != set_val){
    n->log_info("GP set to %d", rcv_gptoggle->data);
  }
  use_global_planner = set_val;
}

void NavCommandCallback(avt_341::msg::Int32Ptr rcv_navcommand) {
  nav_command = rcv_navcommand->data;
  nav_command_rcvd = true;
}

void GoalPoseCallback(avt_341::msg::PoseStampedPtr rcv_goal_pose)
{
  if(verbose_gp_log){
    n->log_info("Setting goal (%.2f, %.2f)", rcv_goal_pose->pose.position.x, rcv_goal_pose->pose.position.y);
  }
  current_waypoints.poses.clear();
  current_waypoints.poses.push_back(*rcv_goal_pose);
  waypoints_rcvd = true;
}

avt_341::msg::Path ToROSPath(const std::vector<std::vector<float>> & path){
  avt_341::msg::Path ros_path;
  ros_path.header.frame_id = "map";
  for (const auto & p: path){
    avt_341::msg::PoseStamped pose;
    pose.pose.position.x = p[0];
    pose.pose.position.y = p[1];
    pose.pose.position.z = 0.0f;
    pose.pose.orientation.w = 1.0f;
    pose.pose.orientation.x = 0.0f;
    pose.pose.orientation.y = 0.0f;
    pose.pose.orientation.z = 0.0f;
    ros_path.poses.push_back(pose);
  }
  return ros_path;
}

avt_341::msg::Int32 state;
int current_waypoint = 0;

void Reset(){
  n->log_info("Resetting node");
  state.data = avt_341::utils::NavStackState::NotInit; // start up state
  current_waypoint = 0;
  odom_rcvd = false;
  shutdown_condition = false;
  current_waypoints.poses.clear();
}

bool reset_called = false;
void ResetCallback(avt_341::msg::StringPtr msg){
  if(msg->data.find(avt_341::node::NodeType::GlobalPlanner) != std::string::npos){
    reset_called = true;
  }
}

int main(int argc, char *argv[])
{
  n = avt_341::node::init_node(argc, argv, "avt_341_global_path_node");

  float goal_dist, global_lookahead,  w_distance, w_occupancy, w_segmentation;
  double local_origin_x, local_origin_y;
  std::vector<double> waypoints_x_list, waypoints_y_list;
  std::string display_type;
  bool debug_visualize, search_diagonals, los_break_on_first, auto_active_on_new_waypoint, use_global_path;
  int los_max_iterations;
  float dilation_factor, max_sep;
  std::string map_topic;

  std::vector<float> goal;
  goal.resize(2, 0.0f);

  n->get_parameter("~goal_dist", goal_dist, 3.0f);
  n->get_parameter("~display", display_type, avt_341::visualization::default_display);
  n->get_parameter("~global_lookahead", global_lookahead, 50.0f);
  n->get_parameter("/waypoints_x", waypoints_x_list, std::vector<double>(0));
  n->get_parameter("/waypoints_y", waypoints_y_list, std::vector<double>(0));
  n->get_parameter("/map_origin_x", local_origin_x, 0.0);
  n->get_parameter("/map_origin_y", local_origin_y, 0.0);
  n->get_parameter("~debug_visualize", debug_visualize, false);
  n->get_parameter("~search_diagonals", search_diagonals, false);
  n->get_parameter("~los_max_iterations", los_max_iterations, 1);
  n->get_parameter("~los_break_on_first", los_break_on_first, true);
  n->get_parameter("~w_distance", w_distance, 1.0f);
  n->get_parameter("~w_occupancy", w_occupancy, 1.0f);
  n->get_parameter("~w_segmentation", w_segmentation, 1.0f);
  n->get_parameter("~auto_active_on_new_waypoint", auto_active_on_new_waypoint, false);
  n->get_parameter("~verbose_gp_log", verbose_gp_log, true);
  n->get_parameter("~dilation_factor", dilation_factor, 0.0f);
  n->get_parameter("~max_separation", max_sep, 1.0f);
  n->get_parameter("~use_global_path", use_global_path, true);
  n->get_parameter("~map_topic", map_topic, std::string("avt_341/occupancy_grid"));

  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> global_path_pre_smooth_pub = nullptr;
  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> global_path_pre_fill_pub = nullptr;
  if(debug_visualize){
    global_path_pre_smooth_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path_pre_smooth", 10);
    global_path_pre_fill_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path_pre_fill", 10);
  }

  int shutdown_behavior = avt_341::utils::NavStackState::Stopped;
  n->get_parameter("~shutdown_behavior", shutdown_behavior, shutdown_behavior);
  if (shutdown_behavior>3 || shutdown_behavior<1)shutdown_behavior = 1;

  if (waypoints_x_list.size() != waypoints_y_list.size())
  {
    std::cerr << "WARNING: " << waypoints_x_list.size() << " X COORDINATES WERE PROVIDED FOR " << waypoints_y_list.size() << " Y COORDINATES." << std::endl;
  }
  if (waypoints_x_list.size() == 0 || waypoints_y_list.size() == 0)
  {
    std::cerr << "WARNING: NO WAYPOINTS WERE LISTED IN /waypoints_x OR /waypoints_y." << std::endl;
    //return 2;
  }

  auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path", 10);
  auto waypoint_pub = n->create_publisher<avt_341::msg::Path>("avt_341/waypoints", 10);
  auto current_waypoint_pub = n->create_publisher<avt_341::msg::PoseStamped>("avt_341/current_waypoint", 10);
  auto dist_to_current_waypoint_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/distance_to_current_waypoint", 10);
  auto goal_reached_pub = n->create_publisher<avt_341::msg::PoseStamped>("avt_341/goal_reached", 10);

  auto odometry_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
  auto map_sub = n->create_subscription<avt_341::msg::OccupancyGrid>(map_topic, 10, MapCallback);
  auto segmentation_map_sub = n->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 10, SegmentationMapCallback);
  auto waypoint_sub = n->create_subscription<avt_341::msg::Path>("avt_341/new_waypoints", 10, WaypointCallback);
  auto gp_toggle_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/gp_toggle", 10, GlobalPlannerToggleCallback);
  auto nav_command_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/nav_command_state", 10, NavCommandCallback);
  auto goal_pose_sub = n->create_subscription<avt_341::msg::PoseStamped>("avt_341/goal_pose", 10, GoalPoseCallback);
  auto reset_sub = n->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
  auto reset_ack_pub = n->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);

  // ctg, 8-19-2021
  // the state values can be
  // -1 - startup (stopped and not shut down)
  // 0 - active 
  // 1 - bring to a smooth stop but do not shut down
  // 2 - bring to a smooth stop and shut down
  // 3 - bring to an immediate stop (hard braking) and shut down
  auto state_pub = n->create_publisher<avt_341::msg::Int32>("avt_341/state", 10);
  state.data = avt_341::utils::NavStackState::NotInit;

  int num_waypoints = std::min(waypoints_x_list.size(), waypoints_y_list.size());
  Reset();

  // Initialize current waypoints with the data from the waypoint yaml params
  current_waypoints.poses.clear();
  current_waypoints.header.frame_id = "map";
  if (num_waypoints > 0)
  {
    //nav_msgs::Path loaded_waypoints;
    for (int32_t i=0;i<num_waypoints;i++){
      avt_341::msg::PoseStamped pose;
      pose.pose.position.x = static_cast<float>(waypoints_x_list[i]) - local_origin_x;
      pose.pose.position.y = static_cast<float>(waypoints_y_list[i]) - local_origin_y;
      pose.pose.position.z = 0.0f;
      pose.pose.orientation.w = 1.0f;
      pose.pose.orientation.x = 0.0f;
      pose.pose.orientation.y = 0.0f;
      pose.pose.orientation.z = 0.0f;
      current_waypoints.poses.push_back(pose);
    }
      // Initialize goal to first waypoint
    goal[0] = waypoints_x_list[0] - local_origin_x;
    goal[1] = waypoints_y_list[0] - local_origin_y;
    state.data = avt_341::utils::NavStackState::Active; // go active
    state_pub->publish(state);
  }

  auto visualizer = avt_341::visualization::create_visualizer(display_type);
  avt_341::planning::Astar astar_planner(visualizer, w_distance, w_occupancy, w_segmentation,
                                         search_diagonals, los_max_iterations, los_break_on_first);

  if (dilation_factor > 0.0)
  {
    astar_planner.SetDilationFactor(dilation_factor);
  }

  avt_341::node::Rate r(20.0f); // Hz
  int nl = 0;
  int shutdown_count = 0;
  auto t1 = std::chrono::system_clock::now();
  //while (avt_341::node::ok() && !goal_reached){
  while (avt_341::node::ok()){

    if(reset_called){
      Reset();

      avt_341::msg::Path ros_path;
      ros_path.poses.clear();
      ros_path.header.frame_id = "map";
      ros_path.header.stamp = n->get_stamp();
      if (use_global_path)
        path_pub->publish(ros_path);
      state.data = avt_341::utils::NavStackState::NotInit;
      state_pub->publish(state);

      avt_341::msg::String reset_ack_msg;
      reset_ack_msg.data = avt_341::node::NodeType::GlobalPlanner;
      reset_ack_pub->publish(reset_ack_msg);

      reset_called = false;
      n->spin_some();
      r.sleep();
      continue;
    }

    // Handle Go command
    if(nav_command_rcvd) {
	    if(nav_command == avt_341::utils::NavStateCmd::GoActive
      && (state.data == avt_341::utils::NavStackState::NotInit) || state.data == avt_341::utils::NavStackState::Stopped) {
        // startup/idling - go active
        state.data = avt_341::utils::NavStackState::Active;
        shutdown_condition = false;
        state_pub->publish(state);
        nav_command_rcvd = false;
        nav_command = avt_341::utils::NavStateCmd::GoInactive;
		//n->log_info("Set state to %d and shutdown condition to %d", state.data, shutdown_condition);
	    }
	  } else if(use_global_planner){
	    state_pub->publish(state);
	  }

    if(use_global_planner) {
      if (waypoints_rcvd) {
        // process a new set of waypoints
        // TODO: find closest point along path -  we probably don't want to reverse back to start point if we're past it.
        current_waypoint = 0;
        goal[0] = current_waypoints.poses[current_waypoint].pose.position.x;
        goal[1] = current_waypoints.poses[current_waypoint].pose.position.y;
        if(verbose_gp_log){
          n->log_info("New waypoints! Updated goal %f, %f", goal[0], goal[1]);
        }
        waypoints_rcvd = false;
        shutdown_condition = false;
        // Maintaining current state - if we're idle, we'll need an explicit GO command unless auto_active option
        if(auto_active_on_new_waypoint){
          state.data = avt_341::utils::NavStackState::Active;  // go active
          state_pub->publish(state);
        }
      }

      if (odom_rcvd && state.data != avt_341::utils::NavStackState::NotInit && current_waypoints.poses.size() > 0){ // data received and not in startup mode
        std::vector<float> pos;
        pos.push_back(odom.pose.pose.position.x);
        pos.push_back(odom.pose.pose.position.y);


        // check the progression along the path
        float dx = goal[0] - odom.pose.pose.position.x;
        float dy = goal[1] - odom.pose.pose.position.y;
        double d = sqrt(dx * dx + dy * dy);
        avt_341::msg::Float64 dist_to_goal;
        dist_to_goal.data = d;

        std::vector<std::vector<float>> path = astar_planner.PlanPath(&current_grid, &segmentation_grid, goal, pos);

        avt_341::msg::Path ros_path = ToROSPath(path);
        // ctg 8/19/21
        // if not on the last waypoint, add a straight path to the next waypoint to the global path
        // this helps the local planner make smooth transitions between waypoints
        if (d<goal_dist || ros_path.poses.size()>1) {
          int cp =current_waypoint;
          while (cp<current_waypoints.poses.size()-1){
            avt_341::utils::vec2 wp1(static_cast<float>(current_waypoints.poses[cp].pose.position.x),
                                     static_cast<float>(current_waypoints.poses[cp].pose.position.y));
            avt_341::utils::vec2 wp2(static_cast<float>(current_waypoints.poses[cp+1].pose.position.x),
                                     static_cast<float>(current_waypoints.poses[cp+1].pose.position.y));
            avt_341::utils::vec2 wp_diff = wp2-wp1;
            avt_341::utils::vec2 wp_diff_norm = wp_diff;
            wp_diff_norm.normalize();
            if(wp_diff.mag() > max_sep) {
              // Add intermediate waypoints
              for(int d = max_sep; d < wp_diff.mag(); d+=max_sep){
                avt_341::msg::PoseStamped pose;
                pose.pose.position.x = wp1.x + d*wp_diff_norm.x;
                pose.pose.position.y = wp1.y + d*wp_diff_norm.y;
                pose.pose.position.z = 0.0f;
                pose.pose.orientation.w = 1.0f;
                pose.pose.orientation.x = 0.0f;
                pose.pose.orientation.y = 0.0f;
                pose.pose.orientation.z = 0.0f;
                ros_path.poses.push_back(pose);
              }
            }
            avt_341::msg::PoseStamped pose;
            pose.pose.position.x = wp2.x;
            pose.pose.position.y = wp2.y;
            pose.pose.position.z = 0.0f;
            pose.pose.orientation.w = 1.0f;
            pose.pose.orientation.x = 0.0f;
            pose.pose.orientation.y = 0.0f;
            pose.pose.orientation.z = 0.0f;
            ros_path.poses.push_back(pose);
            cp++;
          }
        }

        ros_path.header.stamp = n->get_stamp();
        avt_341::node::set_seq(ros_path.header, nl);

        for (int i = 0; i < ros_path.poses.size(); i++){
          ros_path.poses[i].header = ros_path.header;
        }

        if (use_global_path)
          path_pub->publish(ros_path);
        waypoint_pub->publish(current_waypoints);

        if(debug_visualize){
          auto path_pre_smoothing = astar_planner.GetPathWorldPreSmoothing();
          auto ros_path_pre_smoothing = ToROSPath(path_pre_smoothing);
          ros_path_pre_smoothing.header.stamp = n->get_stamp();
          global_path_pre_smooth_pub->publish(ros_path_pre_smoothing);

          auto path_pre_fill = astar_planner.GetPathWorldPreFill();
          auto ros_path_pre_fill = ToROSPath(*path_pre_fill);
          ros_path_pre_fill.header.stamp = n->get_stamp();
          global_path_pre_fill_pub->publish(ros_path_pre_fill);
        }

        if(current_waypoint < current_waypoints.poses.size()){
          current_waypoint_pub->publish(current_waypoints.poses[current_waypoint]);
        }

        dist_to_current_waypoint_pub->publish(dist_to_goal);
        if (nl % 20 == 0 && verbose_gp_log){ //update every second
          auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - t1);
          t1 = std::chrono::system_clock::now();
          n->log_info("Global Path [%f]: Pos %.2f, %.2f Distance to goal (%.2f, %.2f) for %d of %d = %.2f", duration.count(),
                      odom.pose.pose.position.x, odom.pose.pose.position.y, goal[0], goal[1], current_waypoint, current_waypoints.poses.size() - 1, d);
        }
        if (current_waypoint == current_waypoints.poses.size() - 1){  // last waypoint
		      //std::cout << "Goal Dist: " << d << " Shutdown Condition: " << shutdown_condition << std::endl;
          if (d<goal_dist || shutdown_condition){   // reached the goal
		  	    // send arrival notification
            shutdown_condition = true;
            state.data = shutdown_behavior; // request shutdown behavior
            state_pub->publish(state);

            goal_reached_pub->publish(current_waypoints.poses[current_waypoint]);

			      //std::cout << "Shutdown " << shutdown_behavior << std::endl;
			      if(state.data != avt_341::utils::NavStackState::Stopped) {
              shutdown_count++;
            	if (shutdown_count>10)
				      {
					      std::cout << "Shutting down" << std::endl;
					      break;
				      }
			      }
		      }
        }
        else{     // intermediate waypoint
          if (d<goal_dist){   // reached the waypoint
            goal_reached_pub->publish(current_waypoints.poses[current_waypoint]);

            current_waypoint++;
            goal[0] = current_waypoints.poses[current_waypoint].pose.position.x;
            goal[1] = current_waypoints.poses[current_waypoint].pose.position.y;
          }
		      if(state.data != avt_341::utils::NavStackState::Active) {
		  	    std::cout << "Why are we here? Current state: " << state.data << std::endl;
		      }
          state.data = avt_341::utils::NavStackState::Active;         // request active behavior
          state_pub->publish(state);
        }
      } // if odom_recvd
      //else if(state.data != -1){  // not in startup
      //  state.data = 1;       // request smooth stop but don't shutdown (waiting for odom data)
      //  state_pub->publish(state);
      //}
    }else{
      state.data = avt_341::utils::NavStackState::Active;
      state_pub->publish(state);
    }

    n->spin_some();
    r.sleep();
    nl++;
  }

  return 0;
}
