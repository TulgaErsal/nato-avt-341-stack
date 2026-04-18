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
#include "avt_341/node/node_proxy.h"
#include <avt_341/node/occupancy_grid_subscriber.h>
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/planning/global/astar.h"
#include "avt_341/planning/global/fastmarching.h"
#include "avt_341/planning/global/d_star_lite.h"
#include "avt_341/planning/global/fast_marching_square.h"
#include "avt_341/visualization/visualization_factory.h"
#include "avt_341/node/ros_types.h"
#include "avt_341/core/dto_conversion.h"
#include <chrono>
#include <utility>

using avt_341::utils::NavStackState;
using avt_341::utils::IsGoalReached;

#ifdef Bool
#undef Bool // Fix conflicting definition in Xlib.h
#endif

using namespace avt_341::core;
using avt_341::planning::Point;

avt_341::msg::Odometry odom;
bool odom_rcvd = false;
avt_341::msg::OccupancyGrid current_grid;
avt_341::msg::OccupancyGrid segmentation_grid;
avt_341::msg::NavGoalSequence nav_goals;
bool waypoints_rcvd = false;
bool use_global_planner = true;
int nav_command = 0;
bool nav_command_rcvd = false;
bool verbose_gp_log = false;
bool shutdown_condition = false;
std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;
bool use_segmentation = false;
avt_341::msg::NavState state;
int current_waypoint = 0;
bool reset_called = false;

double dft_dist_threshold = 0.0f;
double dft_yaw_threshold = 30.0f;

double goal_start_time = 0.0;
std::shared_ptr<avt_341::node::Publisher<avt_341::msg::NavState>> state_pub = nullptr;
std::shared_ptr<avt_341::node::Publisher<avt_341::msg::NavState>> goal_reached_pub = nullptr;

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom)
{
  odom = *rcv_odom;
  odom_rcvd = true;
}

void MapCallback(avt_341::msg::OccupancyGridPtr rcv_grid)
{
  current_grid = *rcv_grid;
}

void SegmentationMapCallback(avt_341::msg::OccupancyGridPtr rcv_grid)
{
  if (use_segmentation) {
    segmentation_grid = *rcv_grid;
  }
}

// From mission planner
void WaypointCallback(avt_341::msg::NavGoalSequencePtr rcv_waypoints)
{
  // Brute force - overwrite the current global waypoints
  auto nav_goals_in = *rcv_waypoints;
  for (auto & nav_goal : nav_goals_in.goals)
  {
    if (nav_goal.dist_threshold < 0.0)
    {
      nav_goal.dist_threshold = dft_dist_threshold;
    }
    if (nav_goal.yaw_threshold < 0.0)
    {
      nav_goal.yaw_threshold = dft_yaw_threshold;
    }
  }
  nav_goals = nav_goals_in;
  waypoints_rcvd = true;
  if (verbose_gp_log) {
    if (nav_goals.goals.empty()) {
      n->log_info("Empty waypoint sequence received!");
    }else {
      n->log_info("%d waypoint(s) received! %.2f, %.2f @ (dist=%.2f, yaw=%.2f)",
                  nav_goals.goals.size(),
                  nav_goals.goals[0].pose.position.x,
                  nav_goals.goals[0].pose.position.y,
                  nav_goals.goals[0].dist_threshold,
                  nav_goals.goals[0].yaw_threshold
                  );
    }
  }
}

void PublishGoalReached(const avt_341::msg::NavState& msg)
{
  if (avt_341::utils::UseGoalOrientation(msg.goal))
  {
    n->log_info("Goal reached (%.2f, %.2f, %.2f) @ threshold (dist=%.2f, yaw=%.2f) after %.2f seconds",
        msg.goal.pose.position.x,
        msg.goal.pose.position.y,
        avt_341::utils::GetHeadingFromOrientation(msg.goal.pose.orientation)/M_PI*180.0,
        msg.goal.dist_threshold,
        msg.goal.yaw_threshold/M_PI*180.0,
        msg.goal_duration);
  }
  else
  {
    n->log_info("Goal reached (%.2f, %.2f) @ threshold (dist=%.2f) after %.2f seconds",
        msg.goal.pose.position.x,
        msg.goal.pose.position.y,
        msg.goal.dist_threshold,
        msg.goal_duration);
  }

  goal_reached_pub->publish(msg);
}

void GlobalPlannerToggleCallback(avt_341::msg::Int32Ptr rcv_gptoggle)
{
  bool set_val = (bool)rcv_gptoggle->data;
  if(use_global_planner != set_val){
    n->log_info("GP set to %d", rcv_gptoggle->data);
  }
  use_global_planner = set_val;
}

void NavCommandCallback(avt_341::msg::Int32Ptr rcv_navcommand)
{
  nav_command = rcv_navcommand->data;
  nav_command_rcvd = true;
}

// Needs to remain PoseStamped to support RVIZ goal
void GoalPoseCallback(avt_341::msg::PoseStampedPtr rcv_goal_pose)
{
  const auto nav_goal = ToNavGoal(*rcv_goal_pose, dft_dist_threshold, dft_yaw_threshold);
  n->log_info("Setting goal (%.2f, %.2f, %.2f) @ threshold (dist=%.2f, yaw=%.2f)",
    nav_goal.pose.position.x,
    nav_goal.pose.position.y,
    avt_341::utils::GetHeadingFromOrientation(nav_goal.pose.orientation)/M_PI*180.0,
    nav_goal.dist_threshold,
    nav_goal.yaw_threshold/M_PI*180.0
    );
  nav_goals.goals.clear();
  nav_goals.goals.push_back(nav_goal);
  waypoints_rcvd = true;
}

void ResetCallback(avt_341::msg::StringPtr msg)
{
  if (msg->data.find(avt_341::node::NodeType::GlobalPlanner) != std::string::npos) {
    reset_called = true;
  }
}

avt_341::msg::NavGoal GetCurrentGoal()
{
  return current_waypoint < nav_goals.goals.size() ? nav_goals.goals[current_waypoint] : avt_341::msg::NavGoal();
}

void SetRunState(const int run_state)
{
  state.header.stamp = n->get_stamp();
  state.run_state = run_state;
  state_pub->publish(state);
}

void UpdateGoalState(const avt_341::msg::NavGoal& goal)
{
  state.header.stamp = n->get_stamp();

  if (state.run_state == NavStackState::Active) {
    const auto t_now = n->get_now_seconds();
    if (avt_341::utils::GetDistance(goal.pose.position, state.goal.pose.position) > 1e-2) {
      goal_start_time = t_now;
    }
    state.goal = goal;
    double dist_diff, yaw_diff;
    avt_341::utils::GetGoalError(odom.pose.pose, goal, dist_diff, yaw_diff);
    state.goal_distance = dist_diff;
    state.goal_yaw_difference = yaw_diff;
    state.goal_duration = t_now - goal_start_time;
  }else {
    state.goal = avt_341::msg::NavGoal();
    state.goal_distance = 0.0;
    state.goal_yaw_difference = 0.0;
    state.goal_duration = 0.0;
  }
}

void Reset()
{
  n->log_info("Resetting node");
  nav_goals.goals.clear();
  current_waypoint = 0;
  UpdateGoalState(avt_341::msg::NavGoal());
  SetRunState(NavStackState::NotInit);
  odom_rcvd = false;
  shutdown_condition = false;
}

int main(int argc, char* argv[])
{
  n = avt_341::node::init_node(argc, argv, "avt_341_global_path_node");

  float global_lookahead, w_distance, w_occupancy, w_segmentation;
  double local_origin_x, local_origin_y;
  std::vector<double> waypoints_x_list, waypoints_y_list;
  std::string display_type;
  bool debug_visualize, search_diagonals, los_break_on_first, auto_active_on_new_waypoint, use_global_path;
  int los_max_iterations;
  float dilation_factor, max_separation;
  float safety_margin_global, obstacle_threshold, clearance_penalty_scale, clearance_penalty_range, clearance_penalty_exponent;
  int gradient_descent_max_steps, gradient_descent_steps_per_point;
  float clipping_distance;
  std::string map_topic, seg_topic;
  std::string planning_method, clearance_penalty_type, path_extraction_method;

  n->get_parameter("~goal_dist", dft_dist_threshold, 3.0);
  n->get_parameter("~goal_yaw_threshold", dft_yaw_threshold, 360.0);    // Set value > 180.0 degrees to disable
  n->get_parameter("~display", display_type, avt_341::visualization::default_display);
  n->get_parameter("~global_lookahead", global_lookahead, 50.0f);
  n->get_parameter("/waypoints_x", waypoints_x_list, std::vector<double>(0));
  n->get_parameter("/waypoints_y", waypoints_y_list, std::vector<double>(0));
  dft_yaw_threshold *= M_PI / 180.0;

  // TODO: Would like to get rid of these, name if confusing and just does coordinate transform which should be done by ROS2 tf system using frame ids
  // TODO: Or Maybe encode them in file with waypoints?
  n->get_parameter("/map_origin_x", local_origin_x, 0.0);
  n->get_parameter("/map_origin_y", local_origin_y, 0.0);

  n->get_parameter("~debug_visualize", debug_visualize, true);
  n->get_parameter("~search_diagonals", search_diagonals, false);
  n->get_parameter("~los_max_iterations", los_max_iterations, 1);
  n->get_parameter("~los_break_on_first", los_break_on_first, true);
  n->get_parameter("~w_distance", w_distance, 1.0f);
  n->get_parameter("~w_occupancy", w_occupancy, 1.0f);
  n->get_parameter("~w_segmentation", w_segmentation, 1.0f);
  n->get_parameter("~auto_active_on_new_waypoint", auto_active_on_new_waypoint, false);
  n->get_parameter("~verbose_gp_log", verbose_gp_log, true);
  n->get_parameter("~dilation_factor", dilation_factor, 0.0f);
  n->get_parameter("~max_separation", max_separation, 1.0f);
  n->get_parameter("~use_global_path", use_global_path, true);
  n->get_parameter("~use_segmentation", use_segmentation, true);
  n->get_parameter("~map_topic", map_topic, std::string("avt_341/occupancy_grid_low_res"));
  n->get_parameter("~seg_topic", seg_topic, std::string("avt_341/normal_segmentation_grid"));
  n->get_parameter("~planning_method", planning_method, std::string("astar"));
  n->get_parameter("~safety_margin_global", safety_margin_global, 0.5f);
  n->get_parameter("~clearance_penalty_type", clearance_penalty_type, std::string("repulsive_potential"));
  n->get_parameter("~path_extraction_method", path_extraction_method, std::string("gradient_descent"));
  n->get_parameter("~obstacle_threshold", obstacle_threshold, 0.0f);
  n->get_parameter("~clearance_penalty_scale", clearance_penalty_scale, 20.0f);
  n->get_parameter("~clearance_penalty_range", clearance_penalty_range, 5.0f);
  n->get_parameter("~clearance_penalty_exponent", clearance_penalty_exponent, 2.0f);
  n->get_parameter("~gradient_descent_max_steps", gradient_descent_max_steps, 2000);
  n->get_parameter("~gradient_descent_steps_per_point", gradient_descent_steps_per_point, 10);
  n->get_parameter("~clipping_distance", clipping_distance, 0.0f);

  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> global_path_pre_smooth_pub = nullptr;
  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> global_path_pre_fill_pub = nullptr;
  if(debug_visualize){
    global_path_pre_smooth_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path_pre_smooth", 10);
    global_path_pre_fill_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path_pre_fill", 10);
  }

  int shutdown_behavior;
  n->get_parameter("~shutdown_behavior", shutdown_behavior, static_cast<int>(NavStackState::Stopped));
  if (shutdown_behavior > 3 || shutdown_behavior < 1)shutdown_behavior = 1;

  n->log_info("\nGlobal Planner Settings:\n w_distance: %.2f\n w_occupancy: %.2f\n w_segmentation: %.2f\n method: %s\n clipping_distance: %.2f",
    w_distance, w_occupancy, w_segmentation, planning_method.c_str(), clipping_distance);

  auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path", 1);
  auto waypoint_pub = n->create_publisher<avt_341::msg::Path>("avt_341/waypoints", 10);
  goal_reached_pub = n->create_publisher<avt_341::msg::NavState>("avt_341/goal_reached", 10);

  auto odometry_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
  auto map_sub = avt_341::node::OccupancyGridSubscriber(n, map_topic, 10, MapCallback);
  auto segmentation_map_sub = avt_341::node::OccupancyGridSubscriber(n, seg_topic, 10, SegmentationMapCallback);
  auto waypoint_sub = n->create_subscription<avt_341::msg::NavGoalSequence>("avt_341/new_waypoints", 10, WaypointCallback);
  auto goal_pose_sub = n->create_subscription<avt_341::msg::PoseStamped>("avt_341/goal_pose", 10, GoalPoseCallback);
  auto gp_toggle_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/gp_toggle", 10, GlobalPlannerToggleCallback);
  auto nav_command_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/nav_command_state", 10, NavCommandCallback);
  auto reset_sub = n->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
  auto reset_ack_pub = n->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);
  auto fastmatching_costs_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/fm_map", 1);

  state_pub = n->create_publisher<avt_341::msg::NavState>("avt_341/state", 10);
  state.run_state = NavStackState::NotInit;

  Reset();

  // Initialize current waypoints with the data from the waypoint yaml params
  const auto map_origin = Point{static_cast<float>(local_origin_x), static_cast<float>(local_origin_y)};
  nav_goals = ToNavGoalSequence(waypoints_x_list, waypoints_y_list, map_origin, dft_dist_threshold, dft_yaw_threshold, "map");

  if (!nav_goals.goals.empty()) {
    UpdateGoalState(nav_goals.goals[0]);
    SetRunState(NavStackState::Active);
  }

  auto visualizer = avt_341::visualization::create_visualizer(display_type);

  std::shared_ptr<avt_341::planning::Astar> path_planner;
  if (planning_method == "fast_marching") {
    path_planner = std::make_shared<avt_341::planning::FastMarching>(visualizer,
                                                       w_distance,
                                                       w_occupancy,
                                                       w_segmentation,
                                                       search_diagonals,
                                                       los_max_iterations,
                                                       los_break_on_first,
                                                       safety_margin_global,
                                                       clearance_penalty_type,
                                                       path_extraction_method,
                                                       obstacle_threshold,
                                                       clearance_penalty_scale,
                                                       clearance_penalty_range,
                                                       clearance_penalty_exponent,
                                                       gradient_descent_max_steps,
                                                       gradient_descent_steps_per_point,
                                                       clipping_distance,
                                                       verbose_gp_log);
  } else if (planning_method == "d_star_lite") {
    path_planner = std::make_shared<avt_341::planning::DStarLite>(visualizer,
                                                    w_distance,
                                                    w_occupancy,
                                                    w_segmentation,
                                                    search_diagonals,
                                                    los_max_iterations,
                                                    los_break_on_first);
  } else if (planning_method == "fast_marching_square") {
    path_planner = std::make_shared<avt_341::planning::FastMarchingSquare>(visualizer,
                                                             w_distance,
                                                             w_occupancy,
                                                             w_segmentation,
                                                             search_diagonals,
                                                             los_max_iterations,
                                                             los_break_on_first,
                                                             safety_margin_global,
                                                             clearance_penalty_type,
                                                             path_extraction_method,
                                                             obstacle_threshold,
                                                             clearance_penalty_scale,
                                                             clearance_penalty_range,
                                                             clearance_penalty_exponent,
                                                             gradient_descent_max_steps,
                                                             gradient_descent_steps_per_point,
                                                             clipping_distance,
                                                             verbose_gp_log);
  } else {
    path_planner = std::make_shared<avt_341::planning::Astar>(visualizer,
                                                w_distance,
                                                w_occupancy,
                                                w_segmentation,
                                                search_diagonals,
                                                los_max_iterations,
                                                los_break_on_first);
  }

  if (dilation_factor > 0.0) {
    path_planner->SetDilationFactor(static_cast<int>(dilation_factor));
  }

  avt_341::node::Rate r(20.0f); // Hz
  int nl = 0;
  int shutdown_count = 0;
  auto t1 = std::chrono::system_clock::now();
  //while (avt_341::node::ok() && !goal_reached){
  while (avt_341::node::ok()) {

    if (reset_called) {
      Reset();

      avt_341::msg::Path ros_path;
      ros_path.poses.clear();
      ros_path.header.frame_id = "map";
      ros_path.header.stamp = n->get_stamp();
      if (use_global_path)
        path_pub->publish(ros_path);
      SetRunState(NavStackState::NotInit);

      avt_341::msg::String reset_ack_msg;
      reset_ack_msg.data = avt_341::node::NodeType::GlobalPlanner;
      reset_ack_pub->publish(reset_ack_msg);

      reset_called = false;
      n->spin_some();
      r.sleep();
      continue;
    }

    // Handle Go command
    if (nav_command_rcvd) {
      if (nav_command == avt_341::utils::NavStateCmd::GoActive &&
        (state.run_state == NavStackState::NotInit || state.run_state == NavStackState::Stopped)) {
        // startup/idling - go active
        SetRunState(NavStackState::Active);
        shutdown_condition = false;
        nav_command_rcvd = false;
        nav_command = avt_341::utils::NavStateCmd::GoInactive;
        //n->log_info("Set state to %d and shutdown condition to %d", state.data, shutdown_condition);
      }
    } else if (use_global_planner) {
      state_pub->publish(state);
    }

    if (use_global_planner) {
      if (waypoints_rcvd) {
        // process a new set of waypoints
        // TODO: find closest point along path -  we probably don't want to reverse back to start point if we're past it.
        current_waypoint = 0;
        if (verbose_gp_log) {
          const auto goal = GetCurrentGoal();
          n->log_info("New waypoints! Updated goal %.2f, %.2f @ (dist=%.2f, yaw=%.2f)",
            goal.pose.position.x, goal.pose.position.y, goal.dist_threshold, goal.yaw_threshold);
        }
        waypoints_rcvd = false;
        shutdown_condition = false;
        // Maintaining current state - if we're idle, we'll need an explicit GO command unless auto_active option
        if (auto_active_on_new_waypoint) {
          SetRunState(NavStackState::Active);
        }
      }

      if (odom_rcvd && state.run_state != NavStackState::NotInit
        && !nav_goals.goals.empty()) { // data received and not in startup mode
        Point position{static_cast<float>(odom.pose.pose.position.x), static_cast<float>(odom.pose.pose.position.y)};

        // check the progression along the path
        auto goal = GetCurrentGoal();
        UpdateGoalState(goal);

        std::vector<Point> path = path_planner->PlanPath(&current_grid, &segmentation_grid, ToVec2(goal.pose.position), position);
        if (planning_method == "fast_marching" && !path.empty()) {
          avt_341::msg::OccupancyGrid fast_marching_grid;
          fast_marching_grid.header = current_grid.header;
          fast_marching_grid.info = current_grid.info;
          int height = path_planner->GetGridHeight();
          int width = path_planner->GetGridWidth();
          fast_marching_grid.data.resize(height*width);

          float* fm_data = path_planner->ExtractCosts();
          if (fm_data) {
            for (int i = 0; i < width*height; i++) {
              float value = fm_data[i];
              if (value < 0.0f) {
                fast_marching_grid.data[i] = -1; // Unknown
              } else if (!isfinite(value)) {
                fast_marching_grid.data[i] = -1;
              } else {
                fast_marching_grid.data[i] = static_cast<int>(std::round(value));
              }
            }
          }
          fastmatching_costs_pub->publish(fast_marching_grid);
        }
//        n->log_info("Planned path of size %d", path.size());

        avt_341::msg::Path ros_path = ToPath(path);
        // ctg 8/19/21
        // if not on the last waypoint, add a straight path to the next waypoint to the global path
        // this helps the local planner make smooth transitions between waypoints
        if (IsGoalReached(state, goal) || ros_path.poses.size() > 1) {
          int cp = current_waypoint;
          while (cp < nav_goals.goals.size() - 1) {
            avt_341::utils::vec2 wp1(static_cast<float>(nav_goals.goals[cp].pose.position.x),
                                     static_cast<float>(nav_goals.goals[cp].pose.position.y));
            avt_341::utils::vec2 wp2(static_cast<float>(nav_goals.goals[cp + 1].pose.position.x),
                                     static_cast<float>(nav_goals.goals[cp + 1].pose.position.y));
            avt_341::utils::vec2 wp_diff = wp2 - wp1;
            avt_341::utils::vec2 wp_diff_norm = wp_diff;
            wp_diff_norm.normalize();
            if (wp_diff.mag() > max_separation) {
              // Add intermediate waypoints
              for (int step = max_separation; step < wp_diff.mag(); step += max_separation) {
                float px = wp1.x + step * wp_diff_norm.x;
                float py = wp1.y + step * wp_diff_norm.y;
                ros_path.poses.push_back(ToPoseStamped(ros_path.header.frame_id, px, py));
              }
            }
            ros_path.poses.push_back(ToPoseStamped(ros_path.header.frame_id, wp2.x, wp2.y));
            cp++;
          }
        }

        ros_path.header.stamp = n->get_stamp();

        for (int i = 0; i < ros_path.poses.size(); i++) {
          ros_path.poses[i].header = ros_path.header;
        }

        if (use_global_path) {
          path_pub->publish(ros_path);
        }
        waypoint_pub->publish(ToPath(nav_goals));
//        n->log_info("Published path with %d waypoints", ros_path.poses.size());

        if (debug_visualize) {
          auto path_pre_smoothing = path_planner->GetPathWorldPreSmoothing();
          auto ros_path_pre_smoothing = ToPath(path_pre_smoothing);
          ros_path_pre_smoothing.header.stamp = n->get_stamp();
          global_path_pre_smooth_pub->publish(ros_path_pre_smoothing);

          auto path_pre_fill = path_planner->GetPathWorldPreFill();
          auto ros_path_pre_fill = ToPath(*path_pre_fill);
          ros_path_pre_fill.header.stamp = n->get_stamp();
          global_path_pre_fill_pub->publish(ros_path_pre_fill);
        }

        if (nl % 20 == 0 && verbose_gp_log) { //update every second
          auto t_now = std::chrono::system_clock::now();
          auto calc_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t1);
          n->log_info("Global Path [%d]: Pos (%.2f, %.2f) Distance to goal (%.2f, %.2f) for %d of %d = %.2f",
                      calc_duration_ms.count(),
                      odom.pose.pose.position.x,
                      odom.pose.pose.position.y,
                      goal.pose.position.x,
                      goal.pose.position.y,
                      current_waypoint+1,
                      nav_goals.goals.size(),
                      state.goal_distance);
          t1 = t_now;
        }
        if (current_waypoint == nav_goals.goals.size() - 1) {  // last waypoint
          //std::cout << "Goal Dist: " << d << " Shutdown Condition: " << shutdown_condition << std::endl;
          if (IsGoalReached(state, goal) || shutdown_condition) {   // reached the goal
            // send arrival notification

            if (state.run_state == NavStackState::Active) {
              PublishGoalReached(state);
            }

            shutdown_condition = true;
            SetRunState(shutdown_behavior);// request shutdown behavior

            //std::cout << "Shutdown " << shutdown_behavior << std::endl;
            if (state.run_state != NavStackState::Stopped) {
              shutdown_count++;
              if (shutdown_count > 10) {
                std::cout << "Shutting down" << std::endl;
                break;
              }
            }
          }
        } else {     // intermediate waypoint
          if (IsGoalReached(state, goal)) {   // reached the waypoint
            PublishGoalReached(state);
            current_waypoint++;
          }
          if (state.run_state != NavStackState::Active) {
            std::cout << "Why are we here? Current state: " << state.run_state << std::endl;
          }
          SetRunState(NavStackState::Active); // request active behavior
        }
      } // if odom_recvd
      //else if(state.data != -1){  // not in startup
      //  state.data = 1;       // request smooth stop but don't shutdown (waiting for odom data)
      //  state_pub->publish(state);
      //}
    } else {
      SetRunState(NavStackState::Active);
    }

    n->spin_some();
    r.sleep();
    nl++;
  }

  return 0;
}
