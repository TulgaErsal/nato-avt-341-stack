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
#include <future>
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/core/waypoint_file_parser.hpp"
#include "avt_341/planning/global/astar.h"
#include "avt_341/planning/global/fastmarching.h"
#include "avt_341/planning/global/d_star_lite.h"
#include "avt_341/planning/global/fast_marching_square.h"
#include "avt_341/node/ros_types.h"
#include "avt_341/core/dto_conversion.h"
#include <avt_341/global_planner_params_service.hpp>
#include <avt_341_msgs/srv/compute_global_path.hpp>
#include <chrono>
#include <stdexcept>

using avt_341::utils::NavStackState;
using avt_341::utils::IsGoalReached;

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
std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;
avt_341::msg::NavState state;
int current_waypoint = 0;
bool reset_called = false;

double dft_dist_threshold = 0.0f;
double dft_yaw_threshold = 30.0f;

double goal_start_time = 0.0;
std::shared_ptr<avt_341::node::Publisher<avt_341::msg::NavState>> state_pub = nullptr;
std::shared_ptr<avt_341::node::Publisher<avt_341::msg::NavState>> goal_reached_pub = nullptr;
std::shared_ptr<avt_341::planning::Astar> path_planner = nullptr;

// Async planning state
std::future<std::vector<avt_341::planning::Point>> planning_future;
avt_341::msg::Path last_valid_ros_path;
std::chrono::steady_clock::time_point plan_start;
bool timeout_logged = false;

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom)
{
  odom = *rcv_odom;
  odom_rcvd = true;
}

void MapCallback(avt_341::msg::OccupancyGridPtr rcv_grid)
{
  current_grid = *rcv_grid;
}

void SegmentationMapCallback(avt_341::msg::OccupancyGridPtr rcv_grid,
                             bool use_segmentation)
{
  if (use_segmentation) {
    segmentation_grid = *rcv_grid;
  }
}

// From mission planner
void WaypointCallback(avt_341::msg::NavGoalSequencePtr rcv_waypoints,
                      bool verbose_gp_log)
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
}

static std::vector<Point> TrimPathEnd(const std::vector<Point>& path, double trim_distance)
{
  if (trim_distance <= 0.0 || path.size() < 2) {
    return path;
  }
  double accumulated = 0.0;
  for (int i = static_cast<int>(path.size()) - 1; i > 0; --i) {
    Point seg = path[i] - path[i - 1];
    const double seg_len = seg.mag();
    if (accumulated + seg_len >= trim_distance) {
      const double remaining = trim_distance - accumulated;
      const double t = (seg_len - remaining) / seg_len;
      Point interp;
      interp.x = path[i - 1].x + static_cast<float>(t) * seg.x;
      interp.y = path[i - 1].y + static_cast<float>(t) * seg.y;
      std::vector<Point> trimmed(path.begin(), path.begin() + i);
      trimmed.push_back(interp);
      return trimmed;
    }
    accumulated += seg_len;
  }
  return std::vector<Point>{ path.front() };
}

void ComputeGlobalPathServiceCallback(
    const std::shared_ptr<avt_341_msgs::srv::ComputeGlobalPath::Request> request,
    std::shared_ptr<avt_341_msgs::srv::ComputeGlobalPath::Response> response)
{
  avt_341::msg::Path full_path;
  full_path.header.stamp = n->get_stamp();
  full_path.header.frame_id = "map";

  if (planning_future.valid()) {
    n->log_warning("ComputeGlobalPath: background planning in progress -- request ignored");
    response->path = full_path;
    return;
  }

  if (request->goals.goals.empty()) {
    n->log_warning("ComputeGlobalPath: empty goal sequence received");
    response->path = full_path;
    return;
  }

  Point segment_start = ToVec2(request->start_pose.pose);

  const std::size_t num_goals = request->goals.goals.size();
  for (std::size_t i = 0; i < num_goals; ++i) {
    auto& goal = request->goals.goals[i];
    const auto dist_threshold = goal.dist_threshold < 0.0 ? dft_dist_threshold : goal.dist_threshold;
    const Point goal_pt = ToVec2(goal.pose);

    std::vector<Point> segment = path_planner->PlanPath(
      &current_grid, &segmentation_grid, goal_pt, segment_start);

    if (segment.empty()) {
      n->log_warning("ComputeGlobalPath: planner returned empty segment for goal %zu (%.2f, %.2f)", i, goal_pt.x, goal_pt.y);
      continue;
    }

    if (request->remove_threshold) {
      segment = TrimPathEnd(segment, dist_threshold);
    }

    // Append segment poses, skipping the duplicated junction point (start of this
    // segment matches the end of the previous one) for all but the first segment.
    const std::size_t skip = full_path.poses.empty() ? 0 : 1;
    for (std::size_t k = skip; k < segment.size(); ++k) {
      auto pose = ToPoseStamped(full_path.header.frame_id, segment[k].x, segment[k].y);
      pose.header.stamp = full_path.header.stamp;
      full_path.poses.push_back(pose);
    }

    // Next segment starts from the (possibly trimmed) terminal point of this one.
    segment_start = segment.back();
  }

  response->path = full_path;
  n->log_info("ComputeGlobalPath: produced path with %zu poses across %zu goals",
              full_path.poses.size(), num_goals);
}

int main(int argc, char* argv[])
{
  n = avt_341::node::init_node(argc, argv, "avt_341_global_path_node");
  avt_341::params::global_planner::ParamsListener param_listener(n->get_raw_node());
  const auto params = param_listener.get_params();

  dft_dist_threshold = params.goal_dist;
  dft_yaw_threshold = params.goal_yaw_threshold;    // Set value > 180.0 degrees to disable
  dft_yaw_threshold *= M_PI / 180.0;

  int planning_timeout_ms = static_cast<int>(params.planning_timeout_ms);

  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> global_path_pre_smooth_pub = nullptr;
  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> global_path_pre_fill_pub = nullptr;
  if(params.debug_visualize){
    global_path_pre_smooth_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path_pre_smooth", 10);
    global_path_pre_fill_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path_pre_fill", 10);
  }

  int shutdown_behavior = static_cast<int>(params.shutdown_behavior);

  if (!avt_341::utils::IsValidShutdownBehavior(shutdown_behavior)){
    const std::string error_msg = "Invalid shutdown behavior parameter: " + std::to_string(shutdown_behavior);
    n->log_error("%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  n->log_info("\nGlobal Planner Settings:\n w_distance: %.2f\n w_occupancy: %.2f\n w_segmentation: %.2f\n method: %s\n clipping_distance: %.2f",
    params.w_distance, params.w_occupancy, params.w_segmentation,
    params.planning_method.c_str(), params.clipping_distance);

  auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path", 1);
  auto waypoint_pub = n->create_publisher<avt_341::msg::Path>("avt_341/waypoints", 10);
  goal_reached_pub = n->create_publisher<avt_341::msg::NavState>("avt_341/goal_reached", 10);

  auto odometry_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
  auto map_sub = avt_341::node::OccupancyGridSubscriber(
      n, params.map_topic, 10, params.costmap.publish.method, MapCallback);
  auto segmentation_map_sub = avt_341::node::OccupancyGridSubscriber(
      n, params.seg_topic, 10, params.costmap.publish.method,
      [&params](avt_341::msg::OccupancyGridPtr msg) {
        SegmentationMapCallback(msg, params.use_segmentation);
      });
  auto waypoint_sub = n->create_subscription<avt_341::msg::NavGoalSequence>(
      "avt_341/new_waypoints", 10,
      [&params](avt_341::msg::NavGoalSequencePtr msg) {
        WaypointCallback(msg, params.verbose_gp_log);
      });
  auto goal_pose_sub = n->create_subscription<avt_341::msg::PoseStamped>("avt_341/goal_pose", 10, GoalPoseCallback);
  auto gp_toggle_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/gp_toggle", 10, GlobalPlannerToggleCallback);
  auto nav_command_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/nav_command_state", 10, NavCommandCallback);
  auto reset_sub = n->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
  auto reset_ack_pub = n->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);
  auto fastmatching_costs_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/fm_map", 1);

  state_pub = n->create_publisher<avt_341::msg::NavState>("avt_341/state", 10);
  state.run_state = NavStackState::NotInit;

  Reset();

    // Initialize current waypoints from the initial waypoints file, if given
    const auto waypoints = WaypointFileParser::Parse(params.initial_waypoints);
    const auto map_origin = Point{static_cast<float>(params.gis.origin_x), static_cast<float>(params.gis.origin_y)};
    nav_goals = ToNavGoalSequence(waypoints.x, waypoints.y, map_origin,
        dft_dist_threshold, dft_yaw_threshold, "map");

  if (!nav_goals.goals.empty()) {
    UpdateGoalState(nav_goals.goals[0]);
    SetRunState(NavStackState::Active);
  }

  if (params.planning_method == "fast_marching") {
    path_planner = std::make_shared<avt_341::planning::FastMarching>(params.w_distance,
                                                       params.w_occupancy,
                                                       params.w_segmentation,
                                                       params.search_diagonals,
                                                       static_cast<int>(params.los_max_iterations),
                                                       params.los_break_on_first,
                                                       params.safety_margin_global,
                                                       params.clearance_penalty_type,
                                                       params.path_extraction_method,
                                                       params.obstacle_threshold,
                                                       params.clearance_penalty_scale,
                                                       params.clearance_penalty_range,
                                                       params.clearance_penalty_exponent,
                                                       static_cast<int>(params.gradient_descent_max_steps),
                                                       static_cast<int>(params.gradient_descent_steps_per_point),
                                                       params.clipping_distance,
                                                       params.verbose_gp_log);
  } else if (params.planning_method == "d_star_lite") {
    path_planner = std::make_shared<avt_341::planning::DStarLite>(params.w_distance,
                                                    params.w_occupancy,
                                                    params.w_segmentation,
                                                    params.search_diagonals,
                                                    static_cast<int>(params.los_max_iterations),
                                                    params.los_break_on_first);
  } else if (params.planning_method == "fast_marching_square") {
    path_planner = std::make_shared<avt_341::planning::FastMarchingSquare>(params.w_distance,
                                                             params.w_occupancy,
                                                             params.w_segmentation,
                                                             params.search_diagonals,
                                                             static_cast<int>(params.los_max_iterations),
                                                             params.los_break_on_first,
                                                             params.safety_margin_global,
                                                             params.clearance_penalty_type,
                                                             params.path_extraction_method,
                                                             params.obstacle_threshold,
                                                             params.clearance_penalty_scale,
                                                             params.clearance_penalty_range,
                                                             params.clearance_penalty_exponent,
                                                             static_cast<int>(params.gradient_descent_max_steps),
                                                             static_cast<int>(params.gradient_descent_steps_per_point),
                                                             params.clipping_distance,
                                                             params.verbose_gp_log);
  } else {
    path_planner = std::make_shared<avt_341::planning::Astar>(params.w_distance,
                                                params.w_occupancy,
                                                params.w_segmentation,
                                                params.search_diagonals,
                                                static_cast<int>(params.los_max_iterations),
                                                params.los_break_on_first);
  }

  if (params.dilation_factor > 0.0) {
    path_planner->SetDilationFactor(
        static_cast<int>(params.dilation_factor));
  }

  // Service: compute a global path through a sequence of NavGoals starting from a given pose.
  auto compute_global_path_srv =
      n->get_raw_node()->create_service<avt_341_msgs::srv::ComputeGlobalPath>(
          "avt_341/compute_global_path", &ComputeGlobalPathServiceCallback);

  avt_341::node::Rate r(20.0f); // Hz
  int nl = 0;
  int shutdown_count = 0;
  auto t1 = std::chrono::system_clock::now();
  //while (avt_341::node::ok() && !goal_reached){
  while (avt_341::node::ok()) {

    if (reset_called) {
      if (planning_future.valid()) {
        path_planner->RequestCancel();
        planning_future.get();
        path_planner->ClearCancel();
      }
      last_valid_ros_path = avt_341::msg::Path{};
      Reset();

      avt_341::msg::Path ros_path;
      ros_path.poses.clear();
      ros_path.header.frame_id = "map";
      ros_path.header.stamp = n->get_stamp();
      if (params.use_global_path)
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
      if (nav_command == avt_341::utils::NavStateCmd::GoActive && state.run_state != NavStackState::Active) {
        // startup/idling - go active
        SetRunState(NavStackState::Active);
        nav_command = avt_341::utils::NavStateCmd::GoInactive;
      }
      nav_command_rcvd = false;
    } else if (use_global_planner) {
      state_pub->publish(state);
    }

    if (use_global_planner) {
      if (waypoints_rcvd) {
        // process a new set of waypoints
        // TODO: find closest point along path -  we probably don't want to reverse back to start point if we're past it.
        current_waypoint = 0;
        if (params.verbose_gp_log) {
          const auto goal = GetCurrentGoal();
          n->log_info("New waypoints! Updated goal %.2f, %.2f @ (dist=%.2f, yaw=%.2f)",
            goal.pose.position.x, goal.pose.position.y, goal.dist_threshold, goal.yaw_threshold);
        }
        waypoints_rcvd = false;
        // Maintaining current state - if we're idle, we'll need an explicit GO command unless auto_active option
        if (params.auto_active_on_new_waypoint) {
          SetRunState(NavStackState::Active);
        }
        if (planning_future.valid()) {
          path_planner->RequestCancel();
        }
      }

      if (odom_rcvd && state.run_state != NavStackState::NotInit
        && !nav_goals.goals.empty())
      {
        // data received and not in startup mode
        Point position{static_cast<float>(odom.pose.pose.position.x), static_cast<float>(odom.pose.pose.position.y)};

        auto goal = GetCurrentGoal();
        UpdateGoalState(goal);

        if (planning_future.valid()) {
          if (planning_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            auto new_path = planning_future.get();
            timeout_logged = false;

            if (!new_path.empty()) {
              if (params.planning_method == "fast_marching") {
                avt_341::msg::OccupancyGrid fast_marching_grid;
                fast_marching_grid.header = current_grid.header;
                fast_marching_grid.info = current_grid.info;
                int height = path_planner->GetGridHeight();
                int width = path_planner->GetGridWidth();
                fast_marching_grid.data.resize(height * width);
                float* fm_data = path_planner->ExtractCosts();
                if (fm_data) {
                  for (int i = 0; i < width * height; i++) {
                    float value = fm_data[i];
                    if (value < 0.0f || !isfinite(value)) {
                      fast_marching_grid.data[i] = -1;
                    } else {
                      fast_marching_grid.data[i] = static_cast<int>(std::round(value));
                    }
                  }
                }
                fastmatching_costs_pub->publish(fast_marching_grid);
              }

              if (params.debug_visualize) {
                auto path_pre_smoothing = path_planner->GetPathWorldPreSmoothing();
                auto ros_path_pre_smoothing = ToPath(path_pre_smoothing);
                ros_path_pre_smoothing.header.stamp = n->get_stamp();
                global_path_pre_smooth_pub->publish(ros_path_pre_smoothing);

                auto path_pre_fill = path_planner->GetPathWorldPreFill();
                auto ros_path_pre_fill = ToPath(*path_pre_fill);
                ros_path_pre_fill.header.stamp = n->get_stamp();
                global_path_pre_fill_pub->publish(ros_path_pre_fill);
              }

              avt_341::msg::Path ros_path = ToPath(new_path);
              if (IsGoalReached(state, goal) || ros_path.poses.size() > 1) {
                int cp = current_waypoint;
                while (cp < static_cast<int>(nav_goals.goals.size()) - 1) {
                  avt_341::utils::vec2 wp1(static_cast<float>(nav_goals.goals[cp].pose.position.x),
                                           static_cast<float>(nav_goals.goals[cp].pose.position.y));
                  avt_341::utils::vec2 wp2(static_cast<float>(nav_goals.goals[cp + 1].pose.position.x),
                                           static_cast<float>(nav_goals.goals[cp + 1].pose.position.y));
                  avt_341::utils::vec2 wp_diff = wp2 - wp1;
                  avt_341::utils::vec2 wp_diff_norm = wp_diff;
                  wp_diff_norm.normalize();
                  if (wp_diff.mag() > params.max_separation) {
                    for (double step = params.max_separation;
                         step < wp_diff.mag();
                         step += params.max_separation) {
                      float px = wp1.x + step * wp_diff_norm.x;
                      float py = wp1.y + step * wp_diff_norm.y;
                      ros_path.poses.push_back(ToPoseStamped(ros_path.header.frame_id, px, py));
                    }
                  }
                  ros_path.poses.push_back(ToPoseStamped(ros_path.header.frame_id, wp2.x, wp2.y));
                  cp++;
                }
              }
              last_valid_ros_path = ros_path;
            }
          } else {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - plan_start).count();
            if (elapsed_ms > planning_timeout_ms && !timeout_logged) {
              path_planner->RequestCancel();
              n->log_warning("Planning timeout (%ldms) -- cancelling and re-publishing previous path",
                             static_cast<long>(elapsed_ms));
              timeout_logged = true;
            }
          }
        }

        if (!planning_future.valid()) {
          path_planner->ClearCancel();
          auto grid_snap = current_grid;
          auto seg_snap = segmentation_grid;
          auto goal_pt = ToVec2(goal.pose.position);
          auto pos_snap = position;
          plan_start = std::chrono::steady_clock::now();
          planning_future = std::async(std::launch::async,
              [grid_snap, seg_snap, goal_pt, pos_snap]() mutable -> std::vector<Point> {
                return path_planner->PlanPath(&grid_snap, &seg_snap, goal_pt, pos_snap);
              });
        }

        last_valid_ros_path.header.stamp = n->get_stamp();
        for (auto& pose : last_valid_ros_path.poses) {
          pose.header.stamp = last_valid_ros_path.header.stamp;
        }
        if (params.use_global_path) {
          path_pub->publish(last_valid_ros_path);
        }
        waypoint_pub->publish(ToPath(nav_goals));

        if (nl % 20 == 0 && params.verbose_gp_log) { // update every second
          auto t_now = std::chrono::system_clock::now();
          auto calc_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t1);
          n->log_info("Global Path [%d]: Pos (%.2f, %.2f) Distance to goal (%.2f, %.2f) for %d of %d = %.2f",
                      calc_duration_ms.count(),
                      odom.pose.pose.position.x,
                      odom.pose.pose.position.y,
                      goal.pose.position.x,
                      goal.pose.position.y,
                      current_waypoint + 1,
                      nav_goals.goals.size(),
                      state.goal_distance);
          t1 = t_now;
        }

        if (state.run_state == NavStackState::Active && IsGoalReached(state, goal)) {
          goal_reached_pub->publish(state);

          const bool terminal_goal = current_waypoint == static_cast<int>(nav_goals.goals.size()) - 1;
          current_waypoint += terminal_goal ? 0 : 1;

          if (terminal_goal) {
            SetRunState(shutdown_behavior);
          }
        }
      }
    } else {
      SetRunState(NavStackState::Active);
    }

    n->spin_some();
    r.sleep();
    nl++;
  }

  if (planning_future.valid()) {
    path_planner->RequestCancel();
    planning_future.get();
  }

  return 0;
}
