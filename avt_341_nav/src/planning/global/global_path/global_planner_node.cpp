/**
 * \file global_planner_node.cpp
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
#include <rclcpp/rclcpp.hpp>
#include "avt_341_nav/node/node_types.h"
#include <avt_341_nav/node/occupancy_grid_subscriber.h>
#include "avt_341_nav/node/tf_interface.h"
#include <future>
// local includes
#include "avt_341_nav/core/compute_time_recorder.hpp"
#include "avt_341_nav/core/coord_transform.hpp"
#include "avt_341_nav/core/math_dto.hpp"
#include "avt_341_nav/core/occupancy_grid_utils.hpp"
#include "avt_341_nav/core/ros_msg_utils.hpp"
#include "avt_341_nav/core/waypoint_file_parser.hpp"
#include "avt_341_nav/planning/global/astar.h"
#include "avt_341_nav/planning/global/fastmarching.h"
#include "avt_341_nav/planning/global/d_star_lite.h"
#include "avt_341_nav/planning/global/fast_marching_square.h"
#include "avt_341_msgs/msg/nav_goal.hpp"
#include "avt_341_msgs/msg/nav_goal_sequence.hpp"
#include "avt_341_msgs/msg/nav_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "avt_341_nav/core/dto_conversion.h"
#include <avt_341_nav/global_planner_params_service.hpp>
#include <avt_341_msgs/srv/compute_global_path.hpp>
#include <chrono>
#include <optional>
#include <stdexcept>

using avt_341_nav::core::NavStackState;
using avt_341_nav::core::IsGoalReached;

using namespace avt_341_nav::core;
using avt_341_nav::planning::Point;

nav_msgs::msg::Odometry odom;
bool odom_rcvd = false;
nav_msgs::msg::OccupancyGrid current_grid;
nav_msgs::msg::OccupancyGrid segmentation_grid;
avt_341_msgs::msg::NavGoalSequence nav_goals;
bool waypoints_rcvd = false;
bool use_global_planner = true;
int nav_command = 0;
bool nav_command_rcvd = false;
rclcpp::Node::SharedPtr n = nullptr;
avt_341_msgs::msg::NavState state;
int current_waypoint = 0;
bool reset_called = false;

double dft_dist_threshold = 0.0f;
double dft_yaw_threshold = 30.0f;

bool costmap_crop_enabled = false;
double costmap_crop_padding = 0.0;

constexpr double WAYPOINT_TRANSFORM_TIMEOUT_S = 5.0;

double goal_start_time = 0.0;
std::shared_ptr<rclcpp::Publisher<avt_341_msgs::msg::NavState>> state_pub = nullptr;
std::shared_ptr<rclcpp::Publisher<avt_341_msgs::msg::NavState>> goal_reached_pub = nullptr;
std::shared_ptr<avt_341_nav::planning::Astar> path_planner = nullptr;
std::shared_ptr<avt_341_nav::core::ComputeTimeRecorder> compute_time_recorder = nullptr;

// Async planning state
std::future<std::vector<avt_341_nav::planning::Point>> planning_future;
std::chrono::steady_clock::time_point plan_start;
bool timeout_logged = false;
// Metadata of the (possibly cropped) grid the in-flight plan runs on; single plan in
// flight at a time, guarded by planning_future.valid()
nav_msgs::msg::MapMetaData plan_grid_info;
std::string plan_grid_frame_id = "map";

void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr rcv_odom)
{
  odom = *rcv_odom;
  odom_rcvd = true;
}

void MapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr rcv_grid)
{
  current_grid = *rcv_grid;
}

void SegmentationMapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr rcv_grid,
                             bool use_segmentation)
{
  if (use_segmentation) {
    segmentation_grid = *rcv_grid;
  }
}

// From mission planner
void WaypointCallback(avt_341_msgs::msg::NavGoalSequence::SharedPtr rcv_waypoints,
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
      RCLCPP_INFO(n->get_logger(), "Empty waypoint sequence received!");
    }else {
      RCLCPP_INFO(n->get_logger(), "%d waypoint(s) received! %.2f, %.2f @ (dist=%.2f, yaw=%.2f)", nav_goals.goals.size(), nav_goals.goals[0].pose.position.x, nav_goals.goals[0].pose.position.y, nav_goals.goals[0].dist_threshold, nav_goals.goals[0].yaw_threshold);
    }
  }
}

void PublishGoalReached(const avt_341_msgs::msg::NavState& msg)
{
  if (avt_341_nav::core::UseGoalOrientation(msg.goal))
  {
    RCLCPP_INFO(n->get_logger(), "Goal reached (%.2f, %.2f, %.2f) @ threshold (dist=%.2f, yaw=%.2f) after %.2f seconds", msg.goal.pose.position.x, msg.goal.pose.position.y, avt_341_nav::core::GetHeadingFromOrientation(msg.goal.pose.orientation)/M_PI*180.0, msg.goal.dist_threshold, msg.goal.yaw_threshold/M_PI*180.0, msg.goal_duration);
  }
  else
  {
    RCLCPP_INFO(n->get_logger(), "Goal reached (%.2f, %.2f) @ threshold (dist=%.2f) after %.2f seconds", msg.goal.pose.position.x, msg.goal.pose.position.y, msg.goal.dist_threshold, msg.goal_duration);
  }

  goal_reached_pub->publish(msg);
}

void GlobalPlannerToggleCallback(std_msgs::msg::Int32::SharedPtr rcv_gptoggle)
{
  bool set_val = (bool)rcv_gptoggle->data;
  if(use_global_planner != set_val){
    RCLCPP_INFO(n->get_logger(), "GP set to %d", rcv_gptoggle->data);
  }
  use_global_planner = set_val;
}

void NavCommandCallback(std_msgs::msg::Int32::SharedPtr rcv_navcommand)
{
  nav_command = rcv_navcommand->data;
  nav_command_rcvd = true;
}

// Needs to remain PoseStamped to support RVIZ goal
void GoalPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr rcv_goal_pose)
{
  const auto nav_goal = ToNavGoal(*rcv_goal_pose, dft_dist_threshold, dft_yaw_threshold);
  RCLCPP_INFO(n->get_logger(), "Setting goal (%.2f, %.2f, %.2f) @ threshold (dist=%.2f, yaw=%.2f)", nav_goal.pose.position.x, nav_goal.pose.position.y, avt_341_nav::core::GetHeadingFromOrientation(nav_goal.pose.orientation)/M_PI*180.0, nav_goal.dist_threshold, nav_goal.yaw_threshold/M_PI*180.0);
  nav_goals.goals.clear();
  nav_goals.goals.push_back(nav_goal);
  waypoints_rcvd = true;
}

void ResetCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data.find(avt_341_nav::node::NodeType::GlobalPlanner) != std::string::npos) {
    reset_called = true;
  }
}

avt_341_msgs::msg::NavGoal GetCurrentGoal()
{
  return current_waypoint < nav_goals.goals.size() ? nav_goals.goals[current_waypoint] : avt_341_msgs::msg::NavGoal();
}

void SetRunState(const int run_state)
{
  state.header.stamp = n->now();
  state.run_state = run_state;
  state_pub->publish(state);
}

void UpdateGoalState(const avt_341_msgs::msg::NavGoal& goal)
{
  state.header.stamp = n->now();

  if (state.run_state == NavStackState::Active) {
    const auto t_now = n->now().seconds();
    if (avt_341_nav::core::GetDistance(goal.pose.position, state.goal.pose.position) > 1e-2) {
      goal_start_time = t_now;
    }
    state.goal = goal;
    double dist_diff, yaw_diff;
    avt_341_nav::core::GetGoalError(odom.pose.pose, goal, dist_diff, yaw_diff);
    state.goal_distance = dist_diff;
    state.goal_yaw_difference = yaw_diff;
    state.goal_duration = t_now - goal_start_time;
  }else {
    state.goal = avt_341_msgs::msg::NavGoal();
    state.goal_distance = 0.0;
    state.goal_yaw_difference = 0.0;
    state.goal_duration = 0.0;
  }
}

void Reset()
{
  RCLCPP_INFO(n->get_logger(), "Resetting node");
  nav_goals.goals.clear();
  current_waypoint = 0;
  UpdateGoalState(avt_341_msgs::msg::NavGoal());
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

// Snapshots the given grid, cropped to the padded bounding box when cropping is enabled.
// Falls back to a full copy when the box misses the grid entirely.
static nav_msgs::msg::OccupancyGrid SnapshotGrid(
    const nav_msgs::msg::OccupancyGrid& src,
    const avt_341_nav::core::WorldAabb& box)
{
  if (costmap_crop_enabled) {
    auto cropped = avt_341_nav::core::CropGridToWorldAabb(src, box);
    if (cropped) {
      return std::move(*cropped);
    }
  }
  return src;
}

// Builds a line-strip marker outlining the extent of the costmap used for planning.
static visualization_msgs::msg::MarkerArray BuildCropBoxMarker(
    const nav_msgs::msg::MapMetaData& info,
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp)
{
  visualization_msgs::msg::Marker box;
  box.header.frame_id = frame_id;
  box.header.stamp = stamp;
  box.ns = "costmap_crop";
  box.id = 0;
  box.type = visualization_msgs::msg::Marker::LINE_STRIP;
  box.action = visualization_msgs::msg::Marker::ADD;
  box.pose.orientation.w = 1.0;
  box.scale.x = 0.3;
  box.color.r = 1.0f;
  box.color.g = 1.0f;
  box.color.b = 0.0f;
  box.color.a = 1.0f;

  const double llx = info.origin.position.x;
  const double lly = info.origin.position.y;
  const double urx = llx + info.width * info.resolution;
  const double ury = lly + info.height * info.resolution;
  //slightly above ground to avoid z-fighting with grid map displays
  const double box_z = 0.2;
  auto add_corner = [&box, box_z](double x, double y) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = box_z;
    box.points.push_back(p);
  };
  add_corner(llx, lly);
  add_corner(urx, lly);
  add_corner(urx, ury);
  add_corner(llx, ury);
  add_corner(llx, lly);

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(box);
  return marker_array;
}

void ComputeGlobalPathServiceCallback(
    const std::shared_ptr<avt_341_msgs::srv::ComputeGlobalPath::Request> request,
    std::shared_ptr<avt_341_msgs::srv::ComputeGlobalPath::Response> response)
{
  nav_msgs::msg::Path full_path;
  full_path.header.stamp = n->now();
  full_path.header.frame_id = "map";

  if (planning_future.valid()) {
    RCLCPP_WARN(n->get_logger(), "ComputeGlobalPath: background planning in progress -- request ignored");
    response->path = full_path;
    return;
  }

  if (request->goals.goals.empty()) {
    RCLCPP_WARN(n->get_logger(), "ComputeGlobalPath: empty goal sequence received");
    response->path = full_path;
    return;
  }

  Point segment_start = ToVec2(request->start_pose.pose);

  const std::size_t num_goals = request->goals.goals.size();
  for (std::size_t i = 0; i < num_goals; ++i) {
    auto& goal = request->goals.goals[i];
    const auto dist_threshold = goal.dist_threshold < 0.0 ? dft_dist_threshold : goal.dist_threshold;
    const Point goal_pt = ToVec2(goal.pose);

    nav_msgs::msg::OccupancyGrid* grid_ptr = &current_grid;
    nav_msgs::msg::OccupancyGrid* seg_ptr = &segmentation_grid;
    nav_msgs::msg::OccupancyGrid grid_crop, seg_crop;
    if (costmap_crop_enabled) {
      const auto crop_box = avt_341_nav::core::MakePaddedAabb(
        segment_start.x, segment_start.y, goal_pt.x, goal_pt.y, costmap_crop_padding);
      grid_crop = SnapshotGrid(current_grid, crop_box);
      seg_crop = SnapshotGrid(segmentation_grid, crop_box);
      grid_ptr = &grid_crop;
      seg_ptr = &seg_crop;
    }

    std::vector<Point> segment = path_planner->PlanPath(
      grid_ptr, seg_ptr, goal_pt, segment_start);

    if (segment.empty()) {
      RCLCPP_WARN(n->get_logger(), "ComputeGlobalPath: planner returned empty segment for goal %zu (%.2f, %.2f)", i, goal_pt.x, goal_pt.y);
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
  RCLCPP_INFO(n->get_logger(), "ComputeGlobalPath: produced path with %zu poses across %zu goals", full_path.poses.size(), num_goals);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  n = rclcpp::Node::make_shared("avt_341_global_path_node");
  auto tf = std::make_shared<avt_341_nav::node::TfInterface>(n);
  avt_341_nav::params::global_planner::ParamsListener param_listener(n);
  const auto params = param_listener.get_params();

  compute_time_recorder = std::make_shared<avt_341_nav::core::ComputeTimeRecorder>(
      n, avt_341_nav::core::ComputeTimeRecorder::MakeNodeTag(n));
  {
    avt_341_nav::core::RunningStatsConfig planning_section_config;
    planning_section_config.window_num_samples = 20;
    for (const auto & section_id : {
             avt_341_nav::planning::planner_sections::PLAN_PATH,
             avt_341_nav::planning::planner_sections::GRID_INGEST,
             avt_341_nav::planning::planner_sections::DILATION,
             avt_341_nav::planning::planner_sections::EDT,
             avt_341_nav::planning::planner_sections::CLEARANCE_SHIFTS,
             avt_341_nav::planning::planner_sections::SOLVE}) {
      compute_time_recorder->Configure(section_id, planning_section_config);
    }
  }

  dft_dist_threshold = params.goal_dist;
  dft_yaw_threshold = params.goal_yaw_threshold;    // Set value > 180.0 degrees to disable
  dft_yaw_threshold *= M_PI / 180.0;

  costmap_crop_enabled = params.costmap_crop.is_enabled;
  costmap_crop_padding = params.costmap_crop.padding;

  int planning_timeout_ms = static_cast<int>(params.planning_timeout_ms);

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> global_path_pre_smooth_pub = nullptr;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> global_path_pre_fill_pub = nullptr;
  if(params.debug_visualize){
    global_path_pre_smooth_pub = n->create_publisher<nav_msgs::msg::Path>("avt_341/global_path_pre_smooth", 10);
    global_path_pre_fill_pub = n->create_publisher<nav_msgs::msg::Path>("avt_341/global_path_pre_fill", 10);
  }

  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> crop_box_pub = nullptr;
  if (costmap_crop_enabled) {
    crop_box_pub = n->create_publisher<visualization_msgs::msg::MarkerArray>("avt_341/global_planner/costmap_crop", 1);
  }

  int shutdown_behavior = static_cast<int>(params.shutdown_behavior);

  if (!avt_341_nav::core::IsValidShutdownBehavior(shutdown_behavior)){
    const std::string error_msg = "Invalid shutdown behavior parameter: " + std::to_string(shutdown_behavior);
    RCLCPP_ERROR(n->get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  RCLCPP_INFO(n->get_logger(), "\nGlobal Planner Settings:\n w_distance: %.2f\n w_occupancy: %.2f\n w_segmentation: %.2f\n method: %s\n clipping_distance: %.2f", params.w_distance, params.w_occupancy, params.w_segmentation, params.planning_method.c_str(), params.clipping_distance);

  auto path_pub = n->create_publisher<nav_msgs::msg::Path>("avt_341/global_path", 1);
  auto waypoint_pub = n->create_publisher<nav_msgs::msg::Path>("avt_341/waypoints", 10);
  goal_reached_pub = n->create_publisher<avt_341_msgs::msg::NavState>("avt_341/goal_reached", 10);

  auto odometry_sub = n->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
  auto map_sub = avt_341_nav::node::OccupancyGridSubscriber(
      n, params.map_topic, 10, params.costmap.publish.method, MapCallback);
  auto segmentation_map_sub = avt_341_nav::node::OccupancyGridSubscriber(
      n, params.seg_topic, 10, params.costmap.publish.method,
      [&params](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        SegmentationMapCallback(msg, params.use_segmentation);
      });
  auto waypoint_sub = n->create_subscription<avt_341_msgs::msg::NavGoalSequence>(
      "avt_341/new_waypoints", 10,
      [&params](avt_341_msgs::msg::NavGoalSequence::SharedPtr msg) {
        WaypointCallback(msg, params.verbose_gp_log);
      });
  auto goal_pose_sub = n->create_subscription<geometry_msgs::msg::PoseStamped>("avt_341/goal_pose", 10, GoalPoseCallback);
  auto gp_toggle_sub = n->create_subscription<std_msgs::msg::Int32>("avt_341/gp_toggle", 10, GlobalPlannerToggleCallback);
  auto nav_command_sub = n->create_subscription<std_msgs::msg::Int32>("avt_341/nav_command_state", 10, NavCommandCallback);
  auto reset_sub = n->create_subscription<std_msgs::msg::String>("avt_341/reset", 10, ResetCallback);
  auto reset_ack_pub = n->create_publisher<std_msgs::msg::String>("avt_341/reset_ack", 1);
  auto fastmatching_costs_pub = n->create_publisher<nav_msgs::msg::OccupancyGrid>("avt_341/fm_map", 1);

  state_pub = n->create_publisher<avt_341_msgs::msg::NavState>("avt_341/state", 10);
  state.run_state = NavStackState::NotInit;
  nav_msgs::msg::Path last_valid_ros_path;
  last_valid_ros_path.header.frame_id = "map";

  Reset();


    nav_msgs::msg::Path waypoints = WaypointFileParser::Parse(params.initial_waypoints);
    const CoordTransformer coord_transformer(tf->get_buffer(), n->get_logger());
    coord_transformer.TransformPath(waypoints, "map", tf2::durationFromSec(WAYPOINT_TRANSFORM_TIMEOUT_S));
    nav_goals = ToNavGoalSequence(waypoints, dft_dist_threshold, dft_yaw_threshold);

  if (!nav_goals.goals.empty()) {
    UpdateGoalState(nav_goals.goals[0]);
    SetRunState(NavStackState::Active);
  }

  if (params.planning_method == "fast_marching") {
    path_planner = std::make_shared<avt_341_nav::planning::FastMarching>(params.w_distance,
                                                       params.w_occupancy,
                                                       params.w_segmentation,
                                                       params.search_diagonals,
                                                       static_cast<int>(params.los_max_iterations),
                                                       params.los_break_on_first,
                                                       params.safety_margin_global,
                                                       params.safety_margin_soft,
                                                       params.clearance_penalty_type,
                                                       params.path_extraction_method,
                                                       params.obstacle_threshold,
                                                       params.clearance_penalty_scale,
                                                       params.clearance_penalty_range,
                                                       params.clearance_penalty_exponent,
                                                       static_cast<int>(params.gradient_descent_max_steps),
                                                       static_cast<int>(params.gradient_descent_steps_per_point),
                                                       params.clipping_distance,
                                                       params.verbose_gp_log,
                                                       params.no_segmentation_data_cost);
  } else if (params.planning_method == "d_star_lite") {
    path_planner = std::make_shared<avt_341_nav::planning::DStarLite>(params.w_distance,
                                                    params.w_occupancy,
                                                    params.w_segmentation,
                                                    params.search_diagonals,
                                                    static_cast<int>(params.los_max_iterations),
                                                    params.los_break_on_first,
                                                    params.no_segmentation_data_cost);
  } else if (params.planning_method == "fast_marching_square") {
    path_planner = std::make_shared<avt_341_nav::planning::FastMarchingSquare>(params.w_distance,
                                                             params.w_occupancy,
                                                             params.w_segmentation,
                                                             params.search_diagonals,
                                                             static_cast<int>(params.los_max_iterations),
                                                             params.los_break_on_first,
                                                             params.safety_margin_global,
                                                             params.safety_margin_soft,
                                                             params.clearance_penalty_type,
                                                             params.path_extraction_method,
                                                             params.obstacle_threshold,
                                                             params.clearance_penalty_scale,
                                                             params.clearance_penalty_range,
                                                             params.clearance_penalty_exponent,
                                                             static_cast<int>(params.gradient_descent_max_steps),
                                                             static_cast<int>(params.gradient_descent_steps_per_point),
                                                             params.clipping_distance,
                                                             params.verbose_gp_log,
                                                             params.no_segmentation_data_cost);
  } else {
    path_planner = std::make_shared<avt_341_nav::planning::Astar>(params.w_distance,
                                                params.w_occupancy,
                                                params.w_segmentation,
                                                params.search_diagonals,
                                                static_cast<int>(params.los_max_iterations),
                                                params.los_break_on_first,
                                                params.no_segmentation_data_cost);
  }

  if (params.dilation_factor > 0.0) {
    path_planner->SetDilationFactor(
        static_cast<int>(params.dilation_factor));
  }

  path_planner->SetComputeTimeRecorder(compute_time_recorder);

  // Service: compute a global path through a sequence of NavGoals starting from a given pose.
  auto compute_global_path_srv =
      n->create_service<avt_341_msgs::srv::ComputeGlobalPath>(
          "avt_341/compute_global_path", &ComputeGlobalPathServiceCallback);

  rclcpp::Rate r(20.0f); // Hz
  int nl = 0;
  int shutdown_count = 0;
  double last_compute_time_pub = 0.0;
  auto t1 = std::chrono::system_clock::now();
  //while (rclcpp::ok() && !goal_reached){
  while (rclcpp::ok()) {

    if (reset_called) {
      if (planning_future.valid()) {
        path_planner->RequestCancel();
        planning_future.get();
        path_planner->ClearCancel();
      }
      Reset();

      last_valid_ros_path.poses.clear();
      last_valid_ros_path.header.stamp = n->now();
      if (params.use_global_path)
        path_pub->publish(last_valid_ros_path);
      SetRunState(NavStackState::NotInit);

      std_msgs::msg::String reset_ack_msg;
      reset_ack_msg.data = avt_341_nav::node::NodeType::GlobalPlanner;
      reset_ack_pub->publish(reset_ack_msg);

      reset_called = false;
      rclcpp::spin_some(n);
      r.sleep();
      continue;
    }

    // Handle Go command
    if (nav_command_rcvd) {
      if (nav_command == avt_341_nav::core::NavStateCmd::GoActive && state.run_state != NavStackState::Active) {
        // startup/idling - go active
        SetRunState(NavStackState::Active);
        nav_command = avt_341_nav::core::NavStateCmd::GoInactive;
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
          RCLCPP_INFO(n->get_logger(), "New waypoints! Updated goal %.2f, %.2f @ (dist=%.2f, yaw=%.2f)", goal.pose.position.x, goal.pose.position.y, goal.dist_threshold, goal.yaw_threshold);
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
                nav_msgs::msg::OccupancyGrid fast_marching_grid;
                //use the metadata of the (possibly cropped) grid the plan actually ran on,
                //which always agrees with the planner dims below; current_grid may have
                //changed since the planning snapshot was taken
                fast_marching_grid.header.frame_id = plan_grid_frame_id;
                fast_marching_grid.header.stamp = n->now();
                fast_marching_grid.info = plan_grid_info;
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
                ros_path_pre_smoothing.header.stamp = n->now();
                global_path_pre_smooth_pub->publish(ros_path_pre_smoothing);

                auto path_pre_fill = path_planner->GetPathWorldPreFill();
                auto ros_path_pre_fill = ToPath(*path_pre_fill);
                ros_path_pre_fill.header.stamp = n->now();
                global_path_pre_fill_pub->publish(ros_path_pre_fill);
              }

              nav_msgs::msg::Path ros_path = ToPath(new_path);
              if (IsGoalReached(state, goal) || ros_path.poses.size() > 1) {
                int cp = current_waypoint;
                while (cp < static_cast<int>(nav_goals.goals.size()) - 1) {
                  avt_341_nav::core::vec2 wp1(static_cast<float>(nav_goals.goals[cp].pose.position.x),
                                           static_cast<float>(nav_goals.goals[cp].pose.position.y));
                  avt_341_nav::core::vec2 wp2(static_cast<float>(nav_goals.goals[cp + 1].pose.position.x),
                                           static_cast<float>(nav_goals.goals[cp + 1].pose.position.y));
                  avt_341_nav::core::vec2 wp_diff = wp2 - wp1;
                  avt_341_nav::core::vec2 wp_diff_norm = wp_diff;
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
              RCLCPP_WARN(n->get_logger(), "Planning timeout (%ldms) -- cancelling and re-publishing previous path", static_cast<long>(elapsed_ms));
              timeout_logged = true;
            }
          }
        }

        if (!planning_future.valid()) {
          path_planner->ClearCancel();
          auto goal_pt = ToVec2(goal.pose.position);
          auto pos_snap = position;
          const auto crop_box = avt_341_nav::core::MakePaddedAabb(
              pos_snap.x, pos_snap.y, goal_pt.x, goal_pt.y, costmap_crop_padding);
          auto grid_snap = SnapshotGrid(current_grid, crop_box);
          auto seg_snap = SnapshotGrid(segmentation_grid, crop_box);

          plan_grid_info = grid_snap.info;
          plan_grid_frame_id = grid_snap.header.frame_id.empty() ? "map" : grid_snap.header.frame_id;
          if (crop_box_pub) {
            crop_box_pub->publish(BuildCropBoxMarker(plan_grid_info, plan_grid_frame_id, n->now()));
          }

          plan_start = std::chrono::steady_clock::now();
          planning_future = std::async(std::launch::async,
              [grid_snap, seg_snap, goal_pt, pos_snap]() mutable -> std::vector<Point> {
                // Recording from the planning thread is safe: the recorder guards its sections with
                // a mutex. Only PublishSummary is kept on the main loop, since it publishes.
                auto recording = compute_time_recorder->RecordScope(
                    avt_341_nav::planning::planner_sections::PLAN_PATH);
                return path_planner->PlanPath(&grid_snap, &seg_snap, goal_pt, pos_snap);
              });
        }

        last_valid_ros_path.header.stamp = n->now();
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
          RCLCPP_INFO(n->get_logger(), "Global Path [%d]: Pos (%.2f, %.2f) Distance to goal (%.2f, %.2f) for %d of %d = %.2f", calc_duration_ms.count(), odom.pose.pose.position.x, odom.pose.pose.position.y, goal.pose.position.x, goal.pose.position.y, current_waypoint + 1, nav_goals.goals.size(), state.goal_distance);
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

    // Published outside the active-planning branch above so that compute times keep flowing while
    // the planner is idle, which is when they are most useful for diagnosing a stall.
    const double now_seconds = n->now().seconds();
    if (params.compute_time_publish_period > 0.0 &&
        now_seconds - last_compute_time_pub >= params.compute_time_publish_period) {
      last_compute_time_pub = now_seconds;
      compute_time_recorder->PublishSummary();
    }

    rclcpp::spin_some(n);
    r.sleep();
    nl++;
  }

  if (planning_future.valid()) {
    path_planner->RequestCancel();
    planning_future.get();
  }

  return 0;
}
