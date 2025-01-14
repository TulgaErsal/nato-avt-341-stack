/**
 * \file avt_341_dwa_planner_node.cpp
 * Plan a local trajectory using the dynamic window approach planner
 *
 * \author Dario Sirangelo
 *
 * \contact dsirangelo@aarhusdynamics.com
 *
 * \date 1/23/2023
*/

#include "avt_341/node/node_proxy.h"
#include "avt_341/node/ros_types.h"
#include "avt_341/planning/local/dwa_planner.h"
#include "avt_341/visualization/visualization_factory.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

// Initialise ROS messages.
avt_341::msg::Odometry msg_odom;
avt_341::msg::OccupancyGrid msg_grid_occ;
avt_341::msg::OccupancyGrid msg_grid_seg;
avt_341::msg::Path msg_waypoints;
avt_341::msg::PoseStamped msg_waypoint_pose;
avt_341::msg::Path msg_path;

// Initialise receive flags.
bool rcvd_odom = false;
bool rcvd_grid_occ = false;
bool rcvd_grid_seg = false;
bool rcvd_path = false;
bool rcvd_waypoint = false;
bool reset_called = false;
bool use_current_waypoint = false;

double state_x = 0.0;
double state_y = 0.0;
double global_path_lookahead = 15.0;

// Initialise ROS node parameters.
unsigned int loop_count = 0;

/**
 * @brief Store the AGV odometry and mark it as received.
 *
 * @param msg_rcvd_odom Pointer to the odometry ROS nav_msgs/Odometry message.
 */
void
CallbackOdometry(avt_341::msg::OdometryPtr msg_rcvd_odom) {
    msg_odom = *msg_rcvd_odom;
    rcvd_odom = true;
}

/**
 * @brief Store the occupancy grid and mark it as received.
 *
 * @param msg_rcvd_grid Pointer to the occupancy grid ROS nav_msgs/OccupancyGrid message.
 */
void
CallbackGridOccupancy(avt_341::msg::OccupancyGridPtr msg_rcvd_grid) {
    msg_grid_occ = *msg_rcvd_grid;
    rcvd_grid_occ = true;
}

/**
 * @brief Store the segmentation grid and mark it as received.
 *
 * @param msg_rcvd_grid Pointer to the segmentation grid ROS nav_msgs/OccupancyGrid message.
 */
void
CallbackGridSegmentation(avt_341::msg::OccupancyGridPtr msg_rcvd_grid) {
    msg_grid_seg = *msg_rcvd_grid;
    rcvd_grid_seg = true;
}

/**
 * @brief Store the navigation waypoints and mark them as received.
 *
 * @param msg_rcvd_path Pointer to the navigation waypoints ROS nav_msgs/Path message.
 */
void
CallbackWaypoints(avt_341::msg::PathPtr msg_rcvd_path){
    msg_waypoints = *msg_rcvd_path;
    rcvd_path = true;
}

void
CallbackWaypoint(avt_341::msg::PoseStampedPtr msg_rcvd_waypoint_pose){
    msg_waypoint_pose = *msg_rcvd_waypoint_pose;
    rcvd_waypoint = true;
}

/**
 * @brief Store the global path and mark it as received.
 *
 * @param msg_rcvd_path Pointer to the global path ROS nav_msgs/Path message.
 */
void
CallbackPath(avt_341::msg::PathPtr msg_rcvd_path) {
    msg_path = *msg_rcvd_path;
    rcvd_path = true;
}

void
UpdateState(avt_341::planning::DwaPlanner& planner) {
    // Initialise the pose orientation quaternion.
    tf2::Quaternion orientation(
        msg_odom.pose.pose.orientation.x,
        msg_odom.pose.pose.orientation.y,
        msg_odom.pose.pose.orientation.z,
        msg_odom.pose.pose.orientation.w
    );

    // Get the rotation matrix from the quaternion.
    // NOTE: getRPY() expects a double, hence we cast back to float when setting the state.
    double roll, pitch, yaw;
    tf2::Matrix3x3 rotation(orientation);
    rotation.getRPY(roll, pitch, yaw);

    state_x = msg_odom.pose.pose.position.x;
    state_y = msg_odom.pose.pose.position.y;

    // Update the AGV state.
    planner.SetState(
        msg_odom.pose.pose.position.x,
        msg_odom.pose.pose.position.y,
        (float)yaw,
        msg_odom.twist.twist.linear.x,
        msg_odom.twist.twist.angular.z
    );
}

void
UpdateGoal(avt_341::planning::DwaPlanner& planner) {
    float goal_x, goal_y;

    if (planner.GetUseGlobalPath()) {
        if (use_current_waypoint && rcvd_waypoint) {
            goal_x = msg_waypoint_pose.pose.position.x;
            goal_y = msg_waypoint_pose.pose.position.y;
        } else {
            double min_distance = global_path_lookahead;
            int optimal_pose_index = 0;
            for (int i = 0; i < int(msg_path.poses.size()); ++i) {
                auto curr_distance = std::hypot(state_x - msg_path.poses[i].pose.position.x, state_y - msg_path.poses[i].pose.position.y);
                if(curr_distance > min_distance) { optimal_pose_index = i; break; }
            }

            // Set the goal to the last pose in the global path.
            // TODO: Integrate with the mission planner to pass the next mission waypoint as goal.
            goal_x = msg_path.poses[optimal_pose_index].pose.position.x;
            goal_y = msg_path.poses[optimal_pose_index].pose.position.y;
        }

        // Initialise a new global path in the planner and populate it with the global path poses.
        avt_341::planning::DwaPath path;
        for (auto& pose : msg_path.poses) {
            path.Add(pose.pose.position.x, pose.pose.position.y);
        }

        planner.SetGlobalPath(path);
    } else {
        if (use_current_waypoint && rcvd_waypoint) {
            goal_x = msg_waypoint_pose.pose.position.x;
            goal_y = msg_waypoint_pose.pose.position.y;
        } else {
            // Set the goal to the last waypoint in the list of waypoints.
            // TODO: Integrate with the mission planner to pass the next mission waypoint as goal.
            goal_x = msg_waypoints.poses.back().pose.position.x;
            goal_y = msg_waypoints.poses.back().pose.position.y;
        }
    }

    // Set the planner goal waypoint.
    planner.SetGoal(goal_x, goal_y);
}

void
UpdateGrids(avt_341::planning::DwaPlanner& planner) {
    if (rcvd_grid_occ) {
        planner.SetOccupancyGridWidth(msg_grid_occ.info.width);
        planner.SetOccupancyGridHeight(msg_grid_occ.info.height);
        planner.SetOccupancyGridOriginX(msg_grid_occ.info.origin.position.x);
        planner.SetOccupancyGridOriginY(msg_grid_occ.info.origin.position.y);
        planner.SetOccupancyGridResolution(msg_grid_occ.info.resolution);
        planner.SetOccupancyGridData(msg_grid_occ.data);
    }

    if (rcvd_grid_seg) {
        // TODO: Add support for segmentation grids differing in size from the occupancy grid.
        planner.SetSegmentationGridData(msg_grid_seg.data);
    }
}

void ResetCallback(avt_341::msg::StringPtr msg){
  if(msg->data.find(avt_341::node::NodeType::LocalPlanner) != std::string::npos){
    reset_called = true;
  }
}

int
main(int argc, char* argv[]) {
    // Initialize ROS node.
    auto node = avt_341::node::init_node(argc, argv, "avt_341_dwa_planner_node");

    // Create node subscribers.
    auto sub_odom = node->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, CallbackOdometry);
    auto sub_grid_occ = node->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 10, CallbackGridOccupancy);
    auto sub_grid_seg = node->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 10, CallbackGridSegmentation);
    auto sub_path = node->create_subscription<avt_341::msg::Path>("avt_341/global_path", 10, CallbackPath);
    auto sub_waypoint = node->create_subscription<avt_341::msg::PoseStamped>("avt_341/current_waypoint", 10, CallbackWaypoint);
    auto sub_waypoints = node->create_subscription<avt_341::msg::Path>("avt_341/waypoints", 10, CallbackWaypoints);
    auto reset_sub = node->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
    auto reset_ack_pub = node->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);

    // Create node publishers.
    auto pub_path = node->create_publisher<avt_341::msg::Path>("avt_341/local_path", 10);
    auto pub_ctrl_speed = node->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed", 10);
    auto pub_ctrl_steer = node->create_publisher<avt_341::msg::Float64>("avt_341/cmd_steer", 10);
    auto pub_ctrl_drive = node->create_publisher<avt_341::msg::AckermannDriveStamped>("avt_341/drive", 10);
    auto pub_markers = node->create_publisher<avt_341::msg::MarkerArray>("avt_341/markers", 10);

    // Declare and read node parameters from the ROS parameter server.
    float wheelbase; node->get_parameter("~dwa_wheelbase", wheelbase, 2.72f);
    float speed_lin_min; node->get_parameter("~dwa_speed_lin_min", speed_lin_min, 0.15f);
    float speed_lin_max; node->get_parameter("~dwa_speed_lin_max", speed_lin_max, 4.0f);
    int speed_lin_steps; node->get_parameter("~dwa_speed_lin_steps", speed_lin_steps, 10);
    float accel_max; node->get_parameter("~dwa_accel_max", accel_max, 3.0f);
    float speed_ang_min; node->get_parameter("~dwa_speed_ang_min", speed_ang_min, -0.58f);
    float speed_ang_max; node->get_parameter("~dwa_speed_ang_max", speed_ang_max, 0.58f);
    int speed_ang_steps; node->get_parameter("~dwa_speed_ang_steps", speed_ang_steps, 40);
    float ang_accel_max; node->get_parameter("~dwa_ang_accel_max", ang_accel_max, 4.0f);
    float lat_accel_max; node->get_parameter("~dwa_lat_accel_max", lat_accel_max, 9.81f);
    std::string horizon; node->get_parameter("~dwa_horizon", horizon, std::string("adaptive"));
    float time_span_min; node->get_parameter("~dwa_time_span_min", time_span_min, 2.5f);
    float time_span_max; node->get_parameter("~dwa_time_span_max", time_span_max, 10.0f);
    float time_span_var; node->get_parameter("~dwa_time_span_var", time_span_var, 4.5f);
    float time_span_gain; node->get_parameter("~dwa_time_span_gain", time_span_gain, 1.1f);
    float time_step_min; node->get_parameter("~dwa_time_step_min", time_step_min, 0.2f);
    float w_cost_goal; node->get_parameter("~dwa_w_cost_goal", w_cost_goal, 1.0f);
    float w_cost_head; node->get_parameter("~dwa_w_cost_head", w_cost_head, 0.001f);
    int thresh_obs; node->get_parameter("~dwa_thresh_obs", thresh_obs, 0);
    float collision_radius; node->get_parameter("~dwa_collision_radius", collision_radius, 2.25f);
    std::string obs_search; node->get_parameter("~dwa_obs_search", obs_search, std::string("fixed"));
    float search_radius; node->get_parameter("~dwa_search_radius", search_radius, 10.0f);
    float w_cost_obs; node->get_parameter("~dwa_w_cost_obs", w_cost_obs, 1.5f);
    float w_cost_speed; node->get_parameter("~dwa_w_cost_speed", w_cost_speed, 0.0f);
    bool use_global_path; node->get_parameter("~dwa_use_global_path", use_global_path, false);
    float w_cost_path; node->get_parameter("~dwa_w_cost_path", w_cost_path, 0.0f);
    bool use_segmentation; node->get_parameter("~dwa_use_segmentation", use_segmentation, false);
    float w_cost_seg; node->get_parameter("~dwa_w_cost_seg", w_cost_seg, 0.0f);
    int thresh_seg; node->get_parameter("~dwa_thresh_seg", thresh_seg, 100);
    float w_cost_dev; node->get_parameter("~dwa_w_cost_dev", w_cost_dev, 0.75f);
    bool print_summary; node->get_parameter("~dwa_print_summary", print_summary, false);
    node->get_parameter("~dwa_use_current_waypoint", use_current_waypoint, false);
    node->get_parameter("~dwa_global_path_lookahead", global_path_lookahead, 15.0);

    // Initialise and configure the dynamic window approach (DWA) planner.
    avt_341::planning::DwaPlanner planner;
    planner.SetHorizon(horizon);
    planner.SetWindowLinearSpeedMin(speed_lin_min);
    planner.SetWindowLinearSpeedMax(speed_lin_max);
    planner.SetWindowLinearSpeedSteps(speed_lin_steps);
    planner.SetWindowAngularSpeedMin(speed_ang_min);
    planner.SetWindowAngularSpeedMax(speed_ang_max);
    planner.SetWindowAngularSpeedSteps(speed_ang_steps);
    planner.SetWindowAccelerationMax(accel_max);
    planner.SetWindowAngularAccelerationMax(ang_accel_max);
    planner.SetLateralAccelerationMax(lat_accel_max);
    planner.SetWindowTimeStepMin(time_step_min);
    planner.SetWindowTimeSpanMin(time_span_min);
    planner.SetWindowTimeSpanMax(time_span_max);
    planner.SetWindowTimeSpanVariable(time_span_var);
    planner.SetWindowTimeSpanGain(time_span_gain);
    planner.SetCostGoalWeight(w_cost_goal);
    planner.SetCostHeadingWeight(w_cost_head);
    planner.SetCostSpeedWeight(w_cost_speed);
    planner.SetCostObstacleWeight(w_cost_obs);
    planner.SetCostSegmentationWeight(w_cost_seg);
    planner.SetCostGlobalPathWeight(w_cost_path);
    planner.SetCostDeviationWeight(w_cost_dev);
    planner.SetObstacleThreshold(thresh_obs);
    planner.SetCollisionRadius(collision_radius);
    planner.SetObstacleSearch(obs_search);
    planner.SetObstacleSearchRadius(search_radius);
    planner.SetVehicleWheelbase(wheelbase);
    planner.SetUseSegmentation(use_segmentation);
    planner.SetSegmentationThreshold(thresh_seg);
    planner.SetUseGlobalPath(use_global_path);
    planner.SetPrintSummary(print_summary);

    // Set the node spin rate to 50 Hz.
    avt_341::node::Rate rosrate(50.0f);

    while (avt_341::node::ok()) {
        if (msg_path.poses.size() > 0 && rcvd_odom && msg_grid_occ.data.size() > 0) {
            if (use_segmentation && !(msg_grid_seg.data.size() > 0)) break;
            // Update the planner with the latest information.            
            UpdateState(planner);
            UpdateGoal(planner);
            UpdateGrids(planner);

            // Run the planning step.
            planner.Plan();
            auto trajectories = planner.GetTrajectories();

            // Serialise and publish the local path.
            avt_341::msg::Path msg_path = planner.GetPlannedPathRos();
            msg_path.header.frame_id = "map";
            msg_path.header.stamp = node->get_stamp();
            avt_341::node::set_seq(msg_path.header, loop_count);
            pub_path->publish(msg_path);

            // Serialise and publish the target speed.
            float speed = planner.GetPlannedLinearSpeed();
            avt_341::msg::Float64 msg_ctrl_speed;
            msg_ctrl_speed.data = speed;
            pub_ctrl_speed->publish(msg_ctrl_speed);

            // Serialise and publish the target steering angle.
            float steer = planner.GetPlannedAngularSpeed();
            avt_341::msg::Float64 msg_ctrl_steer;
            msg_ctrl_steer.data = steer;
            pub_ctrl_steer->publish(msg_ctrl_steer);

            avt_341::msg::AckermannDriveStamped msg_ctrl_drive;
            msg_ctrl_drive.header.frame_id = "avt_341";
            msg_ctrl_drive.header.stamp = node->get_stamp();
            msg_ctrl_drive.drive.speed = speed;
            msg_ctrl_drive.drive.steering_angle = steer;
            pub_ctrl_drive->publish(msg_ctrl_drive);

            avt_341::msg::Marker delete_marker;
            delete_marker.header.stamp = node->get_stamp();
            delete_marker.header.frame_id = "map";
            delete_marker.ns = "paths";
            delete_marker.action = avt_341::msg::Marker::DELETEALL;

            avt_341::msg::MarkerArray delete_markers;
            delete_markers.markers.push_back(delete_marker);
            pub_markers->publish(delete_markers);

            avt_341::msg::MarkerArray msg_marker_array;
            for(int i = 0; i < trajectories.size(); ++i) {
                avt_341::msg::Marker msg_marker;
                msg_marker.header.stamp = node->get_stamp();
                msg_marker.header.frame_id = "map";
                msg_marker.ns = "dwa/paths";
                msg_marker.id = i;
                msg_marker.type = avt_341::msg::Marker::LINE_STRIP;
                msg_marker.action = avt_341::msg::Marker::ADD;
                msg_marker.pose.position.x = 0.0;
                msg_marker.pose.position.y = 0.0;
                msg_marker.pose.position.z = 0.0;
                msg_marker.pose.orientation.x = 0.0;
                msg_marker.pose.orientation.x = 0.0;
                msg_marker.pose.orientation.x = 0.0;
                msg_marker.pose.orientation.w = 1.0;
                msg_marker.color.r = trajectories[i].GetCost() / planner.GetMaxCost();
                msg_marker.color.g = 1.0 - trajectories[i].GetCost() / planner.GetMaxCost();
                msg_marker.color.b = 0.0;
                msg_marker.color.a = 1.0 - 1.0 - trajectories[i].GetCost() / planner.GetMaxCost();
                msg_marker.scale.x = 0.05;
                msg_marker.scale.y = 1.0;
                msg_marker.scale.z = 1.0;
                
                for (int ipose = 0; ipose < trajectories[i].GetNumberOfStates(); ++ipose) {
                    avt_341::msg::Point point;
                    point.x = trajectories[i].GetState(ipose).GetX();
                    point.y = trajectories[i].GetState(ipose).GetY();
                    msg_marker.points.push_back(point);
                    
                }
                msg_marker_array.markers.push_back(msg_marker);
                
            }
            pub_markers->publish(msg_marker_array);

        }

        if(reset_called){
          planner.Reset();
          avt_341::msg::String reset_ack_msg;
          reset_ack_msg.data = avt_341::node::NodeType::LocalPlanner;
          reset_ack_pub->publish(reset_ack_msg);
          reset_called = false;
        }

        // Advance the sequence counter.
        loop_count++;

        // Reset the received odometry flag.
        rcvd_odom = false;

        // Run the ROS node at the specified rate.
        node->spin_some();
        rosrate.sleep();
    }

    return EXIT_SUCCESS;
}
