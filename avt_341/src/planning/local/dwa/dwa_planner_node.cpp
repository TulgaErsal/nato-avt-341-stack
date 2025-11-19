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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <avt_341/node/node_proxy.h>
#include <avt_341/node/occupancy_grid_subscriber.h>
#include <avt_341/node/ros_types.h>
#include <avt_341/planning/local/dwa/planner.hpp>
#include <avt_341/visualization/visualization_factory.h>

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
double global_path_lookahead = 15.0;

double state_x = 0.0;
double state_y = 0.0;

// Initialise ROS node parameters.
unsigned int loop_count = 0;

/**
 * @brief Store the AGV odometry and mark it as received.
 *
 * @param msg_rcvd_odom Pointer to the odometry ROS nav_msgs/Odometry message.
 */
void CallbackOdometry(avt_341::msg::OdometryPtr msg_rcvd_odom) {
    msg_odom = *msg_rcvd_odom;
    rcvd_odom = true;
}

/**
 * @brief Store the occupancy grid and mark it as received.
 *
 * @param msg_rcvd_grid Pointer to the occupancy grid ROS nav_msgs/OccupancyGrid
 * message.
 */
void CallbackGridOccupancy(avt_341::msg::OccupancyGridPtr msg_rcvd_grid) {
    msg_grid_occ = *msg_rcvd_grid;
    rcvd_grid_occ = true;
}

/**
 * @brief Store the segmentation grid and mark it as received.
 *
 * @param msg_rcvd_grid Pointer to the segmentation grid ROS
 * nav_msgs/OccupancyGrid message.
 */
void CallbackGridSegmentation(avt_341::msg::OccupancyGridPtr msg_rcvd_grid) {
    msg_grid_seg = *msg_rcvd_grid;
    rcvd_grid_seg = true;
}

/**
 * @brief Store the navigation waypoints and mark them as received.
 *
 * @param msg_rcvd_path Pointer to the navigation waypoints ROS nav_msgs/Path
 * message.
 */
void CallbackWaypoints(avt_341::msg::PathPtr msg_rcvd_path) {
    msg_waypoints = *msg_rcvd_path;
    rcvd_path = true;
}

void CallbackWaypoint(avt_341::msg::PoseStampedPtr msg_rcvd_waypoint_pose) {
    msg_waypoint_pose = *msg_rcvd_waypoint_pose;
    rcvd_waypoint = true;
}

/**
 * @brief Store the global path and mark it as received.
 *
 * @param msg_rcvd_path Pointer to the global path ROS nav_msgs/Path message.
 */
void CallbackPath(avt_341::msg::PathPtr msg_rcvd_path) {
    msg_path = *msg_rcvd_path;
    rcvd_path = true;
}

void UpdateState(avt_341::planning::dwa::Planner& planner) {
    // Initialise the pose orientation quaternion.
    tf2::Quaternion orientation(msg_odom.pose.pose.orientation.x,
                                msg_odom.pose.pose.orientation.y,
                                msg_odom.pose.pose.orientation.z,
                                msg_odom.pose.pose.orientation.w);

    // Get the rotation matrix from the quaternion.
    double roll, pitch, yaw;
    tf2::Matrix3x3 rotation(orientation);
    rotation.getRPY(roll, pitch, yaw);

    state_x = msg_odom.pose.pose.position.x;
    state_y = msg_odom.pose.pose.position.y;

    // Update the AGV state.
    planner.SetState(msg_odom.pose.pose.position.x,
                     msg_odom.pose.pose.position.y,
                     yaw,
                     msg_odom.twist.twist.linear.x,
                     msg_odom.twist.twist.angular.z);
}

void UpdateGoal(avt_341::planning::dwa::Planner& planner) {
    double goal_x, goal_y;

    if(planner.GetUseGlobalPath()) {
        if(use_current_waypoint && rcvd_waypoint) {
            goal_x = msg_waypoint_pose.pose.position.x;
            goal_y = msg_waypoint_pose.pose.position.y;
        } else {
            double min_distance = global_path_lookahead;
            int optimal_pose_index = 0;
            for(int i = 0; i < int(msg_path.poses.size()); ++i) {
                auto curr_distance =
                    std::hypot(state_x - msg_path.poses[i].pose.position.x,
                               state_y - msg_path.poses[i].pose.position.y);
                if(curr_distance > min_distance) {
                    optimal_pose_index = i;
                    break;
                }
            }

            // Set the goal to the last pose in the global path.
            // TODO: Integrate with the mission planner to pass the next mission
            // waypoint as goal.
            goal_x = msg_path.poses[optimal_pose_index].pose.position.x;
            goal_y = msg_path.poses[optimal_pose_index].pose.position.y;
        }

        // Initialise a new global path in the planner and populate it with the
        // global path poses.
        avt_341::planning::dwa::Path path;
        for(auto& pose : msg_path.poses) {
            path.Add(pose.pose.position.x, pose.pose.position.y);
        }

        planner.SetGlobalPath(path);
    } else {
        if(use_current_waypoint && rcvd_waypoint) {
            goal_x = msg_waypoint_pose.pose.position.x;
            goal_y = msg_waypoint_pose.pose.position.y;
        } else {
            // Set the goal to the last waypoint in the list of waypoints.
            // TODO: Integrate with the mission planner to pass the next mission
            // waypoint as goal.
            goal_x = msg_waypoints.poses.back().pose.position.x;
            goal_y = msg_waypoints.poses.back().pose.position.y;
        }
    }

    // Set the planner goal waypoint.
    planner.SetGoal(goal_x, goal_y);
}

void UpdateGrids(avt_341::planning::dwa::Planner& planner) {
    if(rcvd_grid_occ) {
        planner.SetOccupancyGridWidth(msg_grid_occ.info.width);
        planner.SetOccupancyGridHeight(msg_grid_occ.info.height);
        planner.SetOccupancyGridOriginX(msg_grid_occ.info.origin.position.x);
        planner.SetOccupancyGridOriginY(msg_grid_occ.info.origin.position.y);
        planner.SetOccupancyGridResolution(msg_grid_occ.info.resolution);
        planner.SetOccupancyGridData(msg_grid_occ.data);
    }

    if(rcvd_grid_seg) {
        // TODO: Add support for segmentation grids differing in size from the
        // occupancy grid.
        planner.SetSegmentationGridData(msg_grid_seg.data);
    }
}

void ResetCallback(avt_341::msg::StringPtr msg) {
    if(msg->data.find(avt_341::node::NodeType::LocalPlanner) !=
       std::string::npos) {
        reset_called = true;
    }
}

avt_341::msg::MarkerArray
GetClearMarkersMessage(std::shared_ptr<avt_341::node::NodeProxy> node) {
    avt_341::msg::Marker delete_marker;
    delete_marker.header.stamp = node->get_stamp();
    delete_marker.header.frame_id = "map";
    delete_marker.ns = "paths";
    delete_marker.action = avt_341::msg::Marker::DELETEALL;

    avt_341::msg::MarkerArray marker_array_message;
    marker_array_message.markers.push_back(delete_marker);

    return marker_array_message;
}

avt_341::msg::MarkerArray
GetTrajectoryMarkersMessage(std::shared_ptr<avt_341::node::NodeProxy> node,
                            const avt_341::planning::dwa::Planner& planner) {
    auto trajectories = planner.GetTrajectories();
    avt_341::msg::MarkerArray marker_array_message;
    for(size_t trajectory_index = 0; trajectory_index < trajectories.size();
        ++trajectory_index) {
        avt_341::msg::Marker trajectory_marker_message;
        trajectory_marker_message.header.stamp = node->get_stamp();
        trajectory_marker_message.header.frame_id = "map";
        trajectory_marker_message.ns = "dwa/paths";
        trajectory_marker_message.id = trajectory_index;
        trajectory_marker_message.type = avt_341::msg::Marker::LINE_STRIP;
        trajectory_marker_message.action = avt_341::msg::Marker::ADD;
        trajectory_marker_message.pose.position.x = 0.0;
        trajectory_marker_message.pose.position.y = 0.0;
        trajectory_marker_message.pose.position.z = 0.0;
        trajectory_marker_message.pose.orientation.x = 0.0;
        trajectory_marker_message.pose.orientation.x = 0.0;
        trajectory_marker_message.pose.orientation.x = 0.0;
        trajectory_marker_message.pose.orientation.w = 1.0;
        trajectory_marker_message.color.r =
            (trajectories[trajectory_index].GetTotalCost() - planner.GetMinCost())/
            (planner.GetMaxCost() - planner.GetMinCost());
        trajectory_marker_message.color.g = 1.0 -
            (trajectories[trajectory_index].GetTotalCost() - planner.GetMinCost()) /
                (planner.GetMaxCost() - planner.GetMinCost());
        trajectory_marker_message.color.b = 0.0;
        trajectory_marker_message.color.a = 1.0 -
            (trajectories[trajectory_index].GetTotalCost() - planner.GetMinCost()) /
                (planner.GetMaxCost() - planner.GetMinCost());
        trajectory_marker_message.scale.x = 0.05;
        trajectory_marker_message.scale.y = 1.0;
        trajectory_marker_message.scale.z = 1.0;

        for(int state_index = 0;
            state_index < trajectories[trajectory_index].GetNumberOfStates();
            ++state_index) {
            avt_341::msg::Point state_point_message;
            state_point_message.x =
                trajectories[trajectory_index].GetState(state_index).GetX();
            state_point_message.y =
                trajectories[trajectory_index].GetState(state_index).GetY();
            trajectory_marker_message.points.push_back(state_point_message);
        }

        marker_array_message.markers.push_back(trajectory_marker_message);
    }

    return marker_array_message;
}

avt_341::msg::DwaInfo
GetInfoMessage(std::shared_ptr<avt_341::node::NodeProxy> node,
               avt_341::planning::dwa::Planner& planner) {
    avt_341::msg::DwaInfo info_message;
    info_message.header.frame_id = "dwa";
    info_message.header.stamp = node->get_stamp();
    for(auto& trajectory : planner.GetTrajectories()) {
        info_message.planned_trajectories.push_back(
            trajectory.GetROSTrajectoryMessage());
    }
    info_message.optimal_trajectory =
        planner.GetOptimalTrajectory().GetROSTrajectoryMessage();
    return info_message;
}

int main(int argc, char* argv[]) {
    // Initialize ROS node.
    auto node =
        avt_341::node::init_node(argc, argv, "avt_341_dwa_planner_node");

#ifndef USE_OPENMP
    node->log_warning("DWA planner was not compiled with OpenMP enabled. "
                      "Planning will be significantly slower.");
#endif


    // Create node subscribers.
    auto sub_odom =
        node->create_subscription<avt_341::msg::Odometry>("avt_341/odometry",
                                                          1,
                                                          CallbackOdometry);
    avt_341::node::OccupancyGridSubscriber sub_grid_occ(node,
        "avt_341/occupancy_grid",
        1,
        CallbackGridOccupancy);
    avt_341::node::OccupancyGridSubscriber sub_grid_seg(node,
        "avt_341/segmentation_grid",
        1,
        CallbackGridSegmentation);
    auto sub_path =
        node->create_subscription<avt_341::msg::Path>("avt_341/global_path",
                                                      1,
                                                      CallbackPath);
    auto sub_waypoint = node->create_subscription<avt_341::msg::PoseStamped>(
        "avt_341/current_waypoint",
        1,
        CallbackWaypoint);
    auto sub_waypoints =
        node->create_subscription<avt_341::msg::Path>("avt_341/waypoints",
                                                      1,
                                                      CallbackWaypoints);
    auto reset_sub =
        node->create_subscription<avt_341::msg::String>("avt_341/reset",
                                                        1,
                                                        ResetCallback);
    auto reset_ack_pub =
        node->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);

    // Create node publishers.
    auto pub_path =
        node->create_publisher<avt_341::msg::Path>("avt_341/local_path", 1);
    auto pub_ctrl_speed =
        node->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed",
                                                      1);
    auto pub_ctrl_steer =
        node->create_publisher<avt_341::msg::Float64>("avt_341/cmd_steer", 1);
    auto pub_ctrl_drive =
        node->create_publisher<avt_341::msg::AckermannDriveStamped>(
            "avt_341/drive",
            1);
    auto pub_markers =
        node->create_publisher<avt_341::msg::MarkerArray>("avt_341/markers",
                                                          1);
    auto pub_info =
        node->create_publisher<avt_341::msg::DwaInfo>("avt_341/dwa/info",
                                                          1);

    // Declare and read node parameters from the ROS parameter server.
    bool use_segmentation;
    node->get_parameter("~dwa_use_segmentation", use_segmentation, false);


    node->get_parameter("~dwa_use_current_waypoint",
                        use_current_waypoint,
                        false);

    node->get_parameter("~dwa_global_path_lookahead",
                        global_path_lookahead,
                        15.0);

    // Initialise and configure the dynamic window approach (DWA) planner.
    avt_341::planning::dwa::Planner planner;
    planner.SetHorizon(node->get_parameter("~dwa_horizon", std::string("adaptive")));
    planner.SetWindowLinearSpeedMin(node->get_parameter("~dwa_speed_lin_min", 0.15));
    planner.SetWindowLinearSpeedMax(node->get_parameter("~dwa_speed_lin_max", 4.0));
    planner.SetWindowLinearSpeedSteps(node->get_parameter("~dwa_speed_lin_steps", 10));
    planner.SetWindowAngularSpeedMin(node->get_parameter("~dwa_speed_ang_min", -0.58));
    planner.SetWindowAngularSpeedMax(node->get_parameter("~dwa_speed_ang_max", 0.58));
    planner.SetWindowAngularSpeedSteps(node->get_parameter("~dwa_speed_ang_steps", 40));
    planner.SetWindowAccelerationMax(node->get_parameter("~dwa_accel_max", 3.0));
    planner.SetWindowAngularAccelerationMax(node->get_parameter("~dwa_ang_accel_max", 4.0));
    planner.SetLateralAccelerationMax(node->get_parameter("~dwa_lat_accel_max", 9.81));
    planner.SetTimeStep(node->get_parameter("~dwa_time_step_min", 0.2));
    planner.SetWindowTimeSpanMin(node->get_parameter("~dwa_time_span_min", 2.5));
    planner.SetWindowTimeSpanMax(node->get_parameter("~dwa_time_span_max", 10.0));
    planner.SetWindowTimeSpanVariable(node->get_parameter("~dwa_time_span_var", 4.5));
    planner.SetWindowTimeSpanGain(node->get_parameter("~dwa_time_span_gain", 1.1));
    planner.SetCostGoalWeight(node->get_parameter("~dwa_w_cost_goal", 1.0));
    planner.SetCostHeadingWeight(node->get_parameter("~dwa_w_cost_head", 0.001));
    planner.SetCostSpeedWeight(node->get_parameter("~dwa_w_cost_speed", 0.0));
    planner.SetCostObstacleWeight(node->get_parameter("~dwa_w_cost_obs", 1.5));
    planner.SetCostSegmentationWeight(node->get_parameter("~dwa_w_cost_seg", 0.0));
    planner.SetCostGlobalPathWeight(node->get_parameter("~dwa_w_cost_path", 0.0));
    planner.SetCostDeviationWeight(node->get_parameter("~dwa_w_cost_dev", 0.75));
    planner.SetObstacleThreshold(node->get_parameter("~dwa_thresh_obs", 0));
    planner.SetCollisionRadius(node->get_parameter("~dwa_collision_radius", 2.25));
    planner.SetObstacleSearch(node->get_parameter("~dwa_obs_search", std::string("fixed")));
    planner.SetObstacleSearchRadius(node->get_parameter("~dwa_search_radius", 10.0));
    planner.SetVehicleWheelbase(node->get_parameter("~dwa_wheelbase", 2.72));
    planner.SetUseSegmentation(use_segmentation);
    planner.SetSegmentationThreshold(node->get_parameter("~dwa_thresh_seg", 100));
    planner.SetUseGlobalPath(node->get_parameter("~dwa_use_global_path", false));
    planner.SetPrintSummary(node->get_parameter("~dwa_print_summary", false));

    // Set the node spin rate to 50 Hz.
    avt_341::node::Rate rosrate(50.0f);

    while(avt_341::node::ok()) {
        if(msg_path.poses.size() > 0 && rcvd_odom &&
           msg_grid_occ.data.size() > 0) {
            if(use_segmentation && !(msg_grid_seg.data.size() > 0)) break;
            // Update the planner with the latest information.
            UpdateState(planner);
            UpdateGoal(planner);
            UpdateGrids(planner);

            // Run the planning step.
            planner.Plan();

            auto optimal_trajectory = planner.GetOptimalTrajectory();

            // Serialise and publish the local path.
            avt_341::msg::Path msg_path = planner.GetPlannedPathRos();
            msg_path.header.frame_id = "map";
            msg_path.header.stamp = node->get_stamp();
            avt_341::node::set_seq(msg_path.header, loop_count);
            pub_path->publish(msg_path);

            // Serialise and publish the target speed.
            avt_341::msg::Float64 msg_ctrl_speed;
            msg_ctrl_speed.data = planner.GetPlannedLinearSpeed();
            pub_ctrl_speed->publish(msg_ctrl_speed);

            // Serialise and publish the target steering angle.
            avt_341::msg::Float64 msg_ctrl_steer;
            msg_ctrl_steer.data = planner.GetPlannedAngularSpeed();
            pub_ctrl_steer->publish(msg_ctrl_steer);

            avt_341::msg::AckermannDriveStamped msg_ctrl_drive;
            msg_ctrl_drive.header.frame_id = "avt_341";
            msg_ctrl_drive.header.stamp = node->get_stamp();
            msg_ctrl_drive.drive.speed = planner.GetPlannedLinearSpeed();
            msg_ctrl_drive.drive.steering_angle =
                planner.GetPlannedAngularSpeed();
            pub_ctrl_drive->publish(msg_ctrl_drive);

            auto clear_markers_message = GetClearMarkersMessage(node);
            pub_markers->publish(clear_markers_message);

            auto trajectory_markers_message = GetTrajectoryMarkersMessage(node, planner);
            pub_markers->publish(trajectory_markers_message);
        
            auto info_message = GetInfoMessage(node, planner);
            pub_info->publish(info_message);
        }

        if(reset_called) {
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
