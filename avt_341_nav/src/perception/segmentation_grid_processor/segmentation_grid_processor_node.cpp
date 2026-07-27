#include <vector>
#include <avt_341/node/occupancy_grid_subscriber.h>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <avt_341/mpc_local_planner_params_service.hpp>

// Global variables
rclcpp::Node::SharedPtr node;

double max_speed;

double cell_size_meters = 0.0;

avt_341::params::mpc_local_planner::Params node_params;

rclcpp::Time init_time;

rclcpp::Time vehicle_odom_input_stamp;

rclcpp::Time last_vehicle_odom_stamp;

nav_msgs::msg::Odometry vehicle_odom_input;

nav_msgs::msg::OccupancyGrid segmentation_grid_input;

std::vector<double> cells;

std::vector<visualization_msgs::msg::Marker> cell_markers;

void callback_veh(nav_msgs::msg::Odometry::SharedPtr veh) {
    vehicle_odom_input = *veh;
    vehicle_odom_input_stamp = vehicle_odom_input.header.stamp;
}

void callback_seg(nav_msgs::msg::OccupancyGrid::SharedPtr obs) {
    segmentation_grid_input = *obs;
}

void callback_speed(std_msgs::msg::Float64::SharedPtr speed) {
    max_speed = speed->data;
}

bool new_input_available(const nav_msgs::msg::OccupancyGrid& grid, const nav_msgs::msg::Odometry& vehicle_odom) {
    if (vehicle_odom_input_stamp == last_vehicle_odom_stamp || grid.header.stamp == init_time
        || vehicle_odom_input_stamp == init_time) {
        return false;
    }

    last_vehicle_odom_stamp = vehicle_odom.header.stamp;

    auto q = vehicle_odom.pose.pose.orientation;
    double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
    double x_vehicle = vehicle_odom.pose.pose.position.x +
        node_params.vehicle_axle_distance_front * cos(yaw);  // x position of front axle
    double y_vehicle = vehicle_odom.pose.pose.position.y +
        node_params.vehicle_axle_distance_front * sin(yaw);  // y position of front axle

    // Rotation matrix
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix[0] = {cos(yaw), -sin(yaw), 0.0};
    rotation_matrix[1] = {sin(yaw), cos(yaw), 0.0};
    rotation_matrix[2] = {0.0, 0.0, 1.0};

    // Observational region determined by right/left vectors
    tf2::Vector3 left_vector = {
        cos(node_params.front_angle_segmentation),
        sin(node_params.front_angle_segmentation), 0};  // 45deg to left
    tf2::Vector3 right_vector = {
        cos(node_params.front_angle_segmentation),
        -sin(node_params.front_angle_segmentation), 0}; // 45deg to right
    auto left_boundary_vector = rotation_matrix * left_vector;
    auto right_boundary_vector = rotation_matrix * right_vector;

    cell_size_meters = grid.info.resolution;
    cells.clear();
    cell_markers.clear();
    int cell_number = 0;
    for (int i = 0; i < grid.info.height; i++) {
        for (int j = 0; j < grid.info.width; j++) {
            float cell_val = grid.data[i*grid.info.width + j];
            if (cell_val > 0.0) {
                std::vector<double> point = {(j + 0.5) * grid.info.resolution + grid.info.origin.position.x,
                                             (i + 0.5) * grid.info.resolution + grid.info.origin.position.y};
                tf2::Vector3 cell_vector = {point[0] - x_vehicle, point[1] - y_vehicle, 0};

                // Add obstacle if it is within range of prediction time horizon driving distance or within observation region
                if (cell_vector.length() >
                        (node_params.prediction_time_horizon + 0.1) * max_speed
                    || cell_vector[0] * left_boundary_vector[1] - left_boundary_vector[0] * cell_vector[1] < 0
                    ||  // Only comparing last element of cross product vector,
                        cell_vector[0] * right_boundary_vector[1] - right_boundary_vector[0] * cell_vector[1]
                            > 0)  // gives same result as cross(a, b)[2].
                {
                    continue;
                } else {
                    cell_number = cell_number + 1;
                    cells.push_back(point[0]);
                    cells.push_back(point[1]);
                    cells.push_back(cell_val);
                    if (node_params.obstacles_vizualize) {
                        visualization_msgs::msg::Marker cell_marker;
                        cell_marker.header.frame_id = "map";
                        cell_marker.header.stamp = node->now();
                        cell_marker.id = cell_number;
                        cell_marker.type = visualization_msgs::msg::Marker::CUBE;
                        cell_marker.action = visualization_msgs::msg::Marker::ADD;
                        cell_marker.scale.x = cell_size_meters;
                        cell_marker.scale.y = cell_size_meters;
                        cell_marker.scale.z = cell_size_meters;
                        float cell_color = 1.0-(cell_val / 100.0);
                        cell_marker.color.a = 1.0;
                        cell_marker.color.r = cell_color;
                        cell_marker.color.g = cell_color;
                        cell_marker.color.b = cell_color;
                        cell_marker.pose.position.x = point[0];
                        cell_marker.pose.position.y = point[1];
                        cell_marker.pose.position.z = 0.0;
                        cell_markers.push_back(cell_marker);
                    }
                }
            }
        }
    }
    return true;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("segmentation_processor_node");
    double rate = 10.0;
    rclcpp::Rate ros_rate(rate);

    init_time = node->now();
    segmentation_grid_input.header.stamp = init_time;
    vehicle_odom_input_stamp = init_time;
    last_vehicle_odom_stamp = init_time;

    // Load parameters
    avt_341::params::mpc_local_planner::ParamsListener param_listener(node);
    node_params = param_listener.get_params();
    max_speed = node_params.max_speed;

    // Create publishers and subscribers
    auto seg_grid_sub = avt_341::node::OccupancyGridSubscriber(
        node, "avt_341/segmentation_grid", 1, node_params.costmap.publish.method, callback_seg);
    auto odometry_sub = node->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry", 1, callback_veh);
    auto speed_sub = node->create_subscription<std_msgs::msg::Float64>("avt_341/speed_setpoint", 1, callback_speed);
    auto cells_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("avt_341/segmentation_cells", 1);
    auto cell_marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("avt_341/cell_markers", 1);

    while (rclcpp::ok()) {
        if (new_input_available(segmentation_grid_input, vehicle_odom_input)) {
            // Publish obstacle message
            std_msgs::msg::Float64MultiArray cells_msg;
            cells_msg.data = cells;
            cells_pub->publish(cells_msg);

            if (node_params.obstacles_vizualize) {
                visualization_msgs::msg::MarkerArray cell_marker_msg;
                cell_marker_msg.markers = cell_markers;
                cell_marker_pub->publish(cell_marker_msg);
            }
        }

        rclcpp::spin_some(node);
        ros_rate.sleep();
    }

    return 0;
}
