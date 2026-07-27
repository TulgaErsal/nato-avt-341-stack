#include <cmath>
#include <vector>
#include <algorithm>
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

double obstacle_size_meters = 0.0;

avt_341::params::mpc_local_planner::Params node_params;

rclcpp::Time init_time;

rclcpp::Time vehicle_odom_input_stamp;

rclcpp::Time last_vehicle_odom_stamp;

nav_msgs::msg::Odometry vehicle_odom_input;

nav_msgs::msg::OccupancyGrid occupancy_grid_input;

nav_msgs::msg::OccupancyGrid segmentation_grid_input;

std::vector<std::vector<bool>> obstacles;

std::vector<std::vector<bool>> cluster_mask;

std::vector<double> obstacles_origin;

std::vector<double> obstacles_clustered;

std::vector<visualization_msgs::msg::Marker> obstacle_markers;

double prediction_horizon;

void callback_veh(nav_msgs::msg::Odometry::SharedPtr veh) {
    vehicle_odom_input = *veh;
    vehicle_odom_input_stamp = vehicle_odom_input.header.stamp;
}

void callback_obs(nav_msgs::msg::OccupancyGrid::SharedPtr obs) {
    occupancy_grid_input = *obs;
}

void callback_seg(nav_msgs::msg::OccupancyGrid::SharedPtr seg) {
    segmentation_grid_input = *seg;
}

void callback_speed(std_msgs::msg::Float64::SharedPtr speed_setpoint) {
    max_speed = speed_setpoint->data;
    prediction_horizon =
        (node_params.prediction_time_horizon + 0.1) * max_speed;
}

// Helper function for cluster_occupied_cells
bool is_occupied(std::vector<std::vector<bool>>& obs, double xi, double yi) {
    if (xi < 0 || yi < 0 || xi >= obs.size() || yi >= obs[0].size()) {
        return false;
    }
    return obs[xi][yi];
}

// Helper function for cluster_occupied_cells
int largest_square(std::vector<std::vector<bool>>& obs, double xi, double yi) {
    int max_size = 0;
    bool max_size_found = false;
    while (!max_size_found) {
        if (!is_occupied(obs, xi, yi+max_size) 
                || !is_occupied(obs, xi+max_size, yi)
                || !is_occupied(obs, xi+max_size, yi+max_size)) {
            max_size_found = true;
            return max_size;
        }
        max_size++;
    }
}

std::vector<double> cluster_occupied_cells(double r) {
    std::vector<std::vector<bool>> obs = obstacles;

    std::vector<double> output;
    for(int xi = 0; xi < obs.size(); xi++) {
        for(int yi = 0; yi < obs[0].size(); yi++) {
            // Check obstacle limit
            if (int(output.size() / 3) >= node_params.max_num_obs) {
                std::cerr << "Number of obstacles exceeds limit ("
                          << int(output.size() / 3) << ">"
                          << node_params.max_num_obs
                          << "). Consider increasing max_num_obs.\n";
                return output;
            }

            if (!obs[xi][yi]) continue;

            // Find largest occupied square
            int max_size = (double)largest_square(obs,xi,yi);
            double max_size_m = (double)max_size*r;

            // Calculate centroid
            double x = (double)xi*r + obstacles_origin[0] + max_size_m / 2.0;
            double y = (double)yi*r + obstacles_origin[1] + max_size_m / 2.0;

            // Push obstacle
            output.push_back(x);
            output.push_back(y);
            output.push_back(max_size_m);

            // Update clustered cells
            int xi1 = std::max(0,std::min((int)obs.size(),xi));
            int yi1 = std::max(0,std::min((int)obs[0].size(),yi));
            int xi2 = std::max(0,std::min((int)obs.size(),xi+max_size));
            int yi2 = std::max(0,std::min((int)obs[0].size(),yi+max_size));
            for (int i=xi1; i<xi2; i++) {
                for (int j=yi1; j<yi2; j++) {
                    obs[i][j] = false;
                }
            }
        }
    }
    return output;
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
        cos(node_params.front_angle_obstacle),
        sin(node_params.front_angle_obstacle), 0};  // 45deg to left
    tf2::Vector3 right_vector = {
        cos(node_params.front_angle_obstacle),
        -sin(node_params.front_angle_obstacle), 0}; // 45deg to right
    auto left_boundary_vector = rotation_matrix * left_vector;
    auto right_boundary_vector = rotation_matrix * right_vector;

    obstacle_size_meters = grid.info.resolution;
    int poi_width = (int)(prediction_horizon / obstacle_size_meters * 2.0);
    // Snap origin to grid resolution so that cell→index→world reconstruction
    // is exact regardless of vehicle position (avoids cluster-center drift).
    const double r = obstacle_size_meters;
    obstacles_origin = {std::floor((x_vehicle - prediction_horizon) / r) * r,
                        std::floor((y_vehicle - prediction_horizon) / r) * r};
    std::vector<std::vector<double>> obstacle_cells;
    obstacles.clear();
    std::vector<bool> obstacle_row;
    obstacle_row.resize(poi_width, false);
    obstacles.resize(poi_width, obstacle_row);
    obstacle_markers.clear();
    int obstacle_number = 0;
    bool isObstacle = false;
    for (int i = 0; i < grid.info.height; i++) {
        for (int j = 0; j < grid.info.width; j++) {
            isObstacle = grid.data[i * grid.info.width + j] > 0.0; 
        
            if (node_params.project_segmentation_onto_occupancy_grid) {
                // Compute the world coordinates of the current occupancy grid cell
                double world_x = (j + 0.5) * grid.info.resolution + grid.info.origin.position.x;
                double world_y = (i + 0.5) * grid.info.resolution + grid.info.origin.position.y;
            
                // Compute corresponding indices in the segmentation grid
                int seg_j = (int)((world_x - segmentation_grid_input.info.origin.position.x) / segmentation_grid_input.info.resolution);
                int seg_i = (int)((world_y - segmentation_grid_input.info.origin.position.y) / segmentation_grid_input.info.resolution);
            
                // Check if calculated indices are within the bounds of the segmentation grid
                // If yes, check if the cell is non-traversable
                // If yes, consider as obstacle
                if (seg_i >= 0 && seg_i < segmentation_grid_input.info.height && seg_j >= 0 && seg_j < segmentation_grid_input.info.width) {
                    isObstacle = isObstacle ||
                        (segmentation_grid_input.data[
                            seg_i * segmentation_grid_input.info.width +
                            seg_j] < node_params.traversability_threshold);
                }
            }
        
            if (isObstacle) {
                std::vector<double> point = {(j + 0.5) * grid.info.resolution + grid.info.origin.position.x,
                                             (i + 0.5) * grid.info.resolution + grid.info.origin.position.y};
                tf2::Vector3 obstacle = {point[0] - x_vehicle, point[1] - y_vehicle, 0};

                // Add obstacle if it is within range of prediction time horizon driving distance or within observation region
                if (obstacle.length() >
                        (node_params.prediction_time_horizon + 0.1) * max_speed
                    || obstacle[0] * left_boundary_vector[1] - left_boundary_vector[0] * obstacle[1] < 0
                    ||  // Only comparing last element of cross product vector,
                        obstacle[0] * right_boundary_vector[1] - right_boundary_vector[0] * obstacle[1]
                            > 0)  // gives same result as cross(a, b)[2].
                {
                    continue;
                } else {
                    obstacle_number = obstacle_number + 1;
                    int px = (int)((point[0]-obstacles_origin[0]) / obstacle_size_meters);
                    int py = (int)((point[1]-obstacles_origin[1]) / obstacle_size_meters);
                    if (px < 0 || py < 0 || px >= obstacles.size() || py >= obstacles[0].size()) {
                        continue;
                    }
                    obstacles[px][py] = true;
                    obstacle_cells.push_back({point[0],point[1]});
                }
            }
        }
    }

    // Clear all previous markers
    visualization_msgs::msg::Marker obs_marker_clear;
    obs_marker_clear.header.frame_id = "map";
    obs_marker_clear.header.stamp = node->now();
    obs_marker_clear.id = 0;
    obs_marker_clear.action = visualization_msgs::msg::Marker::DELETEALL;
    obstacle_markers.push_back(obs_marker_clear);

    // Cluster obstacles
    obstacles_clustered = cluster_occupied_cells(obstacle_size_meters);
    if (node_params.obstacles_vizualize) {
        for (int i=0; i < obstacles_clustered.size()/3; i++) {
            double x = obstacles_clustered[3*i];
            double y = obstacles_clustered[3*i+1];
            double obs_size = obstacles_clustered[3*i+2];

            visualization_msgs::msg::Marker obs_marker;
            obs_marker.header.frame_id = "map";
            obs_marker.header.stamp = node->now();
            obs_marker.id = i + 1; // 0 is reserved for the DELETEALL marker
            obs_marker.type = visualization_msgs::msg::Marker::CUBE;
            obs_marker.action = visualization_msgs::msg::Marker::ADD;
            obs_marker.scale.x = obs_size;
            obs_marker.scale.y = obs_size;
            obs_marker.scale.z = obs_size;
            obs_marker.color.a = 1.0;
            obs_marker.color.r = 1.0;
            obs_marker.color.g = 0.5;
            obs_marker.color.b = 0.0;
            obs_marker.pose.position.x = x;
            obs_marker.pose.position.y = y;
            obs_marker.pose.position.z = 0.0;
            obstacle_markers.push_back(obs_marker);
        }
    }

    return true;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("occupancy_processor_node");
    double rate = 30.0;
    rclcpp::Rate ros_rate(rate);

    init_time = node->now();
    occupancy_grid_input.header.stamp = init_time;
    vehicle_odom_input_stamp = init_time;
    last_vehicle_odom_stamp = init_time;

    // Load parameters
    avt_341::params::mpc_local_planner::ParamsListener param_listener(node);
    node_params = param_listener.get_params();
    max_speed = node_params.max_speed;

    // Create publishers and subscribers
    auto occupancy_grid_sub = avt_341::node::OccupancyGridSubscriber(
        node, "avt_341/occupancy_grid", 10, node_params.costmap.publish.method, callback_obs);
    auto seg_grid_sub = avt_341::node::OccupancyGridSubscriber(
        node, "avt_341/segmentation_grid", 1, node_params.costmap.publish.method, callback_seg);
    auto odometry_sub = node->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry", 10, callback_veh);
    auto speed_sub = node->create_subscription<std_msgs::msg::Float64>("avt_341/speed_setpoint", 1, callback_speed);
    auto obstacle_clusters_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("avt_341/obstacle_clusters", 1);
    auto obstacles_marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("avt_341/obstacle_markers", 1);

    int count = 0;
    prediction_horizon =
        (node_params.prediction_time_horizon + 0.1) * max_speed;

    while (rclcpp::ok()) {
        if (new_input_available(occupancy_grid_input, vehicle_odom_input)) {
            // Publish obstacle clusters message
            std_msgs::msg::Float64MultiArray obs_cluster_msg;
            obs_cluster_msg.data = obstacles_clustered;
            obstacle_clusters_pub->publish(obs_cluster_msg);

            if (node_params.obstacles_vizualize) {
                visualization_msgs::msg::MarkerArray obs_marker_msg;
                obs_marker_msg.markers = obstacle_markers;
                obstacles_marker_pub->publish(obs_marker_msg);
            }

            count++;
        }

        rclcpp::spin_some(node);
        ros_rate.sleep();
    }

    return 0;
}
