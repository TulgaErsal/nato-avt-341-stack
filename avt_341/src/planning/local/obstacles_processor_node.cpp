#include <vector>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

// Global variables
int max_obstacle_number;

double max_speed;

double prediction_time_horizon;

double axle_distance_front;

double obstacle_size_meters = 0.0;

avt_341::msg::Time init_time;

avt_341::msg::Time vehicle_odom_input_stamp;

avt_341::msg::Time last_vehicle_odom_stamp;

avt_341::msg::Odometry vehicle_odom_input;

avt_341::msg::OccupancyGrid occupancy_grid_input;

std::vector<double> obstacles;

void callback_veh(avt_341::msg::OdometryPtr veh) {
    vehicle_odom_input = *veh;
    vehicle_odom_input_stamp = vehicle_odom_input.header.stamp;
}

void callback_obs(avt_341::msg::OccupancyGridPtr obs) {
    occupancy_grid_input = *obs;
}

bool new_input_available(const avt_341::msg::OccupancyGrid& grid, const avt_341::msg::Odometry& vehicle_odom) {
    if (vehicle_odom_input_stamp == last_vehicle_odom_stamp || grid.header.stamp == init_time
        || vehicle_odom_input_stamp == init_time) {
        return false;
    }

    last_vehicle_odom_stamp = vehicle_odom.header.stamp;

    auto q = vehicle_odom.pose.pose.orientation;
    double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
    double x_vehicle = vehicle_odom.pose.pose.position.x + axle_distance_front * cos(yaw);  // x position of front axle
    double y_vehicle = vehicle_odom.pose.pose.position.y + axle_distance_front * sin(yaw);  // y position of front axle

    avt_341::msg_tf::Matrix3x3 rotation_matrix;
    obstacle_size_meters = grid.info.resolution;
    obstacles.clear();
    int obstacle_number = 0;
    for (int i = 0; i < grid.info.height; i++) {
        for (int j = 0; j < grid.info.width; j++) {
            if (grid.data[i * grid.info.width + j] > 0.0 && obstacle_number < max_obstacle_number) {
                std::vector<double> point = {(j + 0.5) * grid.info.resolution + grid.info.origin.position.x,
                                             (i + 0.5) * grid.info.resolution + grid.info.origin.position.y};
                avt_341::msg_tf::Vector3 obstacle = {point[0] - x_vehicle, point[1] - y_vehicle, 0};

                // Rotation matrix
                rotation_matrix[0] = {cos(yaw), -sin(yaw), 0.0};
                rotation_matrix[1] = {sin(yaw), cos(yaw), 0.0};
                rotation_matrix[2] = {0.0, 0.0, 1.0};

                // Observational region determined by right/left vectors
                avt_341::msg_tf::Vector3 left_vector = {0.707107, 0.707107, 0};  // 45deg to left
                avt_341::msg_tf::Vector3 right_vector = {0.707107, -0.707107, 0}; // 45deg to right
                auto left_boundary_vector = rotation_matrix * left_vector;
                auto right_boundary_vector = rotation_matrix * right_vector;

                // Add obstacle if it is within range of prediction time horizon driving distance or within observation region
                if (obstacle.length() > (prediction_time_horizon + 0.1) * max_speed
                    || obstacle[0] * left_boundary_vector[1] - left_boundary_vector[0] * obstacle[1] < 0
                    ||  // Only comparing last element of cross product vector,
                        obstacle[0] * right_boundary_vector[1] - right_boundary_vector[0] * obstacle[1]
                            > 0)  // gives same result as cross(a, b)[2].
                {
                    continue;
                } else {
                    obstacle_number = obstacle_number + 1;
                    obstacles.push_back(point[0]);
                    obstacles.push_back(point[1]);
                    if (int(obstacles.size() / 2) == max_obstacle_number) {
                        std::cerr << "Number of obstacles exceeds limit. Consider increasing max_obstacle_number.\n";
                    }
                    if (int(obstacles.size() / 2) > max_obstacle_number) {
                        std::cerr << "Number of obstacles exceeds limit. Consider increasing max_obstacle_number.\n";
                    }
                }
            }
        }
    }
    return true;
}

int main(int argc, char* argv[]) {
    auto node = avt_341::node::init_node(argc, argv, "occupancy_processor_node");
    double rate = 10.0;
    avt_341::node::Rate ros_rate(rate);

    init_time = node->get_stamp();
    occupancy_grid_input.header.stamp = init_time;
    vehicle_odom_input_stamp = init_time;
    last_vehicle_odom_stamp = init_time;

    // Create publishers and subscribers
    auto occupancy_grid_sub =
        node->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 10, callback_obs);
    auto odometry_sub = node->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, callback_veh);
    auto obstacles_pub = node->create_publisher<avt_341::msg::Obstacles>("avt_341/obstacles", 1);

    // Load parameters
    node->get_parameter("~mpc_obstacles_max_obstacle_number", max_obstacle_number, 1000);
    node->get_parameter("~mpc_bounds_longitudinal_speed_max", max_speed, 10.0);
    node->get_parameter("~mpc_solver_time_span", prediction_time_horizon, 2.0);
    node->get_parameter("~mpc_vehicle_axle_distance_front", axle_distance_front, 1.5521);

    int count = 0;

    while (avt_341::node::ok()) {
        if (new_input_available(occupancy_grid_input, vehicle_odom_input)) {
            avt_341::msg::Obstacles obs_msg;
            obs_msg.id = count;
            obs_msg.obstacle_size_meters = obstacle_size_meters;
            obs_msg.data = obstacles;

            // Publish obstacle message
            obstacles_pub->publish(obs_msg);

            count++;
        }

        node->spin_some();
        ros_rate.sleep();
    }

    return 0;
}
