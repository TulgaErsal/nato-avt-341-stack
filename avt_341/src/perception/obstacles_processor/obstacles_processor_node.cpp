#include <vector>
#include <algorithm>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

// Global variables
std::shared_ptr<avt_341::node::NodeProxy> node;

int max_obstacle_number;

double max_speed;

double prediction_time_horizon;

double axle_distance_front;

double obstacle_size_meters = 0.0;

double obstacles_angle = 0.707107;

bool viz = false;

avt_341::msg::Time init_time;

avt_341::msg::Time vehicle_odom_input_stamp;

avt_341::msg::Time last_vehicle_odom_stamp;

avt_341::msg::Odometry vehicle_odom_input;

avt_341::msg::OccupancyGrid occupancy_grid_input;

std::vector<std::vector<bool>> obstacles;

std::vector<std::vector<bool>> cluster_mask;

std::vector<double> obstacles_origin;

std::vector<double> obstacles_clustered;

std::vector<avt_341::msg::Marker> obstacle_markers;

double prediction_horizon;

void callback_veh(avt_341::msg::OdometryPtr veh) {
    vehicle_odom_input = *veh;
    vehicle_odom_input_stamp = vehicle_odom_input.header.stamp;
}

void callback_obs(avt_341::msg::OccupancyGridPtr obs) {
    occupancy_grid_input = *obs;
}

void callback_speed(avt_341::msg::Float64Ptr speed_setpoint) {
    max_speed = speed_setpoint->data;
    prediction_horizon = (prediction_time_horizon + 0.1) * max_speed;
}

// Helper function for cluster_occupied_cells
bool is_occupied(double x, double y, double r) {
    int px = (int)((x-obstacles_origin[0])/r);
    int py = (int)((y-obstacles_origin[1])/r);
    if (px < 0 || py < 0 || px >= obstacles.size() || py >= obstacles[0].size()) {
        return false;
    }
    return obstacles[px][py] && !cluster_mask[px][py];
}

// Helper function for cluster_occupied_cells
double largest_square(double x, double y, double r) {
    double max_size = 0;
    bool max_size_found = false;
    while (!max_size_found) {
        max_size+=1.0;
        for (int i=0; i<max_size; i++) {
            if (!is_occupied(x+i*r, y+max_size*r, r) 
                    || !is_occupied(x+max_size*r, y+i*r, r)) {
                max_size_found = true;
                return max_size;
            }
        }
    }
}

std::vector<double> cluster_occupied_cells(std::vector<std::vector<double>> cells, double r) {
    std::vector<std::vector<double>> points = cells;
    
    // Reset cluster mask
    cluster_mask.clear();
    std::vector<bool> obstacle_row;
    obstacle_row.resize(obstacles.size(), false);
    cluster_mask.resize(obstacles.size(), obstacle_row);

    std::vector<double> output;
    while (!points.empty()) {
        // Check obstacle limit
        if (int(output.size() / 3) >= max_obstacle_number) {
            std::cerr << "Number of obstacles exceeds limit ("<<int(obstacles_clustered.size() / 3)<<">"<<max_obstacle_number<<"). Consider increasing max_obstacle_number.\n";
        }
        
        // Find largest occupied square
        double x = points[0][0];
        double y = points[0][1];
        double max_size = largest_square(x,y,r) * r;

        // Push clustered obstacle to output
        output.push_back(x + (max_size - r) / 2.0);
        output.push_back(y + (max_size - r) / 2.0);
        output.push_back(max_size);

        // Update clustered cells
        int xi1 = std::max(0,std::min((int)cluster_mask.size(),(int)((x-obstacles_origin[0])/r)));
        int yi1 = std::max(0,std::min((int)cluster_mask.size(),(int)((y-obstacles_origin[1])/r)));
        int xi2 = std::max(0,std::min((int)cluster_mask.size(),(int)((x-obstacles_origin[0]+max_size)/r)));
        int yi2 = std::max(0,std::min((int)cluster_mask.size(),(int)((y-obstacles_origin[1]+max_size)/r)));
        for (int i=xi1; i<xi2; i++) {
            for (int j=yi1; j<yi2; j++) {
                cluster_mask[i][j] = true;
            }
        }

        std::vector<std::vector<double>> new_points;
        for (auto pt: points) {
            if (!((pt[0] >= x && pt[0] < x+max_size) && (pt[1] >= y && pt[1] < y+max_size))) {
                new_points.push_back(pt);
            }
        }
        points = new_points;
    }

    return output;
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

    // Rotation matrix
    avt_341::msg_tf::Matrix3x3 rotation_matrix;
    rotation_matrix[0] = {cos(yaw), -sin(yaw), 0.0};
    rotation_matrix[1] = {sin(yaw), cos(yaw), 0.0};
    rotation_matrix[2] = {0.0, 0.0, 1.0};

    // Observational region determined by right/left vectors
    avt_341::msg_tf::Vector3 left_vector = {cos(obstacles_angle), sin(obstacles_angle), 0};  // 45deg to left
    avt_341::msg_tf::Vector3 right_vector = {cos(obstacles_angle), -sin(obstacles_angle), 0}; // 45deg to right
    auto left_boundary_vector = rotation_matrix * left_vector;
    auto right_boundary_vector = rotation_matrix * right_vector;

    obstacle_size_meters = grid.info.resolution;
    int poi_width = (int)(prediction_horizon / obstacle_size_meters * 2.0);
    obstacles_origin = {x_vehicle-prediction_horizon,y_vehicle-prediction_horizon};
    std::vector<std::vector<double>> obstacle_cells;
    obstacles.clear();
    std::vector<bool> obstacle_row;
    obstacle_row.resize(poi_width, false);
    obstacles.resize(poi_width, obstacle_row);
    obstacle_markers.clear();
    int obstacle_number = 0;
    for (int i = 0; i < grid.info.height; i++) {
        for (int j = 0; j < grid.info.width; j++) {
            if (grid.data[i * grid.info.width + j] > 0.0) {
                std::vector<double> point = {(j + 0.5) * grid.info.resolution + grid.info.origin.position.x,
                                             (i + 0.5) * grid.info.resolution + grid.info.origin.position.y};
                avt_341::msg_tf::Vector3 obstacle = {point[0] - x_vehicle, point[1] - y_vehicle, 0};

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

    // Cluster obstacles
    obstacles_clustered = cluster_occupied_cells(obstacle_cells, obstacle_size_meters);
    if (viz) {
        for (int i=0; i < obstacles_clustered.size()/3; i++) {
            double x = obstacles_clustered[3*i];
            double y = obstacles_clustered[3*i+1];
            double obs_size = obstacles_clustered[3*i+2];

            avt_341::msg::Marker obs_marker;
            obs_marker.header.frame_id = "map";
            obs_marker.header.stamp = node->get_stamp();
            obs_marker.id = i;
            obs_marker.type = avt_341::msg::Marker::CUBE;
            obs_marker.action = avt_341::msg::Marker::ADD;
            obs_marker.scale.x = obs_size;
            obs_marker.scale.y = obs_size;
            obs_marker.scale.z = obs_size;
            obs_marker.color.a = 1.0;
            obs_marker.color.r = 1.0;
            obs_marker.color.g = 0.0;
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
    node = avt_341::node::init_node(argc, argv, "occupancy_processor_node");
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
    auto speed_sub = node->create_subscription<avt_341::msg::Float64>("avt_341/speed_setpoint", 1, callback_speed);
    auto obstacle_clusters_pub = node->create_publisher<avt_341::msg::Float64MultiArray>("avt_341/obstacle_clusters", 1);
    auto obstacles_marker_pub = node->create_publisher<avt_341::msg::MarkerArray>("avt_341/obstacle_markers", 1);

    // Load parameters
    node->get_parameter("~max_num_obs", max_obstacle_number, 1000);
    node->get_parameter("~max_speed", max_speed, 10.0);
    node->get_parameter("~prediction_time_horizon", prediction_time_horizon, 2.0);
    node->get_parameter("~vehicle_axle_distance_front", axle_distance_front, 1.5521);
    node->get_parameter("~front_angle_obstacle", obstacles_angle, 0.707107);
    node->get_parameter("~obstacles_vizualize", viz, false);

    int count = 0;
    prediction_horizon = (prediction_time_horizon + 0.1) * max_speed;

    while (avt_341::node::ok()) {
        if (new_input_available(occupancy_grid_input, vehicle_odom_input)) {
            // Publish obstacle clusters message
            avt_341::msg::Float64MultiArray obs_cluster_msg;
            obs_cluster_msg.data = obstacles_clustered;
            obstacle_clusters_pub->publish(obs_cluster_msg);

            if (viz) {
                avt_341::msg::MarkerArray obs_marker_msg;
                obs_marker_msg.markers = obstacle_markers;
                obstacles_marker_pub->publish(obs_marker_msg);
            }

            count++;
        }

        node->spin_some();
        ros_rate.sleep();
    }

    return 0;
}
