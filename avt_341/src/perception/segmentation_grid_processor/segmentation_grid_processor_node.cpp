#include <vector>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

// Global variables
std::shared_ptr<avt_341::node::NodeProxy> node;

double max_speed;

double prediction_time_horizon;

double axle_distance_front;

double cell_size_meters = 0.0;

double segmentation_angle = 0.707107;

bool viz = false;

avt_341::msg::Time init_time;

avt_341::msg::Time vehicle_odom_input_stamp;

avt_341::msg::Time last_vehicle_odom_stamp;

avt_341::msg::Odometry vehicle_odom_input;

avt_341::msg::OccupancyGrid segmentation_grid_input;

std::vector<double> cells;

std::vector<avt_341::msg::Marker> cell_markers;

void callback_veh(avt_341::msg::OdometryPtr veh) {
    vehicle_odom_input = *veh;
    vehicle_odom_input_stamp = vehicle_odom_input.header.stamp;
}

void callback_seg(avt_341::msg::OccupancyGridPtr obs) {
    segmentation_grid_input = *obs;
}

void callback_speed(avt_341::msg::Float64Ptr speed) {
    max_speed = speed->data;
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
    avt_341::msg_tf::Vector3 left_vector = {cos(segmentation_angle), sin(segmentation_angle), 0};  // 45deg to left
    avt_341::msg_tf::Vector3 right_vector = {cos(segmentation_angle), -sin(segmentation_angle), 0}; // 45deg to right
    auto left_boundary_vector = rotation_matrix * left_vector;
    auto right_boundary_vector = rotation_matrix * right_vector;

    cell_size_meters = grid.info.resolution;
    cells.clear();
    cell_markers.clear();
    int cell_number = 0;
    for (int i = 0; i < grid.info.height; i++) {
        for (int j = 0; j < grid.info.width; j++) {
            float cell_val = 100.0-grid.data[i*grid.info.width + j];
            if (cell_val > 0.0) {
                std::vector<double> point = {(j + 0.5) * grid.info.resolution + grid.info.origin.position.x,
                                             (i + 0.5) * grid.info.resolution + grid.info.origin.position.y};
                avt_341::msg_tf::Vector3 cell_vector = {point[0] - x_vehicle, point[1] - y_vehicle, 0};

                // Add obstacle if it is within range of prediction time horizon driving distance or within observation region
                if (cell_vector.length() > (prediction_time_horizon + 0.1) * max_speed
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
                    if (viz) {
                        avt_341::msg::Marker cell_marker;
                        cell_marker.header.frame_id = "map";
                        cell_marker.header.stamp = node->get_stamp();
                        cell_marker.id = cell_number;
                        cell_marker.type = avt_341::msg::Marker::CUBE;
                        cell_marker.action = avt_341::msg::Marker::ADD;
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
    node = avt_341::node::init_node(argc, argv, "segmentation_processor_node");
    double rate = 10.0;
    avt_341::node::Rate ros_rate(rate);

    init_time = node->get_stamp();
    segmentation_grid_input.header.stamp = init_time;
    vehicle_odom_input_stamp = init_time;
    last_vehicle_odom_stamp = init_time;

    // Create publishers and subscribers
    auto seg_grid_sub = node->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 1, callback_seg);
    auto odometry_sub = node->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 1, callback_veh);
    auto speed_sub = node->create_subscription<avt_341::msg::Float64>("avt_341/speed_setpoint", 1, callback_speed);
    auto cells_pub = node->create_publisher<avt_341::msg::Float64MultiArray>("avt_341/segmentation_cells", 1);
    auto cell_marker_pub = node->create_publisher<avt_341::msg::MarkerArray>("avt_341/cell_markers", 1);

    // Load parameters
    node->get_parameter("~max_speed", max_speed, 10.0);
    node->get_parameter("~prediction_time_horizon", prediction_time_horizon, 2.0);
    node->get_parameter("~vehicle_axle_distance_front", axle_distance_front, 1.5521);
    node->get_parameter("~front_angle_segmentation", segmentation_angle, 0.707107);
    node->get_parameter("~obstacles_vizualize", viz, false);

    while (avt_341::node::ok()) {
        if (new_input_available(segmentation_grid_input, vehicle_odom_input)) {
            // Publish obstacle message
            avt_341::msg::Float64MultiArray cells_msg;
            cells_msg.data = cells;
            cells_pub->publish(cells_msg);

            if (viz) {
                avt_341::msg::MarkerArray cell_marker_msg;
                cell_marker_msg.markers = cell_markers;
                cell_marker_pub->publish(cell_marker_msg);
            }
        }

        node->spin_some();
        ros_rate.sleep();
    }

    return 0;
}
