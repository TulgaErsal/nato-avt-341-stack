#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include "avt_341_msgs/msg/communication.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341/avt_341_utils.h"
#include <avt_341/speed_zones_params_service.hpp>
#include "avt_341/node/tf_interface.h"


rclcpp::Node::SharedPtr node;
std::shared_ptr<avt_341::node::TfInterface> tf;
nav_msgs::msg::Odometry odom;
bool odom_rcvd = false;

class SpeedZone {
public:
    SpeedZone() {
        id = 0;
        frame = "map";
        corners = {};
    }
    SpeedZone(int id, std::string frame, std::vector<std::vector<double>> corners) : id(id), frame(frame), corners(corners) {}

    bool IsContained(geometry_msgs::msg::PoseStamped pose) {
        bool is_contained = false;
        int n = corners.size();
        geometry_msgs::msg::Point p = pose.pose.position;
        int i,j;
        for(int i = 0, j = n-1; i < n; j = i++) {
            if (((corners[i][1] > p.y) != (corners[j][1] > p.y)) && 
                    (p.x < (corners[j][0] - corners[i][0]) * (p.y - corners[i][1]) / (corners[j][1] - corners[i][1]) + corners[i][0])) {
                is_contained = !is_contained;
            }
        }
        return is_contained;
    }

    int id;
    std::string frame;
    std::vector<std::vector<double>> corners;
};

void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr rcv_odom) {
	odom = *rcv_odom;
    odom_rcvd = true;
}

std::vector<std::string> GetLine(std::istream& stream) {
    std::vector<std::string> result;
    std::string line;
    std::getline(stream, line);
    std::stringstream lineStream(line);
    std::string cell;

    while(std::getline(lineStream, cell, ',')) {
        result.push_back(cell);
    }
    return result;
}

std::vector<SpeedZone> ReadSpeedZones(std::string filepath, std::string frame) {
    std::string line;
    std::vector<std::string> line_contents;
    std::vector<SpeedZone> zones;
    int row = 0;

    if (filepath.empty()) {
        return zones;
    }

    // Load the speed zones from file
    std::ifstream infile(filepath);
    if(infile.is_open()) {
        try
        {
            while(std::getline(infile, line))
            {
                std::istringstream iss(line);
                line_contents = GetLine(iss);
                std::vector<std::vector<double>> corners;
                if (row >= 1 && !line_contents.empty()) {
                    int id = std::stoi(line_contents[0]);
                    for (int i = 1; i < line_contents.size(); i+=2) {
                        corners.push_back({
                            std::stod(line_contents[i]),
                            std::stod(line_contents[i+1])
                        });
                    }
                    SpeedZone zone(id,frame,corners);
                    zones.push_back(zone);
                }
                row++;
            }
        }
        catch(std::exception& e)
        {
            RCLCPP_ERROR(node->get_logger(), "Error reading speed zones %s: %s", filepath.c_str(), e.what());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Could not open speed zones file %s", filepath.c_str());
    }
    return zones;
}

bool TransformSpeedZone(SpeedZone& zone, std::string target_frame) {
    std::vector<std::vector<double>> new_corners;
    bool is_transformed = true;
    for (auto corner: zone.corners) {
        geometry_msgs::msg::PoseStamped corner_pose, corner_pose_new;
        corner_pose.header.frame_id = zone.frame;
        corner_pose.header.stamp = node->now();
        corner_pose.pose.position.x = corner[0];
        corner_pose.pose.position.y = corner[1];
        is_transformed &= tf->transform_pose(corner_pose, corner_pose_new, target_frame, 0.5);
        new_corners.push_back({corner_pose_new.pose.position.x,corner_pose_new.pose.position.y});
    }
    if (is_transformed) {
        zone.corners = new_corners;
    }
    return is_transformed;
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("avt_341_speed_zones_node");
    avt_341::params::speed_zones::ParamsListener param_listener(node);
    const auto params = param_listener.get_params();
    tf = std::make_shared<avt_341::node::TfInterface>(node);

    // Create subscribers/publishers
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        params.vehicle_odom_topic, 1, OdometryCallback);
    auto comm_pub = node->create_publisher<avt_341_msgs::msg::Communication>("avt_341/comm_messages",1);

    // Get vehicle name
    std::string my_name = std::string(node->get_namespace());
    std::transform(my_name.begin(), my_name.end(), my_name.begin(), [](unsigned char c){ return std::toupper(c); });    // Uppercase name
    my_name.erase(0, 1);    // Erase '/' in namespace

    // Parse speed zones
    RCLCPP_INFO(node->get_logger(), "Attempting to read file %s", params.zones_filepath.c_str());
    std::vector<SpeedZone> speed_zones =
        ReadSpeedZones(params.zones_filepath, params.zones_frame);

    if (speed_zones.empty()) {
        RCLCPP_WARN(node->get_logger(), "No speed zones defined. Node existing.");
        exit(EXIT_SUCCESS);
    }

    if (params.zone_speeds.size() != speed_zones.size())
    {
        RCLCPP_ERROR(node->get_logger(), "Size mismatch between zone speeds (%d) and zone geometry (%d).", params.zone_speeds.size(), speed_zones.size());
        exit(EXIT_FAILURE);
    }

    RCLCPP_INFO(node->get_logger(), "Loaded %d speed zones.", speed_zones.size());

    rclcpp::Rate node_rate(10.0);
    int current_zone = -1;
    int last_zone = -1;
    while (rclcpp::ok())
    {
        // Update vehicle state
        if (odom_rcvd) {
            bool is_fixed = true;

            // Extract stamped pose from odom
            geometry_msgs::msg::PoseStamped pose;
            pose.header = odom.header;
            pose.pose = odom.pose.pose;

            // Transform pose
            if (pose.header.frame_id != params.zones_frame) {
                geometry_msgs::msg::PoseStamped pose_fixed;
                is_fixed = tf->transform_pose(
                    pose, pose_fixed, params.zones_frame, 0.2);
                pose = pose_fixed;
            }

            if (is_fixed) {
                // Determine which zones the vehicle is in
                current_zone = -1;
                for (auto zone: speed_zones) {
                    if (zone.IsContained(pose)) {
                        current_zone = zone.id;
                    }
                }
            }
            odom_rcvd = false;
        }

        // Check if new zone has been entered
        if (current_zone != last_zone && (current_zone < 0 || current_zone >= params.zone_speeds.size())) {
            RCLCPP_INFO(node->get_logger(), "Speed not defined for zone #%d", current_zone);
            last_zone = current_zone;
        }
        else if (current_zone != last_zone) {
            RCLCPP_INFO(node->get_logger(), "SETTING SPEED TO %.2lf [Zone #%d]", params.zone_speeds[current_zone], current_zone);

            // Send SET_SPEED msg
            avt_341_msgs::msg::Communication comm_msg;
            comm_msg.sender_name = my_name;
            comm_msg.msg_id = 0;
            comm_msg.type = "SET_SPEED";
            comm_msg.receiver_name = my_name;
            comm_msg.desired_speed = params.zone_speeds[current_zone];
            comm_msg.priority_type = "PREEMPT";
            comm_pub->publish(comm_msg);

            last_zone = current_zone;
        }

        rclcpp::spin_some(node);
        node_rate.sleep();
    }

}
