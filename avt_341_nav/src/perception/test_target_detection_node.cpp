// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
// ros includes
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "avt_341_msgs/msg/target_reports.hpp"
#include "nav_msgs/msg/path.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341_nav/node/node_utils.h"

nav_msgs::msg::Odometry current_pose;
bool odom_rcvd = false;

double CalcDistanceSquaredToTarget(const geometry_msgs::msg::Point& odom_pose, const geometry_msgs::msg::Point32& point)
{
	double dx = odom_pose.x - point.x;
	double dy = odom_pose.y - point.y;
	double dz = odom_pose.z - point.z;
	return dx*dx + dy*dy + dz*dz;
}

void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr rcv_odom){
	current_pose = *rcv_odom;
	odom_rcvd = true;
}

int main(int argc, char *argv[]) {

	// Initialize the node
	rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("test_target_detection_node");
	// Subscriptions
    auto odom_sub = n->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry",10, OdometryCallback);
	// Publishers
    auto detect_pub = n->create_publisher<avt_341_msgs::msg::TargetReports>("avt_341/target_reports", 10);

	// handle parameters
	double detection_range = 10.0f;
	avt_341_nav::node::get_parameter(n, "~detection_range", detection_range, 10.0);
	double detection_range_squared = detection_range * detection_range;

	std::vector<std::string> target_name;
	std::vector<double> target_x;
	std::vector<double> target_y;
	avt_341_nav::node::get_parameter(n, "/targets_name", target_name, std::vector<std::string>(0));                                                                 
	avt_341_nav::node::get_parameter(n, "/targets_x", target_x, std::vector<double>(0));                                                                 
    avt_341_nav::node::get_parameter(n, "/targets_y", target_y, std::vector<double>(0));

	geometry_msgs::msg::Point32 target_pt;
	avt_341_msgs::msg::TargetReports targets_pt;
	const rclcpp::Time start = n->now();

	rclcpp::Rate rate(10.0);
	while (rclcpp::ok()){
		targets_pt.reports.clear();
		targets_pt.header.stamp = n->now();
		targets_pt.header.frame_id = "map";

		// loop through the list of targets 
		for(int i = 0; i < target_name.size(); i++) {
			// if distance from robot to the target is less than detection_range, signal detection
			target_pt.x = target_x[i];
			target_pt.y = target_y[i];

			if(CalcDistanceSquaredToTarget(current_pose.pose.pose.position, target_pt) < detection_range_squared) {
				avt_341_msgs::msg::TargetReport report;
				report.target_class = target_name[i];
				report.track_start = start + rclcpp::Duration(0, i + 1);
				report.position.x = target_x[i];
				report.position.y = target_y[i];
				report.ranging = avt_341_msgs::msg::TargetReport::RANGING_LIDAR;
				report.observer = current_pose.pose.pose.position;
				targets_pt.reports.push_back(report);
			}
		}

		detect_pub->publish(targets_pt);
		rclcpp::spin_some(n);
		rate.sleep();
	}

	return 0;
}
