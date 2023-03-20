// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

avt_341::msg::Odometry current_pose;
bool odom_rcvd = false;

double CalcDistanceSquaredToTarget(const avt_341::msg::Point& odom_pose, const avt_341::msg::Point32& point)
{
	double dx = odom_pose.x - point.x;
	double dy = odom_pose.y - point.y;
	double dz = odom_pose.z - point.z;
	return dx*dx + dy*dy + dz*dz;
}

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
	current_pose = *rcv_odom;
	odom_rcvd = true;
}

int main(int argc, char *argv[]) {

	// Initialize the node
	auto n = avt_341::node::init_node(argc, argv, "test_target_detection_node");
	// Subscriptions
    auto odom_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry",10, OdometryCallback);
	// Publishers
    auto detect_pub = n->create_publisher<avt_341::msg::Point>("avt_341/target_contacts", 1);

	// handle parameters
	double detection_range = 10.0f;
	n->get_parameter("~detection_range", detection_range, 10.0);
	double detection_range_squared = detection_range * detection_range;

	std::vector<std::string> target_name;
	std::vector<double> target_x;
	std::vector<double> target_y;
	n->get_parameter("/targets_name", target_name, std::vector<std::string>(0));                                                                 
	n->get_parameter("/targets_x", target_x, std::vector<double>(0));                                                                 
    n->get_parameter("/targets_y", target_y, std::vector<double>(0));

	avt_341::msg::Point32 target_pt; 

	avt_341::node::Rate rate(10.0);
	while (avt_341::node::ok()){

		// loop through the list of targets 
		for(int i = 0; i < target_name.size(); i++) {
			// if distance from robot to the target is less than detection_range, signal detection
			target_pt.x = target_x[i];
			target_pt.y = target_y[i];
			if(CalcDistanceSquaredToTarget(current_pose.pose.pose.position, target_pt) < detection_range_squared) {
				// Signal detection
				std::cout << "Target " << target_name[i] << " detected at " << target_x[i] << ", " << target_y[i] << std::endl;
				detect_pub->publish(current_pose.pose.pose.position);
			}
		}
		n->spin_some();
		rate.sleep();
	}

	return 0;
}
