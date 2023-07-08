#ifdef ROS_1
#include <string>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

nav_msgs::Odometry odometry;
geometry_msgs::Pose &pose = odometry.pose.pose;
void OdometryCallback(const nav_msgs::Odometry::ConstPtr& rcv_odom){
	//std::cout<<"State publisher recieved odometry "<<std::endl;
	odometry = *(rcv_odom.get());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "avt_state_publisher");
    ros::NodeHandle n;
    ros::Subscriber odom_sub = n.subscribe("avt_341/odometry", 100, OdometryCallback);

    tf::TransformBroadcaster broadcaster;
    
    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // set up parent and child frames
	geometry_msgs::TransformStamped map_trans;
	map_trans.header.frame_id = "map";
	map_trans.child_frame_id = "odom";

    ros::Rate loop_rate(100.0);
    while (ros::ok()) {
        ros::spinOnce();

		//don't do anything until odometry is valid
    	if(odometry.header.frame_id != "") {
			//send the transform
			odom_trans.header.seq = odometry.header.seq;
			odom_trans.header.stamp = odometry.header.stamp;
			odom_trans.transform.translation.x = pose.position.x;
			odom_trans.transform.translation.y = pose.position.y;
			odom_trans.transform.translation.z = pose.position.z;
			odom_trans.transform.rotation = pose.orientation;
			broadcaster.sendTransform(odom_trans);

			//joint states handled by robot_state_publisher in base.launch

			//map->odom handled by static_transform_publisher in base.launch

			/*
			map_trans.header.seq = odometry.header.seq;
			map_trans.header.stamp = odometry.header.stamp;
			map_trans.transform.translation.x = 0.0f;
			map_trans.transform.translation.y = 0.0f;
			map_trans.transform.translation.z = 0.0f;
			map_trans.transform.rotation.x = 0.0f;
			map_trans.transform.rotation.y = 0.0f;
			map_trans.transform.rotation.z = 0.0f;
			map_trans.transform.rotation.w = 1.0f;
			broadcaster.sendTransform(map_trans);
			*/
		}

        loop_rate.sleep();
    }

    return 0;
}

#else 
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <queue>

std::queue<nav_msgs::msg::Odometry> odometry_msgs;
void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr rcv_odom){
  odometry_msgs.push(*rcv_odom);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("avt_state_publisher");
  auto odom_sub = n->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry",10, OdometryCallback);

  std::string frame_prefix;
  bool publish_map_to_odom;
  n->declare_parameter("frame_prefix", std::string(""));
  n->get_parameter("frame_prefix", frame_prefix);
  n->declare_parameter("publish_map_to_odom", false);
  n->get_parameter("publish_map_to_odom", publish_map_to_odom);

  tf2_ros::TransformBroadcaster broadcaster(n);

  // message declarations
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = frame_prefix + "base_link";

  // set up parent and child frames
  geometry_msgs::msg::TransformStamped tf_map_to_odom;
  tf_map_to_odom.header.frame_id = std::string("map");
  tf_map_to_odom.child_frame_id = std::string("odom");

  rclcpp::Rate loop_rate(50.0);
  while (rclcpp::ok()) {
    while(!odometry_msgs.empty()) {
      auto odometry = odometry_msgs.front();
      odometry_msgs.pop();

      odom_trans.header.stamp = odometry.header.stamp;
      odom_trans.transform.translation.x = odometry.pose.pose.position.x;
      odom_trans.transform.translation.y = odometry.pose.pose.position.y;
      odom_trans.transform.translation.z = odometry.pose.pose.position.z;
      odom_trans.transform.rotation = odometry.pose.pose.orientation;

      broadcaster.sendTransform(odom_trans);

      if (publish_map_to_odom) {
        // map to odom broadcast transform
        tf_map_to_odom.header.stamp = odometry.header.stamp;
        tf_map_to_odom.transform.translation.x = 0.0;
        tf_map_to_odom.transform.translation.y = 0.0;
        tf_map_to_odom.transform.translation.z = 0.0;
        tf_map_to_odom.transform.rotation.x = 0.0;
        tf_map_to_odom.transform.rotation.y = 0.0;
        tf_map_to_odom.transform.rotation.z = 0.0;
        tf_map_to_odom.transform.rotation.w = 1.0;

        broadcaster.sendTransform(tf_map_to_odom);
      }
    }

    // This will adjust as needed per iteration
    rclcpp::spin_some(n);
    loop_rate.sleep();
  }


  return 0;
}

#endif // ROS_1
