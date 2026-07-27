//ros includes
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341_nav/node/node_utils.h"
// point cloud includes
#include "avt_341_nav/perception/point_cloud_generator.h"

geometry_msgs::msg::Twist twist;
void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr rcv_msg){
  twist.linear.x = rcv_msg->linear.x; // throttle
  twist.linear.y = rcv_msg->linear.y; // braking
  twist.angular.z = rcv_msg->angular.z; // steering
}

int main(int argc, char **argv){

  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("avt_341_simulation_test_node");

  auto twist_sub = n->create_subscription<geometry_msgs::msg::Twist>("avt_341/cmd_vel",1, TwistCallback);

  auto lidar_pub = n->create_publisher<sensor_msgs::msg::PointCloud2>("avt_341/points",1);
  auto odom_pub = n->create_publisher<nav_msgs::msg::Odometry>("avt_341/odometry",1);
  auto mpc_state_pub = n->create_publisher<std_msgs::msg::Float64MultiArray>("avt_341/veh",1);


  // create and populate the odometry message that will be published
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose.position.x = -55.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 1.0;
  odom_msg.pose.pose.orientation.w = 1.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.0;
  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  // create and populate the point cloud message that will be published
  sensor_msgs::msg::PointCloud2 pc2;
  std::vector<avt_341_nav::core::vec3> points {
    avt_341_nav::core::vec3(50.0, 0.0, 0.0),
    avt_341_nav::core::vec3(15.1, 7.8, 5.0),
    avt_341_nav::core::vec3(15.1, 7.8, 1.0),
    avt_341_nav::core::vec3(14.5, 8.5, 7.0),
    avt_341_nav::core::vec3(14.5, 8.5, 1.0),
    avt_341_nav::core::vec3(14.6, 8.2, 4.5),
    avt_341_nav::core::vec3(14.6, 8.2, 1.5),
    avt_341_nav::core::vec3(15.1, -7.8, 0.0),
    avt_341_nav::core::vec3(14.5, -8.5, 0.0),
    avt_341_nav::core::vec3(14.6, -8.2, 0.1)
  };
    std::vector<int> seg_values = {
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            1,
            1,
            1
    };
  std::vector<double> veh_data = {0.0, -50.0, 1.8, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std_msgs::msg::Float64MultiArray mpc_data_msg;
  //mpc_data_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mpc_data_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  mpc_data_msg.layout.dim[0].size = veh_data.size();
  mpc_data_msg.layout.dim[0].stride = 1;
  mpc_data_msg.layout.dim[0].label = "x";

  avt_341_nav::perception::PointCloudGenerator::toROSMsg(points, seg_values,pc2);
  pc2.header.frame_id = "odom";

  //odometry published at 100 Hz, point clout at 10 Hz
  double dt = 0.01;
  rclcpp::Rate rate(1.0/dt);

  int nloops = 0;
  float desired_speed = 5.0f;
  // ros simulation loop
  while (rclcpp::ok()) {

    // publish the odometry message
    odom_msg.header.stamp = n->now();
    odom_msg.pose.pose.position.x += twist.linear.x*desired_speed*dt;
    odom_msg.twist.twist.linear.x = desired_speed*twist.linear.x;
    odom_pub->publish(odom_msg);
    veh_data[1] = odom_msg.pose.pose.position.x;
    veh_data[2] = odom_msg.pose.pose.position.y;
    veh_data[3] = odom_msg.twist.twist.linear.x;
    veh_data[4] = odom_msg.twist.twist.linear.y;
    mpc_data_msg.data.clear();
    mpc_data_msg.data.insert(mpc_data_msg.data.end(), veh_data.begin(), veh_data.end());
    mpc_state_pub->publish(mpc_data_msg);


    if (nloops%10==0){
      // publish the point cloud at 10 Hz
      pc2.header.stamp = n->now();
      lidar_pub->publish(pc2);
    }

    rate.sleep();

    rclcpp::spin_some(n);
    nloops++;
  } //while ros OK


  return 0;
}

