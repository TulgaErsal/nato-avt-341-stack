//ros includes
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include <rclcpp/rclcpp.hpp>
// point cloud includes
#include "avt_341/perception/point_cloud_generator.h"
nav_msgs::msg::Odometry odom_msg;
geometry_msgs::msg::Twist twist;
bool odom_rcvd = false;
void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr rcv_msg){
  twist.linear.x = rcv_msg->linear.x; // throttle
  twist.linear.y = rcv_msg->linear.y; // braking
  twist.angular.z = rcv_msg->angular.z; // steering
}
void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr rcv_odom){
  odom_msg = *rcv_odom;
  odom_rcvd = true;
}
int main(int argc, char **argv){

  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("vehicle_state_node");
  auto mpc_state_pub = n->create_publisher<std_msgs::msg::Float64MultiArray>("avt_341/veh",1);
  auto odometry_sub = n->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);

  std::vector<double> veh_data = {0.0, -50.0, 1.8, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std_msgs::msg::Float64MultiArray mpc_data_msg;
  //mpc_data_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mpc_data_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  mpc_data_msg.layout.dim[0].size = veh_data.size();
  mpc_data_msg.layout.dim[0].stride = 1;
  mpc_data_msg.layout.dim[0].label = "x";


  int nloops = 0;
  float desired_speed = 5.0f;
  double dt = 0.01;
  rclcpp::Rate rate(1.0/dt);
  // ros simulation loop
  while (rclcpp::ok()) {

    // create vehicle state message
    veh_data[1] = odom_msg.pose.pose.position.x;
    veh_data[2] = odom_msg.pose.pose.position.y;
    veh_data[3] = odom_msg.twist.twist.linear.x;
    veh_data[4] = odom_msg.twist.twist.linear.y;
    mpc_data_msg.data.clear();
    mpc_data_msg.data.insert(mpc_data_msg.data.end(), veh_data.begin(), veh_data.end());
    mpc_state_pub->publish(mpc_data_msg);

    rate.sleep();

    rclcpp::spin_some(n);
    nloops++;
  } //while ros OK


  return 0;
}

