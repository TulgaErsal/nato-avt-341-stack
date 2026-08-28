/**
 * \file veh_converter_node.cpp
 * Simple node publishing avt_341/veh Float64MultiArray using twist and other topics
 *
 * \author Marius Thoresen
 *
 * \contact marius.thoresen@ffi.no
 *
 * \date 5/7/2023
*/

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "avt_341_nav/veh_converter_params_service.hpp"

std::shared_ptr <rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> pub_veh;

nav_msgs::msg::Odometry g_odometry;

double g_acceleration{0.0};

double g_steering_angle{0.0};

double g_steering_scale{1.0};
double g_steering_offset{0.0};

bool g_received_odometry{false};

bool g_received_acceleration{false};

bool g_received_steering_angle{false};

double quaternionMsg2Yaw(const geometry_msgs::msg::Quaternion& orientation_msg) {
    tf2::Quaternion orientation(orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w);
    // NOTE: getRPY() expects a double, hence we can cast back to float when setting the state.
    double roll, pitch, yaw;
    tf2::Matrix3x3 rotation(orientation);
    rotation.getRPY(roll, pitch, yaw);
    return yaw;
}

void callbackOdometry(nav_msgs::msg::Odometry::SharedPtr msg_received_odometry) {
    g_odometry = *msg_received_odometry;
    g_received_odometry = true;
    std_msgs::msg::Float64MultiArray veh;
    double time = 0.0;
    double x = g_odometry.pose.pose.position.x;
    double y = g_odometry.pose.pose.position.y;
    double speed_longitudinal = g_odometry.twist.twist.linear.x;
    double speed_lateral = g_odometry.twist.twist.linear.y;
    double steering_angle = g_steering_angle;
    double yaw = quaternionMsg2Yaw(g_odometry.pose.pose.orientation);
    double yaw_rate = g_odometry.twist.twist.angular.z;
    double acceleration = (g_received_acceleration) ? g_acceleration : 0.0;
    double front_wheel_slip{0.0};
    double rear_wheel_slip{0.0};
    veh.data =
        {time, x, y, speed_longitudinal, speed_lateral, steering_angle, yaw, yaw_rate, acceleration, front_wheel_slip,
         rear_wheel_slip};
    pub_veh->publish(veh);
}

void callbackImu(sensor_msgs::msg::Imu::ConstSharedPtr msg_received_imu) {
    g_acceleration = msg_received_imu->linear_acceleration.x;
    g_received_acceleration = true;
}

void callbackAccel(geometry_msgs::msg::AccelStamped::SharedPtr msg_accel) {
    g_acceleration = msg_accel->accel.linear.x;
    g_received_acceleration = true;
}

void callbackSteeringAngle(std_msgs::msg::Float64::SharedPtr msg_received_steering_angle) {
    g_steering_angle = msg_received_steering_angle->data * g_steering_scale + g_steering_offset;
    g_received_steering_angle = true;
}

int main(int argc, char* argv[]) {
    // Initialize ROS node.
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("avt_341_veh_converter_node");

    avt_341_nav::params::veh_converter::ParamsListener param_listener(node);
    const auto params = param_listener.get_params();
    g_steering_scale = params.steering_scale;
    g_steering_offset = params.steering_offset;

    // Create node subscribers.
    auto sub_odometry = node->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry", 1, callbackOdometry);
    auto sub_steering_angle =
        node->create_subscription<std_msgs::msg::Float64>("avt_341/steering_angle", 1, callbackSteeringAngle);
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>("/mavs_ros/imu", 1, callbackImu);
    auto sub_accel = node->create_subscription<geometry_msgs::msg::AccelStamped>("/avt_341/acceleration", 1, callbackAccel);

    // Create node publishers.
    pub_veh = node->create_publisher<std_msgs::msg::Float64MultiArray>("avt_341/veh", 1);

    rclcpp::Rate rosrate(100.0f);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rosrate.sleep();
    }
}