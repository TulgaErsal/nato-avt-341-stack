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

#include "avt_341/node/node_proxy.h"
#include "avt_341/node/ros_types.h"

std::shared_ptr <avt_341::node::Publisher<avt_341::msg::Float64MultiArray>> pub_veh;

avt_341::msg::Odometry g_odometry;

double g_acceleration{0.0};

double g_steering_angle{0.0};

bool g_received_odometry{false};

bool g_received_acceleration{false};

bool g_received_steering_angle{false};

double quaternionMsg2Yaw(const avt_341::msg::Quaternion& orientation_msg) {
    avt_341::msg_tf::Quaternion orientation(orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w);
    // NOTE: getRPY() expects a double, hence we can cast back to float when setting the state.
    double roll, pitch, yaw;
    avt_341::msg_tf::Matrix3x3 rotation(orientation);
    rotation.getRPY(roll, pitch, yaw);
    return yaw;
}

void callbackOdometry(avt_341::msg::OdometryPtr msg_received_odometry) {
    g_odometry = *msg_received_odometry;
    g_received_odometry = true;
    avt_341::msg::Float64MultiArray veh;
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

void callbackImu(avt_341::msg::ImuPtr msg_received_imu) {
    g_acceleration = msg_received_imu->linear_acceleration.x;
    g_received_acceleration = true;
}

void callbackSteeringAngle(avt_341::msg::Float64Ptr msg_received_steering_angle) {
    g_steering_angle = msg_received_steering_angle->data;
    g_received_steering_angle = true;
}

int main(int argc, char* argv[]) {
    // Initialize ROS node.
    auto node = avt_341::node::init_node(argc, argv, "avt_341_veh_converter_node");

    // Create node subscribers.
    auto sub_odometry = node->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 1, callbackOdometry);
    auto sub_steering_angle =
        node->create_subscription<avt_341::msg::Float64>("avt_341/steering_angle", 1, callbackSteeringAngle);
    auto sub_imu = node->create_subscription<avt_341::msg::Imu>("/mavs_ros/imu", 1, callbackImu);


    // Create node publishers.
    pub_veh = node->create_publisher<avt_341::msg::Float64MultiArray>("avt_341/veh", 1);

    avt_341::node::Rate rosrate(100.0f);
    while (avt_341::node::ok()) {
        node->spin_some();
        rosrate.sleep();
    }
}