/**
 * \file avt_341_speed_control_node.cpp
 *
 * ROS node to subscribe to a steering and speed message and do the throttle control
 * Intended to be used with MPC or DWA controller that output desired speed and steering angle
 *
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 *
 * \date 2/9/23
 */
// avt - ros includes
#include "avt_341_msgs/msg/nav_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341_nav/node/node_types.h"
#include "avt_341_nav/avt_341_utils.h"
//avt_341 includes
#include "avt_341_nav/control/pid_controller.h"
#include <avt_341_nav/speed_control_params_service.hpp>
#include <algorithm>

using avt_341_nav::utils::NavStackState;

nav_msgs::msg::Odometry state;
int current_run_state = NavStackState::NotInit;   // startup state
double mrzr_speedometer = 0.0;
bool speedometer_rcvd = false;
double desired_speed = 0.0;
double desired_speed_factor = 1.0;
double desired_steer_radians = 0.0;

void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr rcv_state) {
	state = *rcv_state;
}

void SpeedCallback(std_msgs::msg::Float64::SharedPtr rcv_speed) {
	mrzr_speedometer = rcv_speed->data;
  speedometer_rcvd = true;
}

void DesiredSpeedCallback(std_msgs::msg::Float64::SharedPtr rcv_des_speed) {
	desired_speed = rcv_des_speed->data;
}

void DesiredSpeedFactorCallback(std_msgs::msg::Float64::SharedPtr speed_factor_msg) {
  desired_speed_factor = speed_factor_msg->data;
}

void DesiredSteerCallback(std_msgs::msg::Float64::SharedPtr rcv_des_steer) {
	desired_steer_radians = rcv_des_steer->data;
}

void StateCallback(avt_341_msgs::msg::NavState::SharedPtr rcv_state){
  current_run_state = rcv_state->run_state;
}

bool reset_called = false;
void ResetCallback(const std_msgs::msg::String::SharedPtr msg){
  if(msg->data.find(avt_341_nav::node::NodeType::Control) != std::string::npos){
    reset_called = true;
  }
}

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("avt_341_speed_control_node");

  avt_341_nav::params::speed_control::ParamsListener param_listener(n);
  const auto params = param_listener.get_params();

  auto dc_pub = n->create_publisher<geometry_msgs::msg::Twist>("avt_341/cmd_vel",1);

  auto state_sub = n->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry",1, OdometryCallback);

  auto control_sub = n->create_subscription<avt_341_msgs::msg::NavState>("avt_341/state",1,StateCallback);

  auto speed_sub = n->create_subscription<std_msgs::msg::Float64>("avt_341/forward_speed",1,SpeedCallback);

  auto desired_speed_sub = n->create_subscription<std_msgs::msg::Float64>("avt_341/desired_speed",1,DesiredSpeedCallback);
  auto desired_speed_factor_sub = n->create_subscription<std_msgs::msg::Float64>("avt_341/desired_speed_factor",1,DesiredSpeedFactorCallback);

  auto desired_steer_sub = n->create_subscription<std_msgs::msg::Float64>("avt_341/cmd_steer",1,DesiredSteerCallback);
  auto reset_sub = n->create_subscription<std_msgs::msg::String>("avt_341/reset", 10, ResetCallback);
  auto reset_ack_pub = n->create_publisher<std_msgs::msg::String>("avt_341/reset_ack", 1);

  const double max_steer_angle_rad =
      params.vehicle_max_steer_angle_degrees * M_PI / 180.0;

  avt_341_nav::control::PidController controller(
      params.anti_windup_method, params.pid_output_min,
      params.pid_output_max);

  RCLCPP_INFO(n->get_logger(), "PID Values:\n  kp=%.2f\n  ki=%.2f\n  kd=%.2f\n  "
      "anti_windup_method=%s", params.throttle_kp, params.throttle_ki, params.throttle_kd, params.anti_windup_method.c_str());

  controller.SetKp(params.throttle_kp);
  controller.SetKi(params.throttle_ki);
  controller.SetKd(params.throttle_kd);
  controller.SetUseFeedForward(params.use_feed_forward);
  if (params.use_feed_forward){
    controller.SetForwardModelParams(params.ff_a0, params.ff_a1, params.ff_a2);
  }
  if (params.anti_windup_method ==
      avt_341_nav::control::AntiWindupMethod::IntegralClamping){
    controller.SetIntegralAbsMax(params.integral_abs_max);
  }

  auto next_waypoint_pub = n->create_publisher<geometry_msgs::msg::PointStamped>("avt_341/control_next_waypoint", 1);

  double rate = 100.0;
  double dt = 1.0/rate;
  const double brake_step = dt / params.time_to_max_brake;
  const double max_throttle_step = dt / params.time_to_max_throttle;
  double current_brake_value = 0.0;
  double current_throttle_value = 0.0;
  rclcpp::Rate r(rate);

  while (rclcpp::ok()){
    geometry_msgs::msg::Twist dc;

    if(reset_called){
      controller.Reset();
      desired_speed = 0.0;
      current_run_state = NavStackState::NotInit;
      std_msgs::msg::String reset_ack_msg;
      reset_ack_msg.data = avt_341_nav::node::NodeType::Control;
      reset_ack_pub->publish(reset_ack_msg);
      reset_called = false;
    }

    // tell the controller the current vehicle state
    double vel = 0.0;
    if (speedometer_rcvd){
      vel = mrzr_speedometer;
    }
    else{
      vel = sqrtf(state.twist.twist.linear.x*state.twist.twist.linear.x + state.twist.twist.linear.y*state.twist.twist.linear.y);
    }

    if (current_run_state==NavStackState::Active){
      // Active goal being navigated to
      controller.SetSetpoint(desired_speed_factor*desired_speed);
      dc.linear.x = params.use_speed_controller
          ? controller.GetControlVariable(vel,dt)
          : desired_speed_factor*desired_speed;
      dc.angular.z = std::clamp(
          desired_steer_radians * params.steering_gain,
          -max_steer_angle_rad, max_steer_angle_rad);
      dc.linear.y = 0.0;
    }
    else
    {
      // No active goal, depending on state: 1) coast, 2) gradual brake, 3) or hard max brake stop
      controller.SetSetpoint(0.0);
      dc.linear.x = params.use_speed_controller
          ? controller.GetControlVariable(vel, dt) : 0.0;
      dc.angular.z = 0.0;
      dc.linear.y = 0.0;
      if (params.use_speed_controller)
      {
        if (current_run_state == NavStackState::InactiveGradualStop)
        {
          dc.linear.y = std::min(1.0, current_brake_value + brake_step);
        }
        else if (current_run_state == NavStackState::InactiveHardStop)
        {
          dc.linear.y = 1.0;
        }
      }
    }

    // Throttle post-processing
    if (params.use_speed_controller)
    {
      dc.linear.x *= params.throttle_coefficient;
      dc.linear.x = dc.linear.y > 0.0 ? 0.0 : dc.linear.x;    // make sure the throttle is zero when braking

      // TODO: Bug? this doesn't account for decreasing throttle
      if (dc.linear.x-current_throttle_value > max_throttle_step){
        dc.linear.x = current_throttle_value + max_throttle_step;   // throttle can change at most max_throttle_step
      }

      dc.linear.x = std::clamp(dc.linear.x, 0.0, 1.0);
    }

    // Enforce maximum lateral acceleration
    geometry_msgs::msg::Twist dc_safe = dc;
    double lat_accel_g = (vel*vel) * tan(dc_safe.angular.z) /
        params.vehicle_wheelbase / 9.81;
    if (abs(lat_accel_g) > params.max_desired_lateral_g) {
      RCLCPP_INFO(n->get_logger(), "Lateral acceleration limit activated: %f g", lat_accel_g);
      // Calculate maximum speed for commanded steering angle
      dc_safe.linear.x = sqrt(abs(
          (params.max_desired_lateral_g * 9.81) *
          params.vehicle_wheelbase / tan(dc_safe.angular.z)));
      if (dc_safe.linear.x > dc.linear.x) dc_safe.linear.x = dc.linear.x;
      // Calculate maximum steering angle for current speed
      dc_safe.angular.z = atan(
          (params.max_desired_lateral_g * 9.81) *
          params.vehicle_wheelbase / (vel*vel)) *
          (dc_safe.angular.z/abs(dc_safe.angular.z));
    }

    if (params.output_steering_percent) {
      dc_safe.angular.z /= max_steer_angle_rad;
    }

    // Publish command
    dc_pub->publish(dc_safe);
    current_brake_value = dc_safe.linear.y;
    current_throttle_value = dc_safe.linear.x;

    geometry_msgs::msg::PointStamped next_waypoint_msg;
    next_waypoint_msg.point.x = state.pose.pose.position.x;
    next_waypoint_msg.point.y = state.pose.pose.position.y;
    next_waypoint_msg.point.z = state.pose.pose.position.z;
    next_waypoint_msg.header.frame_id = "map";
    next_waypoint_msg.header.stamp = n->now();
    next_waypoint_pub->publish(next_waypoint_msg);

    rclcpp::spin_some(n);

    r.sleep();
  }

  return 0;
}
