/**
 * \file avt_341_control_node.cpp
 *
 * ROS node to subsribe to a trajectory message and 
 * convert it to a driving command using the pure-pursuit algorithm
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 7/13/2018
 */

#include "avt_341_msgs/msg/nav_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>
//avt_341 includes
#include "avt_341_nav/control/pure_pursuit_controller.h"
#include <avt_341_nav/control_params_service.hpp>
#include "avt_341_nav/core/math_dto.hpp"
#include "avt_341_nav/core/ros_msg_utils.hpp"

nav_msgs::msg::Path control_msg;
nav_msgs::msg::Odometry state;
int current_run_state = -1;   // startup state
bool shutdown_condition = false;
double mrzr_speedometer = 0.0;
bool speedometer_rcvd = false;
bool des_speed_rcvd = false;
double desired_speed = 0.0;

using avt_341_nav::core::NavStackState;

void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr rcv_state) {
	state = *rcv_state; 
}

void SpeedCallback(std_msgs::msg::Float64::SharedPtr rcv_speed) {
	mrzr_speedometer = rcv_speed->data;
  speedometer_rcvd = true; 
}

void DesiredSpeedCallback(std_msgs::msg::Float64::SharedPtr rcv_des_speed) {
	desired_speed = rcv_des_speed->data;
  des_speed_rcvd = true; 
}


void PathCallback(nav_msgs::msg::Path::SharedPtr rcv_control){
  control_msg.poses = rcv_control->poses;
  control_msg.header = rcv_control->header;
}

void StateCallback(avt_341_msgs::msg::NavState::SharedPtr rcv_state){
  current_run_state = rcv_state->run_state;
  if (current_run_state==NavStackState::InactiveGradualStop) shutdown_condition = true;
}

double length(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b){
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

double TriangleArea(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b, geometry_msgs::msg::Point c) {
	double area = std::fabs(a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y));
	return area;
}

double MengerCurvature(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b, geometry_msgs::msg::Point c) {
	double curv = 0.0;
	double denom = length(a, b)*length(b, c)*length(c, b);
	if (denom == 0.0) {
		curv = std::numeric_limits<double>::max();
	}
	else {
		double area = TriangleArea(a, b, c);
		curv = 4.0*area / denom;
	}
	return curv;
}

double GetMaxCurvature(nav_msgs::msg::Path path){
  double max_curvature = 0.0;
  if (path.poses.size() > 2) {
		for (int i = 1; i < path.poses.size() - 1; i++){
		  double curvature = MengerCurvature(path.poses[i - 1].pose.position, path.poses[i].pose.position, path.poses[i + 1].pose.position);
      if (curvature>max_curvature)max_curvature = curvature;
		}
	}
  return max_curvature;
}


int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("avt_341_control_node");

  avt_341_nav::params::control::ParamsListener param_listener(n);
  const auto params = param_listener.get_params();

  auto dc_pub = n->create_publisher<geometry_msgs::msg::Twist>("avt_341/cmd_vel",1);

  auto path_sub = n->create_subscription<nav_msgs::msg::Path>("avt_341/local_path",1, PathCallback);

  auto state_sub = n->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry",1, OdometryCallback);

  auto control_sub = n->create_subscription<avt_341_msgs::msg::NavState>("avt_341/state",1,StateCallback);

  auto speed_sub = n->create_subscription<std_msgs::msg::Float64>("avt_341/forward_speed",1,SpeedCallback);

  auto desired_speed_sub = n->create_subscription<std_msgs::msg::Float64>("avt_341/desired_speed",1,DesiredSpeedCallback);


  avt_341_nav::control::PurePursuitController controller;

  if (params.skid_steered){
    controller.IsSkidSteered(true);
    controller.SetSkidSteerParams(params.skid_kl, params.skid_kt);
  }
  else{
    controller.SetSteeringParams(
        params.steering_coefficient,
        params.pursuit_k,
        params.pursuit_kp,
        params.pursuit_kd);
    controller.SetThrottleCoeff(params.throttle_coefficient);
    controller.SetWheelbase(params.vehicle_wheelbase);
	  controller.SetMaxSteering(
        params.vehicle_max_steer_angle_degrees * 3.14159 / 180.0);
    controller.SetSpeedControllerParams(
        params.throttle_kp,
        params.throttle_ki,
        params.throttle_kd);
  }

  if (params.use_feed_forward){
    controller.GetPidSpeedController()->SetUseFeedForward(true);
    controller.GetPidSpeedController()->SetForwardModelParams(
        params.ff_a0,
        params.ff_a1,
        params.ff_a2);
  }
  
  controller.SetDesiredSpeed(params.vehicle_speed);
  if (params.turn_off_velocity_overshoot_corrector){
    controller.GetPidSpeedController()->SetOvershootLimiter(false);
  }

  auto next_waypoint_pub = n->create_publisher<geometry_msgs::msg::PointStamped>("avt_341/control_next_waypoint", 1);

  const double rate = 100.0;
  const double dt = 1.0/rate;
  const double brake_step = dt / params.time_to_max_brake;
  const double max_throttle_step = dt / params.time_to_max_throttle;
  double current_brake_value = 0.0;
  double current_throttle_value = 0.0;
  rclcpp::Rate r(rate);
  avt_341_nav::core::vec2 goal;
  int nl = 0;
  while (rclcpp::ok()){
    geometry_msgs::msg::Twist dc;
    bool time_to_quit = false;

    // tell the controller the current vehicle state
    double vel = 0.0;
    if (speedometer_rcvd){
      vel = mrzr_speedometer;
    }
    else{
      vel = std::hypot(
          state.twist.twist.linear.x, state.twist.twist.linear.y);
    }

    controller.SetVehicleState(state);
    controller.SetVehicleSpeed(vel);

    if (shutdown_condition){  // current_run_state = 2 
      // bring to a smooth stop and shut down
      controller.SetDesiredSpeed(0.0);
      if (vel<0.5)time_to_quit = true;
      dc = controller.GetDcFromTraj(control_msg, goal);
      dc.linear.x = 0.0;
      dc.angular.z = 0.0;
    }
    else if (current_run_state==NavStackState::Active){    // active running state
      double max_curvature = GetMaxCurvature(control_msg);
      double lateral_g_force = ((vel*vel)*max_curvature)/9.806;
      double desired_velocity = params.vehicle_speed;
      if (lateral_g_force > params.max_desired_lateral_g){
        desired_velocity = std::sqrt(
            9.806 * params.max_desired_lateral_g / max_curvature);
        if (desired_velocity > params.vehicle_speed) {
          desired_velocity = params.vehicle_speed;
        }
      }
      if (des_speed_rcvd){
        desired_velocity = desired_speed;
      }
      controller.SetDesiredSpeed(desired_velocity);
      dc = controller.GetDcFromTraj(control_msg, goal);
    }
    else if (current_run_state==NavStackState::NotInit || current_run_state==NavStackState::InactiveCoast){
      // bring to a smooth stop and wait / idle
	    // std::cout << " Setting desired speed to 0 " << std::endl;
      controller.SetDesiredSpeed(0.0);
      //dc = controller.GetDcFromTraj(control_msg, goal);
	    // Current controller is overshooting - changing to hard stop.
	    dc.linear.x = 0.0;
	    dc.linear.y = 1.0;
	    dc.angular.z = 0.0;
    }
    else if (current_run_state==NavStackState::InactiveHardStop){
      // bring to a hard stop and shut down
      dc.linear.x = 0.0;
      dc.linear.y = 1.0;
      dc.angular.z = 0.0;
      time_to_quit = true;
    }

    if (!params.skid_steered){
      // check braking and throttle
      if (dc.linear.y!=0.0){
        // apply the ramp up to the brake
        if (current_brake_value>dc.linear.y){
          dc.linear.y = current_brake_value - brake_step;
          if (dc.linear.y<-1.0)dc.linear.y = -1.0;
          if (dc.linear.y>0.0)dc.linear.y = 0.0;
        }
      // make sure the throttle is zero when braking
        dc.linear.x = 0.0;
      }
      // apply the throttle ramp up
      if (dc.linear.x-current_throttle_value > max_throttle_step){
        dc.linear.x = current_throttle_value + max_throttle_step;
      }
    }

    // Clamp the throttle effort
    dc.linear.x = std::max(0.0, std::min(dc.linear.x, 1.0));

    // publish the driving command
    dc_pub->publish(dc);
    current_brake_value = dc.linear.y;
    current_throttle_value = dc.linear.x;

    // TODO: Issue #145 Add a better way to display this data instead of constant log (ex: rviz display or maybe just ros2 topic echo)
    // if (nl % int(rate) == 0){ //update every second
    //   std::cout << " Driving Command: " << current_run_state << " Brake: " << current_brake_value << " Throttle: " << current_throttle_value << std::endl;
    // }
      
    // break the loop when an end state is reached
    if (time_to_quit) break;
    
    geometry_msgs::msg::PointStamped next_waypoint_msg;
    next_waypoint_msg.point.x = goal.x;
    next_waypoint_msg.point.y = goal.y;
    next_waypoint_msg.point.z = state.pose.pose.position.z;
    next_waypoint_msg.header.frame_id = "map";
    next_waypoint_msg.header.stamp = n->now();
    next_waypoint_pub->publish(next_waypoint_msg);

    rclcpp::spin_some(n);
    nl++;
    r.sleep();
  }

  return 0;
}
