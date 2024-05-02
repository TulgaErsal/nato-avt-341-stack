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
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/avt_341_utils.h"
//avt_341 includes
#include "avt_341/control/pid_controller.h"


avt_341::msg::Odometry state;
int current_run_state = avt_341::utils::NavStackState::NotInit;   // startup state
bool shutdown_condition = false;
double mrzr_speedometer = 0.0;
bool speedometer_rcvd = false;
float desired_speed = 0.0f;
float desired_speed_factor = 1.0f;
float desired_steer_radians = 0.0f;

void OdometryCallback(avt_341::msg::OdometryPtr rcv_state) {
	state = *rcv_state; 
}

void SpeedCallback(avt_341::msg::Float64Ptr rcv_speed) {
	mrzr_speedometer = rcv_speed->data;
  speedometer_rcvd = true; 
}

void DesiredSpeedCallback(avt_341::msg::Float64Ptr rcv_des_speed) {
	desired_speed = rcv_des_speed->data;
}

void DesiredSpeedFactorCallback(avt_341::msg::Float64Ptr speed_factor_msg) {
  desired_speed_factor = speed_factor_msg->data;
}

void DesiredSteerCallback(avt_341::msg::Float64Ptr rcv_des_steer) {
	desired_steer_radians = rcv_des_steer->data;
}

void StateCallback(avt_341::msg::Int32Ptr rcv_state){
  current_run_state = rcv_state->data;
  if (current_run_state==avt_341::utils::NavStackState::Shutdown)shutdown_condition = true;
}

bool reset_called = false;
void ResetCallback(avt_341::msg::StringPtr msg){
  if(msg->data.find(avt_341::node::NodeType::Control) != std::string::npos){
    reset_called = true;
  }
}

int main(int argc, char *argv[]){
  auto n = avt_341::node::init_node(argc,argv,"avt_341_control_node");

  auto dc_pub = n->create_publisher<avt_341::msg::Twist>("avt_341/cmd_vel",1);

  auto state_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry",1, OdometryCallback);

  auto control_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/state",1,StateCallback);

  auto speed_sub = n->create_subscription<avt_341::msg::Float64>("mrzr_velocity",1,SpeedCallback);

  auto desired_speed_sub = n->create_subscription<avt_341::msg::Float64>("avt_341/desired_speed",1,DesiredSpeedCallback);
  auto desired_speed_factor_sub = n->create_subscription<avt_341::msg::Float64>("avt_341/desired_speed_factor",1,DesiredSpeedFactorCallback);

  auto desired_steer_sub = n->create_subscription<avt_341::msg::Float64>("avt_341/cmd_steer",1,DesiredSteerCallback);
  auto reset_sub = n->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
  auto reset_ack_pub = n->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);

  // The PID params are tuned with this value in mind
  // so it's not a good idea to change it
  float time_to_max_throttle = 3.0f; //seconds
	// Set controller parameters
  float ff_a0, ff_a1, ff_a2;
  bool use_feed_forward;
	float throttle_coeff, time_to_max_brake;
  float throttle_kp, throttle_ki, throttle_kd;
	std::string display, anti_windup_method;
  float vehicle_max_steer_angle_degrees;
  double output_max, output_min;
  bool use_speed_controller;
  n->get_parameter("~vehicle_max_steer_angle_degrees", vehicle_max_steer_angle_degrees, 25.0f);
  n->get_parameter("~throttle_coefficient", throttle_coeff, 1.0f);
  n->get_parameter("~time_to_max_brake", time_to_max_brake, 4.0f);
  n->get_parameter("~time_to_max_throttle", time_to_max_throttle, 3.0f);
  n->get_parameter("~ff_a0", ff_a0, 0.0402f);
  n->get_parameter("~ff_a1", ff_a1, 0.0814f);
  n->get_parameter("~ff_a2", ff_a2, -0.0023f);
  n->get_parameter("~use_feed_forward", use_feed_forward, true);
  n->get_parameter("~throttle_kp", throttle_kp, 0.462f);
  n->get_parameter("~throttle_ki", throttle_ki, 0.222f);
  n->get_parameter("~throttle_kd", throttle_kd, 0.24f);
  n->get_parameter("~pid_output_max", output_max, 1.0);
  n->get_parameter("~pid_output_min", output_min, 0.0);
  n->get_parameter("~anti_windup_method", anti_windup_method, avt_341::control::AntiWindupMethod::ResetOnSetpoint);
  n->get_parameter("~display", display, std::string("none"));
  n->get_parameter("~use_speed_controller", use_speed_controller, true);

  avt_341::control::PidController controller(anti_windup_method, output_min, output_max);

  n->log_info("PID Values:\n  kp=%.2f\n  ki=%.2f\n  kd=%.2f\n  anti_windup_method=%s", throttle_kp, throttle_ki, throttle_kd, anti_windup_method.c_str());

  controller.SetKp(throttle_kp);
  controller.SetKi(throttle_ki);
  controller.SetKd(throttle_kd);
  controller.SetUseFeedForward(use_feed_forward);
  if (use_feed_forward){
    controller.SetForwardModelParams(ff_a0, ff_a1, ff_a2);
  }


  bool display_rviz = display == "rviz";
  auto next_waypoint_pub = display_rviz ? n->create_publisher<avt_341::msg::PointStamped>("avt_341/control_next_waypoint", 1) : nullptr;

  float rate = 100.0f;
  float dt = 1.0f/rate;
  float brake_step = dt/time_to_max_brake;
  float max_throttle_step = dt/time_to_max_throttle;
  float current_brake_value = 0.0f;
  float current_throttle_value = 0.0f;
  avt_341::node::Rate r(rate);

  while (avt_341::node::ok()){
    avt_341::msg::Twist dc;
    bool time_to_quit = false;

    if(reset_called){
      controller.Reset();
      desired_speed = 0.0f;
      current_run_state = avt_341::utils::NavStackState::NotInit;
      avt_341::msg::String reset_ack_msg;
      reset_ack_msg.data = avt_341::node::NodeType::Control;
      reset_ack_pub->publish(reset_ack_msg);
      reset_called = false;
    }

    // tell the controller the current vehicle state
    float vel = 0.0f;
    if (speedometer_rcvd){
      vel = mrzr_speedometer;
    }
    else{
      vel = sqrtf(state.twist.twist.linear.x*state.twist.twist.linear.x + state.twist.twist.linear.y*state.twist.twist.linear.y);
    }

    if (shutdown_condition){  // current_run_state = 2 
      // bring to a smooth stop and shut down
      controller.SetSetpoint(0.0f);
      if (vel<0.5f)time_to_quit = true;
      dc.linear.x = 0.0f;
      dc.angular.z = 0.0f;

    }
    else if (current_run_state==avt_341::utils::NavStackState::Active){    // active running state
      controller.SetSetpoint(desired_speed_factor*desired_speed);
      dc.linear.x = (use_speed_controller) ? controller.GetControlVariable(vel,dt) : desired_speed_factor*desired_speed;
    }
    else if (current_run_state==avt_341::utils::NavStackState::NotInit || current_run_state==avt_341::utils::NavStackState::Stopped){
      // bring to a smooth stop and wait / idle
      controller.SetSetpoint(0.0f);
       dc.linear.x = (use_speed_controller) ? controller.GetControlVariable(vel,dt) : 0.0f;
    }
    else if (current_run_state==avt_341::utils::NavStackState::HardShutdown){
      // bring to a hard stop and shut down
      dc.linear.x = 0.0f;
      dc.linear.y = (use_speed_controller) ? 1.0f : 0.0f;
      dc.angular.z = 0.0f;
      time_to_quit = true;
    }

    // apply the throttle scaling coefficient
    if (use_speed_controller)
      {
      dc.linear.x *= throttle_coeff;
      // check braking and throttle
      if (std::abs(dc.linear.y) > 1e-5){
        // apply the ramp up to the brake
        if (current_brake_value>dc.linear.y){
          dc.linear.y = current_brake_value - brake_step;
          if (dc.linear.y<-1.0)dc.linear.y = -1.0;
          if (dc.linear.y>0.0)dc.linear.y = 0.0;
        }
        // make sure the throttle is zero when braking
        dc.linear.x = 0.0f;
      }
      // apply the throttle ramp up
      if (dc.linear.x-current_throttle_value > max_throttle_step){
        dc.linear.x = current_throttle_value + max_throttle_step;
      }
    }

    // Clamp the throttle effort
    dc.linear.x = std::max(0.0, std::min(dc.linear.x, 1.0));

    // publish the driving command
    dc.angular.z = std::max(-1.0f,std::min(1.0f,(180.0f*desired_steer_radians/3.14159265358979f)/vehicle_max_steer_angle_degrees));
    dc_pub->publish(dc);
    current_brake_value = dc.linear.y;
    current_throttle_value = dc.linear.x;

    // break the loop when an end state is reached
    if (time_to_quit)break;
    
    if(display_rviz){
      avt_341::msg::PointStamped next_waypoint_msg;
      next_waypoint_msg.point.x = state.pose.pose.position.x;
      next_waypoint_msg.point.y = state.pose.pose.position.y;
      next_waypoint_msg.point.z = state.pose.pose.position.z;
      next_waypoint_msg.header.frame_id = "map";
      next_waypoint_msg.header.stamp = n->get_stamp();
      next_waypoint_pub->publish(next_waypoint_msg);
    }

    n->spin_some();

    r.sleep();
  }

  return 0;
}
