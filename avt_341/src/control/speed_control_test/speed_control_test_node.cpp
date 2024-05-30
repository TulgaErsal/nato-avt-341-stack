/**
 * \file speed_control_test_node.cpp
 *
 * Node to test the speed control algorithm. 
 * Publishes a sinusoidally varying desired speed.
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 1/19/2023
 */

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"


int main(int argc, char *argv[]){
  auto n = avt_341::node::init_node(argc,argv,"speed_control_test_node");

  auto speed_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed",10);

  // some params
  float max_desired_speed = 10.0f; //m/s
  float time_to_max = 15.0f; //seconds;
  float pi = 3.14159265358979f;
  float b = time_to_max/pi;

  float rate = 50.0f;
  float dt = 1.0f/rate;
  float elapsed_time = 0.0f;
  avt_341::node::Rate r(rate);
  while (avt_341::node::ok()){
    avt_341::msg::Float64 desired_speed;
    desired_speed.data = max_desired_speed*0.5f*(sinf( (elapsed_time/b-(0.5f*pi)))+1.0f);

    speed_pub->publish(desired_speed);

    elapsed_time += dt;

    n->spin_some();

    r.sleep();
  }

  return 0;
}
