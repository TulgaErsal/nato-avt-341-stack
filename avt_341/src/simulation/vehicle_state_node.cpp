//ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/node/clock_publisher.h"
// point cloud includes
#include "avt_341/perception/point_cloud_generator.h"
avt_341::msg::Odometry odom_msg;
avt_341::msg::Twist twist;
bool odom_rcvd = false;
void TwistCallback(avt_341::msg::TwistPtr rcv_msg){
  twist.linear.x = rcv_msg->linear.x; // throttle
  twist.linear.y = rcv_msg->linear.y; // braking
  twist.angular.z = rcv_msg->angular.z; // steering
}
void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
  odom_msg = *rcv_odom;
  odom_rcvd = true;
}
int main(int argc, char **argv){

  auto n = avt_341::node::init_node(argc,argv,"vehicle_state_node");
  auto mpc_state_pub = n->create_publisher<avt_341::msg::Float64MultiArray>("avt_341/veh",1);
  auto odometry_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);


  // determine if sim time will be used
  bool use_sim_time;
  n->get_parameter("use_sim_time", use_sim_time, false);
  std::shared_ptr<avt_341::node::ClockPublisher> clock_pub;
  if (use_sim_time){
    clock_pub = avt_341::node::ClockPublisher::make_shared("clock", 1, n);
  }

  std::vector<double> veh_data = {0.0, -50.0, 1.8, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  avt_341::msg::Float64MultiArray mpc_data_msg;
  //mpc_data_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mpc_data_msg.layout.dim.push_back(avt_341::msg::MultiArrayDimension());
  mpc_data_msg.layout.dim[0].size = veh_data.size();
  mpc_data_msg.layout.dim[0].stride = 1;
  mpc_data_msg.layout.dim[0].label = "x";


  // variables for tracking time if "use_sim_time" is on
  double elapsed_time = 0.0;
  int nloops = 0;
  float desired_speed = 5.0f;
  double dt = 0.01;
  avt_341::node::Rate rate(1.0/dt);
  // ros simulation loop
  while (avt_341::node::ok()) {

    // create vehicle state message
    veh_data[1] = odom_msg.pose.pose.position.x;
    veh_data[2] = odom_msg.pose.pose.position.y;
    veh_data[3] = odom_msg.twist.twist.linear.x;
    veh_data[4] = odom_msg.twist.twist.linear.y;
    mpc_data_msg.data.clear();
    mpc_data_msg.data.insert(mpc_data_msg.data.end(), veh_data.begin(), veh_data.end());
    mpc_state_pub->publish(mpc_data_msg);
    avt_341::node::inc_seq(odom_msg.header);

    // update and publish time if necessary
    if (use_sim_time ){
      clock_pub->publish(elapsed_time);
      elapsed_time += dt;
    }
    else {
      rate.sleep();
    }

    n->spin_some();
    nloops++;
  } //while ros OK


  return 0;
}

