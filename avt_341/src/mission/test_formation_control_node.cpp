/**
 * @file test_formation_control_node.cpp
 * @author Chris Goodin (cgoodin@cavs.msstate.edu)
 * @brief This node runs with the formation controller to provide a simple unit test
 *  It simulates the vehicles aligned with the x-direction and only going in the x-direction
 * @date 2023-02-23
 */
// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

// The formation controller publishes a speed and path
// this node subscribes to them
avt_341::msg::Path path;
avt_341::msg::Float64 desired_speed;
avt_341::msg::String my_name;
bool path_rcvd = false;
bool speed_rcvd = false;

void PathCallback(avt_341::msg::PathPtr rcv_path){
    path = *rcv_path;
    path_rcvd = true;
}

void SpeedCallback(avt_341::msg::Float64Ptr rcv_speed){
    desired_speed = *rcv_speed;
    speed_rcvd = true;
}

int main(int argc, char **argv){

    auto n = avt_341::node::init_node(argc,argv,"test_formation_controller");

    // Create the odometry message and status message that will be used by the formation controller
    auto odometry_pub = n->create_publisher<avt_341::msg::Odometry>("avt_341/odometry", 10);
    auto status_pub = n->create_publisher<avt_341::msg::FollowerStatus>("avt_341/follower_status", 10);
    
    // subscribe to the path and speed message from the formation controller
    auto path_sub = n->create_subscription<avt_341::msg::Path>("avt_341/global_path", 10, PathCallback);
    auto speed_sub = n->create_subscription<avt_341::msg::Float64>("avt_341/desired_speed", 10, SpeedCallback);

    // Input, set to true of the vehicle is the leader
    bool is_leader = false;
    n->get_parameter("~is_leader", is_leader, false);

    // Set the desired offsets that will go in the status message
    float x_offset = 0.0f;
    n->get_parameter("~x_offset", x_offset, 0.0f);
    float y_offset = 0.0f;
    n->get_parameter("~y_offset", y_offset, 0.0f);

    // this is a simulation parameter, simulating the lead vehicle driving downthe x-axis at leader_speed
    float leader_speed = 5.0f;
    n->get_parameter("~leader_speed", leader_speed, 5.0f);

    n->get_parameter("~name", my_name.data, std::string("AGV1"));

    // Status message sent to the formation controller
    avt_341::msg::FollowerStatus status;
    status.x_offset = x_offset;
    status.y_offset = y_offset;
    status.use_leader = !is_leader;

    status_pub->publish(status);
    // odometry message sent to the formation controller
    avt_341::msg::Odometry odom;
    odom.header.frame_id = my_name.data;
    odom.pose.pose.position.x = -x_offset;
    odom.pose.pose.position.y = y_offset;
    odom.pose.pose.position.z = 0.0f;
    odom.pose.pose.orientation.w = 1.0f;
    odom.pose.pose.orientation.x = 0.0f;
    odom.pose.pose.orientation.y = 0.0f;
    odom.pose.pose.orientation.z = 0.0f;
    odom.twist.twist.linear.x = 0.0f;
    odom.twist.twist.linear.y = 0.0f;
    odom.twist.twist.linear.z = 0.0f;
    odom.twist.twist.angular.x = 0.0f;
    odom.twist.twist.angular.y = 0.0f;
    odom.twist.twist.angular.z = 0.0f;

    // set up looping and simulation parameters
    float update_rate = 10.0f;
    float dt = 1.0f/update_rate;
    avt_341::node::Rate loop_rate(10);
    
    while(avt_341::node::ok()){

        if (is_leader){
            // if this vehicle is the leader, drive forward at leader_speed
            odom.pose.pose.position.x += dt*leader_speed;
            odom.twist.twist.linear.x = leader_speed;
        }
        else{
            // if it is a follower, drive forward at the desired_speed from the formation controller
            odom.pose.pose.position.x += dt*odom.twist.twist.linear.x;
            odom.twist.twist.linear.x = desired_speed.data;
        }

        // publish the status and odometry
        //status_pub->publish(status);
        odometry_pub->publish(odom);

        n->spin_some();
        loop_rate.sleep();
    }
}
