// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

avt_341::msg::Path path;
avt_341::msg::Float64 desired_speed;
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

    auto odometry_pub = n->create_publisher<avt_341::msg::Odometry>("avt_341/odometry", 10);
    auto status_pub = n->create_publisher<avt_341::msg::FollowerStatus>("avt_341/follower_status", 10);
    
    auto path_sub = n->create_subscription<avt_341::msg::Path>("avt_341/global_path", 10, PathCallback);
    auto speed_sub = n->create_subscription<avt_341::msg::Float64>("avt_341/desired_speed", 10, SpeedCallback);

    bool is_leader = false;
    n->get_parameter("~is_leader", is_leader, false);

    float x_offset = 0.0f;
    n->get_parameter("~x_offset", x_offset, 0.0f);

    float y_offset = 0.0f;
    n->get_parameter("~y_offset", y_offset, 0.0f);

    float leader_speed = 5.0f;
    n->get_parameter("~leader_speed", leader_speed, 5.0f);

    avt_341::msg::FollowerStatus status;
    status.x_offset = x_offset;
    status.y_offset = y_offset;
    status.use_leader = !is_leader;

    avt_341::msg::Odometry odom;
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

    float update_rate = 10.0f;
    float dt = 1.0f/update_rate;
    avt_341::node::Rate loop_rate(10);
    
    while(avt_341::node::ok()){

        if (is_leader){
            odom.pose.pose.position.x += dt*leader_speed;
            odom.twist.twist.linear.x = leader_speed;
        }
        else{
            odom.pose.pose.position.x += dt*odom.twist.twist.linear.x;
            odom.twist.twist.linear.x = desired_speed.data;
        }
        
        status_pub->publish(status);
        odometry_pub->publish(odom);

        n->spin_some();
        loop_rate.sleep();
    }
}
