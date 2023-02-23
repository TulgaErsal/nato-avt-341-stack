// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/mission/formation_controller.h"


avt_341::msg::Odometry odom;
avt_341::msg::Odometry ldr_odom;
avt_341::msg::FollowerStatus status;
bool odom_rcvd = false;
bool ldr_odom_rcvd = false;
bool status_rcvd = false;

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
    odom = *rcv_odom;
    odom_rcvd = true;
}

void LeaderOdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
    ldr_odom = *rcv_odom;
    ldr_odom_rcvd = true;
}

void StatusCallback(avt_341::msg::FollowerStatusPtr rcv_status){
    status = *rcv_status;
    status_rcvd = true;
}

int main(int argc, char **argv){

    auto n = avt_341::node::init_node(argc,argv,"formation_controller");

    auto odometry_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
    auto leader_odom_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/leader_odometry", 10, LeaderOdometryCallback);
    auto status_sub = n->create_subscription<avt_341::msg::FollowerStatus>("avt_341/follower_status", 10, StatusCallback);
    auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path", 10);
    auto speed_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed", 10);

    avt_341::mission::FormationController controller;

    avt_341::node::Rate loop_rate(10);
    
    while(avt_341::node::ok()){

        if (odom_rcvd && ldr_odom_rcvd && status_rcvd){
            controller.Update(ldr_odom, odom, status);
            speed_pub->publish(controller.GetSpeed());
            path_pub->publish(controller.GetPath());
        }
        n->spin_some();
        loop_rate.sleep();
    }
}
