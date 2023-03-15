// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/mission/formation_controller.h"

avt_341::msg::Odometry odom;
avt_341::msg::Odometry ldr_odom;
avt_341::msg::FollowerStatus status;
avt_341::msg::String my_name;
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

    // create the node
    auto n = avt_341::node::init_node(argc,argv,"formation_controller");

    // set up subscriptions
    // all vehicles subscribe to their own odometry and the leader odometry separately
    auto odometry_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
    auto leader_odom_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/leader_odometry", 10, LeaderOdometryCallback);
    auto status_sub = n->create_subscription<avt_341::msg::FollowerStatus>("avt_341/follower_status", 10, StatusCallback);
    
    // create the publishers, each vehicle publishes a path and a desired speed
    auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path", 10);
    auto speed_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed", 10);
    auto gptoggle_pub = n->create_publisher<avt_341::msg::Int32>("avt_341/gp_toggle", 10);
    avt_341::msg::Int32 gp_toggle; 

    // load the parameters
    // specify if the vehicle is the leader
    bool is_leader = false;
    n->get_parameter("~is_leader", is_leader, false);

    // parameter of the formation controller
    float dist_gain = 1.0f;
    n->get_parameter("~follower_dist_gain", dist_gain, 1.0f);

    // another parameter of the formation controller
    float path_point_dist = 1.0f;
    n->get_parameter("~global_path_point_dist", path_point_dist, 1.0f);

    n->get_parameter("~name", my_name.data, std::string("AGV1"));

    // create the controller and set the parameters loaded from the launch file
    avt_341::mission::FormationController controller;
    controller.SetGlobalPathPointsDist(path_point_dist);
    controller.SetFollowerDistGain(dist_gain);

    // set the node loop ratre to 10 Hz
    avt_341::node::Rate loop_rate(10);
    
    // start the loop
    while(avt_341::node::ok()){

        // Update leader status
        if(status_rcvd) {
            // if not currently leader and status is not telling me to use the leader, I'm the leader
            if(!is_leader) {
                if(!status.use_leader) {
                    is_leader = true;
                    gp_toggle.data = 1; 
                    gptoggle_pub->publish(gp_toggle);
                    ROS_INFO("%s Formation Control: %s is now the leader", ros::this_node::getName().c_str(), my_name.data.c_str());    
                } else {
                    //ROS_INFO("%s Formation Control: %s is Following %s", ros::this_node::getName().c_str(), my_name.data.c_str(), status.leader_name.c_str());
                }
            } else {
                // if I am the leader and status is telling me to use the leader, I'm the follower
                if(status.use_leader) {
                    is_leader = false;
                    gp_toggle.data = 0; 
                    gptoggle_pub->publish(gp_toggle);
                    ROS_INFO("%s Formation Control: %s is no longer the leader", ros::this_node::getName().c_str(), my_name.data.c_str());
                } else {
                    //ROS_INFO("%s Formation Control: %s is still the leader", ros::this_node::getName().c_str(), my_name.data.c_str());
                }
            }
        }


        //ROS_INFO("%s Formation Control: Message status: odom %d, ldr odom %d, status %d, is_leader %d", ros::this_node::getName().c_str(), odom_rcvd, ldr_odom_rcvd, status_rcvd, is_leader); 
        if ( (odom_rcvd && ldr_odom_rcvd && status_rcvd && !is_leader) ){
            //ROS_INFO("%s Formation Control: Updating follower controller.", ros::this_node::getName().c_str());
            // update the controller IF all the required messages have been received
            controller.Update(ldr_odom, odom, status);
            // publish the controller state
            //ROS_INFO("%s Formation Control: Publishing speed and path %0.2f", ros::this_node::getName().c_str(), controller.GetSpeed().data);
                
            if(controller.GetSpeed().data > 10.0) {
                avt_341::msg::Float64 spd;
                spd.data = 10.0;
                speed_pub->publish(spd);
            }
            speed_pub->publish(controller.GetSpeed());
            path_pub->publish(controller.GetPath());
        } 

        n->spin_some();
        loop_rate.sleep();
    }
}
