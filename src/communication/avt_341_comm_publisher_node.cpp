/**
 * \file avt_341_comm_publisher_node.cpp
 *
 * ROS node to publish test messages for the comm system 
 * Publishes test message to avt_341/comm_messages
 * 
 * \author Daniel Carruth
 *
 * \contact dwc2@cavs.msstate.edu
 * 
 * \date 2/19/2023
 */

#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <std_msgs/String.h>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

int main(int argc, char** argv)
{
    auto nh = avt_341::node::init_node(argc,argv,"avt_341_comm_publisher_node");
    auto test_comm_pub = nh->create_publisher<std_msgs::String>("avt_341/comm_messages", 100);
    ros::Rate loop_rate(1);
    
    std::vector<std::string> comms_list; 
    std::string myid = "vehicle";
    nh->get_parameter("~comms_list", comms_list, std::vector<std::string>(0));
    nh->get_parameter("~myid", myid, std::string("vehicle"));

    int count = 0;
    while(ros::ok()) {
        std_msgs::String msg;
        
    
        if(comms_list.size() > 0) {
            if(count > comms_list.size()-1) {
                //ROS_INFO("Resetting Comm Count");
                count = 0;
            } else {
                //ROS_INFO("Publishing msg: %d, %s ", count, comms_list[count].c_str());
                msg.data = comms_list[count].c_str(); 
                test_comm_pub->publish(msg);
                //ROS_INFO("Published msg");
                ++count;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}