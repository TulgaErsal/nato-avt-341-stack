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
    
    std::string comms_file = "";
    std::string myid = "vehicle";
    nh->get_parameter("~comms_file", comms_file, std::string("test.txt"));
    nh->get_parameter("~myid", myid, std::string("vehicle"));

    const char* comms[12] = {"vehicle,0,FORM,LINE,AGV1,AGV2,CGV1,CGV2,MISSIONPOINT_A,4",
             "vehicle,1,ACK,vehicle,0",
             "vehicle,2,ARRIVE,MISSIONPOINT_A",
             "vehicle,3,TASK_COMPLETE,vehicle,0",
             "vehicle,4,SET_OBJECTIVE,MISSIONPOINT_B"};

    int count = 0;
    while(ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        //ss << "test_node," << count << ",TEST";
        ss << comms[count];
        msg.data = ss.str();

        test_comm_pub->publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        if(count > 12) count=0;
    }
    return 0;
}