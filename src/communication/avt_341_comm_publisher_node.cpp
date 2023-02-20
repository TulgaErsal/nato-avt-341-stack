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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avt_341_comm_publisher_node");
    ros::NodeHandle nh;
    ros::Publisher test_comm_pub = nh.advertise<std_msgs::String>("avt_341/comm_messages", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Test message " << count;
        msg.data = ss.str();

        test_comm_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}