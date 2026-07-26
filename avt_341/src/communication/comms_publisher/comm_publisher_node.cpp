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

#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341/node/node_utils.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("avt_341_comm_publisher_node");
    auto test_comm_pub = nh->create_publisher<std_msgs::msg::String>("avt_341/comm_messages", 100);
    rclcpp::Rate loop_rate(1);
    
    std::vector<std::string> comms_list; 
    std::string myid = "vehicle";
    avt_341::node::get_parameter(nh, "~comms_list", comms_list, std::vector<std::string>(0));
    avt_341::node::get_parameter(nh, "~myid", myid, std::string("vehicle"));

    int count = 0;
    while(rclcpp::ok()) {
        std_msgs::msg::String msg;
        
    
        if(comms_list.size() > 0) {
            if(count > comms_list.size()-1) {
                //log_info("Resetting Comm Count");
                count = 0;
            } else {
                //log_info("Publishing msg: %d, %s ", count, comms_list[count].c_str());
                msg.data = comms_list[count].c_str(); 
                test_comm_pub->publish(msg);
                //log_info("Published msg");
                ++count;
            }
        }
        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }
    return 0;
}
