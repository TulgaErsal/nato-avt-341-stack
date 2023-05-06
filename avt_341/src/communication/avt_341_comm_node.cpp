/**
 * \file avt_341_comm_node.cpp
 *
 * ROS node to handle communications with other vehicles on a network
 * 
 * \author Daniel Carruth
 *
 * \contact dwc2@cavs.msstate.edu
 * 
 * \date 2/19/2023
 */

#include <algorithm>
#include <vector>
#include <sstream>
#include <queue>

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/communication/tcp_socket_proxy.h"

std::queue<std::string> pending_msgs;
char message[256] = { 0 };
std::string my_name;
std::shared_ptr<avt_341::node::NodeProxy> nh = nullptr;

void MessageCallback(avt_341::msg::StringPtr msg) {
    pending_msgs.push(msg->data);
    nh->log_info("Received %s to broadcast.", msg->data.c_str());
}

avt_341::msg::Communication packageMessage(std::vector<std::string> tokens) {
    avt_341::msg::Communication message;
    if(tokens.size() < 3){
        return message;
    }
    message.sender_name = tokens[0];
    message.msg_id = atoi(tokens[1].c_str());
    message.type = tokens[2];
    message.priority_type = "Q";

    // <sender>,<msg_id>,FORM,<formation>,<leader>,<f1>,<f2>,<f3>,<objective>,<speed>
    if(message.type == "FORM") {
        message.formation = tokens[3];
        message.leader_name = tokens[4];
        message.follower1_name = tokens[5];
        message.follower2_name = tokens[6];
        message.follower3_name = tokens[7];
        message.objective_name = tokens[8];
        message.desired_speed = tokens[9];
        message.x_scale = -1.0;
        message.y_scale = -1.0;

        if(tokens.size() == 11) {
            message.priority_type = tokens[10];
        }else if(tokens.size() > 11) {
            message.x_scale = std::stod(tokens[10]);
            message.y_scale = std::stod(tokens[11]);
            if(tokens.size() > 12){
                message.priority_type = tokens[12];
            }
        }
    } 
    // <sender>,<msg_id>,ACK,<orig_msg_sender>,<orig_msg_id>
    else if(message.type == "ACK") {          
        message.original_sender = tokens[3];
        message.original_msg_id = tokens[4];
    } 
    // <sender>,<msg_id>,ARRIVE,<objective>
    else if(message.type == "ARRIVE") {
        message.objective_name = tokens[3];
    } 
    // <sender>,<msg_id>,TASK_COMPLETE,<orig_msg_sender>,<orig_msg_id>
    else if(message.type == "TASK_COMPLETE") {
        message.original_sender = tokens[3];
        message.original_msg_id = tokens[4];
    } 
    // <sender>,<msg_id>,MOVETO,<receiver>,<objective>
    else if(message.type == "MOVETO") {
        message.receiver_name = tokens[3];
        message.objective_name = tokens[4];
        if(tokens.size() > 5) {
          message.priority_type = tokens[5];
        }
    } 
    // <sender>,<msg_id>,SHUTDOWN,<receiver>
    else if(message.type == "SHUTDOWN") {
        message.receiver_name = tokens[3];
    }
    // <sender>,<msg_id>,SET_SPEED,<receiver>,<speed>
    else if(message.type == "SET_SPEED") {
        message.receiver_name = tokens[3];
        message.desired_speed = tokens[4];
    }
    else if(message.type == "CANCEL") {
      message.receiver_name = tokens[3];
      message.target_msg_id = atoi(tokens[4].c_str());
    }
    else if(message.type == "CANCEL_ALL") {
      message.receiver_name = tokens[3];
    }
    
    return message;
}

std::vector<std::string> tokenize_msg(std::string input){
    std::vector<std::string> tokens;
    size_t pos = 0;
    std::string token;
    while ((pos = input.find(",")) != std::string::npos) {
        token = input.substr(0, pos);
        tokens.push_back(token);
        input.erase(0, pos + 1);
    }
    tokens.push_back(input);
    return tokens;
}

int main(int argc, char* argv[])
{
    // Initialize the node
    nh = avt_341::node::init_node(argc,argv,"avt_341_comm_node");
    avt_341::node::Rate loop_rate(100.0);

    // Set up subscriptions
    // Subscribe to avt_341/comm_messages to catch messages that should be relayed to the network
    auto msg_sub = nh->create_subscription<avt_341::msg::String>("avt_341/comm_messages", 10, MessageCallback);

    // Set up publishers
    auto msg_pub = nh->create_publisher<avt_341::msg::Communication>("avt_341/recv_comms", 10);

    int port, msg_count = 0;
    bool disable_socket_comms, broadcast_internal, add_name_id_to_msg;
    char buffer[256];
    std::string hostname;
    avt_341::msg::Communication packed_msg;

    // load parameters
    nh->get_parameter("~host", hostname, std::string("localhost"));
    nh->get_parameter("~port", port, 9000);
    nh->get_parameter("~name", my_name, std::string("AGV1"));
    nh->get_parameter("~disable_socket_comms", disable_socket_comms, false);
    nh->get_parameter("~broadcast_internal", broadcast_internal, false);
    nh->get_parameter("~add_name_id_to_msg", add_name_id_to_msg, true);

    nh->log_info("Connecting to server: %s:%d, name: %s, disable_socket_comms: %d, broadcast_internal: %d",
                 hostname.c_str(), port, my_name.c_str(), disable_socket_comms, broadcast_internal);

    // Create the socket
    std::shared_ptr<avt_341::communication::TcpSocketClientBase> client = nullptr;
    if(disable_socket_comms){
        client = std::make_shared<avt_341::communication::NullTcpSocketClient>();
    }else{
        client = std::make_shared<avt_341::communication::TcpSocketClient>(hostname, port);
    }
    broadcast_internal |= disable_socket_comms;

    // connect to the server
    if(!client->connect())
        nh->log_info("Error connecting to the server\n");

    while (avt_341::node::ok()) {
        std::fill(std::begin(buffer), std::end(buffer), '\0');

        // Check for any messages ready to send
        while(!pending_msgs.empty()) {

            std::string next_msg = pending_msgs.front();
            pending_msgs.pop();

            if(broadcast_internal){
                auto comm_msg = packageMessage(tokenize_msg(next_msg));
                if(!comm_msg.type.empty()){
                    msg_pub->publish(comm_msg);
                }
            }

            // append name and message
            if(add_name_id_to_msg){
                std::ostringstream stream;
                stream << my_name << "," << msg_count++ << "," << next_msg;
                next_msg = stream.str();
            }

            nh->log_info("Broadcasting %s", next_msg.c_str());

            strcpy(buffer, next_msg.c_str());
            int n = client->write(buffer, strlen(buffer));
            if(n < 0)
                nh->log_error("Error writing to socket\n");
            else {
                nh->log_debug("Message sent and buffer cleared.\n");
            }
        }

        std::fill(std::begin(buffer), std::end(buffer), '\0');

        // read the socket
        if(client->read_available(buffer, 256) > 0)
        {
            std::string buffer_str = std::string(buffer);
            nh->log_info("Read %s", buffer_str.c_str());
            packed_msg = packageMessage(tokenize_msg(buffer_str));
            msg_pub->publish(packed_msg);
        }

        nh->spin_some();
        loop_rate.sleep();
    }
    return 0;
}

