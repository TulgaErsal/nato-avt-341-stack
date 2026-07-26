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
#include "avt_341/mission/mission_manager_parser.h"
#include <avt_341/socket_comms_params_service.hpp>

std::queue<avt_341::msg::Communication> pending_msgs;
char message[256] = { 0 };
std::shared_ptr<avt_341::node::NodeProxy> nh = nullptr;

void MessageCallback(avt_341::msg::CommunicationPtr msg,
                     const bool verbose_comm_log) {
    pending_msgs.push(*msg);
    if(verbose_comm_log){
      nh->log_info("Received sender=%s,msg_id=%d,type=%s,receiver=%s to broadcast.", msg->sender_name.c_str(), msg->msg_id, msg->type.c_str(), msg->receiver_name.c_str());
    }
}

int main(int argc, char* argv[])
{
    // Initialize the node
    nh = avt_341::node::init_node(argc,argv,"avt_341_comm_node");
    avt_341::params::socket_comms::ParamsListener param_listener(nh->get_raw_node());
    const auto params = param_listener.get_params();
    avt_341::node::Rate loop_rate(100.0);

    // Set up subscriptions
    // Subscribe to avt_341/comm_messages to catch messages that should be relayed to the network
    auto msg_sub = nh->create_subscription<avt_341::msg::Communication>(
        "avt_341/comm_messages", 10,
        [&params](avt_341::msg::CommunicationPtr msg) {
            MessageCallback(msg, params.verbose_comm_log);
        });

    // Set up publishers
    auto msg_pub = nh->create_publisher<avt_341::msg::Communication>("avt_341/comm_messages", 10);

    int msg_count = 0;
    char buffer[256];
    avt_341::msg::Communication packed_msg;
    const int port = static_cast<int>(params.port);

    // If broadcast_over_ros, comm node publishes to other vehicles on ros network instead of using tcp client
    std::vector<std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Communication>>> other_veh_pubs;
    if(params.broadcast_over_ros){
      for(const auto & veh_ns: params.vehicle_namespaces){
        std::string veh_ns_upper = veh_ns;
        std::transform(veh_ns_upper.begin(), veh_ns_upper.end(), veh_ns_upper.begin(), [](unsigned char c){ return std::toupper(c); });
        if(veh_ns_upper != params.name){
          other_veh_pubs.push_back(nh->create_publisher<avt_341::msg::Communication>("/" + veh_ns + "/avt_341/comm_messages", 10));
        }
      }
    }

    const bool disable_socket_comms = params.broadcast_over_ros;
    nh->log_info("Connecting to server: %s:%d, name: %s, disable_socket_comms: %d, broadcast_over_ros: %d, other_veh_pubs: %d",
                 params.host.c_str(), port, params.name.c_str(),
                 disable_socket_comms, params.broadcast_over_ros,
                 other_veh_pubs.size());

    // Create the socket
    std::shared_ptr<avt_341::communication::TcpSocketClientBase> client = nullptr;
    if(disable_socket_comms){
        client = std::make_shared<avt_341::communication::NullTcpSocketClient>();
    }else{
        client = std::make_shared<avt_341::communication::TcpSocketClient>(
            params.host, port);
    }

    // connect to the server
    if(!client->connect())
        nh->log_info("Error connecting to the server\n");

    while (avt_341::node::ok()) {
        std::fill(std::begin(buffer), std::end(buffer), '\0');

        // Check for any messages ready to send
        while(!pending_msgs.empty()) {

            avt_341::msg::Communication next_msg = pending_msgs.front();
            pending_msgs.pop();

            // Only broadcast messages to other vehicles that are from myself
            if(next_msg.sender_name != params.name){
              continue;
            }

            std::string msg_serialized = rosToSerializedMsg(next_msg);
            if(params.verbose_comm_log){
              nh->log_info("Broadcasting %s", msg_serialized.c_str());
            }

            if(params.broadcast_over_ros){
              for(const auto & pub: other_veh_pubs){
                pub->publish(next_msg);
              }
            }else{
              strcpy(buffer, msg_serialized.c_str());
              int n = client->write(buffer, strlen(buffer));
              if(n < 0)
                nh->log_error("Error writing to socket\n");
              else {
                nh->log_debug("Message sent and buffer cleared.\n");
              }
            }
        }

        std::fill(std::begin(buffer), std::end(buffer), '\0');

        // read the socket
        if(client->read_available(buffer, 256) > 0)
        {
            std::string buffer_str = std::string(buffer);
            if(params.verbose_comm_log){
              nh->log_info("Read %s", buffer_str.c_str());
            }
            packed_msg = serializedToROSMsg(buffer_str);
            msg_pub->publish(packed_msg);
        }

        nh->spin_some();
        loop_rate.sleep();
    }
    return 0;
}

