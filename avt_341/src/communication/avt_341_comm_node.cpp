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

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <vector>
#include <sstream>

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

char message[256] = { 0 };
bool messages_ready = 0;
int res = 0;
avt_341::msg::String my_name;
avt_341::msg::String leader_name;

void MessageCallback(avt_341::msg::StringPtr msg) {
    memset(message, 0, 256);
    strcpy(message, msg->data.c_str());
    messages_ready = 1;
    std::cout << "Comm Node has received message " << message << " to broadcast to the network.";
}

void ClearMessages() {
    messages_ready = 0;
    bzero(message, 256);
}

avt_341::msg::Communication packageMessage(std::vector<std::string> tokens) {
    avt_341::msg::Communication message;
    message.sender_name = tokens[0];
    message.msg_id = atoi(tokens[1].c_str());
    message.type = tokens[2];
    
    // <sender>,<msg_id>,FORM,<formation>,<leader>,<f1>,<f2>,<f3>,<objective>,<speed>
    if(message.type == "FORM") {
        message.formation = tokens[3];
        message.leader_name = tokens[4];
        message.follower1_name = tokens[5];
        message.follower2_name = tokens[6];
        message.follower3_name = tokens[7];
        message.objective_name = tokens[8];
        message.desired_speed = tokens[9];
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
    
    return message;
}

int main(int argc, char* argv[])
{
    // Initialize the node
    auto nh = avt_341::node::init_node(argc,argv,"avt_341_comm_node");
    avt_341::node::Rate loop_rate(100.0);

    // Set up subscriptions
    // Subscribe to avt_341/comm_messages to catch messages that should be relayed to the network
    auto msg_sub = nh->create_subscription<avt_341::msg::String>("avt_341/comm_messages", 1, MessageCallback);

    // Set up publishers
    auto msg_pub = nh->create_publisher<avt_341::msg::Communication>("avt_341/recv_comms", 10);

    int n, sockfd, port, ready, msg_count = 0;
    fd_set read_fds;
    struct timeval timeout;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];
    std::string hostname;
    avt_341::msg::Communication packed_msg;

    // load parameters
    nh->get_parameter("~host", hostname, std::string("localhost"));
    nh->get_parameter("~port", port, 9000);
    nh->get_parameter("~name", my_name.data, std::string("AGV1"));
    
//    nh->log_info("Connecting to host: %s port: %d", hostname.c_str(), port);
    // Create the socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 9) 
        nh->log_error("Error opening socket\n");
    server = gethostbyname(hostname.c_str());
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
        (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
    serv_addr.sin_port = htons(port);

    // connect to the server
    if (connect(sockfd,(struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        nh->log_info("Error connecting to the server\n");
    
    while (avt_341::node::ok()) {
        bzero(buffer, 256);

        // Check for any messages ready to send
        if(messages_ready) {
            // append name and message
            std::ostringstream stream;
            stream << my_name.data << "," << msg_count << "," << message;
            std::cout << stream.str() << std::endl;
            msg_count++;
            
            strcpy(buffer,stream.str().c_str());
            
            n = write(sockfd, buffer, strlen(buffer));
            if(n < 0)
                nh->log_error("Error writing to socket\n");
            else {
                ClearMessages();
                nh->log_debug("Message sent and buffer cleared.\n");
            }
        }

        bzero(buffer, 256);

        FD_ZERO(&read_fds);
        FD_SET(sockfd, &read_fds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        // check for message from the server
        ready = select(sockfd + 1, &read_fds, NULL, NULL, &timeout);
        if(ready > 0 && FD_ISSET(sockfd, &read_fds)) {
            // read the socket
            n = read(sockfd, buffer, 256);
            if(n > 0) 
            {
                //nh->log_info("Read %d bytes: '%s'\n", n, buffer);
                char* token;
                std::vector <std::string> tokens;

                // parse the message, then send appropriate messages on to other ROS nodes
                strcpy(message, buffer);

                // Tokenize incoming message
                token = strtok(buffer, ",");
                //tokens.push_back(token);
                while (token != NULL)
                {
                    tokens.push_back(token);
                    //printf("token: %s\n", token);
                    token = strtok(NULL, ",");
                }
                
                // Package the tokens into message struct
                packed_msg = packageMessage(tokens);

                // Publish the packed_msg
                msg_pub->publish(packed_msg);
            }                
        } else {
            //nh->log_debug("No socket ready to read\n");
        }      
        nh->spin_some();
        loop_rate.sleep();
    }
    return 0;
}

