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

#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <vector>
#include <string>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/Communication.h"

char message[256] = { 0 };
bool messages_ready = 0;
int res = 0;
char my_name[80] = "AGV1";

void MessageCallback(avt_341::msg::StringPtr msg) {
    memset(message, 0, 256);
    strcpy(message, msg->data.c_str());
    messages_ready = 1;
    ROS_INFO("Comm Node has received message '%s' to broadcast to the network.", message);
}

void ClearMessages() {
    messages_ready = 0;
    bzero(message, 256);
}

avt_341::Communication packageMessage(std::vector<std::string> tokens) {
    avt_341::Communication message;
    message.sender_name = tokens.front();
    return message;
}

int main(int argc, char* argv[])
{
    if (argc < 3) {
        fprintf(stderr, "Usage: $ rosrun avt_341_comm_node <hostname> <port>");
        exit(0);
    }

    // Initialize the node
    auto nh = avt_341::node::init_node(argc,argv,"avt_341_comm_node");
    avt_341::node::Rate loop_rate(100.0);

    // Set up subscriptions
    // Subscribe to avt_341/comm_messages to catch messages that should be relayed to the network
    auto msg_sub = nh->create_subscription<avt_341::msg::String>("avt_341/comm_messages", 1, MessageCallback);

    // Set up publishers
    auto msg_pub = nh->create_publisher<avt_341::Communication>("avt_341/recv_comms", 10);
       
    int n, sockfd, port, ready;
    fd_set read_fds;
    struct timeval timeout;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];
    char message[256];
    avt_341::Communication packed_msg;
    
    // Create the socket
    port = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 9) 
        ROS_ERROR("Error opening socket\n");
    server = gethostbyname(argv[1]);
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
        (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
    serv_addr.sin_port = htons(port);

    // connect to the server
    if (connect(sockfd,(struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        ROS_INFO("Error connecting to the server\n");
    
    while (ros::ok()) {
        bzero(buffer, 256);

        // Check for any messages ready to send
        if(messages_ready) {
            strcpy(buffer,message);
            ROS_INFO("Sending %s to server\n", buffer);
            n = write(sockfd, buffer, strlen(buffer));
            if(n < 0)
                ROS_ERROR("Error writing to socket\n");
            else {
                ClearMessages();
                ROS_DEBUG("Message sent and buffer cleared.\n");
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
                ROS_DEBUG("Read %d bytes: '%s'\n", n, buffer);
                char* token;
                std::vector <std::string> tokens;

                // parse the message, then send appropriate messages on to other ROS nodes
                strcpy(message, buffer);

                // Tokenize incoming message
                token = strtok(buffer, ",");
                tokens.push_back(token);
                while (token != NULL)
                {
                    tokens.push_back(token);
                    printf("%s\n", token);
                    token = strtok(NULL, ",");
                }

                // Package the tokens into message struct
                packed_msg = packageMessage(tokens);

                // Publish the packed_msg
                msg_
            }                
        } else {
            //ROS_DEBUG("No socket ready to read\n");
        }      
        nh->spin_some();
        loop_rate.sleep();
    }
    return 0;
}


// handle FORM
// 0            1         2     3        4              5          6          7          8                 9 
// <sender_id>, <msg_id>, FORM, <shape>, <leader_name>, <f1_name>, <f2_name>, <f3_name>, <objective_name>, <desired_speed>
//void handle_form_message() {
    //      set leader_name = tokens[4]
    // if I'm the leader
    //      set use_leader = false
    //      set x_offset = 0
    //      set y_offset = 1
    // if I'm a follower
    //      set use_leader = true
    //      if f1_name = my_name {position = 2}
    //      if f2_name = my_name {position = 3}
    //      if f3_name = my_name {position = 4}
    //      get_offset(shape = tokens[3], position)

//}