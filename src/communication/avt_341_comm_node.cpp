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

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/Communication.h"
#include "avt_341/FollowerStatus.h"

char message[256] = { 0 };
bool messages_ready = 0;
bool odom_rcvd = false;
int res = 0;
avt_341::msg::String my_name;
avt_341::msg::String leader_name;
avt_341::msg::Odometry leader_odom;
float vehicle_scale = 1;

struct formation {
    avt_341::msg::Point follower1;
    avt_341::msg::Point follower2;
    avt_341::msg::Point follower3;
};

std::map<std::string, formation> formations;

struct formation f;
avt_341::FollowerStatus follower_status_message;

void MessageCallback(avt_341::msg::StringPtr msg) {
    memset(message, 0, 256);
    strcpy(message, msg->data.c_str());
    messages_ready = 1;
    ROS_INFO("Comm Node has received message '%s' to broadcast to the network.", message);
}

void VehOdomCallback(avt_341::msg::OdometryPtr msg) {
    // grab leader odom to forward
    if(!strcmp(msg->child_frame_id.c_str(), leader_name.data.c_str())) {
        //ROS_INFO("%s : %s received odometry from the leader: %s Leader: %s", ros::this_node::getName().c_str(), my_name.data.c_str(), msg->child_frame_id.c_str(), leader_name.data.c_str());
        leader_odom = *msg;
        odom_rcvd = true;
    }
}

void ClearMessages() {
    messages_ready = 0;
    bzero(message, 256);
}

int handleFormationRequest(avt_341::Communication message) {
    if(strcmp(message.type.c_str(),"FORM")) {
        return 0;
    }
    // create a follower status message
    follower_status_message.leader_name = message.leader_name;
    
    if(!strcmp(message.leader_name.c_str(), my_name.data.c_str())) {
            ROS_INFO("%s %s taking Lead Position", ros::this_node::getName().c_str(), my_name.data.c_str());
            follower_status_message.x_offset = 0;
            follower_status_message.y_offset = 0;
            follower_status_message.use_leader = false;
            leader_name.data = message.leader_name;
    } else {
        formation f = formations[message.formation.c_str()];
        if(!strcmp(message.follower1_name.c_str(), my_name.data.c_str())) {
            ROS_INFO("%s %s Setting x,y offset: %f, %f", ros::this_node::getName().c_str(), my_name.data.c_str(), f.follower1.x * vehicle_scale, f.follower1.y * vehicle_scale);
            follower_status_message.x_offset = f.follower1.x * vehicle_scale;
            follower_status_message.y_offset = f.follower1.y * vehicle_scale;
        } else if (!strcmp(message.follower2_name.c_str(), my_name.data.c_str())) {
            ROS_INFO("%s %s Setting x,y offset: %f, %f", ros::this_node::getName().c_str(), my_name.data.c_str(), f.follower2.x * vehicle_scale, f.follower2.y * vehicle_scale);
            follower_status_message.x_offset = f.follower2.x * vehicle_scale;
            follower_status_message.y_offset = f.follower2.y * vehicle_scale;
        } else if (!strcmp(message.follower3_name.c_str(), my_name.data.c_str())) {
            ROS_INFO("%s %s Setting x,y offset: %f, %f", ros::this_node::getName().c_str(), my_name.data.c_str(), f.follower3.x * vehicle_scale, f.follower3.y * vehicle_scale);
            follower_status_message.x_offset = f.follower3.x * vehicle_scale;
            follower_status_message.y_offset = f.follower3.y * vehicle_scale;
        } else {
            ROS_INFO("%s %s: Formation message is not for me.", ros::this_node::getName().c_str(), my_name.data.c_str());
            return 0;
        }
        follower_status_message.use_leader = true;
        leader_name.data = message.leader_name;
        ROS_INFO("%s %s setting %s as leader", ros::this_node::getName().c_str(), my_name.data.c_str(), leader_name.data.c_str());
    }
    return 1;
}

avt_341::Communication packageMessage(std::vector<std::string> tokens) {
    avt_341::Communication message;
    message.sender_name = tokens[0];
    message.msg_id = atoi(tokens[1].c_str());
    message.type = tokens[2];
    
    if(!strcmp(tokens[2].c_str(), "FORM")) {
        message.formation = tokens[3];
        message.leader_name = tokens[4];
        message.follower1_name = tokens[5];
        message.follower2_name = tokens[6];
        message.follower3_name = tokens[7];
        message.objective_name = tokens[8];
        message.desired_speed = tokens[9];
    } else if(!strcmp(message.type.c_str(), "ACK")) {
        message.original_sender = tokens[3];
        message.original_msg_id = tokens[4];
    } else if(!strcmp(message.type.c_str(), "ARRIVE")) {
        message.objective_name = tokens[3];
    } else if(!strcmp(message.type.c_str(), "TASK_COMPLETE")) {
        message.original_sender = tokens[3];
        message.original_msg_id = tokens[4];
    } else if(!strcmp(message.type.c_str(), "SET_OBJECTIVE")) {
        message.objective_name = tokens[3];
    } else if(message.type == "MOVETO") {
        message.objective_name = tokens[3];
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
    auto veh1_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/veh1_odometry", 10, VehOdomCallback);
    auto veh2_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/follower1_odometry", 10, VehOdomCallback);
    auto veh3_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/follower2_odometry", 10, VehOdomCallback);
    auto veh4_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/follower3_odometry", 10, VehOdomCallback);

    // Set up publishers
    auto msg_pub = nh->create_publisher<avt_341::Communication>("avt_341/recv_comms", 10);
    auto formation_pub = nh->create_publisher<avt_341::FollowerStatus>("avt_341/follower_status",10);
    auto leader_pub = nh->create_publisher<avt_341::msg::Odometry>("avt_341/leader_odometry", 10);
       
    f.follower1.x = 0;
    f.follower1.y = -1;
    f.follower2.x = 0;
    f.follower2.y = -2;
    f.follower3.x = 0;
    f.follower3.y = -3;
    formations["LINE"] = f; 
    f.follower1.x = -1;
    f.follower1.y = 0;
    f.follower2.x = -2;
    f.follower2.y = 0;
    f.follower3.x = -3;
    f.follower3.y = 0;
    formations["COLUMN"] = f; 
    f.follower1.x = -1;
    f.follower1.y = 1;
    f.follower2.x = -2;
    f.follower2.y = 0;
    f.follower3.x = -3;
    f.follower3.y = 1;
    formations["STAGGER_COL"] = f; 
    f.follower1.x = -1;
    f.follower1.y = 1;
    f.follower2.x = -1;
    f.follower2.y = -1;
    f.follower3.x = -2;
    f.follower3.y = 0;
    formations["DIAMOND"] = f; 
    f.follower1.x = -1;
    f.follower1.y = 1;
    f.follower2.x = 0;
    f.follower2.y = -1;
    f.follower3.x = -1;
    f.follower3.y = -2;
    formations["WEDGE"] = f; 
    f.follower1.x = -1;
    f.follower1.y = 1;
    f.follower2.x = -2;
    f.follower2.y = 2;
    f.follower3.x = -3;
    f.follower3.y = 3;
    formations["ECH_LEFT"] = f; 
    f.follower1.x = -1;
    f.follower1.y = -1;
    f.follower2.x = -2;
    f.follower2.y = -2;
    f.follower3.x = -3;
    f.follower3.y = -3;
    formations["ECH_RIGHT"] = f; 

    int n, sockfd, port, ready;
    fd_set read_fds;
    struct timeval timeout;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];
    std::string hostname;
    avt_341::Communication packed_msg;

    // load parameters
    nh->get_parameter("~host", hostname, std::string("localhost"));
    nh->get_parameter("~port", port, 9000);
    nh->get_parameter("~name", my_name.data, std::string("AGV1"));
    nh->get_parameter("~scale", vehicle_scale, 1.0f);
    
    ROS_INFO("Connecting to host: %s port: %d", hostname.c_str(), port);
    // Create the socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 9) 
        ROS_ERROR("Error opening socket\n");
    server = gethostbyname(hostname.c_str());
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

        if(odom_rcvd) {
            //ROS_INFO("%s %s Publishing odom of leader %s", ros::this_node::getName().c_str(), my_name.data.c_str(), leader_odom.child_frame_id.c_str());
            leader_pub->publish(leader_odom);
        }
        // Check for any messages ready to send
        if(messages_ready) {
            strcpy(buffer,message);
            //ROS_INFO("Sending %s to server", buffer);
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
                //ROS_INFO("Read %d bytes: '%s'\n", n, buffer);
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

                if(!(strcmp(packed_msg.type.c_str(),"FORM"))) {
                    if(handleFormationRequest(packed_msg) == 1) {
                        // publish our formation message
                        formation_pub->publish(follower_status_message);
                    }
                }
                // Publish the packed_msg
                msg_pub->publish(packed_msg);
            }                
        } else {
            //ROS_DEBUG("No socket ready to read\n");
        }      
        nh->spin_some();
        loop_rate.sleep();
    }
    return 0;
}

