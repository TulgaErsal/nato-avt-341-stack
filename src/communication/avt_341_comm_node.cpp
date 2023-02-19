#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

enum FormationShape{LINE, COLUMN, DIAMOND, WEDGE};
const char* shape_name[4] = {"LINE", "COLUMN", "DIAMOND", "WEDGE"};

char message[256] = { 0 };
bool messages_ready = 0;
int res = 0;
char my_name[80] = "AGV1";

void MessageCallback(avt_341::msg::StringPtr msg) {
    memset(message, 0, 256);
    strcpy(message, msg->data.c_str());
    messages_ready = 1;
    ROS_INFO("Comm Node has received message '%s' to send", message);
}

void ClearMessages() {
    messages_ready = 0;
    bzero(message, 256);
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
    auto msg_sub = nh->create_subscription<avt_341::msg::String>("avt_341/comm_messages", 1, MessageCallback);
    // Set up publishers
    auto waypt_pub = nh->create_publisher<avt_341::msg::Path>("/avt_341/new_waypoints", 10);    

    // set up socket
    int n, sockfd, port, ready;
    fd_set read_fds;
    struct timeval timeout;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];
    char message[256];

    // message content
    // message header
    char sender_name[80];
    uint msg_id;
    // FORM command: FORM,<SHAPE>,<LEADER_NAME>,<F1_NAME>,<F2_NAME>,<F3_NAME>,<F4_NAME>,<OBJECTIVE_NAME>,
    FormationShape shape;
    char leader_name[80];
    char follower1_name[80];
    char follower2_name[80];
    char follower3_name[80];
    char objective_name[80];
    // ACK message: ACKNOWLEDGE,<ORIGINAL_SENDER_NAME>,<ORIGINAL_MSG_ID>,
    char original_sender_name[80];
    uint original_msg_id;
    // ARRIVE message: ARRIVE,<OBJECTIVE_NAME>,
    // - use objective_name from FORM
    // COMPLETE message: TASK_COMPLETE,<ORIGINAL_SENDER_NAME>,<ORIGINAL_MSG_ID>
    // - use original_sender_name and original_msg_id vars defined for Ack
    char original_command[256];
    // PATH message: PATH,<NUM_ELS>,<X1>,<Y1>,...,<XN>,<YN>,
    // complicated. 
    // SET_OBJECTIVE message: SET_OBJECTIVE,<OBJECTIVE_NAME>,<TARGET_SPEED>,
    float target_speed;
    // MOVETO message: MOVETO,<OBJECTIVE_NAME>,

    

    port = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 9) 
        fprintf(stderr,"Error opening socket\n");
    server = gethostbyname(argv[1]);
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
        (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
    serv_addr.sin_port = htons(port);

    // connect
    if (connect(sockfd,(struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        ROS_INFO("Error connecting\n");
    
    while (ros::ok()) {
        bzero(buffer, 256);

        // Check Messenger for any messages ready to send
        if(messages_ready) {
            strcpy(buffer,message);
            // Send message to the server
            ROS_INFO("Sending %s to server\n", buffer);
            n = write(sockfd, buffer, strlen(buffer));
            if(n < 0)
                ROS_INFO("Error writing to socket\n");
            else {
                ClearMessages();
                ROS_DEBUG("Message sent and buffer cleared.\n");
            }
        }

        // Check for messages from server
        bzero(buffer, 256);

        FD_ZERO(&read_fds);
        FD_SET(sockfd, &read_fds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        ready = select(sockfd + 1, &read_fds, NULL, NULL, &timeout);
        if(ready > 0 && FD_ISSET(sockfd, &read_fds)) {
            // read the socket
            n = read(sockfd, buffer, 256);
            if(n > 0) 
            {
                ROS_DEBUG("Read %d bytes: '%s'\n", n, buffer);
                char* token;
                
                // parse the message, then send appropriate messages on to other ROS nodes
                strcpy(message, buffer);
                // Message header SENDER_NAME, MSG_ID, ...
                token = strtok(buffer, ",");
                if(token == NULL) return -1;
                strcpy(sender_name,token);      // store SENDER_NAME
                ROS_INFO("Sender Name: %s", sender_name);
                token = strtok(NULL, ",");
                if(token == NULL) {
                    ROS_ERROR("Malformed FORM command: %s", message);
                    return -1;
                } 
                msg_id = atoi(token);           // store MSG_ID 

                token = strtok(NULL, ",");
// BEGIN FORM
                if(!strcmp(token,"FORM")) {
                    ROS_DEBUG("Handling Formation Command: %s", buffer);
                    FormationShape shape;
                    
                    // Formation Command format: "<SHAPE>,<LEADER>, <F1>, <F2>, <F3>, <OBJNAME>,"
                    token = strtok(NULL, ",");      // extract SHAPE
                    if(!strcmp(token,"LINE")) {
                        shape = LINE;
                        ROS_INFO("Formation Shape = LINE");
                    } else if(!strcmp(token, "COLUMN")) {
                        shape = COLUMN;
                        ROS_INFO("Formation Shape = COLUMN");
                    } else if(!strcmp(token, "DIAMOND")) {
                        shape = DIAMOND;
                        ROS_INFO("Formation Shape = DIAMOND");
                    } else if(!strcmp(token, "WEDGE")) {
                        shape = WEDGE;
                        ROS_INFO("Formation Shape = WEDGE");
                    } else {
                        ROS_INFO("Error in shape definition for formation: %s", token);
                        return -1;
                    }

                    // extract leader_name
                    token = strtok(NULL, ",");
                    if(token == NULL) {
                        ROS_ERROR("Malformed FORM command leader name: %s", message);
                        return -1;
                    } 
                    strcpy(leader_name, token);
                    ROS_INFO("Formation Leader Name: %s", leader_name);
                
                    token = strtok(NULL, ",");
                    if(token == NULL) {
                        ROS_ERROR("Malformed FORM command follower1 name: %s", message);
                        return -1;
                    } 
                    strcpy(follower1_name, token);
                    ROS_INFO("Formation 1 Name: %s", follower1_name);
                
                    token = strtok(NULL, ",");
                    if(token == NULL) {
                        ROS_ERROR("Malformed FORM command follower2 name: %s", message);
                        return -1;
                    } 
                    strcpy(follower2_name, token);
                    ROS_INFO("Formation 2 Name: %s", follower2_name);
                    
                    token = strtok(NULL, ",");
                    if(token == NULL) {
                        ROS_ERROR("Malformed FORM command follower3 name: %s", message);
                        return -1;
                    } 
                    strcpy(follower3_name, token);
                    ROS_INFO("Formation 3 Name: %s", follower3_name);

                    token = strtok(NULL, ",");
                    if(token == NULL) {
                        ROS_ERROR("Malformed FORM command objective name: %s", message);
                        return -1;
                    } 
                    strcpy(objective_name, token);
                    ROS_INFO("Objective Name: %s", objective_name);

                    ROS_INFO("Parsed FORM command from %s %d: %s, %s, %s, %s, %s, %s", sender_name, 
                            msg_id, shape_name[shape], leader_name, follower1_name, follower2_name, 
                            follower3_name, objective_name);

                    // Handle the information
                    if(!strcmp(leader_name, my_name)) {
                        ROS_INFO("I'm the leader!");
                        // leader sets its own path to the goal, assumes the other vehicles are following
                        ROS_INFO("Publish message(s) to internal systems to set goal and start moveto behavior");
                                        
                    } else if(~strcmp(follower1_name, my_name)) {
                        ROS_INFO("I'm in 1st position.");
                        // followers just follow - set leader, set appropriate x/y offset for the formation
                        // should probably be a service
                        ROS_INFO("Set behavior to follow, set leader, set x, set y");

                    }else if(~strcmp(follower2_name, my_name)) {
                        ROS_INFO("I'm in 2nd position.");
                        // followers just follow - set leader, set appropriate x/y offset for the formation
                        ROS_INFO("Publish message(s) to follow, to set leader, to set x, to set y");
                        
                    }else if(~strcmp(follower3_name, my_name)) {
                        ROS_INFO("I'm in 3rd position.");
                        // followers just follow - set leader, set appropriate x/y offset for the formation
                        ROS_INFO("Publish message(s) to follow, to set leader, to set x, to set y");
                        
                    } else {
                        ROS_INFO("I'm not listed in the FORM command. Assuming command is not relevant to me.");
                    }
// END FORM
// BEGIN ACK
                } else if(!strcmp(token,"ACK")) {
                    ROS_INFO("Handling acknowledgement");
                    // ACK message: ACKNOWLEDGE,<ORIGINAL_SENDER_NAME>,<ORIGINAL_MSG_ID>,
                    token = strtok(NULL, ",");
                    if(token == NULL) {
                        ROS_ERROR("Malformed ACK message original sender name: %s", message);
                        return -1;
                    } 
                    strcpy(original_sender_name, token);
                    ROS_INFO("Original Sender Name: %s", original_sender_name);

                    token = strtok(NULL, ",");
                    if(token == NULL) {
                        ROS_ERROR("Malformed ACK message original sender name: %s", message);
                        return -1;
                    } 
                    original_msg_id = atoi(token);
                    ROS_INFO("Original Sender ID: %d", original_msg_id);

                    // Handle the information
                    if(!strcmp(original_sender_name, my_name)) {
                        ROS_INFO("My message %d was acknowledged by %s.", original_msg_id, sender_name);
                    } else {
                        ROS_INFO("The message is for someone else."); 
                        // May want to evaluate for understanding other agents' actions. 
                    }
// END ACK
// BEGIN ARRIVE
                } else if(!strcmp(token, "ARRIVE")) {
                    // ARRIVE message: ARRIVE,<OBJECTIVE_NAME>,
                    token = strtok(NULL, ",");
                    if(token == NULL) {
                        ROS_ERROR("Malformed ARRIVE message - objective name: %s", message);
                        return -1;
                    }
                    strcpy(objective_name, token);
                    ROS_INFO("Objective name: %s", objective_name);

                    // Handle the information
                    // Part of task completion
// END ARRIVE
// BEGIN COMPLETE
                } else if(!strcmp(token, "COMPLETE")) {
                    // COMPLETE message: TASK_COMPLETE,<ORIGINAL_SENDER_NAME>,<ORIGINAL_MSG_ID>
                    token = strtok(NULL, ",");
                    if(token == NULL) {
                        ROS_ERROR("Malformed COMPLETE message - original sender name: %s", message);
                        return -1;
                    }
                    strcpy(original_sender_name, token);
                    ROS_INFO("Original sender name: %s", original_sender_name);

                    token = strtok(NULL, ",");
                     if(token == NULL) {
                        ROS_ERROR("Malformed COMPLETE message - original message id: %s", message);
                        return -1;
                    }
                    original_msg_id = atoi(token);

                    ROS_INFO("Original message ID: %d", original_msg_id);

                    if(!strcmp(original_sender_name, my_name)) {
                        ROS_INFO("I asked for this task in my message %d.", original_msg_id);

                    } else {
                        ROS_INFO("The message is for someone else.");
                    }
// END COMPLETE
// BEGIN UPDATE_OBJECTIVE
                } else if(!strcmp(token, "UPDATE_OBJECTIVE")) {
// END UPDATE_OBJECTIVE
// BEGIN UPDATE_SPEED
                } else if (!strcmp(token, "UPDATE_SPEED")) {
// END UPDATE_SPEED
// BEGIN DETECTED_TARGET
                } else if (!strcmp(token, "DETECTED_TARGET")) {
// END DETECTED_TARGET
// BEGIN DETECTED_THREAT
                } else if (!strcmp(token, "DETECTED_THREAT")) {
// END DETECTED_THREAT

                } else {
                    ROS_INFO("Unknown message type: %s", token);
                }
            } else {
                ROS_INFO("Error reading socket\n");
            }
        } else {
            //fprintf(stderr, "No socket ready to read\n");
        }
              
        nh->spin_some();
        loop_rate.sleep();
    }
    return 0;
}