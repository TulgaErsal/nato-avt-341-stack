// ros includes
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/mission/mission_manager.h"

avt_341::msg::Communication rcvd_msg;
bool message_rcvd = false;

avt_341::msg::Odometry leader_odom;
bool leader_odom_rcvd = false;

std::string leader_name;
std::string mission_definition_filename;

// Receive updates from comms
void CommunicationCallback(avt_341::msg::CommunicationPtr msg) {
    std::cout << ros::this_node::getName() << " Mission Manager received communication" << std::endl;
    rcvd_msg = *msg;
    message_rcvd = true;
}

// Receive updated odometry information
void VehicleOdometryCallback(avt_341::msg::OdometryPtr msg) {
    // std::cout << ros::this_node::getName() << " Mission Manager received odometry" << std::endl;
    if(msg->child_frame_id == leader_name) {
        leader_odom = *msg;
        leader_odom_rcvd = true;
    }
}

int main(int argc, char **argv) {

    // initialize the node
    auto nh = avt_341::node::init_node(argc, argv, "mission_manager");
    avt_341::node::Rate loop_rate(10);

    // set up subscriptions
    auto communication_sub = nh->create_subscription<avt_341::msg::Communication>("avt_341/recv_comms", 10, CommunicationCallback);
    auto veh1_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/veh1_odometry", 10, VehicleOdometryCallback);
    auto veh2_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/veh2_odometry", 10, VehicleOdometryCallback);
    auto veh3_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/veh3_odometry", 10, VehicleOdometryCallback);
    auto veh4_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/veh4_odometry", 10, VehicleOdometryCallback);

    // create the publishers, each vehicle publishes a path and a desired speed
    auto waypoint_pub = nh->create_publisher<avt_341::msg::Path>("avt_341/new_waypoints", 10);
    auto follower_status_pub = nh->create_publisher<avt_341::msg::FollowerStatus>("avt_341/follower_status",10);
    auto leader_pub = nh->create_publisher<avt_341::msg::Odometry>("avt_341/leader_odometry", 10);
	auto navcommand_pub = nh->create_publisher<avt_341::msg::Int32>("avt_341/nav_command_state", 10);

    // create the manager
    avt_341::mission::MissionManager mgr;

    // load the parameters
    nh->get_parameter("~name", mgr.my_name, std::string("AGV1"));
    nh->get_parameter("~mission_definition_file", mission_definition_filename, std::string("mission.csv"));
    nh->get_parameter("~follow_scale", mgr.follow_scale, 1.0f);
    
    std::cout << "Load Mission" << std::endl;
    mgr.loadMissionDefinition(mission_definition_filename);

    // start the loop
    while(avt_341::node::ok()){
        if(leader_odom_rcvd) {
            std::cout << ros::this_node::getName() << " is publishing odometry of the leader " << mgr.leader_name << std::endl;
            leader_pub->publish(leader_odom); 
        }
        if(message_rcvd) {
            std::cout << ros::this_node::getName() << " handling message" << std::endl;
            message_rcvd = false;

            if(rcvd_msg.type == "FORM") {
                mgr.handleFormationRequest(rcvd_msg);
                if(mgr.follower_status_msg_updated) {
                    leader_name = mgr.follower_status_message.leader_name;
                    follower_status_pub->publish(mgr.follower_status_message);
                    mgr.follower_status_msg_updated = false;
                }
            } else if(rcvd_msg.type == "MOVETO") {
                mgr.handleMoveTo(rcvd_msg);
                if(mgr.path_msg_updated) {
					avt_341::msg::Int32 go_command;
					go_command.data = 1;
                    waypoint_pub->publish(mgr.path_msg);
					navcommand_pub->publish(go_command);
                    mgr.path_msg_updated = false;
                }
            }
        }

        nh->spin_some();
        loop_rate.sleep();
    }
}
