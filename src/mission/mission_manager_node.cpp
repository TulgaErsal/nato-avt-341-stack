// ros includes
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/mission/mission_manager.h"

avt_341::msg::Communication rcvd_msg;
bool message_rcvd = false;

avt_341::msg::Odometry odom;
bool odom_rcvd = false;

avt_341::msg::Odometry leader_odom;
bool leader_odom_rcvd = false;

avt_341::msg::Path contacts;
bool contacts_rcvd = false;

avt_341::msg::Int32 nav_state;
bool nav_state_rcvd = false;

std::string leader_name;
std::string mission_definition_filename;
float sodist_threshold;

// Receive updates from comms
void CommunicationCallback(avt_341::msg::CommunicationPtr msg) {
    std::cout << ros::this_node::getName() << " Mission Manager received communication" << std::endl;
    rcvd_msg = *msg;
    message_rcvd = true;
}

void EgoOdometryCallback(avt_341::msg::OdometryPtr msg) {
	// need to know our own position
	odom = *msg;
	odom_rcvd = true;
}

// Receive updated odometry information
void VehicleOdometryCallback(avt_341::msg::OdometryPtr msg) {
    // std::cout << ros::this_node::getName() << " Mission Manager received odometry" << std::endl;
    if(msg->child_frame_id == leader_name) {
        leader_odom = *msg;
        leader_odom_rcvd = true;
    }
}

// Receive information on target contacts
void TargetContactsCallback(avt_341::msg::PathPtr msg) {
	//std::cout << ros::this_node::getName() << " Mission Manager received " << msg->poses.size() << " target contacts" << std::endl;
	contacts = *msg;
	contacts_rcvd = true;
}

// Receive information on navigation state (-1 startup, 0 active, 1 stopping, 2 shutdown, 3 shutdown hard)
void NavStateCallback(avt_341::msg::Int32Ptr msg) {
	//std::cout << ros::this_node::getName() << " Mission Manager received " << msg->data << " navigation state" << std::endl;
	nav_state = *msg;
	nav_state_rcvd = true;
}

int main(int argc, char **argv) {

    // initialize the node
    auto nh = avt_341::node::init_node(argc, argv, "mission_manager");
    avt_341::node::Rate loop_rate(10);

    // set up subscriptions
    auto communication_sub = nh->create_subscription<avt_341::msg::Communication>("avt_341/recv_comms", 10, CommunicationCallback);
	auto odom_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 100, EgoOdometryCallback);
	auto nav_state_sub = nh->create_subscription<avt_341::msg::Int32>("avt_341/state", 10, NavStateCallback);
	auto detect_sub = nh->create_subscription<avt_341::msg::Path>("avt_341/target_contacts", 1, TargetContactsCallback);
    auto veh1_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/veh1_odometry", 10, VehicleOdometryCallback);
    auto veh2_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/veh2_odometry", 10, VehicleOdometryCallback);
    auto veh3_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/veh3_odometry", 10, VehicleOdometryCallback);
    auto veh4_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/veh4_odometry", 10, VehicleOdometryCallback);

    // create the publishers, each vehicle publishes a path and a desired speed
    auto waypoint_pub = nh->create_publisher<avt_341::msg::Path>("avt_341/new_waypoints", 10);
    auto follower_status_pub = nh->create_publisher<avt_341::msg::FollowerStatus>("avt_341/follower_status",10);
    auto leader_pub = nh->create_publisher<avt_341::msg::Odometry>("avt_341/leader_odometry", 10);
	auto navcommand_pub = nh->create_publisher<avt_341::msg::Int32>("avt_341/nav_command_state", 10);
	auto communication_pub = nh->create_publisher<avt_341::msg::String>("avt_341/comm_messages", 100); 

    // create the manager
    avt_341::mission::MissionManager mgr;

    // load the parameters
    nh->get_parameter("~name", mgr.my_name, std::string("AGV1"));
    nh->get_parameter("~mission_definition_file", mission_definition_filename, std::string("mission.csv"));
    nh->get_parameter("~follow_scale", mgr.follow_scale, 1.0f);
	nh->get_parameter("~same_object_distance_threshold", sodist_threshold, 1.0f);
	mgr.same_object_distance_threshold_sq = sodist_threshold * sodist_threshold;
    
    std::cout << "Load Mission" << std::endl;
    mgr.loadMissionDefinition(mission_definition_filename);

    // start the loop
    while(avt_341::node::ok()){
		// Handle external notifications
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

		// Internal notifications
		// Update navigation state
		if(nav_state_rcvd) {
			// store the state in manager
			mgr.nav_state = nav_state.data;
			nav_state_rcvd = false;

			// only handling is ARRIVAL messages - assume state = 1 means we've arrived
			if(mgr.nav_state == 1) {
				std::ostringstream stream;
				stream << mgr.my_name << ",1,ARRIVE,TEMP_NAME";
				avt_341::msg::String msg;
				msg.data = stream.str();
				communication_pub->publish(msg);
			}
		}
		// Update our own odometry
		if(odom_rcvd) {
			//std::cout << "Updated own odometry" << std::endl;
			mgr.odometry = odom;
			odom_rcvd = false;
		}
		// Forward leader odometry to formation control
        if(leader_odom_rcvd) {
            //std::cout << ros::this_node::getName() << " is publishing odometry of the leader " << mgr.leader_name << std::endl;
            leader_pub->publish(leader_odom); 
			leader_odom_rcvd = false;
        }
		// Handle detection of potential targets of interest
		if(contacts_rcvd) {
			//std::cout << ros::this_node::getName() << " handling " << contacts.poses.size() << " contacts" << std::endl;
			if(mgr.nav_state != -1) {
				contacts_rcvd = false;
				mgr.handleContacts(contacts); 
			}
		}

        nh->spin_some();
        loop_rate.sleep();
    }
}
