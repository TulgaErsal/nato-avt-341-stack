// ros includes
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/mission/mission_manager.h"
#include <queue>

std::queue<avt_341::msg::Communication> comm_msgs;
std::queue<avt_341::msg::PoseStamped> reached_goals;
std::queue<avt_341::msg::Path> contacts;

avt_341::msg::Odometry odom;
bool odom_rcvd = false;

avt_341::msg::Int32 nav_state;
bool nav_state_rcvd = false;

std::string mission_definition_filename;
float sodist_threshold;

std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Odometry>> leader_pub;
std::shared_ptr<avt_341::mission::MissionManager> mgr;

std::map<std::string, avt_341::msg::Odometry> formation_poses;
std::shared_ptr<avt_341::node::NodeProxy> nh = nullptr;

// Receive updates from comms
void CommunicationCallback(avt_341::msg::CommunicationPtr msg) {
    comm_msgs.push(*msg);
}

void EgoOdometryCallback(avt_341::msg::OdometryPtr msg) {
	// need to know our own position
	odom = *msg;
	odom_rcvd = true;
}

// Receive updated odometry information
void VehicleOdometryCallback(avt_341::msg::OdometryPtr msg) {
    // Ensure same case as leader_name
    std::string child_frame_id = msg->child_frame_id;
    avt_341::mission::Task* current_task = mgr->currentTask();

    std::transform(child_frame_id.begin(), child_frame_id.end(), child_frame_id.begin(), [](unsigned char c){ return std::toupper(c); });
    std::string veh_name = child_frame_id.substr(0, child_frame_id.find('/'));
    formation_poses[veh_name] = *msg;

    if(current_task == nullptr || !current_task->hasFormation()){
      return;
    }

    const std::string leader_name = current_task->getFormationDef()->followedVehicle();

    if(!leader_name.empty() && child_frame_id.find(leader_name) != std::string::npos ) {
      leader_pub->publish(*msg);
    }
}

// Receive information on target contacts
void TargetContactsCallback(avt_341::msg::PathPtr msg) {
	//std::cout << ros::this_node::getName() << " Mission Manager received " << msg->poses.size() << " target contacts" << std::endl;
  // Handle detection of potential targets of interest
  if(mgr->nav_state != avt_341::utils::NavStackState::NotInit) {       // avoid premature detection of contacts
    contacts.push(*msg);
  }
}

// Receive information on navigation state (-1 startup, 0 active, 1 stopping, 2 shutdown, 3 shutdown hard)
void NavStateCallback(avt_341::msg::Int32Ptr msg) {
	//std::cout << ros::this_node::getName() << " Mission Manager received " << msg->data << " navigation state" << std::endl;
	nav_state = *msg;
	nav_state_rcvd = true;
}

bool reset_called = false;
void ResetCallback(avt_341::msg::Int32Ptr msg){
  reset_called = true;
}

void GoalReachedCallback(avt_341::msg::PoseStampedPtr msg){
  reached_goals.push(*msg);
}

bool current_goal_rcvd = false;
avt_341::msg::PoseStamped gp_goal_rcvd;
void CurrentGoalCallback(avt_341::msg::PoseStampedPtr msg){
  gp_goal_rcvd = *msg;
  current_goal_rcvd = true;
}

int main(int argc, char **argv) {

    // initialize the node
    nh = avt_341::node::init_node(argc, argv, "mission_manager");
    avt_341::node::Rate loop_rate(10);

    // load the parameters
    avt_341::mission::FormationSpeedControlParams fsc_params{};
    avt_341::mission::FormationParameters formation_params;
    avt_341::mission::ToiParameters toi_params;
    std::string fsc_type;
    std::vector<std::string> veh_namespaces;

    nh->get_parameter("~name", formation_params.my_name, std::string("AGV1"));
    nh->get_parameter("~mission_definition_file", mission_definition_filename, std::string("mission.csv"));
    nh->get_parameter("~follow_scale_x", formation_params.follow_scale_x, 1.0f);
    nh->get_parameter("~follow_scale_y", formation_params.follow_scale_y, 1.0f);
    nh->get_parameter("~same_object_distance_threshold", sodist_threshold, 1.0f);

    nh->get_parameter("~oof_threshold", fsc_params.oof_threshold, 15.0);
    nh->get_parameter("~oof_const_term", fsc_params.oof_const_term, 0.3);
    nh->get_parameter("~oof_lin_slope", fsc_params.oof_lin_slope, 0.03);
    nh->get_parameter("~oof_mult", fsc_params.oof_mult, 1.5);
    nh->get_parameter("~formation_debug_visualize", fsc_params.debug_visualize, false);
    nh->get_parameter("~offsets_from_leader", formation_params.offsets_from_leader, true);
    nh->get_parameter("~follower_dist_break", fsc_params.follower_dist_break, 10.0);
    nh->get_parameter("~follower_dot_threshold", fsc_params.follower_dot_threshold, 0.0);
    nh->get_parameter("~follower_dot_range", fsc_params.follower_dot_range, 30.0);
    nh->get_parameter("~fsc_type", fsc_type, FormationSpeedControlType::SPEED_UP_FOLLOWER);
    nh->get_parameter("~veh_namespaces", veh_namespaces, std::vector<std::string>{"agv1", "agv2", "cgv1", "cgv2"});

    bool add_name_id_to_msg;
    nh->get_parameter("~toi_approach_dist", toi_params.approach_dist, 15.0f);
    nh->get_parameter("~toi_encircle_radius",  toi_params.encircle_radius, 10.0f);
    nh->get_parameter("~toi_encircle_degrees", toi_params.encircle_degrees, 180.0f);
    nh->get_parameter("~toi_encircle_cw", toi_params.encircle_cw, true);
    nh->get_parameter("~toi_goal_threshold", toi_params.goal_threshold, 5.0f);
    nh->get_parameter("~add_name_id_to_msg", add_name_id_to_msg, false);

    bool use_slow_down_speed_control = fsc_type == FormationSpeedControlType::SLOW_DOWN_LEADER;

    mgr = std::make_shared<avt_341::mission::MissionManager>(formation_params, toi_params, nh, add_name_id_to_msg);
    mgr->sodist_threshold = sodist_threshold;

    avt_341::mission::FormationSpeedController speedController(formation_params.my_name, fsc_params, nh);

    nh->log_info("%s loading definition file %s", mgr->my_name.c_str(), mission_definition_filename.c_str());
    mgr->loadMissionDefinition(mission_definition_filename);

    // set up subscriptions
    auto communication_sub = nh->create_subscription<avt_341::msg::Communication>("avt_341/recv_comms", 10, CommunicationCallback);
    auto odom_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 100, EgoOdometryCallback);
    auto nav_state_sub = nh->create_subscription<avt_341::msg::Int32>("avt_341/state", 10, NavStateCallback);
    auto detect_sub = nh->create_subscription<avt_341::msg::Path>("avt_341/target_contacts", 1, TargetContactsCallback);
    auto veh1_sub = nh->create_subscription<avt_341::msg::Odometry>("/" + veh_namespaces[0] + "/avt_341/odometry", 10, VehicleOdometryCallback);
    auto veh2_sub = nh->create_subscription<avt_341::msg::Odometry>("/" + veh_namespaces[1] + "/avt_341/odometry", 10, VehicleOdometryCallback);
    auto veh3_sub = nh->create_subscription<avt_341::msg::Odometry>("/" + veh_namespaces[2] + "/avt_341/odometry", 10, VehicleOdometryCallback);
    auto veh4_sub = nh->create_subscription<avt_341::msg::Odometry>("/" + veh_namespaces[3] + "/avt_341/odometry", 10, VehicleOdometryCallback);
    auto reset_sub = nh->create_subscription<avt_341::msg::Int32>("avt_341/reset", 10, ResetCallback);
    auto goal_reached_sub = nh->create_subscription<avt_341::msg::PoseStamped>("avt_341/goal_reached", 10, GoalReachedCallback);
    auto current_waypoint_sub = nh->create_subscription<avt_341::msg::PoseStamped>("avt_341/current_waypoint", 10, CurrentGoalCallback);

    auto speed_factor_pub = nh->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed_factor", 10);
    leader_pub = nh->create_publisher<avt_341::msg::Odometry>("avt_341/leader_odometry", 10);

    // start the loop
    while(avt_341::node::ok()){
    // Handle external notifications

        if(reset_called){
          nh->log_info("Resetting node");
          mgr->reset();
          reset_called = false;
        }

        if(current_goal_rcvd){
          mgr->current_gp_goal = gp_goal_rcvd;
          current_goal_rcvd = false;
        }

        while(!reached_goals.empty()){
            auto goal = reached_goals.front();
            reached_goals.pop();
            mgr->onGoalReached(goal);
        }

        while(!comm_msgs.empty()){
            auto rcvd_msg = comm_msgs.front();
            comm_msgs.pop();
            if(!mgr->isMsgForSelf(rcvd_msg)){
              continue;
            }
            if(rcvd_msg.type != "TASK_COMPLETE"){
                nh->log_info("%s handling message: type=%s, id=%d", mgr->my_name.c_str(), rcvd_msg.type.c_str(), rcvd_msg.msg_id);
            }

            if(rcvd_msg.type == "FORM") {
                mgr->handleFormationRequest(rcvd_msg);
            } else if(rcvd_msg.type == "ACK") {
                mgr->handleAcknowledge(rcvd_msg);
            } else if(rcvd_msg.type == "ARRIVE") {
                mgr->handleArrive(rcvd_msg);
            } else if(rcvd_msg.type == "TASK_COMPLETE") {
                mgr->handleTaskComplete(rcvd_msg);
            } else if(rcvd_msg.type == "MOVETO") {
                mgr->handleMoveTo(rcvd_msg);
            } else if(rcvd_msg.type == "SHUTDOWN") {
                std::cout << mgr->my_name << " is shutting down" << std::endl;
                break;
            } else if(rcvd_msg.type == "SET_SPEED") {
                mgr->handleSetSpeed(rcvd_msg);
            } else if(rcvd_msg.type == "CANCEL") {
                mgr->handleCancelTask(rcvd_msg);
            } else if(rcvd_msg.type == "CANCEL_ALL"){
                mgr->handleCancelAllTask(rcvd_msg);
            } else if(rcvd_msg.type == "OVERWATCH"){
                mgr->handleOverwatch(rcvd_msg);
            }
            else{
              nh->log_warning("Unknown message type: %s", rcvd_msg.type.c_str());
            }
        }

        while(!contacts.empty()){
          auto rcvd_msg = contacts.front();
          contacts.pop();
          mgr->handleContacts(rcvd_msg, formation_poses);
        }

        // Incoming internal notifications
        // Monitor navigation state
        if(nav_state_rcvd) {
            // Update navigation state in manager
            mgr->nav_state = nav_state.data;
            nav_state_rcvd = false;
        }
        // Update our own odometry
        if(odom_rcvd) {
            //std::cout << "Updated own odometry" << std::endl;
            mgr->odometry = odom;
            odom_rcvd = false;
        }

        // update tasks
        mgr->updateTasks();

        // post-update tasks
        mgr->postUpdateTasks();

        avt_341::mission::Task* task = mgr->currentTask();
        if(task != nullptr && use_slow_down_speed_control){
            avt_341::msg::Float64 speed_msg;
            speed_msg.data = speedController.getSpeedFactor(task->getFormationDef(), task->terminalPose(), formation_poses);
            speed_factor_pub->publish(speed_msg);
        }
        
        nh->spin_some();
        loop_rate.sleep();
    }
}
