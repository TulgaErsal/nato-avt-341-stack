// ros includes
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/mission/mission_manager.h"
#include "avt_341/mission/mission_manager_parser.h"
#include <queue>

#include "avt_341/mission/goal_filtering/goal_filter_factory.hpp"
#include "avt_341/mission/goal_filtering/obs_avoid_goal_filter.hpp"

std::queue<avt_341::msg::Communication> comm_msgs;
std::queue<avt_341::msg::PoseStamped> reached_goals;
std::queue<avt_341::msg::Path> contacts;

avt_341::msg::Odometry odom;
bool odom_rcvd = false;

avt_341::msg::Int32 nav_state;
bool nav_state_rcvd = false;

std::string mission_definition_filename, mission_paths_file;
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

std::string toUpper(const std::string & str){
  std::string str_upper = str;
  std::transform(str.begin(), str.end(), str_upper.begin(), [](unsigned char c){ return std::toupper(c); });
  return str_upper;
}

// Receive updated odometry information
void VehicleOdometryCallback(avt_341::msg::OdometryPtr msg) {
    // Ensure same case as leader_name
    std::string child_frame_id = msg->child_frame_id;
    avt_341::mission::Task* current_task = mgr->currentTask();

    child_frame_id = toUpper(child_frame_id);
    std::string veh_name = child_frame_id.substr(0, child_frame_id.find('/'));
    formation_poses[veh_name] = *msg;

    if(current_task == nullptr || !current_task->hasFormation()){
      return;
    }

    const std::string leader_name = current_task->getFormationDef()->followedVehicle();

    if(!leader_name.empty() && child_frame_id.find(leader_name) != std::string::npos ) {
      avt_341::msg::Odometry leader_odom = *msg;

      // Check if odometry is in map frame
      if (msg->header.frame_id != "map") {
        avt_341::msg::PoseStamped leader_pose, leader_pose_map;
        leader_pose.header = msg->header;
        leader_pose.pose = leader_odom.pose.pose;
        nh->transform_pose(leader_pose, leader_pose_map, "map", 0.2);
        leader_odom.pose.pose = leader_pose_map.pose;
      }
      
      mgr->leader_odometry = leader_odom;
      mgr->rcvd_leader_odom = true;
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
void ResetCallback(avt_341::msg::StringPtr msg){
  if(msg->data.find(avt_341::node::NodeType::Mission) != std::string::npos){
    reset_called = true;
  }
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

auto get_veh_odom_sub(const std::vector<std::string> & veh_namespaces, const std::string & my_name, int target_idx,
                      const std::string & tracking_veh, const std::string & tracked_veh){

  bool target_veh_present = target_idx < veh_namespaces.size();
  std::string target_veh_ns = target_veh_present ? veh_namespaces[target_idx] : "";
  std::string sub_postfix = my_name == tracking_veh && !tracked_veh.empty() && toUpper(target_veh_ns) == tracked_veh ? "/tracked" : "";
  return target_veh_present
    ? nh->create_subscription<avt_341::msg::Odometry>("/" + target_veh_ns + "/avt_341/odometry" + sub_postfix, 10, VehicleOdometryCallback)
    : nullptr;
}

int main(int argc, char **argv) {

    // initialize the node
    nh = avt_341::node::init_node(argc, argv, "mission_manager");
    nh->initialize_tf_listener();
    avt_341::node::Rate loop_rate(10);

    // load the parameters
    avt_341::mission::FormationSpeedControlParams fsc_params{};
    avt_341::mission::FormationParameters formation_params;
    avt_341::mission::ToiParameters toi_params;
    std::string fsc_type, tracked_veh, tracking_veh;
    std::vector<std::string> veh_namespaces;

    nh->get_parameter("~name", formation_params.my_name, std::string("AGV1"));
    formation_params.my_name = toUpper(formation_params.my_name);
    nh->get_parameter("~mission_definition_file", mission_definition_filename, std::string("mission.csv"));
    nh->get_parameter("~mission_paths_file", mission_paths_file, std::string("mission_paths.csv"));
    nh->get_parameter("~follow_scale_x", formation_params.follow_scale_x, 1.0f);
    nh->get_parameter("~follow_scale_y", formation_params.follow_scale_y, 1.0f);
    nh->get_parameter("~global_path_point_dist", formation_params.global_path_points_dist, 1.0f);
    nh->get_parameter("~use_leader_breadcrumbs", formation_params.use_breadcrumbs, true);
    nh->get_parameter("~x_offset_on_path", formation_params.x_offset_on_path, false);
    nh->get_parameter("~formation_prune_gp", formation_params.prune_global_path, false);
    nh->get_parameter("~follow_goal_threshold", formation_params.follow_goal_threshold, 10.0f);
    nh->get_parameter("~same_object_distance_threshold", sodist_threshold, 1.0f);


    nh->get_parameter("~oof_threshold", fsc_params.oof_threshold, 15.0);
    nh->get_parameter("~fsc_max_speed_factor", fsc_params.max_speed_factor, 2.0);
    nh->get_parameter("~fsc_follower_obt_stop", fsc_params.follower_obt_stop, false);
    nh->get_parameter("~oof_const_term", fsc_params.oof_const_term, 0.3);
    nh->get_parameter("~oof_lin_slope", fsc_params.oof_lin_slope, 0.03);
    nh->get_parameter("~oof_mult", fsc_params.oof_mult, 1.5);
    nh->get_parameter("~formation_debug_visualize", fsc_params.debug_visualize, false);
    nh->get_parameter("~offsets_from_leader", formation_params.offsets_from_leader, true);
    nh->get_parameter("~follower_dist_break", fsc_params.follower_dist_break, 10.0);
    nh->get_parameter("~follower_dot_threshold", fsc_params.follower_dot_threshold, 0.0);
    nh->get_parameter("~follower_dot_range", fsc_params.follower_dot_range, 30.0);
    nh->get_parameter("~fsc_type", fsc_type, FormationSpeedControlType::SPEED_UP_FOLLOWER);
    nh->get_parameter("~vehicle_namespaces", veh_namespaces, std::vector<std::string>{"agv1", "agv2", "cgv1", "cgv2"});

    nh->get_parameter("~toi_approach_dist", toi_params.approach_dist, 15.0f);
    nh->get_parameter("~toi_encircle_radius",  toi_params.encircle_radius, 10.0f);
    nh->get_parameter("~toi_encircle_degrees", toi_params.encircle_degrees, 180.0f);
    nh->get_parameter("~toi_encircle_cw", toi_params.encircle_cw, true);
    nh->get_parameter("~toi_goal_threshold", toi_params.goal_threshold, 5.0f);

    nh->get_parameter("~ot_tracking_veh", tracking_veh, std::string(""));
    nh->get_parameter("~ot_tracked_veh", tracked_veh, std::string(""));

    std::string goal_filter_method;
    nh->get_parameter("~formation_goal_filter", goal_filter_method, std::string("none"));
    std::shared_ptr<avt_341::mission::GoalFilter> goal_filter = avt_341::mission::create_goal_filter(formation_params.my_name, goal_filter_method, nh);

    tracking_veh = toUpper(tracking_veh);
    tracked_veh = toUpper(tracked_veh);

    mgr = std::make_shared<avt_341::mission::MissionManager>(formation_params, toi_params, nh, goal_filter);
    mgr->sodist_threshold = sodist_threshold;

    std::shared_ptr<avt_341::mission::FormationSpeedController> speedController = avt_341::mission::createFormationSpeedController(fsc_type, formation_params.my_name, fsc_params, nh);

    nh->log_info("Mission Manager Settings:\n  fsc_type=%s\n  use_leader_breadcrumbs=%d\n  x_offset_on_path=%d\n  formation_prune_gp=%d",
                fsc_type.c_str(), formation_params.use_breadcrumbs, formation_params.x_offset_on_path, formation_params.prune_global_path);
    nh->log_info("%s loading definition file %s", mgr->my_name.c_str(), mission_definition_filename.c_str());
    mgr->loadMissionDefinition(mission_definition_filename);
    nh->log_info("%s loading paths file %s", mgr->my_name.c_str(), mission_paths_file.c_str());
    mgr->loadMissionPaths(mission_paths_file);

    // set up subscriptions
    auto communication_sub = nh->create_subscription<avt_341::msg::Communication>("avt_341/comm_messages", 10, CommunicationCallback);
    auto odom_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 100, EgoOdometryCallback);
    auto nav_state_sub = nh->create_subscription<avt_341::msg::Int32>("avt_341/state", 10, NavStateCallback);
    auto detect_sub = nh->create_subscription<avt_341::msg::Path>("avt_341/target_contacts", 1, TargetContactsCallback);
    auto veh1_sub =  get_veh_odom_sub(veh_namespaces, formation_params.my_name, 0, tracking_veh, tracked_veh);
    auto veh2_sub =  get_veh_odom_sub(veh_namespaces, formation_params.my_name, 1, tracking_veh, tracked_veh);
    auto veh3_sub =  get_veh_odom_sub(veh_namespaces, formation_params.my_name, 2, tracking_veh, tracked_veh);
    auto veh4_sub =  get_veh_odom_sub(veh_namespaces, formation_params.my_name, 3, tracking_veh, tracked_veh);

    auto reset_sub = nh->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
    auto goal_reached_sub = nh->create_subscription<avt_341::msg::PoseStamped>("avt_341/goal_reached", 10, GoalReachedCallback);
    auto current_waypoint_sub = nh->create_subscription<avt_341::msg::PoseStamped>("avt_341/current_waypoint", 10, CurrentGoalCallback);

    auto speed_factor_pub = nh->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed_factor", 10);
    auto reset_ack_pub = nh->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);
    leader_pub = nh->create_publisher<avt_341::msg::Odometry>("avt_341/leader_odometry", 10);

    // start the loop
    while(avt_341::node::ok()){
    // Handle external notifications

        if(reset_called){
          nh->log_info("Resetting node");
          mgr->reset();
          avt_341::msg::String reset_ack_msg;
          reset_ack_msg.data = avt_341::node::NodeType::Mission;
          reset_ack_pub->publish(reset_ack_msg);
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
            std::string msg_text = rosToSerializedMsg(rcvd_msg);
            if(!isMsgFor(mgr->my_name, rcvd_msg)){
              nh->log_info("%s ignoring message: %s", mgr->my_name.c_str(), msg_text.c_str());
              continue;
            }
            //std::string msg_text = rosToSerializedMsg(rcvd_msg);
            nh->log_info("%s handling message: %s", mgr->my_name.c_str(), msg_text.c_str());

            if(rcvd_msg.type == MissionMsgType::Formation) {
                mgr->handleFormationRequest(FormationMsg(rcvd_msg));
            } else if(rcvd_msg.type == MissionMsgType::Acknowledge) {
                mgr->handleAcknowledge(AcknowledgeMsg(rcvd_msg));
            } else if(rcvd_msg.type == MissionMsgType::Arrived) {
                mgr->handleArrive(ArrivedMsg(rcvd_msg));
            } else if(rcvd_msg.type == MissionMsgType::TaskComplete) {
                mgr->handleTaskComplete(TaskCompleteMsg(rcvd_msg));
            } else if(rcvd_msg.type == MissionMsgType::MoveTo) {
                mgr->handleMoveTo(MoveToMsg(rcvd_msg));
            } else if(rcvd_msg.type == MissionMsgType::Shutdown) {
                std::cout << mgr->my_name << " is shutting down" << std::endl;
                break;
            } else if(rcvd_msg.type == MissionMsgType::SetSpeed) {
                mgr->handleSetSpeed(SetSpeedMsg(rcvd_msg));
            } else if(rcvd_msg.type == MissionMsgType::Cancel) {
                mgr->handleCancelTask(CancelMsg(rcvd_msg));
            } else if(rcvd_msg.type == MissionMsgType::CancelAll){
                mgr->handleCancelAllTask(CancelAllMsg(rcvd_msg));
            } else if(rcvd_msg.type == MissionMsgType::Overwatch){
                mgr->handleOverwatch(OverwatchMsg(rcvd_msg));
            } else if(rcvd_msg.type == MissionMsgType::PathFollow){
                mgr->handlePathFollow(PathFollowMsg(rcvd_msg));
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

        // Publish leader status
        mgr->publishLeaderStatus();

        avt_341::mission::Task* task = mgr->currentTask();
        if(task != nullptr){
            avt_341::msg::Float64 speed_msg;
            speed_msg.data = speedController->getSpeedFactor(task->getFormationDef(), task->terminalPose(), formation_poses);
            speed_factor_pub->publish(speed_msg);
        }else{
          speedController->clearVisualization();
        }
        
        nh->spin_some();
        loop_rate.sleep();
    }
}
