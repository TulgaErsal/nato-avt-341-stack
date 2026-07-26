// ros includes
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/mission/mission_manager.h"
#include "avt_341/mission/mission_manager_parser.h"
#include <queue>
#include <avt_341/core/dto_conversion.h>
#include <optional>
#include <set>
#include <cmath>

#include <avt_341_msgs/srv/set_nav_point_definitions.hpp>
#include <avt_341_msgs/srv/check_speed.hpp>
#include <avt_341_msgs/srv/get_odometry.hpp>
#include "avt_341/mission/goal_filtering/goal_filter_factory.hpp"
#include "avt_341/mission/goal_filtering/obs_avoid_goal_filter.hpp"
#include <avt_341/mission_manager_params_service.hpp>

std::queue<avt_341::msg::Communication> comm_msgs;
std::queue<avt_341::msg::PoseStamped> reached_goals;
std::queue<avt_341::msg::Path> contacts;

avt_341::msg::Odometry odom;
bool odom_rcvd = false;

std::optional<int> nav_run_state;

std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Odometry>> leader_pub;
std::shared_ptr<avt_341::mission::MissionManager> mgr;

std::map<std::string, avt_341::msg::Odometry> formation_odoms;
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
    formation_odoms[veh_name] = *msg;

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

bool current_goal_rcvd = false;
avt_341::msg::PoseStamped gp_goal_rcvd;
void NavStateCallback(avt_341::msg::NavStatePtr msg) {
    nav_run_state = msg->run_state;
    if (avt_341::core::HasActiveGoal(msg))
    {
        gp_goal_rcvd = avt_341::core::ToPoseStamped(msg->goal);
        current_goal_rcvd = true;
    }
}

bool reset_called = false;
void ResetCallback(avt_341::msg::StringPtr msg){
  if(msg->data.find(avt_341::node::NodeType::Mission) != std::string::npos){
    reset_called = true;
  }
}

void GoalReachedCallback(avt_341::msg::NavStatePtr msg){
  reached_goals.push(avt_341::core::ToPoseStamped(msg->goal));
}

auto get_veh_odom_sub(const std::vector<std::string> & veh_namespaces, int target_idx,
                      bool use_avt_tracker, const std::string & my_name) {
  bool target_veh_present = target_idx < static_cast<int>(veh_namespaces.size());
  const std::string target_veh_ns = target_veh_present ? veh_namespaces[target_idx] : "";
  bool use_estimated = use_avt_tracker && (toUpper(target_veh_ns) != my_name);
  const std::string topic = use_estimated
    ? "avt_341/odometry/estimated/" + target_veh_ns
    : "/" + target_veh_ns + "/avt_341/odometry";
  return target_veh_present
    ? nh->create_subscription<avt_341::msg::Odometry>(topic, 10, VehicleOdometryCallback)
    : nullptr;
}

void CheckSpeedServiceImpl(
    const std::shared_ptr<avt_341_msgs::srv::CheckSpeed::Request> request,
    std::shared_ptr<avt_341_msgs::srv::CheckSpeed::Response> response) {

    const double ego_speed = std::abs(odom.twist.twist.linear.x);
    const std::string & op = request->operation;

    if (op == "eq") {
        response->is_true = std::abs(ego_speed - request->speed) <= request->threshold;
    } else if (op == "ne") {
        response->is_true = std::abs(ego_speed - request->speed) > request->threshold;
    } else if (op == "lt") {
        response->is_true = ego_speed < request->speed + request->threshold;
    } else if (op == "gt") {
        response->is_true = ego_speed > request->speed - request->threshold;
    } else {
        nh->log_warning("CheckSpeed: unknown operation \"%s\" (expected eq, ne, lt or gt).", op.c_str());
        response->is_true = false;
    }
}

void GetOdometryServiceImpl(
    const std::shared_ptr<avt_341_msgs::srv::GetOdometry::Request> request,
    std::shared_ptr<avt_341_msgs::srv::GetOdometry::Response> response) {

    if (request->vehicle_id.empty()) {
        response->odom = odom;
        return;
    }

    const auto it = formation_odoms.find(toUpper(request->vehicle_id));
    if (it == formation_odoms.end()) {
        nh->log_warning("GetOdometry: no odometry received for vehicle \"%s\".", request->vehicle_id.c_str());
        return;
    }
    response->odom = it->second;
}

void SetNavPointDefinitionsServiceImpl(
    const std::shared_ptr<avt_341_msgs::srv::SetNavPointDefinitions::Request> request,
    std::shared_ptr<avt_341_msgs::srv::SetNavPointDefinitions::Response> response)
{
    auto fail = [&](const std::string & reason) {
        response->success = false;
        response->message = reason;
        nh->log_warning("SetNavPointDefinitions rejected: %s", reason.c_str());
    };
    if (request->labels.size() != request->poses.size()) {
        fail("labels size (" + std::to_string(request->labels.size()) + ") does not match poses size ("
             + std::to_string(request->poses.size()) + ").");
        return;
    }

    std::vector<avt_341::mission::MissionPoint> mission_points;
    mission_points.reserve(request->labels.size());
    std::set<std::string> seen_labels;

    for (size_t i = 0; i < request->labels.size(); i++) {
        const std::string & label = request->labels[i];
        const auto & pose = request->poses[i];

        if (label.empty())
        {
            fail("label at position " + std::to_string(i) + " is empty.");
            return;
        }

        avt_341::mission::MissionPoint mission_point;
        mission_point.name = label;
        mission_point.pos_x = pose.position.x;
        mission_point.pos_y = pose.position.y;
        mission_point.pos_z = pose.position.z;
        mission_point.rot_x = pose.orientation.x;
        mission_point.rot_y = pose.orientation.y;
        mission_point.rot_z = pose.orientation.z;
        mission_point.rot_w = pose.orientation.w;
        mission_points.push_back(mission_point);
    }

    mgr->setMissionPoints(mission_points);
    response->success = true;
    response->message = "Set " + std::to_string(mission_points.size()) + " nav point definitions.";
}

int main(int argc, char **argv) {

    // initialize the node
    nh = avt_341::node::init_node(argc, argv, "mission_manager");
    nh->initialize_tf_listener();
    avt_341::params::mission_manager::ParamsListener param_listener(nh->get_raw_node());
    const auto params = param_listener.get_params();
    avt_341::node::Rate loop_rate(10);

    const std::string my_name = toUpper(params.name);

    std::shared_ptr<avt_341::mission::GoalFilter> goal_filter =
        avt_341::mission::create_goal_filter(
            my_name, params.formation_goal_filter, nh,
            params.fgf_obs_avoid, params.costmap.publish.method);

    mgr = std::make_shared<avt_341::mission::MissionManager>(
        params, my_name, nh, goal_filter);

    std::shared_ptr<avt_341::mission::FormationSpeedController>
        speedController = avt_341::mission::createFormationSpeedController(
            my_name, params.fsc, nh);

    nh->log_info("Mission Manager Settings:\n  fsc.type=%s\n  formation.use_breadcrumbs=%d\n  formation.x_offset_on_path=%d\n  formation.prune_global_path=%d",
                params.fsc.type.c_str(), params.formation.use_breadcrumbs,
                params.formation.x_offset_on_path,
                params.formation.prune_global_path);
    nh->log_info("%s loading definition file %s", mgr->my_name.c_str(),
                 params.mission_definition_file.c_str());
    mgr->loadMissionDefinition(params.mission_definition_file);
    nh->log_info("%s loading paths file %s", mgr->my_name.c_str(),
                 params.mission_paths_file.c_str());
    mgr->loadMissionPaths(params.mission_paths_file);

    // set up subscriptions
    auto communication_sub = nh->create_subscription<avt_341::msg::Communication>("avt_341/comm_messages", 10, CommunicationCallback);
    auto odom_sub = nh->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 100, EgoOdometryCallback);
    auto nav_state_sub = nh->create_subscription<avt_341::msg::NavState>("avt_341/state", 10, NavStateCallback);
    auto detect_sub = nh->create_subscription<avt_341::msg::Path>("avt_341/target_contacts", 1, TargetContactsCallback);
    auto veh1_sub = get_veh_odom_sub(
        params.vehicle_namespaces, 0, params.use_avt_tracker,
        my_name);
    auto veh2_sub = get_veh_odom_sub(
        params.vehicle_namespaces, 1, params.use_avt_tracker,
        my_name);
    auto veh3_sub = get_veh_odom_sub(
        params.vehicle_namespaces, 2, params.use_avt_tracker,
        my_name);
    auto veh4_sub = get_veh_odom_sub(
        params.vehicle_namespaces, 3, params.use_avt_tracker,
        my_name);

    auto reset_sub = nh->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
    auto goal_reached_sub = nh->create_subscription<avt_341::msg::NavState>("avt_341/goal_reached", 10, GoalReachedCallback);

    auto speed_factor_pub = nh->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed_factor", 10);
    auto reset_ack_pub = nh->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);
    leader_pub = nh->create_publisher<avt_341::msg::Odometry>("avt_341/leader_odometry", 10);

    // Services
    auto set_nav_point_definitions_srv =
        nh->get_raw_node()->create_service<avt_341_msgs::srv::SetNavPointDefinitions>(
            "avt_341/set_nav_point_definitions", &SetNavPointDefinitionsServiceImpl);

    auto check_speed_srv =
        nh->get_raw_node()->create_service<avt_341_msgs::srv::CheckSpeed>(
            "avt_341/check_speed", &CheckSpeedServiceImpl);

    auto get_odometry_srv =
        nh->get_raw_node()->create_service<avt_341_msgs::srv::GetOdometry>(
            "avt_341/get_odometry", &GetOdometryServiceImpl);

    // start the loop
    while(avt_341::node::ok()){
    // Handle external notifications

        if(reset_called){
          nh->log_info("Resetting node");
          mgr->reset();
          current_goal_rcvd = false;
          mgr->rcvd_leader_odom = false;
          while(!reached_goals.empty()) reached_goals.pop();
          while(!contacts.empty()) contacts.pop();
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
                mgr->handleSetSpeedMsg(SetSpeedMsg(rcvd_msg));
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
          mgr->handleContacts(rcvd_msg, formation_odoms);
        }

        // Incoming internal notifications
        // Monitor navigation state
        if(nav_run_state.has_value()) {
            // Update navigation state in manager
            mgr->nav_state = nav_run_state.value();
        }
        // Update our own odometry
        if(odom_rcvd) {
            //std::cout << "Updated own odometry" << std::endl;
            mgr->odometry = odom;
            odom_rcvd = false;
        }

        // update tasks
        mgr->updateTasks();

        // Publish mission task status
        mgr->publishTaskStatus();
        // TODO: Can remove lead status and follower status messages. Use single MissionTaskStatus message.
        mgr->publishLeaderStatus();

        avt_341::mission::Task* task = mgr->currentTask();
        if(task != nullptr){
            avt_341::msg::Float64 speed_msg;
            speed_msg.data = speedController->getSpeedFactor(task->getFormationDef(), task->terminalPose(), formation_odoms, mgr->getSpeedSetpoint());
            speed_factor_pub->publish(speed_msg);
        }else{
          speedController->clearVisualization();
        }
        
        nh->spin_some();
        loop_rate.sleep();
    }
}
