// ros includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include "avt_341_nav/node/node_types.h"
#include "avt_341_nav/node/tf_interface.h"
// local includes
#include "avt_341_nav/mission/mission_manager.h"
#include "avt_341_nav/mission/mission_manager_parser.h"
#include "avt_341_nav/mission/speed_zone_monitor.hpp"
#include "avt_341_nav/core/string_utils.hpp"
#include <queue>
#include <avt_341_nav/core/dto_conversion.h>
#include <optional>
#include <regex>
#include <set>
#include <cmath>

#include <avt_341_msgs/srv/set_nav_point_definitions.hpp>
#include <avt_341_msgs/srv/check_speed.hpp>
#include <avt_341_msgs/srv/get_odometry.hpp>
#include "avt_341_nav/mission/goal_filtering/goal_filter_factory.hpp"
#include "avt_341_nav/mission/goal_filtering/obs_avoid_goal_filter.hpp"
#include <avt_341_nav/mission_manager_params_service.hpp>
#include "avt_341_msgs/msg/communication.hpp"
#include "avt_341_msgs/msg/nav_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

std::queue<avt_341_msgs::msg::Communication> comm_msgs;
std::queue<geometry_msgs::msg::PoseStamped> reached_goals;
std::queue<nav_msgs::msg::Path> contacts;

nav_msgs::msg::Odometry odom;
bool odom_rcvd = false;

std::optional<int> nav_run_state;

std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> leader_pub;
std::shared_ptr<avt_341_nav::mission::MissionManager> mgr;

std::map<std::string, nav_msgs::msg::Odometry> formation_odoms;
rclcpp::Node::SharedPtr nh = nullptr;
std::shared_ptr<avt_341_nav::node::TfInterface> tf = nullptr;

std::shared_ptr<rclcpp::AsyncParametersClient> tracker_param_client = nullptr;
std::optional<std::string> applied_toi_regex;

// Reconfigure the object tracking node with the targets of interest regex of a formation command.
// An empty regex is a valid value: it tells the tracker to ignore all targets of interest.
void UpdateTrackerToiRegex(const std::string & toi_regex) {
    if(applied_toi_regex.has_value() && applied_toi_regex.value() == toi_regex) {
        return;
    }

    try {
        std::regex{toi_regex};
    } catch (const std::regex_error & e) {
        RCLCPP_WARN(nh->get_logger(), "Ignoring invalid toi_regex \"%s\" in formation command: %s",
            toi_regex.c_str(), e.what());
        return;
    }

    if(!tracker_param_client->service_is_ready()) {
        RCLCPP_WARN(nh->get_logger(), "Object tracking node parameter service not available, cannot apply toi_regex \"%s\".",
            toi_regex.c_str());
        return;
    }

    tracker_param_client->set_parameters(
        {rclcpp::Parameter("target_selection.toi_regex", toi_regex)},
        [toi_regex](const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
            const auto results = future.get();
            if(results.empty() || !results.front().successful) {
                const std::string reason = results.empty() ? "no result returned" : results.front().reason;
                RCLCPP_WARN(nh->get_logger(), "Failed to apply toi_regex \"%s\" to the object tracking node: %s",
                    toi_regex.c_str(), reason.c_str());
                return;
            }
            // Only remember the value once it is known to have been applied, so that a failed
            // reconfigure is retried on the next formation command.
            applied_toi_regex = toi_regex;
            RCLCPP_INFO(nh->get_logger(), "Applied toi_regex \"%s\" to the object tracking node.", toi_regex.c_str());
        });
}

// Receive updates from comms
void CommunicationCallback(avt_341_msgs::msg::Communication::SharedPtr msg) {
    comm_msgs.push(*msg);
}

void EgoOdometryCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
	// need to know our own position
	odom = *msg;
	odom_rcvd = true;
}

std::string toUpper(const std::string & str){
  std::string str_upper = str;
  std::transform(str.begin(), str.end(), str_upper.begin(), [](unsigned char c){ return std::toupper(c); });
  return str_upper;
}

// Publishes the static transform placing the map frame origin at
// (gis.origin_x, gis.origin_y) within the GIS crs frame.
void PublishGisStaticTransform(const avt_341_nav::params::core::Frames & frames) {
  if (frames.gis.crs.empty()) {
    RCLCPP_INFO(nh->get_logger(), "No GIS crs configured, skipping map georeference static transform.");
    return;
  }

  const std::string crs_frame = avt_341_nav::core::CrsToFrameId(frames.gis.crs);
  geometry_msgs::msg::PoseStamped map_origin;
  map_origin.pose.position.x = frames.gis.origin_x;
  map_origin.pose.position.y = frames.gis.origin_y;
  map_origin.pose.orientation.w = 1.0;
  tf->publish_static_tf(crs_frame, frames.map, map_origin);
  RCLCPP_INFO(nh->get_logger(), "Published static transform %s -> %s at (%.3f, %.3f).",
      crs_frame.c_str(), frames.map.c_str(), frames.gis.origin_x, frames.gis.origin_y);
}

// Receive updated odometry information
void VehicleOdometryCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
    // Ensure same case as leader_name
    std::string child_frame_id = msg->child_frame_id;
    avt_341_nav::mission::Task* current_task = mgr->currentTask();

    child_frame_id = toUpper(child_frame_id);
    std::string veh_name = child_frame_id.substr(0, child_frame_id.find('/'));
    formation_odoms[veh_name] = *msg;

    if(current_task == nullptr || !current_task->hasFormation()){
      return;
    }

    const std::string leader_name = current_task->getFormationDef()->followedVehicle();

    if(!leader_name.empty() && child_frame_id.find(leader_name) != std::string::npos ) {
      nav_msgs::msg::Odometry leader_odom = *msg;

      // Check if odometry is in map frame
      if (msg->header.frame_id != "map") {
        geometry_msgs::msg::PoseStamped leader_pose, leader_pose_map;
        leader_pose.header = msg->header;
        leader_pose.pose = leader_odom.pose.pose;
        tf->transform_pose(leader_pose, leader_pose_map, "map", 0.2);
        leader_odom.pose.pose = leader_pose_map.pose;
      }
      
      mgr->leader_odometry = leader_odom;
      mgr->rcvd_leader_odom = true;
      leader_pub->publish(*msg);
    }
}

// Receive information on target contacts
void TargetContactsCallback(nav_msgs::msg::Path::SharedPtr msg) {
	//std::cout << ros::this_node::getName() << " Mission Manager received " << msg->poses.size() << " target contacts" << std::endl;
  // Handle detection of potential targets of interest
  if(mgr->nav_state != avt_341_nav::core::NavStackState::NotInit) {       // avoid premature detection of contacts
    contacts.push(*msg);
  }
}

bool current_goal_rcvd = false;
geometry_msgs::msg::PoseStamped gp_goal_rcvd;
void NavStateCallback(avt_341_msgs::msg::NavState::SharedPtr msg) {
    nav_run_state = msg->run_state;
    if (avt_341_nav::core::HasActiveGoal(msg))
    {
        gp_goal_rcvd = avt_341_nav::core::ToPoseStamped(msg->goal);
        current_goal_rcvd = true;
    }
}

bool reset_called = false;
void ResetCallback(const std_msgs::msg::String::SharedPtr msg){
  if(msg->data.find(avt_341_nav::node::NodeType::Mission) != std::string::npos){
    reset_called = true;
  }
}

void GoalReachedCallback(avt_341_msgs::msg::NavState::SharedPtr msg){
  reached_goals.push(avt_341_nav::core::ToPoseStamped(msg->goal));
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
    ? nh->create_subscription<nav_msgs::msg::Odometry>(topic, 10, VehicleOdometryCallback)
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
        RCLCPP_WARN(nh->get_logger(), "CheckSpeed: unknown operation \"%s\" (expected eq, ne, lt or gt).", op.c_str());
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
        RCLCPP_WARN(nh->get_logger(), "GetOdometry: no odometry received for vehicle \"%s\".", request->vehicle_id.c_str());
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
        RCLCPP_WARN(nh->get_logger(), "SetNavPointDefinitions rejected: %s", reason.c_str());
    };
    if (request->labels.size() != request->poses.size()) {
        fail("labels size (" + std::to_string(request->labels.size()) + ") does not match poses size ("
             + std::to_string(request->poses.size()) + ").");
        return;
    }

    std::vector<avt_341_nav::mission::MissionPoint> mission_points;
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

        avt_341_nav::mission::MissionPoint mission_point;
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
    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("mission_manager");
    tf = std::make_shared<avt_341_nav::node::TfInterface>(nh);
    avt_341_nav::params::mission_manager::ParamsListener param_listener(nh);
    const auto params = param_listener.get_params();
    rclcpp::Rate loop_rate(10);

    const std::string my_name = toUpper(params.name);

    PublishGisStaticTransform(params.frames);

    tracker_param_client = std::make_shared<rclcpp::AsyncParametersClient>(nh, params.toi.tracker_node_name);

    std::shared_ptr<avt_341_nav::mission::GoalFilter> goal_filter =
        avt_341_nav::mission::create_goal_filter(
            my_name, params.formation_goal_filter, nh,
            params.fgf_obs_avoid, params.costmap.publish.method);

    mgr = std::make_shared<avt_341_nav::mission::MissionManager>(
        params, my_name, nh, goal_filter);

    std::shared_ptr<avt_341_nav::mission::FormationSpeedController>
        speedController = avt_341_nav::mission::createFormationSpeedController(
            my_name, params.fsc, nh, tf);

    std::shared_ptr<avt_341_nav::mission::SpeedZoneMonitor> speed_zone_monitor = nullptr;
    if (params.use_speed_zones) {
        speed_zone_monitor = std::make_shared<avt_341_nav::mission::SpeedZoneMonitor>(
            nh, tf, mgr, params.speed_zones_file);
    }

    RCLCPP_INFO(nh->get_logger(), "Mission Manager Settings:\n  fsc.type=%s\n  formation.use_breadcrumbs=%d\n  formation.x_offset_on_path=%d\n  formation.prune_global_path=%d", params.fsc.type.c_str(), params.formation.use_breadcrumbs, params.formation.x_offset_on_path, params.formation.prune_global_path);
    RCLCPP_INFO(nh->get_logger(), "%s loading definition file %s", mgr->my_name.c_str(), params.mission_definition_file.c_str());
    mgr->loadMissionDefinition(params.mission_definition_file);
    RCLCPP_INFO(nh->get_logger(), "%s loading paths file %s", mgr->my_name.c_str(), params.mission_paths_file.c_str());
    mgr->loadMissionPaths(params.mission_paths_file);

    // set up subscriptions
    auto communication_sub = nh->create_subscription<avt_341_msgs::msg::Communication>("avt_341/comm_messages", 10, CommunicationCallback);
    auto odom_sub = nh->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry", 100, EgoOdometryCallback);
    auto nav_state_sub = nh->create_subscription<avt_341_msgs::msg::NavState>("avt_341/state", 10, NavStateCallback);
    auto detect_sub = nh->create_subscription<nav_msgs::msg::Path>("avt_341/target_contacts", 1, TargetContactsCallback);
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

    auto reset_sub = nh->create_subscription<std_msgs::msg::String>("avt_341/reset", 10, ResetCallback);
    auto goal_reached_sub = nh->create_subscription<avt_341_msgs::msg::NavState>("avt_341/goal_reached", 10, GoalReachedCallback);

    auto speed_factor_pub = nh->create_publisher<std_msgs::msg::Float64>("avt_341/desired_speed_factor", 10);
    auto reset_ack_pub = nh->create_publisher<std_msgs::msg::String>("avt_341/reset_ack", 1);
    leader_pub = nh->create_publisher<nav_msgs::msg::Odometry>("avt_341/leader_odometry", 10);

    // Services
    auto set_nav_point_definitions_srv =
        nh->create_service<avt_341_msgs::srv::SetNavPointDefinitions>(
            "avt_341/set_nav_point_definitions", &SetNavPointDefinitionsServiceImpl);

    auto check_speed_srv =
        nh->create_service<avt_341_msgs::srv::CheckSpeed>(
            "avt_341/check_speed", &CheckSpeedServiceImpl);

    auto get_odometry_srv =
        nh->create_service<avt_341_msgs::srv::GetOdometry>(
            "avt_341/get_odometry", &GetOdometryServiceImpl);

    // start the loop
    while(rclcpp::ok()){
    // Handle external notifications

        if(reset_called){
          RCLCPP_INFO(nh->get_logger(), "Resetting node");
          mgr->reset();
          mgr->publishTaskChange();
          current_goal_rcvd = false;
          mgr->rcvd_leader_odom = false;
          while(!reached_goals.empty()) reached_goals.pop();
          while(!contacts.empty()) contacts.pop();
          std_msgs::msg::String reset_ack_msg;
          reset_ack_msg.data = avt_341_nav::node::NodeType::Mission;
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
              RCLCPP_INFO(nh->get_logger(), "%s ignoring message: %s", mgr->my_name.c_str(), msg_text.c_str());
              continue;
            }
            //std::string msg_text = rosToSerializedMsg(rcvd_msg);
            RCLCPP_INFO(nh->get_logger(), "%s handling message: %s", mgr->my_name.c_str(), msg_text.c_str());

            if(rcvd_msg.type == MissionMsgType::Formation) {
                UpdateTrackerToiRegex(rcvd_msg.toi_regex);
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
              RCLCPP_WARN(nh->get_logger(), "Unknown message type: %s", rcvd_msg.type.c_str());
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
            if (speed_zone_monitor != nullptr) {
                speed_zone_monitor->UpdateOdometry(odom);
            }
            odom_rcvd = false;
        }

        // update tasks
        mgr->updateTasks();

        // Publish mission task status
        mgr->publishTaskStatus();
        // TODO: Can remove lead status and follower status messages. Use single MissionTaskStatus message.
        mgr->publishLeaderStatus();

        avt_341_nav::mission::Task* task = mgr->currentTask();
        if(task != nullptr){
            std_msgs::msg::Float64 speed_msg;
            speed_msg.data = speedController->getSpeedFactor(task->getFormationDef(), task->terminalPose(), formation_odoms, mgr->getSpeedSetpoint());
            speed_factor_pub->publish(speed_msg);
        }else{
          speedController->clearVisualization();
        }
        
        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }
}
