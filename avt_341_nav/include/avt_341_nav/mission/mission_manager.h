
#ifndef AVT_341_MISSION_MGR_H
#define AVT_341_MISSION_MGR_H

/**
* \class MissionManager
*
* Manager for vehicle global path points based on desired formation.
*
* \author Daniel Carruth
*
* \date 1/31/2023
*/

// c++ includes
#include <string>
#include <chrono>
#include <map>
// local includes
#include "avt_341_msgs/msg/communication.hpp"
#include "avt_341_msgs/msg/map_marker_list.hpp"
#include "avt_341_msgs/msg/mission_module_status.hpp"
#include "avt_341_msgs/msg/mission_task_status.hpp"
#include "avt_341_msgs/msg/nav_goal.hpp"
#include "avt_341_msgs/msg/nav_goal_sequence.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341_nav/mission/task.h"
#include "avt_341_nav/mission/formation_utils.h"
#include "avt_341_nav/mission/formation_definition.h"
#include "avt_341_nav/mission/formation_speed_control.h"
#include "avt_341_nav/mission/mission_manager_dto.h"
#include "avt_341_nav/mission/goal_filtering/goal_filter.hpp"
#include <deque>


namespace avt_341_nav {
namespace mission {

class Task;

struct Contact {
    // storage for contact information
    // timestamp, position, class/name, investigated
    geometry_msgs::msg::PoseStamped pose;
    std::string name;
    bool investigated;
    bool investigating;
    bool is_new;       // true until MoveTo+Encircle tasks are created
    double first_seen_sec;
};

// Signature of a task-creating mission command, used to detect formation changes
struct FormationSignature {
    std::string command_type;
    std::string formation_type;
    bool formation_at_goal = false;
    std::vector<std::string> formation_vehicles;

    FormationSignature() = default;
    explicit FormationSignature(const std::string & command_type_in, const FormationDefinition * formation_def = nullptr)
        : command_type(command_type_in) {
        if (formation_def != nullptr) {
            formation_type = formation_def->getFormationType();
            formation_at_goal = formation_def->formationAtGoal();
            formation_vehicles = formation_def->orderedVehicles();
        }
    }

    // record the latest task-creating mission command
    void record(const std::string & command_type_in, const FormationDefinition * formation_def = nullptr) {
        *this = FormationSignature(command_type_in, formation_def);
    }

    bool operator==(const FormationSignature & other) const {
        return command_type == other.command_type &&
               formation_type == other.formation_type &&
               formation_at_goal == other.formation_at_goal &&
               formation_vehicles == other.formation_vehicles;
    }
    bool operator!=(const FormationSignature & other) const { return !(*this == other); }
};
    
/// Class for formation control
class MissionManager{

  public:
    /// Construct a formation controller
    MissionManager(
        MissionManagerParams params,
        const std::string & manager_name,
        const rclcpp::Node::SharedPtr & node,
        const std::shared_ptr<GoalFilter> & goal_filter);

    ~MissionManager();

    int loadMissionDefinition(std::string filename);

    bool getMissionPoint(MissionPoint& mission_point, std::string posename);

    void setMissionPoints(const std::vector<MissionPoint> & mission_points);

    bool loadMissionPaths(std::string filename);

    bool getMissionPath(MissionPath& mission_path, std::string pathname);

    // internal messages
    void handleContacts(const nav_msgs::msg::Path &, const std::map<std::string, nav_msgs::msg::Odometry> &);

    // external messages
    // Returns the created task, or nullptr if the message was not for this vehicle.
    Task* handleMoveTo(const MoveToMsg & msg, double x_offset=0.0, double y_offset=0.0, FormationDefinition* formation_def = nullptr, double desired_speed = 0.0);
    void handlePathFollow(const PathFollowMsg& msg, FormationDefinition* formation_def = nullptr);
    void handleFormationRequest(FormationMsg msg);
    void handleAcknowledge(const AcknowledgeMsg &);
    void handleArrive(const ArrivedMsg & msg);
    void handleTaskComplete(const TaskCompleteMsg & msg);
    void handleSetSpeedMsg(const SetSpeedMsg & msg);
    void handleOverwatch(const OverwatchMsg & msg);
    void handleCancelTask(const CancelMsg & msg);
    void handleCancelAllTask(const CancelAllMsg & msg);

    std::string my_name;
    nav_msgs::msg::Odometry odometry;
    nav_msgs::msg::Odometry leader_odometry;
    bool rcvd_leader_odom = false;
    int nav_state;
    bool goal_changed;
    bool arrival_announced;
    double local_origin_x;
    double local_origin_y;

    // Task management
    void updateTasks();
    bool addTask(Task * task, const std::string & priority_type = PriorityType::QUEUE);
    void publishPath(const nav_msgs::msg::Path& path);
    void publishGoal(const avt_341_msgs::msg::NavGoal & goal_in);
    void publishGoalPath(const nav_msgs::msg::Path& path);
    void publishNavStateCmd(int state);
    void publishGpToggle(int state);
    void publishArrival(const std::string & sender_name, const std::string & objective);
    void publishTaskStatus();
    // Publishes the latched snapshot of the active and queued tasks. Call after
    // any operation that mutates the task list outside of updateTasks(),
    // otherwise the retained sample is served stale to late-joining subscribers.
    void publishTaskChange();
    avt_341_msgs::msg::MissionTaskStatus createTaskStatusMsg(const Task* task) const;
    void reset();
    void resetTaskList(bool send_completion_msg);
    void cancelTask(int task_id,bool send_completion_msg);
    void onGoalReached(const geometry_msgs::msg::PoseStamped & pose);
    bool hasCompletedTask(const std::string & target_veh, int target_msg_id) const;
    bool hasArrival(const std::string & target_veh, const std::string & objective) const;

    Task* currentTask();
    geometry_msgs::msg::PoseStamped current_gp_goal;
    double getSpeedSetpoint();
    double nowSeconds() const;

  private:

    MissionManagerParams params_;
    std::vector<MissionPoint> mission_data;
    std::vector<MissionPoint> overwatch_positions;
    std::vector<MissionPath> mission_paths;
    std::deque<Task*> task_list;
    std::vector<Contact> mission_contacts;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<GoalFilter> goal_filter_;
    double speed_setpoint_state = -1.0;

    int obj_detection_cnt=9999; // TODO: Hack for task ids of contacts, replace later
    std::vector<TaskCompleteMsg> task_completions_;
    std::vector<ArrivedMsg> arrivals_;

    FormationSignature last_command_signature_;

    std::shared_ptr<rclcpp::Publisher<avt_341_msgs::msg::NavGoalSequence>> waypoint_pub = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> reset_pub = nullptr;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> gp_path_pub = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> navcommand_pub = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> gp_toggle_pub = nullptr;
    std::shared_ptr<rclcpp::Publisher<avt_341_msgs::msg::Communication>> communication_pub = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> speed_pub = nullptr;
    std::shared_ptr<rclcpp::Publisher<avt_341_msgs::msg::MissionTaskStatus>> task_status_pub = nullptr;
    std::shared_ptr<rclcpp::Publisher<avt_341_msgs::msg::MissionModuleStatus>> task_change_pub = nullptr;
    std::shared_ptr<rclcpp::Publisher<avt_341_msgs::msg::MapMarkerList>> map_markers_pub = nullptr;

    // Methods
    bool hasContact(const std::string & name, const geometry_msgs::msg::PoseStamped & pose);
    auto getClosestNewContact();
    MissionPoint getClosestOverwatch();
    void addContact(const std::string & name, const geometry_msgs::msg::PoseStamped & pose);
    void updateExistingContact(std::vector<Contact>::iterator it, const geometry_msgs::msg::PoseStamped & pose);
    void createToiTasks(Contact & contact, const std::map<std::string, nav_msgs::msg::Odometry> & veh_poses);
    void updateOverwatchPositions();
    void publishTaskCompletion(Task * task);
    void publishTaskCompletion(const std::string & sender_name, int msg_id);
    void publishSpeedSetPoint();
    void insertFormationChangeDelay(Task* formation_task, const FormationDefinition & formation_def, bool is_formation_change);

    // Publishes the current mission points as a latched MapMarkerList. Called
    // whenever the mission point list changes (CSV load or service call).
    void publishMapMarkers();

}; // class mission manager

} // namespace mission
} // namespace avt_341_nav


#endif //AVT_341_MISSION_MGR_H
