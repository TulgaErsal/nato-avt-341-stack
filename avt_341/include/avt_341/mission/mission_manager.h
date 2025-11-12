
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
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/mission/task.h"
#include "avt_341/mission/formation_utils.h"
#include "avt_341/mission/formation_definition.h"
#include "avt_341/mission/formation_speed_control.h"
#include "avt_341/mission/mission_manager_dto.h"
#include <deque>

namespace avt_341 {
namespace mission {

class Task;

struct Contact {
    // storage for contact information
    // timestamp, position, class/name, investigated
    avt_341::msg::PoseStamped pose;
    std::string name;
    bool investigated;
    bool investigating;
    bool is_new;
};
    
/// Class for formation control
class MissionManager{

  public:
    /// Construct a formation controller
    MissionManager(const FormationParameters & formation_params, const ToiParameters & toi_params, std::shared_ptr<node::NodeProxy> node_proxy);
    ~MissionManager();

    int loadMissionDefinition(std::string filename);

    bool getMissionPoint(MissionPoint& mission_point, std::string posename);

    bool loadMissionPaths(std::string filename);

    bool getMissionPath(MissionPath& mission_path, std::string pathname);

    // internal messages
    void handleContacts(const avt_341::msg::Path &, const std::map<std::string, avt_341::msg::Odometry> &);

    // external messages
    void handleMoveTo(const MoveToMsg & msg, double x_offset=0.0, double y_offset=0.0, FormationDefinition* formation_def = nullptr, double desired_speed = 0.0);
    void handlePathFollow(const PathFollowMsg& msg, FormationDefinition* formation_def = nullptr);
    void handleFormationRequest(FormationMsg msg);
    void handleAcknowledge(const AcknowledgeMsg &);
    void handleArrive(const ArrivedMsg & msg);
    void handleTaskComplete(const TaskCompleteMsg & msg);
    void handleSetSpeed(const SetSpeedMsg & msg);
    void handleOverwatch(const OverwatchMsg & msg);
    void handleCancelTask(const CancelMsg & msg);
    void handleCancelAllTask(const CancelAllMsg & msg);

    std::string my_name;
    double sodist_threshold;
    avt_341::msg::Odometry odometry;
    avt_341::msg::Odometry leader_odometry;
    int nav_state;
    float desired_speed;
    bool goal_changed;
    bool arrival_announced;
    double local_origin_x;
    double local_origin_y;

    // Task management
    void updateTasks();
    void postUpdateTasks();
    bool addTask(Task * task, const std::string & priority_type = PriorityType::QUEUE);
    void publishPath(const avt_341::msg::Path& path);
    void publishGoal(const avt_341::msg::PoseStamped & target_pose);
    void publishGoalPath(const avt_341::msg::Path& path);
    void publishNavStateCmd(int state);
    void publishGpToggle(int state);
    void publishArrival(const std::string & sender_name, const std::string & objective);
    void publishFormationStatus(avt_341::msg::FollowerStatus & status_msg);
    void publishCurrentTaskInfo();
    void publishTaskInfo(const Task* task);
    void reset();
    void resetTaskList(bool send_completion_msg);
    void cancelTask(int task_id,bool send_completion_msg);
    void onGoalReached(const avt_341::msg::PoseStamped & pose);
    bool hasCompletedTask(const std::string & target_veh, int target_msg_id) const;
    bool hasArrival(const std::string & target_veh, const std::string & objective) const;

    Task* currentTask();
    avt_341::msg::PoseStamped current_gp_goal;

  private:

    const FormationParameters & formation_params;
    const ToiParameters & toi_params_;
    std::vector<MissionPoint> mission_data;
    std::vector<MissionPoint> overwatch_positions;
    std::vector<MissionPath> mission_paths;
    std::deque<Task*> task_list;
    std::vector<Contact> mission_contacts;
    std::shared_ptr<node::NodeProxy> node_proxy_;

    int obj_detection_cnt=9999; // TODO: Hack for task ids of contacts, replace later
    std::vector<TaskCompleteMsg> task_completions_;
    std::vector<ArrivedMsg> arrivals_;

    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> waypoint_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::String>> reset_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> gp_path_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Int32>> navcommand_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Int32>> gp_toggle_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Communication>> communication_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Float64>> speed_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::FollowerStatus>> follower_status_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MissionTaskStatus>> task_status_pub = nullptr;

    // Methods
    bool hasContact(const std::string & name, const avt_341::msg::PoseStamped & pose);
    auto getClosestNewContact();
    MissionPoint getClosestOverwatch();
    void addContact(const std::string & name, const avt_341::msg::PoseStamped & pose);
    void publishTaskCompletion(Task * task);
    void publishTaskCompletion(const std::string & sender_name, int msg_id);

}; // class mission manager

} // namespace mission
} // namespace avt_341


#endif //AVT_341_MISSION_MGR_H
