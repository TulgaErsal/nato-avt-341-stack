
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
#include <deque>

namespace avt_341 {
namespace mission {

class Task;

class PriorityType{
public:
  static const std::string QUEUE;
  static const std::string QUEUE_SHORT;
  static const std::string PREEMPT;
  static const std::string PREEMPT_SHORT;
  static const std::string CANCEL_ALL_PREVIOUS;
  static const std::string CANCEL_ALL_PREVIOUS_SHORT;
  inline static bool isQueue(const std::string &type) { return type.empty() || type == QUEUE || type == QUEUE_SHORT; }
  inline static bool isPreempt(const std::string &type) { return type == PREEMPT || type == PREEMPT_SHORT; }
  inline static bool isCancelAllPrevious(const std::string &type) { return type == CANCEL_ALL_PREVIOUS || type == CANCEL_ALL_PREVIOUS_SHORT; }
};

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

    // internal messages
    void handleContacts(avt_341::msg::Path);

    // external messages
    void handleMoveTo(const avt_341::msg::Communication &, double x_offset=0.0, double y_offset=0.0, FormationDefinition* formation_def = nullptr);
    bool isMsgForSelf(const avt_341::msg::Communication & msg);
    void handleFormationRequest(avt_341::msg::Communication);
    void handleAcknowledge(const avt_341::msg::Communication &);
    void handleArrive(const avt_341::msg::Communication &);
    void handleTaskComplete(const avt_341::msg::Communication &);
    void handleHold(const avt_341::msg::Communication &);
    void handleSetSpeed(const avt_341::msg::Communication &);

    std::string my_name;
    double sodist_threshold;
    avt_341::msg::Odometry odometry;
    int nav_state;
    float desired_speed;
    bool goal_changed;
    bool arrival_announced;

    // Task management
    void updateTasks();
    void postUpdateTasks();
    bool addTask(Task * task, const std::string & priority_type = PriorityType::QUEUE);
    void publishPath(avt_341::msg::Path& path);
    void publishGoal(avt_341::msg::PoseStamped & target_pose);
    void publishNavStateCmd(int state);
    void publishGpToggle(int state);
    void publishCommStr(const std::string & msg_data);
    void publishFormationStatus(avt_341::msg::FollowerStatus & status_msg);
    void reset();
    void resetTaskList(bool send_completion_msg);
    void handleCancelTask(const avt_341::msg::Communication & msg);
    void handleCancelAllTask(const avt_341::msg::Communication & msg);
    void cancelTask(int task_id,bool send_completion_msg);
    void onGoalReached(const avt_341::msg::PoseStamped & pose);
    bool hasCompletedTask(const std::string & target_veh, int target_msg_id);
    Task* currentTask();
    avt_341::msg::PoseStamped current_gp_goal;

  private:
    // TODO: Remove later: Hardcoded value for who does investigation and overwatch
    // ==============================================================================
    const std::string INVESTIGATING_AGV = "AGV1";
    const std::string OVERWATCH_AGV = "AGV2";
    // ==============================================================================

    const FormationParameters & formation_params;
    const ToiParameters & toi_params_;
    std::vector<MissionPoint> mission_data;
    std::vector<MissionPoint> overwatch_positions;
    std::deque<Task*> task_list;
    std::vector<Contact> mission_contacts;
    std::shared_ptr<node::NodeProxy> node_proxy_;

    int obj_detection_cnt=9999; // TODO: Hack for task ids of contacts, replace later
    std::vector<avt_341::msg::Communication> task_completions_;


    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::FollowerStatus>> follower_status_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> waypoint_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> gp_path_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Int32>> navcommand_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Int32>> gp_toggle_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::String>> communication_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Float64>> speed_pub = nullptr;

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
