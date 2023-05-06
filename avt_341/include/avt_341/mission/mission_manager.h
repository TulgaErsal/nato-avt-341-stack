
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
    float x, y;
    bool investigated;
    bool investigating;
    bool is_new;
};
    
/// Class for formation control
class MissionManager{

  public:
    /// Construct a formation controller
    MissionManager(const FormationParameters & formation_params, std::shared_ptr<node::NodeProxy> node_proxy);
    ~MissionManager();

    int loadMissionDefinition(std::string filename);

    bool getMissionPoint(MissionPoint& mission_point, std::string posename);

    // internal messages
    void handleContacts(avt_341::msg::Path);

    // external messages
    void handleMoveTo(const avt_341::msg::Communication &, double x_offset=0.0, double y_offset=0.0, FormationDefinition* formation_def = nullptr);
    void handleFormationRequest(avt_341::msg::Communication);
    void handleAcknowledge(const avt_341::msg::Communication &);
    void handleArrive(const avt_341::msg::Communication &);
    void handleTaskComplete(const avt_341::msg::Communication &);
    void handleHold(const avt_341::msg::Communication &);
    void handleSetSpeed(const avt_341::msg::Communication &);

    std::string my_name;
    float same_object_distance_threshold_sq;
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
    Task* currentTask();

  private:
    const FormationParameters & formation_params;
    std::vector<MissionPoint> mission_data;
    std::deque<Task*> task_list;
    std::vector<Contact> mission_contacts;
    std::shared_ptr<node::NodeProxy> node_proxy_;

    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::FollowerStatus>> follower_status_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> waypoint_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Int32>> navcommand_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Int32>> gp_toggle_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::String>> communication_pub = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Float64>> speed_pub = nullptr;

    // Methods
    auto getContact(std::string name, float x, float y);
    auto getClosestNewContact(float veh_x, float veh_y);
    void addContact(std::string name, float x, float y);
    bool close(float x, float y, float n_x, float n_y);
    float distance_sq(float x1, float y1, float x2, float y2);
    void publishTaskCompletion(Task * task);
    void publishTaskCompletion(const std::string & sender_name, int msg_id);

}; // class mission manager

} // namespace mission
} // namespace avt_341


#endif //AVT_341_MISSION_MGR_H
