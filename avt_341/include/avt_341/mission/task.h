#ifndef AVT_341_TASK_H
#define AVT_341_TASK_H
/**
* \class Task
*
* Class for keeping track of tasks assigned to the vehicle
* Based on Naisense ScenarioRunner
*
* \author Daniel Carruth
*
* \date 1/31/2023
*/

// c++ includes
#include <string>
#include <sstream>
// local includes
#include "avt_341/node/ros_types.h"
#include "avt_341/mission/mission_manager.h"
#include "avt_341/mission/formation_definition.h"
#include "avt_341/mission/formation_path_generator.h"

namespace avt_341 {
namespace mission {

class MissionManager;
struct Contact;

class Task {
public:
    Task(MissionManager* manager, const std::string & sender, int msg_id, FormationDefinition* formation_def = nullptr);
    virtual ~Task();

    virtual void init();
    virtual void run() = 0;
    virtual bool is_done() { return true; }
    virtual void on_done() = 0;
    virtual std::string description() const { return "Task"; }
    virtual void onGoalReached(const avt_341::msg::PoseStamped & pose){};
    virtual void onPreempt() {}

    bool hasFormation() const;

    std::string sender_name;
    int msg_id;
    bool init_done;
    bool is_preemptable;
    bool arrived = false;
    double task_speed = -1.0;
    FormationDefinition* getFormationDef() const { return formation_def_; }
    virtual avt_341::msg::PoseStamped terminalPose() const;


protected:
  virtual void init_() = 0;
  MissionManager* mgr;
  FormationDefinition* formation_def_;

}; // class Task

class MoveTo : public Task {
public:
    static const std::string NONE;
    static const std::string POSE;
    static const std::string MISSION_POINT;
    static const std::string CONTACT;
    static const std::string ACTOR;

    // TODO: Too many parameters, can place most goal parameters in NavGoal structure
    MoveTo(MissionManager* manager, const std::string & sender, int msg_id, FormationDefinition* formation_def = nullptr,
           double x_offset = 0.0, double y_offset = 0.0, double goal_threshold=-1.0, double yaw_threshold=-1.0, double desired_speed=-1.0);
    void init_() override;
    void run() override;
    bool is_done() override;
    void on_done() override;
    void onPreempt() override;
    void onGoalReached(const avt_341::msg::PoseStamped & pose) override;

    bool setGoalByContact(const Contact & contact);
    bool setGoalByPose(const avt_341::msg::PoseStamped & pose);
    bool setGoalByMissionPoint(std::string name);
    avt_341::msg::PoseStamped terminalPose() const override;

    // goal = position and orientation
    avt_341::msg::PoseStamped goal;
    avt_341::msg::PoseStamped target_pose;
    std::string goal_type;
    std::string name;
    bool terminate_on_all_arrived_;
    std::string description() const override;
private:
    bool setGoalInternal(const avt_341::msg::PoseStamped & pose, const std::string & name_in, const std::string & pose_type);
    void applyOffset();
    double x_offset_;
    double y_offset_;
    double goal_threshold_;
    double yaw_threshold_;
}; // class MoveTo

class WaitUntilComplete : public Task {
public:
    WaitUntilComplete(MissionManager * manager, const std::string & sender, int msg_id, const std::string & target_veh, int target_msg_id);
    void init_() override;
    void run() override;
    bool is_done() override;
    void on_done() override;
    std::string description() const override;
private:
  std::string target_veh_;
  int target_msg_id_;
}; // class WaitUntil

class Encircle : public Task {
public:
    Encircle(MissionManager* manager, const std::string & sender, int msg_id, const avt_341::msg::PoseStamped & target,
           const ToiParameters & params);
    Encircle(MissionManager* manager, const std::string & sender, int msg_id, const avt_341::msg::PoseStamped & target,
             double radius=15.0, double angular_range_degrees=180.0, bool is_cw = true, double goal_threshold=5.0);
    void init_() override;
    void run() override;
    bool is_done() override;
    void on_done() override;
    std::string description() const override;
    avt_341::msg::PoseStamped terminalPose() const override;

private:
  bool arrived;
  avt_341::msg::PoseStamped target_;
  double radius_;
  double angular_range_degrees_;
  bool is_cw_;
  double goal_threshold_;
  avt_341::msg::Path circle_path_;
}; // class Encircle

class Follow : public Task {
public:
    Follow(MissionManager* manager, std::string sender, int msg_id, FormationDefinition* formation_def,
        double desired_speed = -1.0, double goal_threshold=-1.0, double yaw_threshold=-1.0);
    void init_() override;
    void run() override;
    bool is_done() override;
    void on_done() override;
    void onPreempt() override;
    std::string description() const override;
    avt_341::msg::PoseStamped terminalPose() const override;

private:
  bool terminate_on_leader_arrived_;
  bool terminate_on_all_arrived_;
  avt_341::mission::FormationPathGenerator path_generator_;
  double goal_threshold_;
  double yaw_threshold_;
}; // class Follow

class PathFollow : public Task {
public:
    PathFollow(MissionManager* manager, const std::string & sender, int msg_id, FormationDefinition* formation_def = nullptr, double desired_speed=-1.0);
    void init_() override;
    void run() override;
    bool is_done() override;
    void on_done() override;
    void onPreempt() override;
    void onGoalReached(const avt_341::msg::PoseStamped & pose) override;

    bool setPathByDef(std::string name);
    avt_341::msg::PoseStamped terminalPose() const override;

    avt_341::msg::Path path;
    avt_341::msg::PoseStamped target_pose;
    std::string goal_type;
    std::string name;
    bool terminate_on_all_arrived_;
    std::string description() const override;
private:
    bool setPathInternal(const avt_341::msg::Path & path_in, const std::string & name_in);

}; // class PathFollow

} // namespace mission
} // namespace avt_341


#endif //AVT_341_TASK_H
