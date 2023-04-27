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
// local includes
#include "avt_341/node/ros_types.h"
#include "avt_341/mission/mission_manager.h"

namespace avt_341 {
namespace mission {

class MissionManager;
struct Contact;

class Task {
public:
    virtual void init() = 0; 
    virtual void run() = 0;
    virtual bool is_done() { return true; }
    virtual void on_done() = 0;
    std::string sender_name;
    int msg_id;
    Task* next_task;
    MissionManager* mgr;
    bool set_busy;
    bool completed;
}; // class Task

class MoveTo : public Task {
public:
    static const int NONE = 0;
    static const int POSE = 1;
    static const int MISSION_POINT = 2;
    static const int CONTACT = 3;
    static const int ACTOR = 4;

    MoveTo(MissionManager* manager, std::string sender, int msg_id);
    void init() override;
    void run() override;
    bool is_done() override;
    void on_done() override;

    bool setGoalByPose(float x, float y, float z, float rot_w, float rot_x, float rot_y, float rot_z);
    bool setGoalByMissionPoint(std::string name);

    // goal = position and orientation
    avt_341::msg::PoseStamped goal;
    int goal_type; 
    std::string name;
    bool arrived;
    Contact * contact;
}; // class MoveTo

class WaitUntil : public Task {
public:
    WaitUntil(MissionManager* manager, std::string sender, int msg_id);
    void init() override;
    void run() override;
    bool is_done() override;
    void on_done() override;
}; // class WaitUntil

class Encircle : public Task {
public:
    Encircle(MissionManager* manager, std::string sender, int msg_id);
    void init() override;
    void run() override;
    bool is_done() override;
    void on_done() override;
}; // class Encircle

class Follow : public Task {
public:
    Follow(MissionManager* manager, std::string sender, int msg_id);
    void init() override;
    void run() override;
    bool is_done() override;
    void on_done() override;
}; // class Follow


} // namespace mission
} // namespace avt_341


#endif //AVT_341_TASK_H
