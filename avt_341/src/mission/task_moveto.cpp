// clas definition
#include <fstream>
#include <sstream>
#include <iostream>

#include "avt_341/mission/task.h"
#include "avt_341/avt_341_utils.h"
#include "avt_341/mission/formation_utils.h"

namespace avt_341 {
namespace mission {

// MoveTo
MoveTo::MoveTo(MissionManager* manager, std::string sender, int id, double x_offset, double y_offset, int priority_in)
: x_offset_(x_offset), y_offset_(y_offset) {
    mgr = manager;
    sender_name = sender;
    msg_id = id;
    next_task = NULL;
    goal_type = 0;
    arrived = false;
    priority = priority_in;
    name="TEMP_NAME";
    goal.pose.position.x = 0.0;
    goal.pose.position.y = 0.0;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;
    set_busy = false;
}

bool MoveTo::setGoalByPose(float x, float y, float z, float rot_x, float rot_y, float rot_z, float rot_w) {
    goal_type = POSE;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation.x = rot_x;
    goal.pose.orientation.y = rot_y;
    goal.pose.orientation.z = rot_z;
    goal.pose.orientation.w = rot_w;

    applyOffset();

    std::ostringstream stream;
    stream << "POSE_" << x << "_" << y;
    name = stream.str();
    return true;
}

void MoveTo::applyOffset(){
  Vec2d vx, vy;
  PoseToForwardRightVectors(goal.pose, vx, vy);
  goal.pose.position.x = goal.pose.position.x + vx[0]*x_offset_ + vy[0]*y_offset_;
  goal.pose.position.y = goal.pose.position.y + vx[1]*x_offset_ + vy[1]*y_offset_;
}

bool MoveTo::setGoalByMissionPoint(std::string mp_name) {
    goal_type = MISSION_POINT;
    MissionPoint target;
    bool validPoint = mgr->getMissionPoint(target, mp_name);
    if(validPoint) {
        goal.pose.position.x = target.pos_x;
        goal.pose.position.y = target.pos_y;
        goal.pose.position.z = target.pos_z;
        goal.pose.orientation.x = target.rot_x;
        goal.pose.orientation.y = target.rot_y;
        goal.pose.orientation.z = target.rot_z;
        goal.pose.orientation.w = target.rot_w;

        applyOffset();

        name = mp_name;
        return true;
    } else {
        std::cout << "Pose " << mp_name << " not found in mission data!" << std::endl;
        return false;
    }
}

void MoveTo::init() {

    avt_341::msg::Path path_msg;
    path_msg.poses.clear();
    path_msg.poses.push_back(goal);
    mgr->publishPath(path_msg);

    mgr->publishNavStateCmd(avt_341::utils::NavStateCmd::GoActive);

    if(set_busy) {
        mgr->busy = true;
    }
}

void MoveTo::run() {
    // get current position
    // calculate distance from goal
    if(mgr->previous_nav_state == avt_341::utils::NavStackState::Active
    && mgr->nav_state == avt_341::utils::NavStackState::Stopped)
    {
        // we've reached a goal and are stopping
        // TODO: Check distance b/c we could be stopping for _any_ reason
        std::cout << mgr->my_name << " detected change in nav_state from active to stopping - assuming arriving at goal" << std::endl;
        arrived = true;
    }
}

bool MoveTo::is_done() {
    return arrived;
}

void MoveTo::on_done() {

    // announce arrival
    std::ostringstream stream;
    stream << "ARRIVE," << name;
    mgr->publishCommStr(stream.str());

    // update contact investigation status
    if(goal_type == CONTACT) {
        if(contact != nullptr) {
            contact->investigated = true;
        }
    }

    completed = true;
    mgr->busy = false;
}

std::string MoveTo::description() const {
  std::ostringstream stream;
  stream << "TASK-MOVE_TO: goal_type=" << goal_type << ", name=" << name << ", x_off=" << x_offset_ << ", y_off=" << y_offset_;
  return stream.str();
}

} // mission 
} // avt_341
