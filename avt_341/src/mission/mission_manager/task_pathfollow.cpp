#include <fstream>
#include <sstream>
#include <iostream>

#include "avt_341/mission/task.h"
#include "avt_341/avt_341_utils.h"
#include "avt_341/mission/formation_utils.h"

namespace avt_341 {
namespace mission {

PathFollow::PathFollow(MissionManager* manager, const std::string & sender, int msg_id, FormationDefinition* formation_def, double desired_speed)
: Task(manager, sender, msg_id, formation_def) {
    setPathInternal(avt_341::msg::Path(), "");
    terminate_on_all_arrived_ = formation_def != nullptr && formation_def->terminationMethod() == "ALL_ARRIVED";
    task_speed = desired_speed;
}

bool PathFollow::setPathInternal(const avt_341::msg::Path & path_in, const std::string & name_in) {
    name = name_in;
    path = path_in;
    arrived = false;
    return true;
}

bool PathFollow::setPathByDef(std::string name) {
    MissionPath target;
    if(mgr->getMissionPath(target, name)) {
        avt_341::msg::Path path_in;
        for (MissionPoint mp: target.poses) {
            avt_341::msg::PoseStamped pose;
            pose.pose.position.x = mp.pos_x;
            pose.pose.position.y = mp.pos_y;
            pose.pose.position.z = mp.pos_z;
            pose.pose.orientation.x = mp.rot_x;
            pose.pose.orientation.y = mp.rot_y;
            pose.pose.orientation.z = mp.rot_z;
            pose.pose.orientation.w = mp.rot_w;
            path_in.poses.push_back(pose);
        }
        return setPathInternal(path_in, name);
    } else {
        std::cout << "Path " << name << " not found in mission data!" << std::endl;
        return false;
    }
}

void PathFollow::init_() {
    target_pose = path.poses.back();

    mgr->publishGoalPath(path);
    mgr->publishNavStateCmd(avt_341::utils::NavStateCmd::GoActive);
    mgr->publishGpToggle(1);
}

void PathFollow::run() {
  // Nothing to do per timestep but wait for goal to be reached
}

bool PathFollow::is_done() {
    if(terminate_on_all_arrived_){
      bool all_arrived = true;
      for(const auto & veh : formation_def_->orderedVehicles()){
        all_arrived = all_arrived && mgr->hasArrival(veh, "TASK_" + std::to_string(msg_id));
      }
      return all_arrived;
    }
    return arrived;
}

void PathFollow::on_done() {
}

void PathFollow::onPreempt(){
  init_done = false;
}

std::string PathFollow::description() const {
  std::ostringstream stream;
  stream << "ID " << msg_id << " PATH_FOLLOW: " << name;
  return stream.str();
}

void PathFollow::onGoalReached(const avt_341::msg::PoseStamped & pose){
  bool goal_reached = std::abs(target_pose.pose.position.x - pose.pose.position.x + target_pose.pose.position.y - pose.pose.position.y) < 1.0;
  if(!arrived && goal_reached){
    mgr->publishArrival(mgr->my_name, "TASK_" + std::to_string(msg_id));
  }
  arrived = arrived || goal_reached;
}

avt_341::msg::PoseStamped PathFollow::terminalPose() const{
  return target_pose;
}

} // mission 
} // avt_341