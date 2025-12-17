// clas definition
#include "avt_341/mission/task.h"
#include <fstream>
#include <iostream>

namespace avt_341 {
namespace mission {

// Follow
Follow::Follow(MissionManager* manager, std::string sender, int id, FormationDefinition* formation_def, double desired_speed)
: Task(manager, sender, id, formation_def), path_generator_(formation_def->params) {
    const std::string termination_method = formation_def->terminationMethod();
    terminate_on_leader_arrived_ = termination_method == "LEADER_ARRIVED";
    terminate_on_all_arrived_ = termination_method == "ALL_ARRIVED";
    task_speed = desired_speed;
}

void Follow::init_() {
    mgr->publishNavStateCmd(avt_341::utils::NavStateCmd::GoActive);
    mgr->publishGpToggle(path_generator_.useBreadcrumbs() ? 0 : 1);

    if(!formation_def_->formationAtGoal()){
      mgr->publishFormationStatus(formation_def_->formation_status);
    }
}

void Follow::run() {
    if (!(mgr->rcvd_leader_odom)) return;
    path_generator_.Update(mgr->leader_odometry, mgr->odometry, formation_def_->formation_status);
    const auto & follower_path = path_generator_.GetPath();
    if(path_generator_.useBreadcrumbs()){
      mgr->publishPath(follower_path);
    }else if(!follower_path.poses.empty()){
      mgr->publishGoal(follower_path.poses.back());
    }
}

avt_341::msg::PoseStamped Follow::terminalPose() const{
  const auto & follower_path = path_generator_.GetPath();
  if(follower_path.poses.empty()){
    return Task::terminalPose();
  }
  return follower_path.poses.back();
}

void Follow::onPreempt(){
  init_done = false;
  path_generator_.Reset();
}

bool Follow::is_done() {
    if(terminate_on_leader_arrived_){
        return mgr->hasCompletedTask(formation_def_->leaderName(), msg_id);
    }
    if(terminate_on_all_arrived_){
      bool leader_arrived = mgr->hasArrival(formation_def_->followedVehicle(), "TASK_" + std::to_string(msg_id));
      bool at_termination_location = leader_arrived && PosePlanarDistance(mgr->odometry.pose.pose.position, terminalPose().pose.position) < formation_def_->params.follow_goal_threshold;
      if(!arrived && at_termination_location){
        mgr->publishArrival(mgr->my_name, "TASK_" + std::to_string(msg_id));
      }
      arrived = arrived || at_termination_location;
      bool all_arrived = true;
      for(const auto & veh : formation_def_->orderedVehicles()){
        all_arrived = all_arrived && mgr->hasArrival(veh, "TASK_" + std::to_string(msg_id));
      }
      return all_arrived;
    }
    return false;
}

void Follow::on_done() {
    std::cout << mgr->my_name << " Follow Task is completed" << std::endl;
}

std::string Follow::description() const {
    std::ostringstream stream;
    stream << "ID " << msg_id << " FOLLOW " << formation_def_->followedVehicle();
    return stream.str();
}

} // mission 
} // avt_341
