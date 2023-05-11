// clas definition
#include <fstream>
#include <sstream>
#include <iostream>

#include "avt_341/mission/task.h"
#include "avt_341/avt_341_utils.h"
#include "avt_341/mission/formation_utils.h"

namespace avt_341 {
namespace mission {

const std::string MoveTo::NONE = "NONE";
const std::string MoveTo::POSE = "POSE";
const std::string MoveTo::MISSION_POINT = "MISSION_POINT";
const std::string MoveTo::CONTACT = "CONTACT";
const std::string MoveTo::ACTOR = "ACTOR";

// MoveTo
MoveTo::MoveTo(MissionManager* manager, std::string sender, int id, FormationDefinition* formation_def,
               double x_offset, double y_offset, double d_approach)
: Task(manager, sender, id, formation_def), x_offset_(x_offset), y_offset_(y_offset), d_approach_(d_approach) {
    setGoalInternal(avt_341::msg::PoseStamped(), "", MoveTo::NONE);
}

bool MoveTo::setGoalInternal(const avt_341::msg::PoseStamped & pose, const std::string & name_in, const std::string & pose_type){
  goal_type = pose_type;
  name = name_in;
  goal = pose;
  arrived = false;
  applyOffset();
  return true;
}

void MoveTo::applyOffset(){
  Vec2d vx, vy;
  PoseToForwardRightVectors(goal.pose, vx, vy);
  goal.pose.position.x = goal.pose.position.x + vx[0]*x_offset_ + vy[0]*y_offset_;
  goal.pose.position.y = goal.pose.position.y + vx[1]*x_offset_ + vy[1]*y_offset_;
}

bool MoveTo::setGoalByContact(const Contact & contact) {
  return setGoalInternal(contact.pose, contact.name, MoveTo::CONTACT);
}

bool MoveTo::setGoalByPose(const avt_341::msg::PoseStamped & pose) {
  return setGoalInternal(pose, "pose", MoveTo::POSE);
}

bool MoveTo::setGoalByMissionPoint(std::string mp_name) {
    MissionPoint target;
    if(mgr->getMissionPoint(target, mp_name)) {
        avt_341::msg::PoseStamped pose;
        pose.pose.position.x = target.pos_x;
        pose.pose.position.y = target.pos_y;
        pose.pose.position.z = target.pos_z;
        pose.pose.orientation.x = target.rot_x;
        pose.pose.orientation.y = target.rot_y;
        pose.pose.orientation.z = target.rot_z;
        pose.pose.orientation.w = target.rot_w;
        return setGoalInternal(pose, mp_name, MoveTo::MISSION_POINT);
    } else {
        std::cout << "Pose " << mp_name << " not found in mission data!" << std::endl;
        return false;
    }
}

void MoveTo::applyApproachDistance(){
  if(d_approach_ > 0.0){
    avt_341::msg::Pose current_pose = mgr->odometry.pose.pose;
    auto unit_vect = avt_341::utils::vec2(goal.pose.position.x - current_pose.position.x, goal.pose.position.y - current_pose.position.y);
    unit_vect.normalize();
    target_pose.pose.position.x = goal.pose.position.x - unit_vect.x*d_approach_;
    target_pose.pose.position.y = goal.pose.position.y - unit_vect.y*d_approach_;
  }

}

void MoveTo::init_() {
    target_pose = goal;

    applyApproachDistance();
    mgr->publishGoal(target_pose);
    mgr->publishNavStateCmd(avt_341::utils::NavStateCmd::GoActive);
    mgr->publishGpToggle(1);

    if(formation_def_ == nullptr){
      // TODO: Need to publish formation def with is_leader to turn off formation_controller from auto-publishing path...
      // TODO: Should find better way to disable formation controller, maybe just put it's logic in FollowTask, since only place it is relevant
      avt_341::msg::FollowerStatus fs;
      fs.use_leader = false;
      mgr->publishFormationStatus(fs);
    }

    if(formation_def_ != nullptr && !formation_def_->formationAtGoal()){
      mgr->publishFormationStatus(formation_def_->formation_status);
    }

}

void MoveTo::run() {
  // Need to recalculate if approach distance set or keep publishing to gp if did not receive callback yet
  if(d_approach_ > 0.0){
    applyApproachDistance();
    mgr->publishGoal(target_pose);
  }else if (PosePlanarDistance(mgr->current_gp_goal.pose.position, target_pose.pose.position) > 1.0){
    mgr->publishGoal(target_pose);
  }
  // Nothing to do per timestep but wait for goal to be reached
}

bool MoveTo::is_done() {
    return arrived;
}

void MoveTo::on_done() {
    // announce arrival
    std::ostringstream stream;
    stream << "ARRIVE," << name;
    mgr->publishCommStr(stream.str());
}

void MoveTo::onPreempt(){
  init_done = false;
  mgr->publishGpToggle(0);
}

std::string MoveTo::description() const {
  std::ostringstream stream;
  stream << "ID " << msg_id << " MOVE_TO: " << goal_type << " " << name << " (" << goal.pose.position.x << "," << goal.pose.position.y << ") " << "off=(" << x_offset_ << "," << y_offset_ << ")" << " d=" << d_approach_;
  return stream.str();
}

void MoveTo::onGoalReached(const avt_341::msg::PoseStamped & pose){
  const double abs_error = std::abs(target_pose.pose.position.x - pose.pose.position.x + target_pose.pose.position.y - pose.pose.position.y);
  arrived = arrived || abs_error < 1.0;
}

avt_341::msg::PoseStamped MoveTo::terminalPose() const{
  return target_pose;
}

} // mission 
} // avt_341
