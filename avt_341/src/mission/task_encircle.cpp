// clas definition
#include "avt_341/mission/task.h"
#include <fstream>
#include <iostream>
#include <math.h>

namespace avt_341 {
namespace mission {

// Encircle
Encircle::Encircle(MissionManager* manager, std::string sender, int id, const avt_341::msg::PoseStamped & target,
                   const avt_341::msg::PoseStamped & current_pose, double radius, double angular_range_degrees, bool is_cw,
                   double goal_threshold)
                   : target_(target), current_pose_(current_pose), radius_(radius), angular_range_degrees_(angular_range_degrees), is_cw_(is_cw), goal_threshold_(goal_threshold) {
    mgr = manager;
    sender_name = sender;
    msg_id = id;
    next_task = nullptr;
    set_busy = false;
    completed = false;
    arrived = false;
}

void Encircle::init() {

  avt_341::msg::Path path_msg;

  const auto pi = std::atan(1.0)*4.0;
  double dx = target_.pose.position.x - current_pose_.pose.position.x;
  double dy = target_.pose.position.y - current_pose_.pose.position.y;
  double starting_rad = std::atan2(dy, dx);
  double sign = is_cw_ ? -1.0 : 1.0;

  for(int r_idx = 0; r_idx < static_cast<int>(angular_range_degrees_); r_idx += 1){
    double r = starting_rad + sign * r_idx / 180.0 * pi;
    avt_341::msg::PoseStamped pose;
    pose.pose.position.x = std::cos(r) * radius_ + target_.pose.position.x;
    pose.pose.position.y = std::sin(r) * radius_ + target_.pose.position.y;
    path_msg.poses.push_back(pose);
  }

  mgr->publishPath(path_msg);
  mgr->publishNavStateCmd(avt_341::utils::NavStateCmd::GoInactive);

  if(set_busy) {
    mgr->busy = true;
  }
}

void Encircle::run() {
  double dx = mgr->odometry.pose.pose.position.x - target_.pose.position.x;
  double dy = mgr->odometry.pose.pose.position.y - target_.pose.position.y;
  arrived = arrived || (dx*dx + dy*dy < goal_threshold_*goal_threshold_);
}

bool Encircle::is_done() {
  return arrived;
}

void Encircle::on_done() {
  completed = true;
  mgr->busy = false;
}

std::string Encircle::description() const {
  std::ostringstream stream;
  stream << "TASK-ENCIRCLE: target_position=(" << target_.pose.position.x << ", " << target_.pose.position.y
  << ") radius=" << radius_ << " angular_range=" << angular_range_degrees_ << " is_cw=" << is_cw_;
  return stream.str();
}


} // mission 
} // avt_341
