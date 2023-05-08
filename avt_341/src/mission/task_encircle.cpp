// clas definition
#include "avt_341/mission/task.h"
#include <fstream>
#include <iostream>
#include <math.h>

namespace avt_341 {
namespace mission {

Encircle::Encircle(MissionManager* manager, const std::string & sender, int msg_id, const avt_341::msg::PoseStamped & target,
  const ToiParameters & params)
    : Encircle(manager, sender, msg_id, target, params.encircle_radius, params.encircle_degrees, params.encircle_cw, params.goal_threshold) {
}

Encircle::Encircle(MissionManager* manager, const std::string & sender, int msg_id, const avt_341::msg::PoseStamped & target,
                   double radius, double angular_range_degrees, bool is_cw, double goal_threshold)
                   : Task(manager, sender, msg_id), target_(target), radius_(radius),
                   angular_range_degrees_(angular_range_degrees), is_cw_(is_cw), goal_threshold_(goal_threshold) {
    arrived = false;
}

void Encircle::init_() {

  circle_path_.poses.clear();
  avt_341::msg::Pose current_pose = mgr->odometry.pose.pose;
  const auto pi = std::atan(1.0)*4.0;
  double dx = current_pose.position.x - target_.pose.position.x;
  double dy = current_pose.position.y - target_.pose.position.y;
  double starting_rad = std::atan2(dy, dx);
  double sign = is_cw_ ? -1.0 : 1.0;

  for(int r_idx = 0; r_idx < static_cast<int>(angular_range_degrees_); r_idx += 1){
    double r = starting_rad + sign * r_idx / 180.0 * pi;
    avt_341::msg::PoseStamped pose;
    pose.pose.position.x = std::cos(r) * radius_ + target_.pose.position.x;
    pose.pose.position.y = std::sin(r) * radius_ + target_.pose.position.y;
    circle_path_.poses.push_back(pose);
  }

  mgr->publishPath(circle_path_);
  mgr->publishGpToggle(0);

}

void Encircle::run() {

  mgr->publishPath(circle_path_);
  mgr->publishGpToggle(0);

  double dx = mgr->odometry.pose.pose.position.x - circle_path_.poses.back().pose.position.x;
  double dy = mgr->odometry.pose.pose.position.y - circle_path_.poses.back().pose.position.y;
  arrived = arrived || (dx*dx + dy*dy < goal_threshold_*goal_threshold_);
}

bool Encircle::is_done() {
  return arrived;
}

void Encircle::on_done() {
}

std::string Encircle::description() const {
  std::ostringstream stream;
  stream << "ID " << msg_id << " ENCIRCLE: target_position=(" << target_.pose.position.x << ", " << target_.pose.position.y
  << ") radius=" << radius_ << " angular_range=" << angular_range_degrees_ << " is_cw=" << is_cw_;
  return stream.str();
}


} // mission 
} // avt_341
