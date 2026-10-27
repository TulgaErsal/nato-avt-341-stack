// clas definition
#include "avt_341_nav/mission/task.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <cmath>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace avt_341_nav {
namespace mission {

Encircle::Encircle(MissionManager* manager, const std::string & sender, int msg_id, const geometry_msgs::msg::PoseStamped & target,
  const ToiParameters & params)
    : Encircle(manager, sender, msg_id, target, params.encircle_radius, params.encircle_degrees, params.encircle_cw, params.goal_threshold) {
  replan_distance_ = params.encircle_replan_distance;
  task_speed = params.encircle_speed;
}

Encircle::Encircle(MissionManager* manager, const std::string & sender, int msg_id, const geometry_msgs::msg::PoseStamped & target,
                   double radius, double angular_range_degrees, bool is_cw, double goal_threshold)
                   : Task(manager, sender, msg_id), target_(target), radius_(radius),
                   angular_range_degrees_(angular_range_degrees), is_cw_(is_cw), goal_threshold_(goal_threshold) {
    arrived = false;
}

void Encircle::init_() {

  geometry_msgs::msg::Pose current_pose = mgr->odometry.pose.pose;
  double dx = current_pose.position.x - target_.pose.position.x;
  double dy = current_pose.position.y - target_.pose.position.y;
  start_rad_ = std::atan2(dy, dx);
  buildCirclePath_(start_rad_, angular_range_degrees_);

  mgr->publishPath(circle_path_);
  mgr->publishGpToggle(0);

}

void Encircle::buildCirclePath_(double from_rad, double degrees) {
  circle_path_.poses.clear();
  const auto pi = std::atan(1.0)*4.0;
  double sign = is_cw_ ? -1.0 : 1.0;

  for(int r_idx = 0; r_idx < std::max(1, static_cast<int>(degrees)); r_idx += 1){
    double r = from_rad + sign * r_idx / 180.0 * pi;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = std::cos(r) * radius_ + target_.pose.position.x;
    pose.pose.position.y = std::sin(r) * radius_ + target_.pose.position.y;
    circle_path_.poses.push_back(pose);
  }
}

bool Encircle::updateTarget(const geometry_msgs::msg::PoseStamped & target) {
  if (!init_done) {
    target_ = target;
    return false;
  }
  const double shift = std::hypot(target.pose.position.x - target_.pose.position.x,
                                  target.pose.position.y - target_.pose.position.y);
  if (replan_distance_ <= 0.0 || shift < replan_distance_) {
    return false;
  }
  target_ = target;

  // Keep the end of the arc at the same bearing from the target and re-plan
  // the part still ahead of the vehicle around the new target.
  const auto pi = std::atan(1.0)*4.0;
  const double sign = is_cw_ ? -1.0 : 1.0;
  const double range_rad = std::max(0.0, angular_range_degrees_ - 1.0) / 180.0 * pi;
  const double end_rad = start_rad_ + sign * range_rad;
  const double current_rad = std::atan2(mgr->odometry.pose.pose.position.y - target_.pose.position.y,
                                        mgr->odometry.pose.pose.position.x - target_.pose.position.x);
  double remaining_rad = std::fmod(sign * (end_rad - current_rad), 2.0 * pi);
  if (remaining_rad < 0.0) {
    remaining_rad += 2.0 * pi;
  }
  if (remaining_rad > range_rad) {
    // Outside the arc: either just past its end or still before its start.
    const double past_end_rad = 2.0 * pi - remaining_rad;
    const double before_start_rad = remaining_rad - range_rad;
    remaining_rad = past_end_rad < before_start_rad ? 0.0 : range_rad;
  }
  // Whole-degree steps back from the end bearing, so the arc ends exactly on it.
  const double steps = std::floor(remaining_rad / pi * 180.0 + 1e-9);
  buildCirclePath_(end_rad - sign * steps / 180.0 * pi, steps + 1.0);
  return true;
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
  if (!contact_name_.empty()) {
    mgr->markContactInvestigated(contact_name_);
  }
}

std::string Encircle::description() const {
  std::ostringstream stream;
  stream << "ID " << msg_id << " ENCIRCLE: target_position=(" << target_.pose.position.x << ", " << target_.pose.position.y
  << ") radius=" << radius_ << " angular_range=" << angular_range_degrees_ << " is_cw=" << is_cw_;
  return stream.str();
}

geometry_msgs::msg::PoseStamped Encircle::terminalPose() const{
  if(circle_path_.poses.empty()){
    return Task::terminalPose();
  }
  return circle_path_.poses.back();
}


} // mission 
} // avt_341_nav
