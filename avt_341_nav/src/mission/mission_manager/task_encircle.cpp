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
  speed_limit = params.encircle_speed;
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
  swept_rad_ = 0.0;
  last_position_ = current_pose.position;
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

  // Re-plan the arc still to sweep around the new target, from the vehicle's bearing.
  const auto pi = std::atan(1.0)*4.0;
  const double remaining_rad = std::clamp(rangeRad_() - swept_rad_, 0.0, 2.0 * pi - pi / 180.0);
  const double steps = std::floor(remaining_rad / pi * 180.0 + 1e-9);
  buildCirclePath_(bearingFromTarget_(mgr->odometry.pose.pose.position), steps + 1.0);
  return true;
}

double Encircle::rangeRad_() const {
  return std::max(0.0, angular_range_degrees_ - 1.0) / 180.0 * std::atan(1.0) * 4.0;
}

double Encircle::bearingFromTarget_(const geometry_msgs::msg::Point & point) const {
  return std::atan2(point.y - target_.pose.position.y, point.x - target_.pose.position.x);
}

void Encircle::run() {
  // Angle the vehicle has swept around the target in the encircle direction, so cutting across to the end of the arc
  // does not count as having encircled it.
  const auto pi = std::atan(1.0)*4.0;
  const auto & position = mgr->odometry.pose.pose.position;
  const double step = std::remainder(bearingFromTarget_(position) - bearingFromTarget_(last_position_), 2.0 * pi);
  swept_rad_ += is_cw_ ? -step : step;
  last_position_ = position;

  mgr->publishPath(circle_path_);
  mgr->publishGpToggle(0);

  arrived = arrived || swept_rad_ >= rangeRad_() - goal_threshold_ / radius_;
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
