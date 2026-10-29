// clas definition
#include "avt_341_nav/mission/task.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace avt_341_nav {
namespace mission {

Encircle::Encircle(MissionManager* manager, const std::string & sender, int msg_id, const geometry_msgs::msg::PoseStamped & target,
  const ToiParameters & params)
    : Encircle(manager, sender, msg_id, target, params.encircle_radius, params.encircle_degrees, params.encircle_cw, params.goal_threshold) {
  replan_distance_ = params.encircle_replan_distance;
  speed_limit = params.encircle_speed;
  blocked_time_ = params.encircle_blocked_time;
  blocked_lookahead_ = params.encircle_blocked_lookahead;
  blocked_fraction_ = params.encircle_blocked_fraction;
  blocked_clearance_ = params.encircle_blocked_clearance;
  tangent_entry_ = params.encircle_tangent_entry;
  stall_time_ = params.encircle_stall_time;
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
  buildPath_(rangeRad_());

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

// A straight line from the vehicle to its tangent point on the circle (when tangent entry is on and the vehicle is
// outside), then the rest of the arc to sweep along the circle. The line's sweep counts, so the path ends where the arc
// from the vehicle's bearing would.
void Encircle::buildPath_(double remaining_rad) {
  const auto pi = std::atan(1.0)*4.0;
  away_from_end_ = false;
  const double sign = is_cw_ ? -1.0 : 1.0;
  const auto & ego = mgr->odometry.pose.pose.position;
  const double entry = std::min(entryRad_(), remaining_rad);
  const double arc = std::clamp(remaining_rad - entry, 0.0, 2.0 * pi - pi / 180.0);
  buildCirclePath_(bearingFromTarget_(ego) + sign * entry, std::floor(arc / pi * 180.0 + 1e-9) + 1.0);
  if (entry <= 0.0) {
    return;
  }
  const auto tangent = circle_path_.poses.front().pose.position;
  const double length = std::hypot(tangent.x - ego.x, tangent.y - ego.y);
  const int n = static_cast<int>(std::ceil(length / (radius_ * pi / 180.0)));
  std::vector<geometry_msgs::msg::PoseStamped> line;
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = ego.x + (tangent.x - ego.x) * i / n;
    pose.pose.position.y = ego.y + (tangent.y - ego.y) * i / n;
    line.push_back(pose);
  }
  circle_path_.poses.insert(circle_path_.poses.begin(), line.begin(), line.end());
}

double Encircle::entryRad_() const {
  const auto & ego = mgr->odometry.pose.pose.position;
  const double distance = std::hypot(ego.x - target_.pose.position.x, ego.y - target_.pose.position.y);
  return tangent_entry_ && distance > radius_ ? std::acos(radius_ / distance) : 0.0;
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
  buildPath_(std::max(rangeRad_() - swept_rad_, 0.0));
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
  if (arrived) {
    return;
  }

  // Reaching the end of the path, e.g. across the arc's gap, ends the encircle: the path has nothing left to follow.
  const auto & end = circle_path_.poses.back().pose.position;
  if (std::hypot(position.x - end.x, position.y - end.y) > goal_threshold_) {
    away_from_end_ = true;
  } else if (away_from_end_) {
    abandoned_ = true;
    if (swept_rad_ < rangeRad_() - pi / 4.0) {
      RCLCPP_WARN(mgr->logger(), "Ending the encircle of \"%s\": reached the end of its path after sweeping %.0f of %.0f deg.",
                  contact_name_.c_str(), swept_rad_ * 180.0 / pi, rangeRad_() * 180.0 / pi);
    }
    return;
  }

  const double now = mgr->nowSeconds();
  if (stall_time_ > 0.0) {
    if (progress_since_ < 0.0 || !mgr->autonomyEnabled() || swept_rad_ > progress_swept_rad_ + 5.0 / 180.0 * pi) {
      progress_since_ = now;
      progress_swept_rad_ = swept_rad_;
    } else if (now - progress_since_ >= stall_time_) {
      abandoned_ = true;
      RCLCPP_WARN(mgr->logger(), "Abandoning the encircle of \"%s\": no progress around it for %.1f s.",
                  contact_name_.c_str(), now - progress_since_);
      return;
    }
  }

  if (blocked_time_ <= 0.0 || !mgr->occupancy_grid) {
    return;
  }
  const double fraction = blockedFractionAhead_(*mgr->occupancy_grid);
  if (fraction < blocked_fraction_) {
    blocked_since_ = -1.0;
  } else if (blocked_since_ < 0.0) {
    blocked_since_ = now;
  } else if (now - blocked_since_ >= blocked_time_) {
    abandoned_ = true;
    RCLCPP_WARN(mgr->logger(), "Abandoning the encircle of \"%s\": %.0f%% of the next %.0f m of the circle has been blocked for %.1f s.",
                contact_name_.c_str(), 100.0 * fraction, blocked_lookahead_, now - blocked_since_);
  }
}

// Fraction of the circle poses within blocked_lookahead_ ahead of the vehicle with an occupied cell within blocked_clearance_.
double Encircle::blockedFractionAhead_(const nav_msgs::msg::OccupancyGrid & grid) const {
  const auto & poses = circle_path_.poses;
  const auto & ego = mgr->odometry.pose.pose.position;
  size_t start = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < poses.size(); ++i) {
    const double d = std::hypot(poses[i].pose.position.x - ego.x, poses[i].pose.position.y - ego.y);
    if (d < best) {
      best = d;
      start = i;
    }
  }
  const double res = grid.info.resolution;
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const int reach = static_cast<int>(std::ceil(blocked_clearance_ / res));
  int checked = 0;
  int blocked = 0;
  double length = 0.0;
  for (size_t i = start; i < poses.size(); ++i) {
    const auto & p = poses[i].pose.position;
    if (i > start) {
      length += std::hypot(p.x - poses[i - 1].pose.position.x, p.y - poses[i - 1].pose.position.y);
      if (length > blocked_lookahead_) {
        break;
      }
    }
    const int col = static_cast<int>(std::floor((p.x - grid.info.origin.position.x) / res));
    const int row = static_cast<int>(std::floor((p.y - grid.info.origin.position.y) / res));
    bool hit = false;
    for (int r = std::max(row - reach, 0); r <= std::min(row + reach, height - 1) && !hit; ++r) {
      for (int c = std::max(col - reach, 0); c <= std::min(col + reach, width - 1) && !hit; ++c) {
        hit = grid.data[static_cast<size_t>(r) * width + c] > 0;
      }
    }
    ++checked;
    blocked += hit ? 1 : 0;
  }
  return checked > 0 ? static_cast<double>(blocked) / checked : 0.0;
}

bool Encircle::is_done() {
  return arrived || abandoned_;
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
