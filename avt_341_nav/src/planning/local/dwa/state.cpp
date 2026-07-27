#include <avt_341/planning/local/dwa/state.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace avt_341 {
namespace planning {
namespace dwa {

State::State() {}

State::State(double x, double y, double yaw, double speed, double speed_ang)
    : x_(x),
      y_(y),
      yaw_(yaw),
      speed_(speed),
      speed_ang_(speed_ang) {}

double State::GetX() const { return x_; }

void State::SetX(double x) { x_ = x; }

double State::GetY() const { return y_; }

void State::SetY(double y) { y_ = y; }

double State::GetYaw() { return yaw_; }

void State::SetYaw(double yaw) { yaw_ = yaw; }

double State::GetSpeed() { return speed_; }

void State::SetSpeed(double speed) { speed_ = speed; }

double State::GetAngularSpeed() { return speed_ang_; }

void State::SetAngularSpeed(double speed_ang) { speed_ang_ = speed_ang; }

geometry_msgs::msg::PoseStamped State::ToRosPoseStamped() {
    geometry_msgs::msg::PoseStamped msg_posestamped;

    msg_posestamped.pose.position.x = x_;
    msg_posestamped.pose.position.y = y_;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);
    msg_posestamped.pose.orientation.x = quaternion.x();
    msg_posestamped.pose.orientation.y = quaternion.y();
    msg_posestamped.pose.orientation.z = quaternion.z();
    msg_posestamped.pose.orientation.w = quaternion.w();

    return msg_posestamped;
}

} // namespace dwa
} // namespace planning
} // namespace avt_341