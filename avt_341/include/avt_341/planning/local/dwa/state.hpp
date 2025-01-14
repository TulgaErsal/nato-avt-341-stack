#pragma once

#include <avt_341/node/ros_types.h>

namespace avt_341 {
namespace planning {
namespace dwa {

class State {
  public:
    State();

    State(double x, double y, double yaw, double speed, double speed_ang);

    double GetX() const;

    void SetX(double x);

    double GetY() const;

    void SetY(double y);

    double GetYaw();

    void SetYaw(double yaw);

    double GetSpeed();

    void SetSpeed(double speed);

    double GetAngularSpeed();

    void SetAngularSpeed(double speed_ang);

    avt_341::msg::PoseStamped ToRosPoseStamped();

  private:
    double x_;
    double y_;
    double yaw_;
    double speed_;
    double speed_ang_;
};

} // namespace dwa
} // namespace planning
} // namespace avt_341