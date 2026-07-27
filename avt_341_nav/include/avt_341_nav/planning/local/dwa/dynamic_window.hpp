#pragma once

#include <cassert>
#include <limits>

namespace avt_341_nav {
namespace planning {
namespace dwa {

class DynamicWindow {
  public:
    DynamicWindow();

    DynamicWindow(double minimum_speed,
                  double maximum_speed,
                  double minimum_steering_rate,
                  double maximum_steering_rate);

    void Update(double minimum_speed,
                double maximum_speed,
                double minimum_steering_rate,
                double maximum_steering_rate);

    const double& GetMinimumSpeed();

    const double& GetMaximumSpeed();

    const double& GetMinimumSteeringRate();

    const double& GetMaximumSteeringRate();

  private:
    double minimum_speed_;

    double maximum_speed_;

    double minimum_steering_rate_;

    double maximum_steering_rate_;
};

} // namespace dwa
} // namespace planning
} // namespace avt_341_nav