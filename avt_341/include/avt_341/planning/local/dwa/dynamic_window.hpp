#pragma once

namespace avt_341 {
namespace planning {
namespace dwa {

class DynamicWindow {
  public:
    DynamicWindow();

    DynamicWindow(double speed_min,
                  double speed_max,
                  double speed_ang_min,
                  double speed_ang_max);

    double speed_min_;
    double speed_max_;
    double speed_ang_min_;
    double speed_ang_max_;
};

} // namespace dwa
} // namespace planning
} // namespace avt_341