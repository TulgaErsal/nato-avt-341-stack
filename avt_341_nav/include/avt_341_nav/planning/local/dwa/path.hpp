#pragma once

#include <cmath>
#include <limits>
#include <vector>

namespace avt_341_nav {
namespace planning {
namespace dwa {

class Path {
  public:
    Path();

    void Add(double x, double y);

    int FindClosestDistance(double x, double y) const;

  private:
    std::vector<double> x_;
    std::vector<double> y_;
};

} // namespace dwa
} // namespace planning
} // namespace avt_341_nav