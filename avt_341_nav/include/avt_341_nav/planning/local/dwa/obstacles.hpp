#pragma once

#include <cmath>
#include <vector>

namespace avt_341_nav {
namespace planning {
namespace dwa {

class Obstacles {
  public:
    Obstacles();

    void Add(double x, double y);

    size_t GetNumberOfObstacles() const;

    double GetDistance(int i, double x, double y) const;

    void Clear();

  private:
    std::vector<double> x_;
    std::vector<double> y_;
};

} // namespace dwa
} // namespace planning
} // namespace avt_341_nav
