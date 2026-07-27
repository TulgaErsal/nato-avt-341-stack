#pragma once

#include <vector>

namespace avt_341 {
namespace planning {
namespace dwa {

class Cells {
  public:
    Cells();

    void Add(double x, double y, double cost);

    int GetNumberOfObstacles();

    void Clear();

  private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<int> costs_;
};

} // namespace dwa
} // namespace planning
} // namespace avt_341