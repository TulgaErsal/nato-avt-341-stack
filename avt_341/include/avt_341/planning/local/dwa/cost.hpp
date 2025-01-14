#pragma once

#include <limits>
#include <vector>

namespace avt_341 {
namespace planning {
namespace dwa {

class Cost {
  public:
    Cost() {}

    Cost(int i) {
        cost_norm_.resize(i);
        cost_.resize(i);
    }

    void Add(int i, double cost) { cost_[i] = cost; }

    double GetCost(int i) const { return cost_[i]; }

  private:
    std::vector<double> cost_;
    std::vector<double> cost_norm_;
    double min_ = -std::numeric_limits<double>::quiet_NaN();
    double max_ = std::numeric_limits<double>::quiet_NaN();
};

} // namespace dwa
} // namespace planning
} // namespace avt_341