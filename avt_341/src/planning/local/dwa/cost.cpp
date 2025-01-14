#pragma once

#include <avt_341/planning/local/dwa/cost.hpp>

namespace avt_341 {
namespace planning {
namespace dwa {

Cost::Cost() {}

Cost::Cost(int i) {
    cost_norm_.resize(i);
    cost_.resize(i);
}

void Cost::Add(int i, double cost) { cost_[i] = cost; }

double Cost::GetCost(int i) const { return cost_[i]; }

} // namespace dwa
} // namespace planning
} // namespace avt_341