#pragma once

#include <avt_341/planning/local/dwa/state.hpp>

namespace avt_341 {
namespace planning {
namespace dwa {

class Trajectory {
  public:
    Trajectory();

    void Add(State state);

    int GetNumberOfStates();

    State GetState(int i);

    State GetLastState();

    avt_341::msg::Path ToRosPath();

    void Reset();

    double GetCost();

    void SetCost(double cost);

  private:
    double cost_ = 0.0;
    std::vector<State> states_;
};

} // namespace dwa
} // namespace planning
} // namespace avt_341