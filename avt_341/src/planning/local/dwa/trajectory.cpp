#include <avt_341/planning/local/dwa/trajectory.hpp>

namespace avt_341 {
namespace planning {
namespace dwa {

Trajectory::Trajectory() {}

void Trajectory::Add(State state) { states_.push_back(state); }

int Trajectory::GetNumberOfStates() { return (int)states_.size(); }

State Trajectory::GetState(int i) { return states_[i]; }

State Trajectory::GetLastState() { return states_.back(); }

avt_341::msg::Path Trajectory::ToRosPath() {
    avt_341::msg::Path msg_path;

    // Fill the poses array with all states in the trajectory.
    for(State& state : states_) {
        msg_path.poses.push_back(state.ToRosPoseStamped());
    }

    return msg_path;
}

void Trajectory::Reset() { states_.clear(); }

double Trajectory::GetCost() { return cost_; }

void Trajectory::SetCost(double cost) { cost_ = cost; }

} // namespace dwa
} // namespace planning
} // namespace avt_341