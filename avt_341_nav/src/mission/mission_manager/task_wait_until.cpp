// clas definition
#include "avt_341_nav/mission/task.h"
#include <avt_341_nav/core/dto_conversion.h>
#include <cmath>
#include <fstream>
#include <iostream>

namespace avt_341_nav {
namespace mission {

// MoveTo
WaitUntilComplete::WaitUntilComplete(MissionManager * manager, const std::string & sender, int msg_id, const std::string & target_veh, int target_msg_id)
: Task(manager, sender, msg_id), target_veh_(target_veh), target_msg_id_(target_msg_id),
  completions_before_(manager->completionCount()) {
}

void WaitUntilComplete::init_() {
    // Hold at the current position: a preempted task leaves the planner driving to its goal.
    if(!mgr->odometry.header.frame_id.empty()) {
        mgr->publishGoal(core::ToNavGoal(terminalPose(), -1.0, 2.0 * M_PI));
    }
}

void WaitUntilComplete::run() {
}

bool WaitUntilComplete::is_done() {
    return mgr->hasCompletedTask(target_veh_, target_msg_id_, completions_before_);
}

void WaitUntilComplete::on_done() {
}

std::string WaitUntilComplete::description() const{
  std::ostringstream stream;
  stream << "ID " << msg_id << " WAIT_UNTIL_COMPLETE: " << target_veh_ << " " << target_msg_id_;
  return stream.str();
}

} // mission 
} // avt_341_nav
