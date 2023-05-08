// clas definition
#include "avt_341/mission/task.h"
#include <fstream>
#include <iostream>

namespace avt_341 {
namespace mission {

// MoveTo
WaitUntilComplete::WaitUntilComplete(MissionManager * manager, const std::string & sender, int msg_id, const std::string & target_veh, int target_msg_id)
: Task(manager, sender, msg_id), target_veh_(target_veh), target_msg_id_(target_msg_id) {
}

void WaitUntilComplete::init_() {
}

void WaitUntilComplete::run() {
}

bool WaitUntilComplete::is_done() {
    return mgr->hasCompletedTask(target_veh_, target_msg_id_);
}

void WaitUntilComplete::on_done() {
}

std::string WaitUntilComplete::description() const{
  std::ostringstream stream;
  stream << "ID " << msg_id << " WAIT_UNTIL_COMPLETE: " << target_veh_ << " " << target_msg_id_;
  return stream.str();
}

} // mission 
} // avt_341
