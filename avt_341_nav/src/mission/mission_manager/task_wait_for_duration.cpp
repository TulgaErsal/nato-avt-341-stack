// clas definition
#include "avt_341_nav/mission/task.h"

namespace avt_341_nav {
namespace mission {

WaitForDuration::WaitForDuration(MissionManager * manager, const std::string & sender, int msg_id, double duration_s)
: Task(manager, sender, msg_id), duration_s_(duration_s), start_time_s_(0.0) {
}

void WaitForDuration::init_() {
    start_time_s_ = mgr->nowSeconds();
}

void WaitForDuration::run() {
}

bool WaitForDuration::is_done() {
    return mgr->nowSeconds() - start_time_s_ >= duration_s_;
}

void WaitForDuration::on_done() {
}

std::string WaitForDuration::description() const{
  std::ostringstream stream;
  stream << "ID " << msg_id << " WAIT_FOR_DURATION: " << duration_s_ << " s";
  return stream.str();
}

} // mission
} // avt_341_nav
