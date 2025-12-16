// clas definition
#include "avt_341/mission/task.h"

namespace avt_341 {
namespace mission {

Task::Task(MissionManager* manager, const std::string & sender, int id, FormationDefinition* formation_def)
: sender_name(sender), msg_id(id), init_done(false), is_preemptable(true), mgr(manager), formation_def_(formation_def) {

}

Task::~Task() {
  delete formation_def_;
}

bool Task::hasFormation() const{
  return formation_def_ != nullptr && formation_def_->has_formation();
}

void Task::init(){
  if(!init_done){
    init_();
    init_done = true;
  }
}

avt_341::msg::PoseStamped Task::terminalPose() const{
  auto pose = avt_341::msg::PoseStamped();
  pose.pose = mgr->odometry.pose.pose;
  pose.header.stamp = mgr->odometry.header.stamp;
  pose.header.frame_id = mgr->odometry.header.frame_id;
  return pose;
}


} // mission 
} // avt_341
