// clas definition
#include "avt_341/mission/task.h"

namespace avt_341 {
namespace mission {

Task::Task(MissionManager* manager, const std::string & sender, int id, FormationDefinition* formation_def)
: mgr(manager), sender_name(sender), msg_id(id), init_done(false), formation_def_(formation_def) {

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



} // mission 
} // avt_341
