// clas definition
#include "avt_341/mission/task.h"
#include <fstream>
#include <iostream>

namespace avt_341 {
namespace mission {

// Follow
Follow::Follow(MissionManager* manager, std::string sender, int id, FormationDefinition* formation_def)
: Task(manager, sender, id, formation_def) {
}

void Follow::init_() {
    mgr->publishNavStateCmd(avt_341::utils::NavStateCmd::GoActive);

    if(!formation_def_->formationAtGoal()){
      mgr->publishFormationStatus(formation_def_->formation_status);
    }
}

void Follow::run() {
    //std::cout << mgr->my_name << " Follow Task is running" << std::endl;
}

void Follow::preempted(){
  init_done = false;
  mgr->publishNavStateCmd(avt_341::utils::NavStateCmd::GoInactive);
}

bool Follow::is_done() {
    return false;
}

void Follow::on_done() {
    std::cout << mgr->my_name << " Follow Task is completed" << std::endl;
}

std::string Follow::description() const {
    std::ostringstream stream;
    stream << "TASK-FOLLOW leader " << formation_def_->followedVehicle();
    return stream.str();
}

} // mission 
} // avt_341
