// clas definition
#include "avt_341/mission/task.h"
#include <fstream>
#include <iostream>

namespace avt_341 {
namespace mission {

// Follow
Follow::Follow(MissionManager* manager, std::string sender, int id) {
    mgr = manager;
    sender_name = sender;
    msg_id = id;
    next_task = NULL;
    set_busy = false;
    completed = false;
}

void Follow::init() {
    mgr->publishNavStateCmd(avt_341::utils::NavStateCmd::GoActive);

    if(set_busy) {
        mgr->busy = true;
    }
}

void Follow::run() {
    //std::cout << mgr->my_name << " Follow Task is running" << std::endl;
}

bool Follow::is_done() {
    return false;
}

void Follow::on_done() {
    std::cout << mgr->my_name << " Follow Task is completed" << std::endl;
}

std::string Follow::description() const {
    std::ostringstream stream;
    stream << "TASK-FOLLOW leader " << mgr->formation_def.followedVehicle();
    return stream.str();
}

} // mission 
} // avt_341
