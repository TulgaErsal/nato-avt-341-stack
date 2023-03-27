// clas definition
#include "avt_341/mission/task.h"
#include <fstream>

namespace avt_341 {
namespace mission {

// Follow
Follow::Follow(MissionManager* manager) {
    mgr = manager;
    next_task = NULL;
    set_busy = false;
    completed = false;
}

void Follow::init() {
    std::cout << mgr->my_name << " setting " << mgr->leader_name << " as leader." << std::endl;

    // send go command to global planner
    if(mgr->nav_state != 0) {
        if(mgr->nav_msg_updated == true) {
            if(mgr->nav_msg.data != 1) {
                std::cout << mgr->my_name << "WARNING Overwriting nav_msg " << mgr->nav_msg.data << " with 1" << std::endl;
            }
        }
        mgr->nav_msg.data = 1;
        mgr->nav_msg_updated = true;
    }

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

} // mission 
} // avt_341