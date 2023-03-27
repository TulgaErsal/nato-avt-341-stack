// clas definition
#include "avt_341/mission/task.h"
#include <fstream>

namespace avt_341 {
namespace mission {

// MoveTo
WaitUntil::WaitUntil(MissionManager * manager, std::string sender, int id) {
    mgr = manager;
    sender_name = sender;
    msg_id = id;
    next_task = NULL;
    set_busy = false;
    completed = false;
}

void WaitUntil::init() {
    std::cout << "Wait Until Task initialized" << std::endl;
}

void WaitUntil::run() {
    std::cout << "Wait Until Task is running" << std::endl;
}

bool WaitUntil::is_done() {
    return false;
}

void WaitUntil::on_done() {
    std::cout << "Wait Until Task is completed" << std::endl;
}

} // mission 
} // avt_341