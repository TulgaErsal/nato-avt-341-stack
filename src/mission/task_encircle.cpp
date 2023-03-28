// clas definition
#include "avt_341/mission/task.h"
#include <fstream>
#include <iostream>

namespace avt_341 {
namespace mission {

// Encircle
Encircle::Encircle(MissionManager* manager, std::string sender, int id) {
    mgr = manager;
    sender_name = sender;
    msg_id = id;
    next_task = NULL;
    set_busy = false;
    completed = false;
}

void Encircle::init() {
    std::cout << "Encircle Task initialized" << std::endl;
}

void Encircle::run() {
    std::cout << "Encircle Task is running" << std::endl;
}

bool Encircle::is_done() {
    return false;
}

void Encircle::on_done() {
    std::cout << "Encircle Task is completed" << std::endl;
}

} // mission 
} // avt_341
