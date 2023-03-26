// clas definition
#include "avt_341/mission/task.h"
#include <fstream>

namespace avt_341 {
namespace mission {

// Follow
Follow::Follow() {

}

void Follow::init() {
    std::cout << "Follow Task initialized" << std::endl;
}

void Follow::run() {
    std::cout << "Follow Task is running" << std::endl;
}

bool Follow::is_done() {
    std::cout << "Follow Task is done" << std::endl;
    return true;
}

void Follow::on_done() {
    std::cout << "Follow Task is completed" << std::endl;
}

} // mission 
} // avt_341