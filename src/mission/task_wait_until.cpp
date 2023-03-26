// clas definition
#include "avt_341/mission/task.h"
#include <fstream>

namespace avt_341 {
namespace mission {

// MoveTo
WaitUntil::WaitUntil() {

}

void WaitUntil::init() {
    std::cout << "Wait Until Task initialized" << std::endl;
}

void WaitUntil::run() {
    std::cout << "Wait Until Task is running" << std::endl;
}

bool WaitUntil::is_done() {
    std::cout << "Wait Until Task is complete" << std::endl;
    return true;
}

void WaitUntil::on_done() {
    std::cout << "Wait Until Task is completed" << std::endl;
}

} // mission 
} // avt_341