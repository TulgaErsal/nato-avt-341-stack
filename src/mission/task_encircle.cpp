// clas definition
#include "avt_341/mission/task.h"
#include <fstream>

namespace avt_341 {
namespace mission {

// Encircle
Encircle::Encircle() {

}

void Encircle::init() {
    std::cout << "Encircle Task initialized" << std::endl;
}

void Encircle::run() {
    std::cout << "Encircle Task is running" << std::endl;
}

bool Encircle::is_done() {
    std::cout << "Encircle Task is complete" << std::endl;
    return true;
}

void Encircle::on_done() {
    std::cout << "Encircle Task is completed" << std::endl;
}

} // mission 
} // avt_341