/**
* \class Task
*
* Class for keeping track of tasks assigned to the vehicle
* Based on Naisense ScenarioRunner
*
* \author Daniel Carruth
*
* \date 1/31/2023
*/

// c++ includes
#include <string>
// local includes
#include "avt_341/node/ros_types.h"

namespace avt_341 {
namespace mission {

class Task {
public:
    void init() {
        std::cout << "Task initialized" << std::endl;
    }
    void run() {
        std::cout << "Task is running" << std::endl;
    }
    void update() {
        std::cout << "Task updated" << std::endl;
    }
    void on_done() {
        std::cout << "Task completed" << std::endl;
    }
}; // class Task

class MoveToPositionTask : public Task {
public:
    void init() {
        std::cout << "MoveToPositionTask initialized" << std::endl;
    }
    void run() {
        std::cout << "MoveToPositionTask is running" << std::endl;
    }
    void update() {
        std::cout << "MoveToPositionTask updated" << std::endl;
    }
    void on_done() {
        std::cout << "MoveToPositionTask completed" << std::endl;
    }
}; // class MoveToPositionTask

} // namespace mission
} // namespace avt_341