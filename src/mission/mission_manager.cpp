// clas definition
#include "avt_341/mission/mission_manager.h"
#include <fstream>

namespace avt_341 {
namespace mission {



MissionManager::MissionManager(){
    
    
}

std::vector<std::string> getLine(std::istream& stream) {
    std::vector<std::string> result;
    std::string line;
    std::getline(stream, line);
    std::stringstream lineStream(line);
    std::string cell;

    while(std::getline(lineStream, cell, ',')) {
        result.push_back(cell);
    }
    return result;
}

int MissionManager::loadMissionDefinition(std::string filename) {
    std::string line;
    std::vector<std::string> contents;
    Pose position;

    //std::cout << "Loading mission " << filename << std::endl;

    // Load the mission from file
    std::ifstream infile(filename);
    if(infile.is_open()) {
        mission_data.clear();
        while(std::getline(infile, line))
        {
            std::istringstream iss(line);
            //std::cout << line << std::endl;
            contents = getLine(iss);
            if(!contents.empty() && strcmp(contents[0].c_str(),"name")) {
                position.name = contents[0];
                position.pos_x = std::strtod(contents[1].c_str(), NULL);
                position.pos_y = std::strtod(contents[2].c_str(), NULL);
                position.pos_z = std::strtod(contents[3].c_str(), NULL);
                position.rot_x = std::strtod(contents[4].c_str(), NULL);
                position.rot_y = std::strtod(contents[5].c_str(), NULL);
                position.rot_z = std::strtod(contents[6].c_str(), NULL);
                position.rot_w = std::strtod(contents[7].c_str(), NULL);
                mission_data.push_back(position);
                //std::cout << "Pose: " << position.name << " " << position.pos_x << " " << position.rot_w << std::endl;
            } else {
                std::cout << "Empty" << std::endl;
            }
        }    
    } else {
        std::cout << "Error reading mission definition " << filename << std::endl;
    }
}

Pose MissionManager::getPose(std::string posename) {
    Pose pose;
    auto result = std::find_if(std::begin(mission_data), std::end(mission_data), 
                [&](const auto& e) {std::cout << "name" << e.name << ":" << posename << std::endl; return e.name == posename; });
    std::cout << "Result: " << (*result).name << std::endl;
    return *result;
}

// Message Handlers
void MissionManager::handleFormationRequest(avt_341::msg::Communication msg) {

}

void MissionManager::handleAcknowledge(avt_341::msg::Communication msg) {

}

void MissionManager::handleArrive(avt_341::msg::Communication msg) {
    // If tracking, update mission tracker

}

void MissionManager::handleTaskComplete(avt_341::msg::Communication msg) {
    // If tracking, mark complete


}

void MissionManager::handleMoveTo(avt_341::msg::Communication msg) {
    // only applies if I'm the leader, otherwise decline
    Pose objective = getPose(msg.objective_name);
    std::cout << "Moving to " << objective.name << ": " << objective.pos_x << ", " << objective.pos_y << std::endl;

}

void MissionManager::handleHold(avt_341::msg::Communication msg) {
    // handle request to wait

}

void MissionManager::handleSetSpeed(avt_341::msg::Communication msg) {
    // handle updated speed

}

} // namespace mission
} // namespace avt_341