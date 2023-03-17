// clas definition
#include "avt_341/mission/mission_manager.h"
#include <fstream>

namespace avt_341 {
namespace mission {



MissionManager::MissionManager(){
    
    my_name = "";
    leader_name = "";

    path_msg_updated = false;
    follower_status_msg_updated = false;

    follow_scale = 1.0;

    // initialize formation data
    struct Formation f;     
    f.follower1.x = 0;
    f.follower1.y = -1;
    f.follower2.x = 0;
    f.follower2.y = -2;
    f.follower3.x = 0;
    f.follower3.y = -3;
    formations["LINE"] = f; 
    f.follower1.x = -1;
    f.follower1.y = 0;
    f.follower2.x = -2;
    f.follower2.y = 0;
    f.follower3.x = -3;
    f.follower3.y = 0;
    formations["COLUMN"] = f; 
    f.follower1.x = -1;
    f.follower1.y = 1;
    f.follower2.x = -2;
    f.follower2.y = 0;
    f.follower3.x = -3;
    f.follower3.y = 1;
    formations["STAGGER_COL"] = f; 
    f.follower1.x = -1;
    f.follower1.y = 1;
    f.follower2.x = -1;
    f.follower2.y = -1;
    f.follower3.x = -2;
    f.follower3.y = 0;
    formations["DIAMOND"] = f; 
    f.follower1.x = -1;
    f.follower1.y = 1;
    f.follower2.x = 0;
    f.follower2.y = -1;
    f.follower3.x = -1;
    f.follower3.y = -2;
    formations["WEDGE"] = f; 
    f.follower1.x = -1;
    f.follower1.y = 1;
    f.follower2.x = -2;
    f.follower2.y = 2;
    f.follower3.x = -3;
    f.follower3.y = 3;
    formations["ECH_LEFT"] = f; 
    f.follower1.x = -1;
    f.follower1.y = -1;
    f.follower2.x = -2;
    f.follower2.y = -2;
    f.follower3.x = -3;
    f.follower3.y = -3;
    formations["ECH_RIGHT"] = f; 
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
    auto result = std::find_if(std::begin(mission_data), std::end(mission_data), 
                [&](const auto& e) {return e.name == posename; });
    std::cout << "Result: " << (*result).name << std::endl;
    return *result;
}

// Message Handlers
void MissionManager::handleFormationRequest(avt_341::msg::Communication msg) {
    follower_status_message.leader_name = msg.leader_name;

    // Publish to /avt_341/new_waypoints
    if(follower_status_msg_updated == true) {
        std::cout << my_name << " Warning: Overwriting follower status message" << std::endl;
    }
    if(msg.leader_name == my_name){
        std::cout << my_name << " is taking the lead position." << std::endl;
        follower_status_message.x_offset = 0;
        follower_status_message.y_offset = 0;
        follower_status_message.use_leader = false;
        follower_status_msg_updated = true;
        leader_name = msg.leader_name;
    } else {
        Formation f = formations[msg.formation.c_str()];
        if(msg.follower1_name == my_name) {
            follower_status_message.x_offset = f.follower1.x * follow_scale; 
            follower_status_message.y_offset = f.follower1.y * follow_scale;
        } else if (msg.follower2_name == my_name) {
            follower_status_message.x_offset = f.follower2.x * follow_scale; 
            follower_status_message.y_offset = f.follower2.y * follow_scale;
        } else if (msg.follower3_name == my_name) {
            follower_status_message.x_offset = f.follower3.x * follow_scale;
            follower_status_message.y_offset = f.follower3.y * follow_scale;
        } else {
            std::cout << my_name << " is ignoring the formation request. Not for me." << std::endl;
            return;
        }
        follower_status_message.use_leader = true;
        follower_status_msg_updated = true;
        leader_name = msg.leader_name;
        std::cout << my_name << " setting " << leader_name << " as leader." << std::endl;
    }
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
    if(is_leader) {
        Pose objective = getPose(msg.objective_name);
        std::cout << "Moving to " << objective.name << ": " << objective.pos_x << ", " << objective.pos_y << std::endl;

        // Create Path element (single point)
        avt_341::msg::PoseStamped pose;
        pose.pose.position.x = objective.pos_x;
        pose.pose.position.y = objective.pos_y;
        pose.pose.position.z = objective.pos_z;
        pose.pose.orientation.w = objective.rot_w;
        pose.pose.orientation.x = objective.rot_x;
        pose.pose.orientation.y = objective.rot_y;
        pose.pose.orientation.z = objective.rot_z;

        // Publish to /avt_341/new_waypoints
        if(path_msg_updated == true) {
            std::cout << my_name << " Warning: Overwriting path message" << std::endl;
        }
        path_msg.poses.clear();
        path_msg.poses.push_back(pose);
        path_msg_updated = true;
    } else {
        std::cout << my_name << " Mission Manager: ignoring MoveTo b/c currently following a leader" << std::endl;
    }
}

void MissionManager::handleHold(avt_341::msg::Communication msg) {
    // handle request to wait

}

void MissionManager::handleSetSpeed(avt_341::msg::Communication msg) {
    // handle updated speed

}

} // namespace mission
} // namespace avt_341