// clas definition
#include "avt_341/mission/mission_manager.h"
#include <fstream>

namespace avt_341 {
namespace mission {

MissionManager::MissionManager(){
    
    my_name = "";
    leader_name = "";
	is_leader = true;
	nav_state = -1;

    path_msg_updated = false;
    follower_status_msg_updated = false;
    comm_msg_updated = false;
    arrival_announced = true;
    goal_changed = false;

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

auto MissionManager::getContact(std::string name, float x, float y) {
	auto result = std::find_if(std::begin(mission_contacts), std::end(mission_contacts),
	            [&](const auto& e) {return (e.name == name && close(e.x, e.y, x, y)); });
	/*
	if(result != mission_contacts.end()) {
		std::cout << "Result: " << (*result).name << std::endl;
	} else {
		std::cout << "Not found" << std::endl;
	}
	*/
	return result;

}

void MissionManager::addContact(std::string name, float in_x, float in_y) {
	Contact new_contact;
	new_contact.name = name;
	new_contact.x = in_x;
	new_contact.y = in_y;
	mission_contacts.push_back(new_contact);
}

bool MissionManager::close(float old_x, float old_y, float new_x, float new_y) {
	float distance_sq = ((old_x - new_x)*(old_x - new_x)) + ((old_y - new_y)*(old_y - new_y));
	if(distance_sq < same_object_distance_threshold_sq) {
		return true;
	} else {
		return false;
	}
}

// Message Handlers
void MissionManager::handleContacts(avt_341::msg::Path contacts) {
	// check if contacts are already in the contact database
	for(auto item: contacts.poses) {
		// check if contact is in database 
		auto it = getContact(item.header.frame_id, item.pose.position.x, item.pose.position.y);
		if(it != mission_contacts.end()) {
			// already in the list
			//std::cout << item.header.frame_id << " is in the list." << std::endl;
		} else {
			// new contact
			std::cout << item.header.frame_id << " is a new contact." << std::endl;
			// Add it to the contacts
			addContact(item.header.frame_id, item.pose.position.x, item.pose.position.y);
		}
	}
	// are we already investigating a contact?
	if(!investigating_contact) {
		// is this a target of interest? 
		// if so, order follower to moveto overwatch position 
		// move to a point near the TOI
		for(auto item: mission_contacts) {
			// review our contacts
			if(item.investigating == false && !investigating_contact) {
				// we have not done anything for this contact
				std::cout << " Requesting move to " << item.name << " at " << item.x << ", " << item.y << std::endl;
				handleMoveTo(item.x, item.y);
				item.investigating = true;
				investigating_contact = true;
				// set a subgoal to encircle the TOI
				// set a subgoal to return to the goal
				// ignore the identification subgoal for now
			}
		}
	}
}

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

void MissionManager::handleArrival() {
    if(arrival_announced == false) {
		std::ostringstream stream;
		stream << my_name << "ARRIVE,TEMP_NAME";
        comm_msg.data = stream.str();
        arrival_announced = true;
	} else if(nav_state == 0 && arrival_announced == true) {
        arrival_announced = false;
    }
}

void MissionManager::handleMoveTo(float x, float y) {
	// only applies if I'm leader, 
	if(is_leader) {
		// create Path element (single point)
        std::cout << "Moving to : " << x << ", " << y << std::endl;
		avt_341::msg::PoseStamped pose;
		pose.pose.position.x = x;
		pose.pose.position.y = y;
		pose.pose.position.z = 0;
		pose.pose.orientation.w = 1;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
	
		if(path_msg_updated == true) {
			std::cout << my_name << " Warning: Overwriting path message" << std::endl;
		}
		path_msg.poses.clear();
		path_msg.poses.push_back(pose);
		path_msg_updated = true;
        goal_changed = true;
    }
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
