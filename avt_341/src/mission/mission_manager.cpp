// clas definition
#include "avt_341/mission/mission_manager.h"
#include <fstream>
#include <iostream>

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
    speed_msg_updated = false;
    nav_msg_updated = false;
    
    arrival_announced = true;

    follow_scale = 1.0;

    active_task = NULL;
    mission_tasks.clear();
    mission_data.clear();
    task_list.clear();
    completed_tasks.clear();
    mission_contacts.clear();

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

MissionManager::~MissionManager() {
    // Clean up the memory allocated to tasks
    for(auto task : task_list) {
        delete task;
    }
    for(auto task: completed_tasks) {
        delete task;
    }
    task_list.clear();
    completed_tasks.clear();
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
    MissionPoint missionPt;

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
                missionPt.name = contents[0];
                missionPt.pos_x = std::strtod(contents[1].c_str(), NULL);
                missionPt.pos_y = std::strtod(contents[2].c_str(), NULL);
                missionPt.pos_z = std::strtod(contents[3].c_str(), NULL);
                missionPt.rot_x = std::strtod(contents[4].c_str(), NULL);
                missionPt.rot_y = std::strtod(contents[5].c_str(), NULL);
                missionPt.rot_z = std::strtod(contents[6].c_str(), NULL);
                missionPt.rot_w = std::strtod(contents[7].c_str(), NULL);
                mission_data.push_back(missionPt);
                //std::cout << "Pose: " << position.name << " " << position.pos_x << " " << position.rot_w << std::endl;
            } else {
                std::cout << "Empty" << std::endl;
            }
        }    
    } else {
        std::cout << "Error reading mission definition " << filename << std::endl;
    }
}

bool MissionManager::getMissionPoint(MissionPoint& mission_point, std::string name) {
    auto it = std::find_if(std::begin(mission_data), std::end(mission_data), 
                [&](const auto& e) {return e.name == name; });
    if(it == mission_data.end()) {
        std::cout << "MissionPoint " << name << " not found in Mission Data" << std::endl;
        return false;
    } 
    std::cout << "MissionPoint " << name << " found in Mission Data" << std::endl;
    mission_point = *it;
    return true;
}

Task* MissionManager::getTask() {
    return active_task;
}

bool MissionManager::setTask(Task* task) {
    // if no active task, put it on the list
    task_list.push_back(task); 
    if(active_task == NULL || !busy) {
        std::cout << "Added task to task_list. Making new task active." << std::endl;
        active_task = task;
        active_task->init();
    } else {
        std::cout << "Added task to task_list. Current task has priority. New task not active." << std::endl;
    }
    return true;
}

auto MissionManager::getNextTask() {
	auto it = std::find_if(std::begin(task_list), std::end(task_list),
	            [&](const auto& e) {return (e->completed == false); });
	return it;
}

void MissionManager::updateTasks() {
    if(active_task != NULL) {
        if(!active_task->completed) {
            if(!active_task->is_done()) {
                active_task->run();
                if(active_task->is_done()) {
                    active_task->on_done();
                }
            }
        } else {
            completed_tasks.push_back(active_task); 
            if(active_task->next_task != NULL) {
                std::cout << "Moving to next task in the sequence" << std::endl;
                active_task = active_task->next_task; 
                active_task->init();
            } else {
                // send COMPLETE msg
                std::ostringstream stream;
                stream << "TASK_COMPLETE," << active_task->sender_name << "," << active_task->msg_id;
                if(comm_msg_updated) {
                    std::cout << "WARNING Overwriting comm_msg " << comm_msg.data << " with " << stream.str() << std::endl;
                }
                comm_msg.data = stream.str();
                comm_msg_updated = true;

                // get the next list off the task_list
                auto it = getNextTask();        // TODO: Need to consider if an old task has expired 
                if(it != task_list.end()) {
                    std::cout << "Grabbing the next task off the queue" << std::endl;
                    active_task = *it;
                    active_task->init();    // TODO: we need to init, may need a resume 
                } else {
                    //std::cout << "No tasks on the queue. Going idle." << std::endl;
                    active_task = NULL;
                }
            }
        }
    }
}

void MissionManager::postUpdateTasks() {
    previous_nav_state = nav_state;
}

// Contact Management
auto MissionManager::getContact(std::string name, float x, float y) {
	auto it = std::find_if(std::begin(mission_contacts), std::end(mission_contacts),
	            [&](const auto& e) {return (e.name == name && close(e.x, e.y, x, y)); });
	return it;
}

auto MissionManager::getClosestNewContact(float veh_x, float veh_y) {
    auto it = std::min_element(mission_contacts.begin(), mission_contacts.end(), [&](const Contact& a, const Contact& b) {
        if((a.investigating || a.investigated) && (b.investigating || b.investigated)) {
            return false;
        }
        if(a.investigating || a.investigated) {
            return false; // Contact b is closer
        }
        if(b.investigating || b.investigated) {
            return true; // Contact a is closer
        }
        double da = distance_sq(veh_x, veh_y, a.x, a.y);
        double db = distance_sq(veh_x, veh_y, b.x, b.y);
        return da < db; 
    });
    return it;
}

void MissionManager::addContact(std::string name, float in_x, float in_y) {
	Contact new_contact;
	new_contact.name = name;
	new_contact.x = in_x;
	new_contact.y = in_y;
    new_contact.investigating = false;
    new_contact.investigated = false;
    new_contact.is_new = true;
	mission_contacts.push_back(new_contact);
}

float MissionManager::distance_sq(float x1, float y1, float x2, float y2) {
    return ((x1 - x2)*(x1 - x2)) + ((y1 - y2)*(y1 - y2));
}

bool MissionManager::close(float old_x, float old_y, float new_x, float new_y) {
	if(distance_sq(old_x, old_y, new_x, new_y) < same_object_distance_threshold_sq) {
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
			//std::cout << item.header.frame_id << " is in the list. Investigating: " << it->investigating << ", " << it->investigated << std::endl;
		} else {
			// new contact
			std::cout << "New contact: " << item.header.frame_id << " at " << item.pose.position.x << ", " << item.pose.position.y << std::endl;
			// Add it to the contacts
			addContact(item.header.frame_id, item.pose.position.x, item.pose.position.y);
		}
	}
	
    // TODO: need to check if this is a target of interest, currently assuming contacts are
    // move to a point near the TOI
    auto it = getClosestNewContact(odometry.pose.pose.position.x, odometry.pose.pose.position.y); 
    if(it != mission_contacts.end()) {
        // found a new contact
        Contact contact = *it;
        if(it->investigating == false) {
            std::cout << " Requesting move to " << contact.name << " at " << contact.x << ", " << contact.y << std::endl;
            //handleMoveTo(contact.x, contact.y);
            MoveTo* investigateTask = new MoveTo(this, my_name, -1);
            bool ret = investigateTask->setGoalByPose(contact.x, contact.y, 0.0, 1.0, 0.0, 0.0, 0.0);
            investigateTask->set_busy = true;   // set as a priority, uninterruptable task
            investigateTask->contact = &(*it);
            investigateTask->contact->investigating = true;
            investigateTask->goal_type = MoveTo::CONTACT;
            setTask(investigateTask);
            
            //investigateTask->nextTask = new Encircle();
            // set a subgoal to encircle the TOI
            // set a subgoal to return to the goal
            // ignore the identification subgoal for now
            // follower should go to OVERWATCH position
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

        // handle objective
        msg.receiver_name = my_name;
        handleMoveTo(msg);

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

        Follow* followTask = new Follow(this, msg.sender_name, msg.msg_id);
        setTask(followTask);
    }
    
    // handle set speed
    handleSetSpeed(msg); 
}

void MissionManager::handleAcknowledge(avt_341::msg::Communication msg) {
    // <sender>,<msg_id>,ACK,<orig_msg_sender>,<orig_msg_id>
    if(msg.original_sender == my_name) {
        std::cout << my_name << ": " << msg.sender_name << " acknowledged my msg #" << msg.original_msg_id << std::endl;
    }
}

// <sender>,<msg_id>,ARRIVE,<objective>
void MissionManager::handleArrive(avt_341::msg::Communication msg) {
    // If tracking, update mission tracker
    
}

// <sender>,<msg_id>,TASK_COMPLETE,<orig_msg_sender>,<orig_msg_id>
void MissionManager::handleTaskComplete(avt_341::msg::Communication msg) {
    // If tracking, mark complete
    if(msg.original_sender == my_name) {
        std::cout << my_name << ": " << msg.sender_name << " has completed the assigned task from my msg #" << msg.original_msg_id << std::endl;
    }
}

void MissionManager::handleMoveTo(avt_341::msg::Communication msg) {
    // only applies if I'm the leader, otherwise decline
    if(msg.receiver_name == my_name) {
        if(is_leader) {
            MoveTo* moveTask = new MoveTo(this, msg.sender_name, msg.msg_id);
            bool ret = moveTask->setGoalByMissionPoint(msg.objective_name);
            setTask(moveTask);
        } else {
            std::cout << my_name << " Mission Manager: ignoring MoveTo b/c currently following a leader" << std::endl;
        }
    } else {
        std::cout << my_name << " Mission Manager: ignoring MoveTo (not for me)." << std::endl;
    }
}

void MissionManager::handleHold(avt_341::msg::Communication msg) {
    // handle request to wait

}

void MissionManager::handleSetSpeed(avt_341::msg::Communication msg) {
    // handle updated speed
    std::cout << "Setting desired speed to " << msg.desired_speed << std::endl;
    if(speed_msg_updated == true) {
        std::cout << my_name << " Warning: Overwriting desired speed message" << std::endl;
    }
    desired_speed = std::stof(msg.desired_speed);
    speed_msg.data = desired_speed;
    speed_msg_updated = true;
}

} // namespace mission
} // namespace avt_341
