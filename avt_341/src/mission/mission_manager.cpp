// clas definition
#include "avt_341/mission/mission_manager.h"
#include <fstream>
#include <iostream>
#include <sstream>

namespace avt_341 {
namespace mission {

MissionManager::MissionManager(FormationDefinition & formation_definition, std::shared_ptr<node::NodeProxy> node_proxy)
: formation_def(formation_definition), node_proxy_(node_proxy){

    my_name = formation_def.my_name;
    nav_state = avt_341::utils::NavStackState::NotInit;
    previous_nav_state = avt_341::utils::NavStackState::NotInit;

    arrival_announced = true;

    active_task = NULL;
    mission_tasks.clear();
    mission_data.clear();
    task_list.clear();
    completed_tasks.clear();
    mission_contacts.clear();

    waypoint_pub = node_proxy_->create_publisher<avt_341::msg::Path>("avt_341/new_waypoints", 10);
    follower_status_pub = node_proxy_->create_publisher<avt_341::msg::FollowerStatus>("avt_341/follower_status",10);
    navcommand_pub = node_proxy_->create_publisher<avt_341::msg::Int32>("avt_341/nav_command_state", 10);
    communication_pub = node_proxy_->create_publisher<avt_341::msg::String>("avt_341/comm_messages", 100);
    speed_pub = node_proxy_->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed", 10);
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
    return 0;
}

bool MissionManager::getMissionPoint(MissionPoint& mission_point, std::string name) {
    auto it = std::find_if(std::begin(mission_data), std::end(mission_data), 
                [&](const auto& e) {return e.name == name; });
    if(it == mission_data.end()) {
        node_proxy_->log_info("Missing Mission Point %s", name.c_str());
        return false;
    }
    node_proxy_->log_info("Found Mission Point %s = (%.2f, %.2f, %.2f) found", name.c_str(), it->pos_x, it->pos_y, it->pos_z);
    mission_point = *it;
    return true;
}

Task* MissionManager::getTask() {
    return active_task;
}

bool MissionManager::addTask(Task* task) {
    // if no active task, put it on the list
    task_list.push_back(task);
    node_proxy_->log_info("QUEUED %s", task->description().c_str());
    return setActiveTask(task);
}

bool MissionManager::setActiveTask(Task* task){
    if(busy && active_task != nullptr && !active_task->completed){
        node_proxy_->log_info("SKIPPED SINCE BUSY %s", task->description().c_str());
        return false;
    }
    active_task = task;
    if(active_task == nullptr) {
        node_proxy_->log_info("NO ACTIVE TASK AVAILABLE");
        return false;
    }
    node_proxy_->log_info(" > EXECUTING %s", task->description().c_str());
    active_task = task;
    active_task->init();
    return true;
}

Task* MissionManager::getNextTask() {
    auto it = std::find_if(std::begin(task_list), std::end(task_list),
                           [&](const auto& e) {return (e->completed == false); });
    return it != task_list.end() ? *it : nullptr;
}

void MissionManager::publishPath(avt_341::msg::Path& path){
    path.header.stamp = node_proxy_->get_stamp();
    path.header.frame_id = "map";
    waypoint_pub->publish(path);
}

void MissionManager::publishNavStateCmd(int state){
    avt_341::msg::Int32 nav_msg;
    nav_msg.data = state;
    navcommand_pub->publish(nav_msg);
}

void MissionManager::publishCommStr(const std::string & msg_data){
    avt_341::msg::String msg;
    msg.data = msg_data;
    communication_pub->publish(msg);
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
            if(active_task->next_task != nullptr) {
                setActiveTask(active_task->next_task);
            } else {
                // send COMPLETE msg
                std::ostringstream stream;
                stream << "TASK_COMPLETE," << active_task->sender_name << "," << active_task->msg_id;
                avt_341::msg::String comm_msg;
                comm_msg.data = stream.str();
                communication_pub->publish(comm_msg);

                // get the next list off the task_list
                Task* next_task = getNextTask();
                setActiveTask(next_task);
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
        if(!it->investigating) {
            std::cout << " Requesting move to " << contact.name << " at " << contact.x << ", " << contact.y << std::endl;
            //handleMoveTo(contact.x, contact.y);
            MoveTo* investigateTask = new MoveTo(this, my_name, -1);
            bool ret = investigateTask->setGoalByPose(contact.x, contact.y, 0.0, 1.0, 0.0, 0.0, 0.0);
            investigateTask->set_busy = true;   // set as a priority, uninterruptable task
            investigateTask->contact = &(*it);
            investigateTask->contact->investigating = true;
            investigateTask->goal_type = MoveTo::CONTACT;
            addTask(investigateTask);
            
            //investigateTask->nextTask = new Encircle();
            // set a subgoal to encircle the TOI
            // set a subgoal to return to the goal
            // ignore the identification subgoal for now
            // follower should go to OVERWATCH position
        }
    }
}

void MissionManager::handleFormationRequest(avt_341::msg::Communication msg) {

    auto formation_status = formation_def.update(msg);

    if(formation_def.isLeader()){
        // handle objective
        msg.receiver_name = my_name;
        handleMoveTo(msg);
    } else if(formation_status.use_leader) {
        Follow* followTask = new Follow(this, msg.sender_name, msg.msg_id);
        addTask(followTask);
    }

    follower_status_pub->publish(formation_status);
    // handle set speed
    handleSetSpeed(msg);
}

void MissionManager::handleAcknowledge(const avt_341::msg::Communication & msg) {
    // <sender>,<msg_id>,ACK,<orig_msg_sender>,<orig_msg_id>
    if(msg.original_sender == my_name) {
        std::cout << my_name << ": " << msg.sender_name << " acknowledged my msg #" << msg.original_msg_id << std::endl;
    }
}

// <sender>,<msg_id>,ARRIVE,<objective>
void MissionManager::handleArrive(const avt_341::msg::Communication & msg) {
    // If tracking, update mission tracker
    
}

// <sender>,<msg_id>,TASK_COMPLETE,<orig_msg_sender>,<orig_msg_id>
void MissionManager::handleTaskComplete(const avt_341::msg::Communication & msg) {
    // If tracking, mark complete
    if(msg.original_sender == my_name) {
        std::cout << my_name << ": " << msg.sender_name << " has completed the assigned task from my msg #" << msg.original_msg_id << std::endl;
    }
}

void MissionManager::handleMoveTo(const avt_341::msg::Communication & msg) {
    // only applies if I'm the leader, otherwise decline
    if(msg.receiver_name == my_name) {
        if(formation_def.isLeader()) {
            MoveTo* moveTask = new MoveTo(this, msg.sender_name, msg.msg_id);
            bool ret = moveTask->setGoalByMissionPoint(msg.objective_name);
            addTask(moveTask);
        } else {
            node_proxy_->log_info("Ignoring MoveTo b/c currently following a leader");
        }
    } else {
        node_proxy_->log_info("Ignoring MoveTo (not for me)");
    }
}

void MissionManager::handleHold(const avt_341::msg::Communication & msg) {
    // handle request to wait

}

void MissionManager::handleSetSpeed(const avt_341::msg::Communication & msg) {
    // handle updated speed
    desired_speed = std::stof(msg.desired_speed);
    node_proxy_->log_info("Setting desired speed to %.2f", desired_speed);
    avt_341::msg::Float64 speed_msg;
    speed_msg.data = desired_speed;
    speed_pub->publish(speed_msg);
}

} // namespace mission
} // namespace avt_341
