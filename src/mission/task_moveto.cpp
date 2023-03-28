// clas definition
#include <fstream>
#include <sstream>
#include <iostream>

#include "avt_341/mission/task.h"

namespace avt_341 {
namespace mission {

// MoveTo
MoveTo::MoveTo(MissionManager* manager, std::string sender, int id) {
    mgr = manager;
    sender_name = sender;
    msg_id = id;
    next_task = NULL;
    goal_type = 0;
    arrived = false;
    name="TEMP_NAME";
    goal.pose.position.x = 0.0;
    goal.pose.position.y = 0.0;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;
    set_busy = false;
}

bool MoveTo::setGoalByPose(float x, float y, float z, float rot_x, float rot_y, float rot_z, float rot_w) {
    goal_type = POSE;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation.x = rot_x;
    goal.pose.orientation.y = rot_y;
    goal.pose.orientation.z = rot_z;
    goal.pose.orientation.w = rot_w;
    std::ostringstream stream;
    stream << "POSE_" << x << "_" << y;
    name = stream.str();
    return true;
}

bool MoveTo::setGoalByMissionPoint(std::string mp_name) {
    goal_type = MISSION_POINT;
    MissionPoint target;
    bool validPoint = mgr->getMissionPoint(target, mp_name);
    if(validPoint) {
        goal.pose.position.x = target.pos_x;
        goal.pose.position.y = target.pos_y;
        goal.pose.position.z = target.pos_z;
        goal.pose.orientation.x = target.rot_x;
        goal.pose.orientation.y = target.rot_y;
        goal.pose.orientation.z = target.rot_z;
        goal.pose.orientation.w = target.rot_w;
        name = mp_name;
        return true;
    } else {
        std::cout << "Pose " << mp_name << " not found in mission data!" << std::endl;
        return false;
    }
}

void MoveTo::init() {
    std::cout << "Move To Task initialized" << std::endl;

    // send waypoints to global planner
    if(mgr->path_msg_updated == true) {
        std::cout << mgr->my_name << " WARNING Overwriting path_msg" << std::endl;
    }
    mgr->path_msg.poses.clear();
    mgr->path_msg.poses.push_back(goal);
    mgr->path_msg_updated = true;

    // send go command to global planner
    if(mgr->nav_state != 0) {
        if(mgr->nav_msg_updated == true) {
            if(mgr->nav_msg.data != 1) {
                std::cout << mgr->my_name << "WARNING Overwriting nav_msg " << mgr->nav_msg.data << " with 1" << std::endl;
            }
        }
        mgr->nav_msg.data = 1;
        mgr->nav_msg_updated = true;
    }

    if(set_busy) {
        mgr->busy = true;
    }
}

void MoveTo::run() {
    // get current position
    // calculate distance from goal
    if(mgr->previous_nav_state == 0 && mgr->nav_state == 1) 
    {
        // we've reached a goal and are stopping
        // TODO: Check distance b/c we could be stopping for _any_ reason
        std::cout << mgr->my_name << " detected change in nav_state from active to stopping - assuming arriving at goal" << std::endl;
        arrived = true;
    }
}

bool MoveTo::is_done() {
    return arrived;
}

void MoveTo::on_done() {
    std::cout << "Move To Task " << name << " is completed" << std::endl;

    // announce arrival
    std::ostringstream stream;
    stream << "ARRIVE," << name;
    if(mgr->comm_msg_updated) {
        std::cout << "WARNING Overwriting comm_msg " << mgr->comm_msg.data << " with " << stream.str() << std::endl;
    }
    mgr->comm_msg.data = stream.str();
    mgr->comm_msg_updated = true;

    // update contact investigation status
    if(goal_type == CONTACT) {
        if(contact != NULL) {
            contact->investigated = true;
        }
    }

    completed = true;
    mgr->busy = false;
}

} // mission 
} // avt_341
