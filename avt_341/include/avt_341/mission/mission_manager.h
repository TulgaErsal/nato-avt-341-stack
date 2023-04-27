
#ifndef AVT_341_MISSION_MGR_H
#define AVT_341_MISSION_MGR_H

/**
* \class MissionManager
*
* Manager for vehicle global path points based on desired formation.
*
* \author Daniel Carruth
*
* \date 1/31/2023
*/

// c++ includes
#include <string>
#include <chrono>
#include <map>
// local includes
#include "avt_341/node/ros_types.h"
#include "avt_341/mission/task.h"

namespace avt_341 {
namespace mission {

class Task;

struct MissionPoint {
    std::string name;
    double pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w;
};

struct Formation {
    avt_341::msg::Point follower1;
    avt_341::msg::Point follower2;
    avt_341::msg::Point follower3;
};

struct Contact {
	// storage for contact information
	// timestamp, position, class/name, investigated
	avt_341::msg::PoseStamped pose;
	std::string name;
	float x, y;
	bool investigated;
	bool investigating;
    bool is_new;
};
    
/// Class for formation control
class MissionManager{

  public:
	/// Construct a formation controller
	MissionManager();
    ~MissionManager();

    int loadMissionDefinition(std::string filename);

    bool getMissionPoint(MissionPoint& mission_point, std::string posename);

	// internal messages
	void handleContacts(avt_341::msg::Path);

	// external messages
    void handleMoveTo(avt_341::msg::Communication);
    void handleFormationRequest(avt_341::msg::Communication);
    void handleAcknowledge(avt_341::msg::Communication);
    void handleArrive(avt_341::msg::Communication);
    void handleTaskComplete(avt_341::msg::Communication);
    void handleHold(avt_341::msg::Communication);
    void handleSetSpeed(avt_341::msg::Communication);

    std::string my_name;
    std::string leader_name;
    bool is_leader;
    float follow_scale;
	float same_object_distance_threshold_sq;
	avt_341::msg::Odometry odometry;
	int previous_nav_state, nav_state; 
    float desired_speed;
    bool goal_changed;
    bool arrival_announced;
	bool busy;	// probably temporary 

    // Task management
    Task* getTask();
    auto getNextTask();
    void updateTasks();
    void postUpdateTasks();
    bool setTask(Task * task);

    // Messages
    bool path_msg_updated;
    avt_341::msg::Path path_msg;

    bool follower_status_msg_updated;
    avt_341::msg::FollowerStatus follower_status_message;

    bool comm_msg_updated;
    avt_341::msg::String comm_msg;

    bool nav_msg_updated;
    avt_341::msg::Int32 nav_msg;

    bool speed_msg_updated;
    avt_341::msg::Float64 speed_msg;

  private:

    std::vector<MissionPoint> mission_data;
    Task * active_task;
    std::vector<Task*> mission_tasks;
    std::vector<Task*> task_list;
    std::vector<Task*> completed_tasks;
    std::map<std::string, Formation> formations;
	std::vector<Contact> mission_contacts;


    // Methods
	auto getContact(std::string name, float x, float y); 
    auto getClosestNewContact(float veh_x, float veh_y);
	void addContact(std::string name, float x, float y);
	bool close(float x, float y, float n_x, float n_y);
    float distance_sq(float x1, float y1, float x2, float y2);
}; // class mission manager

} // namespace mission
} // namespace avt_341


#endif //AVT_341_MISSION_MGR_H
