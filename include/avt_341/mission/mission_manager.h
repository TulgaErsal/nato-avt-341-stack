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
// local includes
#include "avt_341/node/ros_types.h"

namespace avt_341 {
namespace mission {

struct Pose {
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
	ros::Time timestamp;
	std::string name;
	float x, y;
	bool investigated;
	bool investigating;
};
    
/// Class for formation control
class MissionManager{

  public:
	/// Construct a formation controller
	MissionManager();

    int loadMissionDefinition(std::string filename);

    Pose getPose(std::string posename);

	// internal messages
	void handleContacts(avt_341::msg::Path);
    void handleArrival();

	// external messages
    void handleMoveTo(avt_341::msg::Communication);
	void handleMoveTo(float x, float y);
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
	int nav_state; 
    bool goal_changed;
    bool arrival_announced;

    // Messages
    bool path_msg_updated;
    avt_341::msg::Path path_msg;

    bool follower_status_msg_updated;
    avt_341::msg::FollowerStatus follower_status_message;

    bool comm_msg_updated;
    avt_341::msg::String comm_msg;

  private:

    std::vector<Pose> mission_data;
    std::map<std::string, Formation> formations;
	std::vector<Contact> mission_contacts;
	bool investigating_contact;	// probably temporary 

    // Methods
	auto getContact(std::string name, float x, float y); 
	void addContact(std::string name, float x, float y);
	bool close(float x, float y, float n_x, float n_y);
}; // class mission manager

} // namespace mission
} // namespace avt_341
