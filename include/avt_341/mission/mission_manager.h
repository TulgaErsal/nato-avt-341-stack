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
    
/// Class for formation control
class MissionManager{

  public:
	/// Construct a formation controller
	MissionManager();

    int loadMissionDefinition(std::string filename);

    Pose getPose(std::string posename);

    void handleMoveTo(avt_341::msg::Communication);

  private:

    std::vector<Pose> mission_data;

    void handleFormationRequest(avt_341::msg::Communication);

    void handleAcknowledge(avt_341::msg::Communication);

    void handleArrive(avt_341::msg::Communication);

    void handleTaskComplete(avt_341::msg::Communication);

    void handleHold(avt_341::msg::Communication);

    void handleSetSpeed(avt_341::msg::Communication);

}; // class mission manager

} // namespace mission
} // namespace avt_341