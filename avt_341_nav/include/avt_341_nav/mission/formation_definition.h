#ifndef AVT_341_FORMATION_DEFINITION_H
#define AVT_341_FORMATION_DEFINITION_H

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "avt_341_nav/mission/mission_manager_dto.h"
#include <avt_341_nav/mission_manager_params_dto.hpp>
#include <map>
#include <vector>

namespace avt_341_nav {
namespace mission {

struct FormationOffsets {
  geometry_msgs::msg::Point follower1;
  geometry_msgs::msg::Point follower2;
  geometry_msgs::msg::Point follower3;
};

// Formation follower state derived from a formation command: the ego-vehicle's
// offset from the followed vehicle (in its frame) and whether a leader is followed.
struct FollowerStatus {
  double x_offset = 0.0;
  double y_offset = 0.0;
  bool use_leader = false;
};

struct MissionPoint {
  std::string name;
  double pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w;
};

struct MissionPath {
  std::string name;
  std::vector<MissionPoint> poses;
};

using MissionManagerParams = avt_341_nav::params::mission_manager::Params;
using FormationParameters = MissionManagerParams::Formation;
using ToiParameters = MissionManagerParams::Toi;

class FormationDefinition {

public:

  FormationDefinition(const FormationParameters & params_in,
                      const std::string & my_name);
  FormationDefinition(FormationMsg &comm_msg, const MissionPoint & mp,
                      const FormationParameters &params_in,
                      const std::string & my_name);

  FollowerStatus commToFollowerStatus(const std::string &veh_name, int &out_idx) const;
  FollowerStatus commToFollowerStatus(const FormationMsg & comm_msg, const std::string &veh_name, int &out_idx) const;
  bool update(FormationMsg &comm_msg, const MissionPoint & mp);
  FormationOffsets getOffsets(const std::string &formation) const;

  inline bool isLeader() const { return leaderName() == my_name_; }
  inline bool isFollowing() const { return formation_status.use_leader && !formationAtGoal(); }
  inline bool formationAtGoal() const { return formation_at_goal_; }
  inline bool has_formation() const { return !current_formation_msg_.formation.empty(); }
  inline std::string leaderName() const { return current_formation_msg_.leader_name; }
  inline std::string followedVehicle() const { return followed_vehicle_; }
  inline std::string terminationMethod() const { return current_formation_msg_.termination_method; }
  inline std::vector<std::string> orderedVehicles() const { return formation_vehicle_names_; }
  inline int formationIndex() const { return my_index_; }
  inline bool isColumn() const { return current_formation_msg_.formation == "COLUMN"; }
  inline std::string getFormationType() const { return current_formation_msg_.formation; }

  FollowerStatus formation_status;
  geometry_msgs::msg::PoseStamped goal;
  const FormationParameters &params;

private:

  std::map<std::string, FormationOffsets> offsets_map_;

  std::vector<std::string> formation_vehicle_names_;
  FormationMsg current_formation_msg_;
  std::string my_name_;
  std::string followed_vehicle_;
  int my_index_;
  bool formation_at_goal_;
};

}
}

#endif //AVT_341_FORMATION_DEFINITION_H
