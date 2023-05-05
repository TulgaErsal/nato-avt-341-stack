#ifndef AVT_341_FORMATION_DEFINITION_H
#define AVT_341_FORMATION_DEFINITION_H

#include "avt_341/node/ros_types.h"
#include <map>

namespace avt_341 {
namespace mission {

struct FormationOffsets {
  avt_341::msg::Point follower1;
  avt_341::msg::Point follower2;
  avt_341::msg::Point follower3;
};

class FormationDefinition {

public:

  FormationDefinition(const std::string &my_name_in, float follow_scale_x_in, float follow_scale_y_in, bool offsets_from_leader_in, int num_vehicles_in);
  FormationDefinition();

  avt_341::msg::FollowerStatus commToFollowerStatus(const std::string &veh_name, int &out_idx);
  avt_341::msg::FollowerStatus commToFollowerStatus(const avt_341::msg::Communication & comm_msg, const std::string &veh_name, int &out_idx);
  bool update(avt_341::msg::Communication &comm_msg);
  FormationOffsets getOffsets(const std::string &formation);

  inline bool isLeader() const { return leaderName() == my_name; }
  inline bool isFollowing() const { return formation_status.use_leader && !formationAtGoal(); }
  inline bool formationAtGoal() const { return formation_at_goal_; }
  inline std::string formationType() const { return current_formation_msg_.formation; }
  inline std::string leaderName() const { return current_formation_msg_.leader_name; }
  inline std::string followedVehicle() const { return followed_vehicle_; }
  inline std::vector<std::string> orderedVehicles() const { return formation_vehicle_names_; }
  inline int formationIndex() const { return my_index_; }
  inline bool isColumn() const { return current_formation_msg_.formation == "COLUMN"; }

  bool selfInFormation(const avt_341::msg::Communication &comm_msg);
  bool vehicleInFormation(const avt_341::msg::Communication &comm_msg, const std::string & vehicle_name);

  std::string my_name;
  float follow_scale_x;
  float follow_scale_y;
  bool offsets_from_leader;
  int num_vehicles;
  avt_341::msg::FollowerStatus formation_status;

private:

  std::map<std::string, FormationOffsets> offsets_map_;

  std::vector<std::string> formation_vehicle_names_;
  avt_341::msg::Communication current_formation_msg_;
  std::string followed_vehicle_;
  int my_index_;
  bool formation_at_goal_;
};

}
}

#endif //AVT_341_FORMATION_DEFINITION_H
