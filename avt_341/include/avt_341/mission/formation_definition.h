#ifndef AVT_341_FORMATION_DEFINITION_H
#define AVT_341_FORMATION_DEFINITION_H

#include "avt_341/node/ros_types.h"
#include "avt_341/mission/mission_manager_dto.h"
#include <map>
#include <vector>

namespace avt_341 {
namespace mission {

struct FormationOffsets {
  avt_341::msg::Point follower1;
  avt_341::msg::Point follower2;
  avt_341::msg::Point follower3;
};

struct MissionPoint {
  std::string name;
  double pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w;
};

struct MissionPath {
  std::string name;
  std::vector<MissionPoint> poses;
};

struct FormationParameters{
  std::string my_name;
  float follow_scale_x;
  float follow_scale_y;
  bool offsets_from_leader;
  float follow_goal_threshold;
  float global_path_points_dist;
  bool use_breadcrumbs;
  bool x_offset_on_path;
  bool prune_global_path;
};

struct ToiParameters{
  float approach_dist;
  float encircle_radius;
  float encircle_degrees;
  bool encircle_cw;
  float goal_threshold;
};

class FormationDefinition {

public:

  FormationDefinition(const FormationParameters & params_in);
  FormationDefinition(FormationMsg &comm_msg, const MissionPoint & mp, const FormationParameters &params_in);

  avt_341::msg::FollowerStatus commToFollowerStatus(const std::string &veh_name, int &out_idx) const;
  avt_341::msg::FollowerStatus commToFollowerStatus(const FormationMsg & comm_msg, const std::string &veh_name, int &out_idx) const;
  bool update(FormationMsg &comm_msg, const MissionPoint & mp);
  FormationOffsets getOffsets(const std::string &formation) const;

  inline bool isLeader() const { return leaderName() == params.my_name; }
  inline bool isFollowing() const { return formation_status.use_leader && !formationAtGoal(); }
  inline bool formationAtGoal() const { return formation_at_goal_; }
  inline bool has_formation() const { return !current_formation_msg_.formation.empty(); }
  inline std::string leaderName() const { return current_formation_msg_.leader_name; }
  inline std::string followedVehicle() const { return followed_vehicle_; }
  inline std::string terminationMethod() const { return current_formation_msg_.termination_method; }
  inline std::vector<std::string> orderedVehicles() const { return formation_vehicle_names_; }
  inline int formationIndex() const { return my_index_; }
  inline bool isColumn() const { return current_formation_msg_.formation == "COLUMN"; }

  avt_341::msg::FollowerStatus formation_status;
  avt_341::msg::PoseStamped goal;
  const FormationParameters &params;

private:

  std::map<std::string, FormationOffsets> offsets_map_;

  std::vector<std::string> formation_vehicle_names_;
  FormationMsg current_formation_msg_;
  std::string followed_vehicle_;
  int my_index_;
  bool formation_at_goal_;
};

}
}

#endif //AVT_341_FORMATION_DEFINITION_H
