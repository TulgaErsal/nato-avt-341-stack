#include <iostream>
#include "avt_341_nav/mission/formation_definition.h"


namespace avt_341_nav {
namespace mission {

FormationDefinition::FormationDefinition(
    const FormationParameters & params_in, const std::string & my_name)
    : params(params_in), my_name_(my_name) {
  struct FormationOffsets f;
  f.follower1.x = 0;
  f.follower1.y = -1;
  f.follower2.x = 0;
  f.follower2.y = -2;
  f.follower3.x = 0;
  f.follower3.y = -3;
  offsets_map_["LINE"] = f;
  f.follower1.x = 0;
  f.follower1.y = 1;
  f.follower2.x = 0;
  f.follower2.y = 2;
  f.follower3.x = 0;
  f.follower3.y = 3;
  offsets_map_["LINE_RIGHT"] = f;
  f.follower1.x = 0;
  f.follower1.y = 1;
  f.follower2.x = 0;
  f.follower2.y = -1;
  f.follower3.x = 0;
  f.follower3.y = -2;
  offsets_map_["LINE_CENTER"] = f;
  f.follower1.x = -1;
  f.follower1.y = 0;
  f.follower2.x = -2;
  f.follower2.y = 0;
  f.follower3.x = -3;
  f.follower3.y = 0;
  offsets_map_["COLUMN"] = f;
  f.follower1.x = -1;
  f.follower1.y = 1;
  f.follower2.x = -2;
  f.follower2.y = 0;
  f.follower3.x = -3;
  f.follower3.y = 1;
  offsets_map_["STAGGER_COL"] = f;
  f.follower1.x = -1;
  f.follower1.y = 1;
  f.follower2.x = -1;
  f.follower2.y = -1;
  f.follower3.x = -2;
  f.follower3.y = 0;
  offsets_map_["DIAMOND"] = f;
  f.follower1.x = -1;
  f.follower1.y = 1;
  f.follower2.x = 0;
  f.follower2.y = -1;
  f.follower3.x = -1;
  f.follower3.y = -2;
  offsets_map_["WEDGE"] = f;
  f.follower1.x = -1;
  f.follower1.y = 1;
  f.follower2.x = -2;
  f.follower2.y = 2;
  f.follower3.x = -3;
  f.follower3.y = 3;
  offsets_map_["ECH_LEFT"] = f;
  f.follower1.x = -1;
  f.follower1.y = -1;
  f.follower2.x = -2;
  f.follower2.y = -2;
  f.follower3.x = -3;
  f.follower3.y = -3;
  offsets_map_["ECH_RIGHT"] = f;
}

FormationDefinition::FormationDefinition(
    FormationMsg &comm_msg, const MissionPoint & mp,
    const FormationParameters & params_in, const std::string & my_name)
    : FormationDefinition(params_in, my_name) {
  update(comm_msg, mp);
}



FormationOffsets FormationDefinition::getOffsets(const std::string &formation) const {
  FormationOffsets offsets = offsets_map_.at(formation);
  if(formation == "COLUMN" && !formation_at_goal_){
    offsets.follower2.x = offsets.follower1.x;
    offsets.follower3.x = offsets.follower1.x;
  }
  return offsets;
}


FollowerStatus FormationDefinition::commToFollowerStatus(const std::string & veh_name, int & out_idx) const {
  return commToFollowerStatus(current_formation_msg_, veh_name, out_idx);
}

FollowerStatus FormationDefinition::commToFollowerStatus(const FormationMsg & comm_msg, const std::string & veh_name, int & out_idx) const{
  FollowerStatus follower_status;
  if(comm_msg.leader_name == veh_name){
    follower_status.x_offset = 0.0;
    follower_status.y_offset = 0.0;
    follower_status.use_leader = false;
    out_idx = 0;
  }else{
    double x_scale = comm_msg.x_scale <= 0.0 ? params.follow_scale_x : comm_msg.x_scale;
    double y_scale = comm_msg.y_scale <= 0.0 ? params.follow_scale_y : comm_msg.y_scale;

    FormationOffsets f = getOffsets(comm_msg.formation);
    if(comm_msg.follower1_name == veh_name) {
      follower_status.x_offset = f.follower1.x * x_scale;
      follower_status.y_offset = f.follower1.y * y_scale;
      out_idx = 1;
    } else if (comm_msg.follower2_name == veh_name) {
      follower_status.x_offset = f.follower2.x * x_scale;
      follower_status.y_offset = f.follower2.y * y_scale;
      out_idx = 2;
    } else if (comm_msg.follower3_name == veh_name) {
      follower_status.x_offset = f.follower3.x * x_scale;
      follower_status.y_offset = f.follower3.y * y_scale;
      out_idx = 3;
    } else {
      out_idx = -1;
    }

    follower_status.use_leader = true;
  }
  return follower_status;
}

bool FormationDefinition::update(FormationMsg &comm_msg, const MissionPoint & mp){

  size_t substr_pos = comm_msg.formation.find("_AT_GOAL");
  formation_at_goal_ = substr_pos != std::string::npos;
  if(formation_at_goal_){
    comm_msg.formation = comm_msg.formation.substr(0, substr_pos);
  }

  current_formation_msg_ = comm_msg;
  formation_status = commToFollowerStatus(comm_msg, my_name_, my_index_);

  goal.pose.position.x = mp.pos_x;
  goal.pose.position.y = mp.pos_y;
  goal.pose.position.z = mp.pos_z;
  goal.pose.orientation.x = mp.rot_x;
  goal.pose.orientation.y = mp.rot_y;
  goal.pose.orientation.z = mp.rot_z;
  goal.pose.orientation.w = mp.rot_w;

  std::string leader_name = leaderName();
  formation_vehicle_names_ = std::vector<std::string>{leader_name, comm_msg.follower1_name, comm_msg.follower2_name, comm_msg.follower3_name};
  formation_vehicle_names_.erase(std::remove_if(formation_vehicle_names_.begin(), formation_vehicle_names_.end(), 
                                                [](const std::string & s){return s.empty() || s == "NIL" || s == "NONE";}), formation_vehicle_names_.end());

  if(my_index_ == 0 || !isColumn()){
    followed_vehicle_ = leader_name;
  }else{
    followed_vehicle_ = formation_vehicle_names_[my_index_ - 1];
  }

  return true;
}

}
}
