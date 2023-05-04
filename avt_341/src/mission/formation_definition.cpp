#include <iostream>
#include "avt_341/mission/formation_definition.h"


namespace avt_341 {
namespace mission {


FormationDefinition::FormationDefinition(const std::string &my_name_in, float follow_scale_x_in, float follow_scale_y_in, bool offsets_from_leader_in, int num_vehicles_in)
: my_name(my_name_in), follow_scale_x(follow_scale_x_in), follow_scale_y(follow_scale_y_in), offsets_from_leader(offsets_from_leader_in), num_vehicles(num_vehicles_in){
}
FormationDefinition::FormationDefinition(){

}



FormationOffsets FormationDefinition::getOffsets(const std::string &formation) {
  if (offsets_map_.empty()) {
    // initialize formation data
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
    offsets_map_["LINE_LEFT"] = f;
    f.follower1.x = -1;
    f.follower1.y = 0;
    f.follower2.x = offsets_from_leader ? -2 : -1;
    f.follower2.y = 0;
    f.follower3.x = offsets_from_leader ? -3 : -1;
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
  return offsets_map_[formation];

}


avt_341::msg::FollowerStatus FormationDefinition::commToFollowerStatus(const std::string & veh_name, int & out_idx) {
  return commToFollowerStatus(current_formation_msg_, veh_name, out_idx);
}

avt_341::msg::FollowerStatus FormationDefinition::commToFollowerStatus(const avt_341::msg::Communication & comm_msg, const std::string & veh_name, int & out_idx){
  avt_341::msg::FollowerStatus follower_status_msg;
  follower_status_msg.leader_name = comm_msg.leader_name;
  if(comm_msg.leader_name == veh_name){
    follower_status_msg.x_offset = 0.0;
    follower_status_msg.y_offset = 0.0;
    follower_status_msg.use_leader = false;
    out_idx = 0;
  }else{
    FormationOffsets f = getOffsets(comm_msg.formation);
    if(comm_msg.follower1_name == veh_name) {
      follower_status_msg.x_offset = f.follower1.x * follow_scale_x;
      follower_status_msg.y_offset = f.follower1.y * follow_scale_y;
      out_idx = 1;
    } else if (comm_msg.follower2_name == veh_name) {
      follower_status_msg.x_offset = f.follower2.x * follow_scale_x;
      follower_status_msg.y_offset = f.follower2.y * follow_scale_y;
      out_idx = 2;
    } else if (comm_msg.follower3_name == veh_name) {
      follower_status_msg.x_offset = f.follower3.x * follow_scale_x;
      follower_status_msg.y_offset = f.follower3.y * follow_scale_y;
      out_idx = 3;
    } else {
      out_idx = -1;
    }
    follower_status_msg.use_leader = true;
  }
  return follower_status_msg;
}

bool FormationDefinition::update(avt_341::msg::Communication &comm_msg){

  int veh_idx = -1;
  size_t substr_pos = comm_msg.formation.find("_AT_GOAL");
  bool formation_at_goal_in = substr_pos != std::string::npos;
  comm_msg.formation = comm_msg.formation.substr(0, substr_pos);

  auto formation_status_in = commToFollowerStatus(comm_msg, my_name, veh_idx);
  if(veh_idx < 0){
    // my_name not found in message
    return false;
  }

  my_index_ = veh_idx;
  current_formation_msg_ = comm_msg;
  formation_status = formation_status_in;
  formation_at_goal_ = formation_at_goal_in;

  std::string leader_name = leaderName();
  auto formation_vehicle_names_temp = std::vector<std::string>{leader_name, comm_msg.follower1_name, comm_msg.follower2_name, comm_msg.follower3_name};
  formation_vehicle_names_ = std::vector<std::string>(formation_vehicle_names_temp.begin(), formation_vehicle_names_temp.begin() + num_vehicles);

  if(offsets_from_leader || my_index_ == 0 || !isColumn()){
    followed_vehicle_ = leader_name;
  }else{
    followed_vehicle_ = formation_vehicle_names_[my_index_ - 1];
  }

  return true;
}

}
}