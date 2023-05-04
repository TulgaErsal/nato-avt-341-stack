#include "avt_341/mission/formation_speed_control.h"

namespace avt_341 {
namespace mission {

FormationSpeedController::FormationSpeedController(FormationDefinition & formation_def,
                                                   const FormationSpeedControlParams & params,
                                                   std::shared_ptr<avt_341::node::NodeProxy> node_proxy)
                         : my_name_(formation_def.my_name), formation_def_(formation_def), fsc_params_(params), node_proxy_(node_proxy) {

  if(fsc_params_.debug_visualize){
    marker_pub_ = node_proxy_->create_publisher<avt_341::msg::Marker>("avt_341/formation_visualize", 1);
  }
}

avt_341::msg::PoseStamped FormationSpeedController::getFollowerTargetPose(avt_341::msg::Odometry leader_odom, avt_341::msg::FollowerStatus status){
  Vec2d leaderVx, leaderVy;
  PoseToForwardRightVectors(leader_odom.pose.pose, leaderVx, leaderVy);
  double leaderYoffset = status.y_offset;
  double leaderXoffset = status.x_offset;
  avt_341::msg::PoseStamped target_pose;
  target_pose.pose.position.x = leader_odom.pose.pose.position.x + leaderVy[0]*leaderYoffset + leaderVx[0]*leaderXoffset;
  target_pose.pose.position.y = leader_odom.pose.pose.position.y + leaderVy[1]*leaderYoffset + leaderVx[1]*leaderXoffset;
  target_pose.pose.position.z = leader_odom.pose.pose.position.z;
  target_pose.pose.orientation.x = leader_odom.pose.pose.orientation.x;
  target_pose.pose.orientation.y = leader_odom.pose.pose.orientation.y;
  target_pose.pose.orientation.z = leader_odom.pose.pose.orientation.z;
  target_pose.pose.orientation.w = leader_odom.pose.pose.orientation.w;
  return target_pose;
}

double FormationSpeedController::getSpeedFactor(std::map<std::string, avt_341::msg::Odometry> & formation_poses) {
  if (formation_poses.find(my_name_) == formation_poses.end()) {
    std::cout << "FormationSpeedController::getSpeedFactor: my_name_ not found in formation_poses " << my_name_ << std::endl;
    return 0.0;
  }

  // loop through formation_poses, get pose target
  std::vector<std::string> out_formation_veh;
  std::map<std::string, double> delta_pos_map;
  std::map<std::string, avt_341::msg::PoseStamped> target_poses;

  int first_oof_idx = -1;
  const std::string leader_name = formation_def_.leaderName();
  bool is_leader = formation_def_.isLeader();
  bool is_column = formation_def_.isColumn();
  bool self_out_of_formation = false;
  int my_index = formation_def_.formationIndex();
  std::string followed_vehicle = formation_def_.followedVehicle();
  std::vector<std::string> formation_vehicle_names = formation_def_.orderedVehicles();

  // Populate target_poses, out_formation_veh, first_oof_idx, delta_pos_map
  // ==========================================================================================
  for (const auto &veh_name: formation_vehicle_names) {
    // if not leader
    if (veh_name != leader_name) {
      int veh_index;
      const auto formation_status = formation_def_.commToFollowerStatus(veh_name, veh_index);
      std::string followed_vehicle_i;
      if (formation_def_.offsets_from_leader || veh_index == 0 || !is_column) {
        followed_vehicle_i = leader_name;
      } else {
        followed_vehicle_i = formation_vehicle_names[veh_index - 1];
      }

      const auto target_pose = getFollowerTargetPose(formation_poses[followed_vehicle_i], formation_status);
      target_poses[veh_name] = target_pose;

      double delta_pos = PosePlanarDistance(target_pose.pose.position, formation_poses[veh_name].pose.pose.position);
      bool is_out_of_formation = delta_pos > fsc_params_.oof_threshold;
      if (is_out_of_formation && first_oof_idx < 0) {
        first_oof_idx = veh_index;
      }

      if (is_out_of_formation) {
        out_formation_veh.push_back(veh_name);
        self_out_of_formation = (self_out_of_formation || veh_name == my_name_);
      }
      delta_pos_map[veh_name] = delta_pos;
    }
  }

  double speed_factor = 1.0;

  if(formation_def_.formationAtGoal()){
    if (fsc_params_.debug_visualize) {
      visualizeSpeedIndicators(speed_factor, delta_pos_map[my_name_], target_poses[my_name_], formation_poses[my_name_].pose.pose.position);
    }
    return speed_factor;
  }

  // Calculate speed factor if at least one vehicle out of formation
  // ==========================================================================================
  if ((is_column || !self_out_of_formation) && !out_formation_veh.empty() && out_formation_veh[0] != my_name_) {
    std::string first_oof = out_formation_veh[0];

    if (formation_def_.offsets_from_leader) {
      double distance_to_self = PosePlanarDistance(formation_poses[first_oof].pose.pose.position,
                                                   formation_poses[my_name_].pose.pose.position);
      // speed control IF: In formation or own vehicle close to first out of formation vehicle
      if (!self_out_of_formation || distance_to_self < fsc_params_.oof_threshold) {

        auto delta_past_threshold = delta_pos_map[first_oof] - fsc_params_.oof_threshold;
        if (delta_past_threshold > 0.0) {
          // linear range
          double mult = self_out_of_formation ? 1.0/fsc_params_.oof_mult : 1.0;
          speed_factor = std::max(1.0 - mult * fsc_params_.oof_lin_slope * delta_past_threshold - fsc_params_.oof_const_term, 0.0);
        }
      }
    } else {
      auto delta_past_threshold = delta_pos_map[first_oof] - fsc_params_.oof_threshold;
      if ((is_leader || (is_column && my_index < first_oof_idx)) && delta_past_threshold > 0.0) {
        // linear range
        speed_factor = std::max(1.0 - fsc_params_.oof_lin_slope * delta_past_threshold - fsc_params_.oof_const_term, 0.0);
      }
    }
  }

  // Dot product heading filter
  // ==========================================================================================
  if(!is_leader && followed_vehicle != leader_name){
    utils::vec2 followed_dir;
    auto followed_target_pos = target_poses[followed_vehicle].pose.position;
    auto followed_current_pos = formation_poses[followed_vehicle].pose.pose.position;
    followed_dir.x = static_cast<float>(followed_target_pos.x - followed_current_pos.x);
    followed_dir.y = static_cast<float>(followed_target_pos.y - followed_current_pos.y);
    followed_dir.normalize();

    utils::vec2 my_dir;
    auto my_target_pos = target_poses[my_name_].pose.position;
    auto my_current_pos = formation_poses[my_name_].pose.pose.position;
    my_dir.x = static_cast<float>(my_target_pos.x - my_current_pos.x);
    my_dir.y = static_cast<float>(my_target_pos.y - my_current_pos.y);
    const float dir_mag = my_dir.mag();
    my_dir.normalize();

    if(utils::dot(my_dir, followed_dir) < fsc_params_.follower_dot_threshold || dir_mag < fsc_params_.follower_dist_break) {
      speed_factor = 0.0;
    }
  }

  if (fsc_params_.debug_visualize) {
    visualizeSpeedIndicators(speed_factor, delta_pos_map[my_name_], target_poses[my_name_], formation_poses[my_name_].pose.pose.position);
  }

  return speed_factor;
}

void FormationSpeedController::visualizeSpeedIndicators(double speed_factor, double delta_pos,
                                                        const avt_341::msg::PoseStamped &target_pose,
                                                        const avt_341::msg::Point &current_pos) {

  std::string str1 = "map";
  std::string str2 = my_name_ + "_target";
  node_proxy_->publish_tf(str1, str2, target_pose);

  avt_341::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_proxy_->get_stamp();
  marker.id = 0;
  marker.type = avt_341::msg::Marker::TEXT_VIEW_FACING;
  marker.action = avt_341::msg::Marker::MODIFY;
  marker.color.a = 1.0;
  marker.scale.z = 3.0;
  if (speed_factor < 0.4) {
    marker.color.r = 1.0;
  } else if (speed_factor < 1.0 - 1e-3) {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
  } else {
    marker.color.b = 1.0;
  }
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  std::ostringstream out;
  out.precision(2);
  out << std::fixed;
  out << "(" << delta_pos << ", " << speed_factor << ")";

  marker.text = out.str();
  marker.pose.position.x = current_pos.x;
  marker.pose.position.y = current_pos.y + 3.0;
  marker.pose.position.z = 0.1;
  marker_pub_->publish(marker);
}

}
}