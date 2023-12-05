// clas definition
#include "avt_341/mission/formation_path_generator.h"
// c++ includes
#include <math.h>

namespace avt_341 {
namespace mission {

FormationPathGenerator::FormationPathGenerator(const avt_341::mission::FormationParameters & params)
: params_(params){
  gpp2_ = params_.global_path_points_dist*params_.global_path_points_dist;
}

/**
 * @brief Generate the global path for the vehicle
 * 
 * @param leader_odom Current odometry status of the lead vehicle
 * @param status Message containing desired offsets for this vehicle
 * @param leaderVy y (left) norm vector in coordiante frame of leader
 */
void FormationPathGenerator::GenerateLeaderPath(const avt_341::msg::Odometry & leader_odom, const avt_341::msg::Odometry & odom,
                                             avt_341::msg::FollowerStatus status, Vec2d leaderVx, Vec2d leaderVy){
  if(!(bool)status.use_leader) return;

  double leaderYoffset = status.y_offset;
  double leaderXoffset = params_.x_offset_on_path ? 0.0 : status.x_offset;

  avt_341::msg::PoseStamped target_pose;
  target_pose.pose.position.x = leader_odom.pose.pose.position.x + leaderVy[0]*leaderYoffset + leaderVx[0]*leaderXoffset;
  target_pose.pose.position.y = leader_odom.pose.pose.position.y + leaderVy[1]*leaderYoffset + leaderVx[1]*leaderXoffset;
  target_pose.pose.position.z = leader_odom.pose.pose.position.z;

  // If x_offset_on_path, need to keep track of leader path history and only add to desired global path once past x_offset
  if(desired_global_path_.poses.empty()){
    desired_global_path_.poses.push_back(target_pose);
    leader_path_history_.poses.push_back(target_pose);
    return;
  }

  auto & target_path = params_.x_offset_on_path ? leader_path_history_ : desired_global_path_;

  const auto & last_pose = target_path.poses.back();
  double dx = target_pose.pose.position.x - last_pose.pose.position.x;
  double dy = target_pose.pose.position.y - last_pose.pose.position.y;
  double dz = target_pose.pose.position.z - last_pose.pose.position.z;

  double dist2 = dx*dx + dy*dy + dz*dz;
  if(dist2 < gpp2_) return;

  target_path.poses.push_back(target_pose);

  // Extra logic if x_offset_on_path_. Add points to desired_global_path_ from leader_path_history_ that are path x_offset in path distance.
  if(params_.x_offset_on_path){
    double s_length = 0;
    int cutoff_index = -1;
    for(int i = leader_path_history_.poses.size()-2; i > 0; i--){
      double dx_i = leader_path_history_.poses[i].pose.position.x - leader_path_history_.poses[i+1].pose.position.x;
      double dy_i = leader_path_history_.poses[i].pose.position.y - leader_path_history_.poses[i+1].pose.position.y;
      s_length += sqrt(dy_i*dy_i + dx_i*dx_i);
      if(s_length > abs(status.x_offset)){
        cutoff_index = i;
        break;
      }
    }
    if(cutoff_index > -1){
      for(int i = 0; i < cutoff_index; i++){
        desired_global_path_.poses.push_back(leader_path_history_.poses[i]);
      }
      leader_path_history_.poses = std::vector<avt_341::msg::PoseStamped>(leader_path_history_.poses.begin()+cutoff_index,
                                                                          leader_path_history_.poses.end());
    }
  }

  if(params_.prune_global_path){
    // Remove all poses before closest location in desired_global_path_
    double min_dist2 = 1e10;
    int min_index = -1;
    for(int i = 0; i < desired_global_path_.poses.size(); i++){
      double dx_i = desired_global_path_.poses[i].pose.position.x - odom.pose.pose.position.x;
      double dy_i = desired_global_path_.poses[i].pose.position.y - odom.pose.pose.position.y;
      double dist2_i = dx_i*dx_i + dy_i*dy_i;
      if(dist2_i < min_dist2){
        min_dist2 = dist2_i;
        min_index = i;
      }
    }
    if(min_index > 0){
      desired_global_path_.poses = std::vector<avt_341::msg::PoseStamped>(desired_global_path_.poses.begin()+min_index,
                                                                          desired_global_path_.poses.end());
    }
  }

}

/// @brief  Calculate the 2D rotation of the ego vehicle
void FormationPathGenerator::CalcVehicleRotation(avt_341::msg::Odometry odom, Vec2d &vehicleVx){
	Matrix3x3 vehicleRotMatrix;
	TQuat q;
	q[0] = odom.pose.pose.orientation.x; 
	q[1] = odom.pose.pose.orientation.y;
	q[2] = odom.pose.pose.orientation.z; 
	q[3] = odom.pose.pose.orientation.w;
	ConvertQuaternionToRotMat(q, vehicleRotMatrix);
	vehicleVx[0] = 0.5 * ( vehicleRotMatrix[0][0] + vehicleRotMatrix[1][1]); //average cos
	vehicleVx[1] = 0.5 * (-vehicleRotMatrix[0][1] + vehicleRotMatrix[1][0]); //average sin
	NormalizeVec2D(vehicleVx);
}

void FormationPathGenerator::Update(avt_341::msg::Odometry leader_odom, avt_341::msg::Odometry odom, avt_341::msg::FollowerStatus status){

	Vec2d leaderVx, leaderVy;

  PoseToForwardRightVectors(leader_odom.pose.pose, leaderVx, leaderVy);

  GenerateLeaderPath(leader_odom, odom, status, leaderVx, leaderVy);
}

void FormationPathGenerator::Reset(){
  desired_global_path_.poses.clear();
  leader_path_history_.poses.clear();
}

} // namespace mission
} // namespace avt_341
