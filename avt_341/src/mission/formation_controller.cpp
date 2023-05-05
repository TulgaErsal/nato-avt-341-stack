// clas definition
#include "avt_341/mission/formation_controller.h"
// c++ includes
#include <math.h>

namespace avt_341 {
namespace mission {

FormationController::FormationController(){
	global_path_points_dist_ = 1.0f;
	gpp2_ = global_path_points_dist_*global_path_points_dist_;
	follower_dist_gain_ = 1.0f;
	desired_speed_ = 0.0f;
}

/**
 * @brief Generate the global path for the vehicle
 * 
 * @param leader_odom Current odometry status of the lead vehicle
 * @param status Message containing desired offsets for this vehicle
 * @param leaderVy y (left) norm vector in coordiante frame of leader
 */
void FormationController::GenerateLeaderPath(avt_341::msg::Odometry leader_odom, avt_341::msg::FollowerStatus status, Vec2d leaderVx, Vec2d leaderVy){
  if(!(bool)status.use_leader) return;

  double leaderYoffset = status.y_offset;
  double leaderXoffset = x_offset_on_path_? 0.0 : status.x_offset;

  avt_341::msg::PoseStamped target_pose;
  target_pose.pose.position.x = leader_odom.pose.pose.position.x + leaderVy[0]*leaderYoffset + leaderVx[0]*leaderXoffset;
  target_pose.pose.position.y = leader_odom.pose.pose.position.y + leaderVy[1]*leaderYoffset + leaderVx[1]*leaderXoffset;
  target_pose.pose.position.z = leader_odom.pose.pose.position.z;

  // If x_offset_on_path, need to keep track of leader path history and only add to desired global path once past x_offset
  auto & target_path = x_offset_on_path_ ? leader_path_history_ : desired_global_path_;

  if(target_path.poses.empty()){
    target_path.poses.push_back(target_pose);
    return;
  }

  const auto & last_pose = target_path.poses.back();
  double dx = target_pose.pose.position.x - last_pose.pose.position.x;
  double dy = target_pose.pose.position.y - last_pose.pose.position.y;
  double dz = target_pose.pose.position.z - last_pose.pose.position.z;

  double dist2 = dx*dx + dy*dy + dz*dz;
  if(dist2 < gpp2_) return;

  target_path.poses.push_back(target_pose);

  // Extra logic if x_offset_on_path_. Add points to desired_global_path_ from leader_path_history_ that are path x_offset in path distance.
  if(x_offset_on_path_){
    double s_length = 0;
    int cutoff_index = -1;
    for(int i = 1; i < leader_path_history_.poses.size(); i++){
      double dx_i = leader_path_history_.poses[i].pose.position.x - leader_path_history_.poses[i-1].pose.position.x;
      double dy_i = leader_path_history_.poses[i].pose.position.y - leader_path_history_.poses[i-1].pose.position.y;
      s_length += sqrt(dy_i*dy_i + dx_i*dx_i);
      if(s_length > status.x_offset){
        cutoff_index = i;
        break;
      }
    }
    while(cutoff_index > 0 && leader_path_history_.poses.size() > cutoff_index){
      desired_global_path_.poses.push_back(leader_path_history_.poses.back());
      leader_path_history_.poses.pop_back();
    }
  }

}

/**
 * @brief Calculate the desired speed for the follower vehicle
 * 
 * @param leader_odom Current odometry status of the lead vehicle
 * @param odom Current odometry of the ego-vehicle
 * @param status Message containing desired offsets for this vehicle
 * @param leaderVx Orientation of the leader vehicle 
 * @param leaderVy Orientation of the leader vehicle 
 */
void FormationController::CalculateFollowerSpeed(avt_341::msg::Odometry leader_odom, avt_341::msg::Odometry odom, avt_341::msg::FollowerStatus status, Vec2d leaderVx, Vec2d leaderVy){
    float followerHeadingDistLimit_ = 20;
	
	float targetSpeed = 0.0f;
	float leaderXoffset = status.x_offset;
	float leaderYoffset = status.y_offset;
	//Calculate target leader point
	float targetLeaderPoint[2];
	targetLeaderPoint[0] = leader_odom.pose.pose.position.x + leaderVx[0]*leaderXoffset + leaderVy[0]*leaderYoffset;
 	targetLeaderPoint[1] = leader_odom.pose.pose.position.y + leaderVx[1]*leaderXoffset + leaderVy[1]*leaderYoffset;


	//Calculate vector to target leader point
	// CTG 2/23/23 - these were facing the wrong way, added the minus sign
	float vec[2];
	vec[0] = -(odom.pose.pose.position.x - targetLeaderPoint[0]);
	vec[1] = -(odom.pose.pose.position.y - targetLeaderPoint[1]);

	Vec2d vehicleVx;
	CalcVehicleRotation(odom,vehicleVx);

	//Calculate the distance to target leader point
	float dist = sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
	
	//Calculate the dot product of the vehicle heading vector and the vector to target leader point
	float dotP = vehicleVx[0]*vec[0] +  vehicleVx[1]*vec[1];

	float leaderVel[2];
	leaderVel[0] = leader_odom.twist.twist.linear.x;
	leaderVel[1] = leader_odom.twist.twist.linear.y;

	//Speed along vehicle heading
	float speedHeading = vehicleVx[0]*leaderVel[0] + vehicleVx[1]*leaderVel[1];
	
    //If distance between vehicles is less than followerHeadingDistLimit_ meters, the vehicles are too close and distance cannot be used.
	if(dist < followerHeadingDistLimit_){
	        if(dotP > 0){
		         targetSpeed = dotP * follower_dist_gain_ + speedHeading;
		         if(targetSpeed < 0.0f)targetSpeed = 0.0f;
	            }
	        else{
		         targetSpeed = 0.0f; //Follower Vehicle is heading in the wrong direction. So stop.
	            }
	    }
	else{
	     targetSpeed = dist * follower_dist_gain_ + speedHeading;
	    }

	desired_speed_ = targetSpeed;
}

/// @brief  Calculate the 2D rotation of the ego vehicle
void FormationController::CalcVehicleRotation(avt_341::msg::Odometry odom, Vec2d &vehicleVx){
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

void FormationController::Update(avt_341::msg::Odometry leader_odom, avt_341::msg::Odometry odom, avt_341::msg::FollowerStatus status){

	Vec2d leaderVx, leaderVy;

  PoseToForwardRightVectors(leader_odom.pose.pose, leaderVx, leaderVy);

  GenerateLeaderPath(leader_odom, status, leaderVx, leaderVy);
	desired_global_path_.header.frame_id = "map";

	CalculateFollowerSpeed(leader_odom, odom, status, leaderVx, leaderVy);
}

void FormationController::ClearDesiredGlobalPath(){
  desired_global_path_.poses.clear();
  leader_path_history_.poses.clear();
}

void FormationController::Reset(){
  ClearDesiredGlobalPath();
}

} // namespace mission
} // namespace avt_341
