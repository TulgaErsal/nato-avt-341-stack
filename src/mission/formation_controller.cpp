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

void FormationController::ConvertQuaternionToRotMat(TQuat q, Matrix3x3 &R){
	R[0][0]=1.f+2.f*(-q[1]*q[1]-q[2]*q[2]); R[0][1]=2.f*(q[0]*q[1]-q[2]*q[3]); R[0][2]=2.f*(q[0]*q[2]+q[1]*q[3]);
	R[1][0]=2.f*(q[0]*q[1]+q[2]*q[3]); R[1][1]=1.f+2.f*(-q[0]*q[0]-q[2]*q[2]); R[1][2]=2.f*(q[1]*q[2]-q[0]*q[3]);
	R[2][0]=2.f*(q[0]*q[2]-q[1]*q[3]); R[2][1]=2.f*(q[1]*q[2]+q[0]*q[3]); R[2][2]=1.f+2.f*(-q[0]*q[0]-q[1]*q[1]);
}

void FormationController::NormalizeVec2D(Vec2d &v){
	float l=sqrtf(v[0]*v[0]+v[1]*v[1]);
	if(l==0.0f)l=0.0f; else l=1.0f/l;
	v[0]*=l; v[1]*=l;
}

/**
 * @brief Generate the global path for the vehicle
 * 
 * @param leader_odom Current odometry status of the lead vehicle
 * @param status Message containing desired offsets for this vehicle
 * @param leaderVy Orientation of the leader vehicle
 */
void FormationController::GenerateLeaderPath(avt_341::msg::Odometry leader_odom, avt_341::msg::FollowerStatus status, Vec2d leaderVy){
	if(!(bool)status.use_leader) return;

	float leaderYoffset = status.y_offset;

	if(desired_global_path_.poses.size() == 0){
		avt_341::msg::PoseStamped pose;
		pose.pose.position.x = leader_odom.pose.pose.position.x + leaderVy[0]*leaderYoffset;
		pose.pose.position.y = leader_odom.pose.pose.position.y + leaderVy[1]*leaderYoffset;
		pose.pose.position.z = leader_odom.pose.pose.position.z;
		desired_global_path_.poses.push_back(pose);
		//ROS_INFO("Global Path: %i X:%g Y:%g Z:%g", numdesired_global_path_Points, desired_global_path_[0], desired_global_path_[1], desired_global_path_[2]);
		return;
	}

	int n3 = (int)desired_global_path_.poses.size()-1;
	float dx = (leader_odom.pose.pose.position.x + leaderVy[0]*leaderYoffset) - desired_global_path_.poses[n3].pose.position.x;
	float dy = (leader_odom.pose.pose.position.y + leaderVy[1]*leaderYoffset) - desired_global_path_.poses[n3].pose.position.y;
	float dz = leader_odom.pose.pose.position.z - desired_global_path_.poses[n3].pose.position.z;

	float dist2 = dx*dx + dy*dy + dz*dz;
	if(dist2 < gpp2_) return;

	avt_341::msg::PoseStamped pose;
	pose.pose.position.x = leader_odom.pose.pose.position.x + leaderVy[0]*leaderYoffset;
	pose.pose.position.y = leader_odom.pose.pose.position.y + leaderVy[1]*leaderYoffset;
	pose.pose.position.z = leader_odom.pose.pose.position.z;
	desired_global_path_.poses.push_back(pose);

	return;
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

	//Calculate the dot product of the vehicle heading vector and the vector to target leader point
	float dotP = vehicleVx[0]*vec[0] +  vehicleVx[1]*vec[1]; 
	if(dotP>0){
		float relativeVel[2];
		relativeVel[0] = leader_odom.twist.twist.linear.x - odom.twist.twist.linear.x;
		relativeVel[1] = leader_odom.twist.twist.linear.y - odom.twist.twist.linear.y;

		//Speed along vehicle heading
		float speedHeading = vehicleVx[0]*relativeVel[0] + vehicleVx[1]*relativeVel[1];

		targetSpeed = dotP * follower_dist_gain_ + speedHeading;
		if(targetSpeed < 0.0f)targetSpeed = 0.0f;
	}
	else{
		targetSpeed = 0.0f; 
	}

	
	desired_speed_ = targetSpeed;
}

/// @brief  Calculate the 2D rotation of the leader vehicle
void FormationController::CalcLeaderRotation(avt_341::msg::Odometry leader_odom, Vec2d &leaderVx, Vec2d &leaderVy){
	Matrix3x3 leaderRotMatrix;
	TQuat q;
	q[0] = leader_odom.pose.pose.orientation.x; 
	q[1] = leader_odom.pose.pose.orientation.y;
	q[2] = leader_odom.pose.pose.orientation.z; 
	q[3] = leader_odom.pose.pose.orientation.w;
	ConvertQuaternionToRotMat(q, leaderRotMatrix);
	leaderVx[0] = 0.5 * ( leaderRotMatrix[0][0] + leaderRotMatrix[1][1]); //average cos
	leaderVx[1] = 0.5 * (-leaderRotMatrix[0][1] + leaderRotMatrix[1][0]); //average sin
	NormalizeVec2D(leaderVx);
	leaderVy[0] = leaderVx[1];
	leaderVy[1] =-leaderVx[0];
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

	CalcLeaderRotation(leader_odom, leaderVx, leaderVy);

	GenerateLeaderPath(leader_odom, status, leaderVy);

	CalculateFollowerSpeed(leader_odom, odom, status, leaderVx, leaderVy);
}

} // namespace mission
} // namespace avt_341