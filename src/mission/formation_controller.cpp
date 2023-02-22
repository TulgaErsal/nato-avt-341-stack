// clas definition
#include "avt_341/mission/formation_controller.h"
// c++ includes
#include <math.h>


FormationController::FormationController(){
	GlobalPathPointsDist = 1.0f;
	followerDistGain = 1.0f;
}

void FormationController::ConvertQuaternionToRotMat(TQuat q, Matrix3x3 &R){
	//float q[4];
	//q[0] = quat.x; q[1] = quat.y, q[2]=quat.z; q[3] = quat.w;
	R[0][0]=1.f+2.f*(-q[1]*q[1]-q[2]*q[2]); R[0][1]=2.f*(q[0]*q[1]-q[2]*q[3]); R[0][2]=2.f*(q[0]*q[2]+q[1]*q[3]);
	R[1][0]=2.f*(q[0]*q[1]+q[2]*q[3]); R[1][1]=1.f+2.f*(-q[0]*q[0]-q[2]*q[2]); R[1][2]=2.f*(q[1]*q[2]-q[0]*q[3]);
	R[2][0]=2.f*(q[0]*q[2]-q[1]*q[3]); R[2][1]=2.f*(q[1]*q[2]+q[0]*q[3]); R[2][2]=1.f+2.f*(-q[0]*q[0]-q[1]*q[1]);
}

void FormationController::NormalizeVec2D(Vec2d &v){
	float l=sqrtf(v[0]*v[0]+v[1]*v[1]);
	if(l==0.0f)l=0.0f; else l=1.0f/l;
	v[0]*=l; v[1]*=l;
}


//Global Path Generator
void FormationController::GenerateLeaderPath(avt_341::msg::Odometry leader_odom, avt_341::msg::FollowerStatus status, Vec2d leaderVy){
	if(!status.use_leader) return;
	float leaderYoffset = status.y_offset;
	if(GlobalPath.poses.size() == 0){
		avt_341::msg::PoseStamped pose;
		pose.pose.position.x = leader_odom.pose.pose.position.x + leaderVy[0]*leaderYoffset;
		pose.pose.position.x = leader_odom.pose.pose.position.y + leaderVy[1]*leaderYoffset;
		pose.pose.position.y = leader_odom.pose.pose.position.z;
		GlobalPath.poses.push_back(pose);
		//ROS_INFO("Global Path: %i X:%g Y:%g Z:%g", numGlobalPathPoints, GlobalPath[0], GlobalPath[1], GlobalPath[2]);
		return;
	}

	int n3 = (int)GlobalPath.poses.size()-1;
	float dx = (leader_odom.pose.pose.position.x + leaderVy[0]*leaderYoffset) - GlobalPath.poses[n3].pose.position.x;
	float dy = (leader_odom.pose.pose.position.y + leaderVy[1]*leaderYoffset) - GlobalPath.poses[n3].pose.position.y;
	float dz = leader_odom.pose.pose.position.z - GlobalPath.poses[n3].pose.position.z;

	float dist2 = dx*dx + dy*dy + dz*dz;
	if(dist2 < GlobalPathPointsDist * GlobalPathPointsDist) return;

	avt_341::msg::PoseStamped pose;
	pose.pose.position.x = leader_odom.pose.pose.position.x + leaderVy[0]*leaderYoffset;
	pose.pose.position.x = leader_odom.pose.pose.position.y + leaderVy[1]*leaderYoffset;
	pose.pose.position.y = leader_odom.pose.pose.position.z;
	GlobalPath.poses.push_back(pose);

	return;
	//ROS_INFO("Global Path: %i X:%g Y:%g Z:%g", numGlobalPathPoints, GlobalPath[n3], GlobalPath[n3+1], GlobalPath[n3+2]);
}


//Formation Vehicle Speed Calculation
//Output: target follower speed
float FormationController::CalculateFollowerSpeed(avt_341::msg::Odometry leader_odom, avt_341::msg::Odometry odom, avt_341::msg::FollowerStatus status, Vec2d leaderVx, Vec2d leaderVy){
	float targetSpeed = 0.0f;
	float leaderXoffset = status.x_offset;
	float leaderYoffset = status.y_offset;
	//Calculate target leader point
	float targetLeaderPoint[2];
	targetLeaderPoint[0] = leader_odom.pose.pose.position.x + leaderVx[0]*leaderXoffset + leaderVy[0]*leaderYoffset;
 	targetLeaderPoint[1] = leader_odom.pose.pose.position.y + leaderVx[1]*leaderXoffset + leaderVy[1]*leaderYoffset;


	//Calculate vector to target leader point
	float vec[2];
	vec[0] = odom.pose.pose.position.x - targetLeaderPoint[0];
	vec[1] = odom.pose.pose.position.y - targetLeaderPoint[1];

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

		targetSpeed = dotP * followerDistGain + speedHeading;
		if(targetSpeed < 0.0f)targetSpeed = 0.0f;
	}
	else{
		targetSpeed = 0.0f; 
	}
	return targetSpeed;
}


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

	float desired_speed = CalculateFollowerSpeed(leader_odom, odom, status, leaderVx, leaderVy);

	//publish GlobalPath
	//publish desired_speed
}