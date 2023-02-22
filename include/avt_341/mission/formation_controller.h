// c++ includes
#include <string>
// local includes
#include "avt_341/node/ros_types.h"

typedef float Matrix3x3[3][3];

typedef float Vec2d[2];

typedef float TQuat[4];

//struct FollowerStatus{
//	float x_offset;
//	float y_offset;
//	bool use_leader;
//	std::string leader_name;
//};

// class for formation control
class FormationController{

  public:

	FormationController();
	
	void Update(avt_341::msg::Odometry leader_odom, avt_341::msg::Odometry odom, avt_341::msg::FollowerStatus status);


	void SetGlobalPathPointsDist(float d){GlobalPathPointsDist = d;}


	void SetFollowerDistGain(float gain){followerDistGain=gain;}

  private:

	//Global Path Generator
	void  GenerateLeaderPath(avt_341::msg::Odometry leader_odom, avt_341::msg::FollowerStatus status, Vec2d leaderVy);

	//Formation Vehicle Speed Calculation
	float CalculateFollowerSpeed(avt_341::msg::Odometry leader_odom, avt_341::msg::Odometry odom, avt_341::msg::FollowerStatus status, Vec2d leaderVx, Vec2d leaderVy);

	// control parameters
	float GlobalPathPointsDist; 
	float followerDistGain;

	// message published
	avt_341::msg::Path GlobalPath;

	void ConvertQuaternionToRotMat(TQuat q, Matrix3x3 &R);

	void NormalizeVec2D(Vec2d &v);

	void CalcLeaderRotation(avt_341::msg::Odometry leader_odom, Vec2d &leaderVx, Vec2d &leaderVy);

	void CalcVehicleRotation(avt_341::msg::Odometry odom, Vec2d &vehicleVx);

}; // class formation controller
