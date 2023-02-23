// c++ includes
#include <string>
// local includes
#include "avt_341/node/ros_types.h"


namespace avt_341 {
namespace mission {

typedef float Matrix3x3[3][3];

typedef float Vec2d[2];

typedef float TQuat[4];

// class for formation control
class FormationController{

  public:

	FormationController();
	
	void Update(avt_341::msg::Odometry leader_odom, avt_341::msg::Odometry odom, avt_341::msg::FollowerStatus status);

	void SetGlobalPathPointsDist(float d){global_path_points_dist_ = d; gpp2_ = d*d; }

	void SetFollowerDistGain(float gain){follower_dist_gain_ = gain;}

	avt_341::msg::Path GetPath(){return desired_global_path_; }

	avt_341::msg::Float64 GetSpeed(){avt_341::msg::Float64 ds; ds.data = desired_speed_; }

  private:

	//Global Path Generator
	void  GenerateLeaderPath(avt_341::msg::Odometry leader_odom, avt_341::msg::FollowerStatus status, Vec2d leaderVy);

	//Formation Vehicle Speed Calculation
	void CalculateFollowerSpeed(avt_341::msg::Odometry leader_odom, avt_341::msg::Odometry odom, avt_341::msg::FollowerStatus status, Vec2d leaderVx, Vec2d leaderVy);

	// control parameters
	float global_path_points_dist_;
	float gpp2_; // square of global_path_points_dist_ 
	float follower_dist_gain_;

	// outputs / messages published
	avt_341::msg::Path desired_global_path_;
	float desired_speed_;

	void ConvertQuaternionToRotMat(TQuat q, Matrix3x3 &R);

	void NormalizeVec2D(Vec2d &v);

	void CalcLeaderRotation(avt_341::msg::Odometry leader_odom, Vec2d &leaderVx, Vec2d &leaderVy);

	void CalcVehicleRotation(avt_341::msg::Odometry odom, Vec2d &vehicleVx);

}; // class formation controller

} // namespace mission
} // namespace avt_341