/**
* \class FormationController
*
* Controller for vehicle global path points based on desired formation.
* Adapted to ROS by CTG from original code by Tamer Wasfy
*
* \author Tamer Wasfy, Chris Goodin
*
* \date 8/31/2020
*/

#ifndef AVT_341_FORMATION_CONTROLLER_H
#define AVT_341_FORMATION_CONTROLLER_H

// c++ includes
#include <string>
// local includes
#include "avt_341/node/ros_types.h"
#include "avt_341/mission/formation_utils.h"

namespace avt_341 {
namespace mission {

// convenient shorthands for adapting TW's code
typedef float Matrix3x3[3][3];
typedef float Vec2d[2];
typedef float TQuat[4];

/// Class for formation control
class FormationController{

  public:
	/// Construct a formation controller
	FormationController();
	
	/// Update the controller based on the most recent leader odometry, vehicle odometry, and status message
	void Update(avt_341::msg::Odometry leader_odom, avt_341::msg::Odometry odom, avt_341::msg::FollowerStatus status);

	/// Set the global path point distance in meters - this is the spacing between points
	void SetGlobalPathPointsDist(float d){global_path_points_dist_ = d; gpp2_ = d*d; }

	/// Set the follower dist gain. This controls how aggressively the vehicle closes ground, equavilent to the "P" in PID
	void SetFollowerDistGain(float gain){follower_dist_gain_ = gain;}

  void SetXOffsetOnPath(bool x_offset_on_path){x_offset_on_path_ = x_offset_on_path;}

	/// Get the current desired global path
	avt_341::msg::Path GetPath(){return desired_global_path_; }

	/// Get the current desired speed in m/s
	avt_341::msg::Float64 GetSpeed(){avt_341::msg::Float64 ds; ds.data = desired_speed_; return ds; }

  private:

	// Method to generate global path based on formation
  void GenerateLeaderPath(avt_341::msg::Odometry leader_odom, avt_341::msg::FollowerStatus status, Vec2d leaderVx, Vec2d leaderVy);

  // Method to calculate desired speed based on formation
	void CalculateFollowerSpeed(avt_341::msg::Odometry leader_odom, avt_341::msg::Odometry odom, avt_341::msg::FollowerStatus status, Vec2d leaderVx, Vec2d leaderVy);

	// control parameters
	float global_path_points_dist_;
	float gpp2_; // square of global_path_points_dist_ 
	float follower_dist_gain_;

	// outputs / messages published
	avt_341::msg::Path desired_global_path_;
	float desired_speed_;
  bool x_offset_on_path_;

	// utility functions and intermediate calculations
	void CalcVehicleRotation(avt_341::msg::Odometry odom, Vec2d &vehicleVx);

}; // class formation controller

} // namespace mission
} // namespace avt_341

#endif //AVT_341_FORMATION_CONTROLLER_H
