#ifndef AVT_341_FORMATION_PATH_GENERATOR_H
#define AVT_341_FORMATION_PATH_GENERATOR_H

// c++ includes
#include <string>
// local includes
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "avt_341_nav/mission/formation_utils.h"
#include "avt_341_nav/mission/formation_definition.h"

namespace avt_341_nav {
namespace mission {

/// Class for formation control
class FormationPathGenerator{

  public:
	/// Construct a formation controller
  FormationPathGenerator(const avt_341_nav::mission::FormationParameters & params);
	
	/// Update the controller based on the most recent leader odometry, vehicle odometry, and status message
  void Update(nav_msgs::msg::Odometry leader_odom, nav_msgs::msg::Odometry odom, FollowerStatus status);
  const nav_msgs::msg::Path & GetPath() const { return desired_global_path_; }
  inline bool useBreadcrumbs() const { return params_.use_breadcrumbs; }
  void Reset();

  private:

	// Method to generate global path based on formation
  void GenerateLeaderPath(const nav_msgs::msg::Odometry & leader_odom, const nav_msgs::msg::Odometry & odom,
													FollowerStatus status, Vec2d leaderVx, Vec2d leaderVy);

	// control parameters
	double gpp2_; // square of global_path_points_dist_

	// outputs / messages published
	nav_msgs::msg::Path desired_global_path_;
	nav_msgs::msg::Path leader_path_history_;
  const avt_341_nav::mission::FormationParameters & params_;

	// tangent heading state
	double prev_leader_x_;
	double prev_leader_y_;
	bool tangent_heading_valid_;
	float tangent_vx_[2];
	float tangent_vy_[2];

	// utility functions and intermediate calculations
	void CalcVehicleRotation(nav_msgs::msg::Odometry odom, Vec2d &vehicleVx);

}; // class formation controller

} // namespace mission
} // namespace avt_341_nav

#endif //AVT_341_FORMATION_PATH_GENERATOR_H
