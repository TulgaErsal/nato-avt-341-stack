#ifndef AVT_341_FORMATION_SPEED_CONTROL_H
#define AVT_341_FORMATION_SPEED_CONTROL_H

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/avt_341_utils.h"
#include "avt_341/mission/formation_utils.h"
#include "avt_341/mission/formation_definition.h"

namespace avt_341 {
namespace mission {

struct FormationSpeedControlParams {
  double oof_threshold;
  double oof_const_term;
  double oof_lin_slope;
  double oof_mult;
  bool debug_visualize;
  double follower_dist_break;
  double follower_dot_threshold;
};

class FormationSpeedController {

public:

  FormationSpeedController(const std::string & my_name, const FormationSpeedControlParams &params, std::shared_ptr<avt_341::node::NodeProxy> node_proxy);

  double getSpeedFactor(const FormationDefinition* formation_def, std::map<std::string, avt_341::msg::Odometry> & formation_poses);
  void visualizeSpeedIndicators(double speed_factor, double delta_pos, const avt_341::msg::PoseStamped &target_pose,
                                const avt_341::msg::Point &current_pos, bool heading_filter_on, bool follower_dist_break_on);

private:
  avt_341::msg::PoseStamped getFollowerTargetPose(avt_341::msg::Odometry leader_odom, avt_341::msg::FollowerStatus status);

  std::string my_name_;
  FormationSpeedControlParams fsc_params_;
  std::shared_ptr<avt_341::node::NodeProxy> node_proxy_;
  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Marker>> marker_pub_ = nullptr;

};

}
}

#endif //AVT_341_FORMATION_SPEED_CONTROL_H
