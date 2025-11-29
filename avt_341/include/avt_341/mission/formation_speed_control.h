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
      double max_speed_factor;
      bool debug_visualize;
      bool follower_obt_stop;
      double follower_dist_break;
      double follower_dot_threshold;
      double follower_dot_range;
    };

    class FormationSpeedController {

    public:
      explicit FormationSpeedController(const std::string & veh_name, const FormationSpeedControlParams &params);

      virtual double
      getSpeedFactor(const FormationDefinition *formation_def, const avt_341::msg::PoseStamped &terminal_pose,
                     std::map<std::string, avt_341::msg::Odometry> &formation_poses) = 0;

      virtual void
      clearVisualization();

    protected:
      const FormationSpeedControlParams & fsc_params_;
      std::string my_name_;

      avt_341::msg::PoseStamped
      getFollowerTargetPose(avt_341::msg::Odometry leader_odom, avt_341::msg::FollowerStatus status);
    };

    class NullFormationSpeedController : public FormationSpeedController {
    public:
      explicit NullFormationSpeedController(const std::string & veh_name, const FormationSpeedControlParams &params);

      double getSpeedFactor(const FormationDefinition *formation_def, const avt_341::msg::PoseStamped &terminal_pose,
                            std::map<std::string, avt_341::msg::Odometry> &formation_poses) override;
    };

    class SlowLeaderFormationSpeedController : public FormationSpeedController {

    public:
      SlowLeaderFormationSpeedController(const std::string &my_name, const FormationSpeedControlParams &params,
                                         std::shared_ptr<avt_341::node::NodeProxy> node_proxy);

      double getSpeedFactor(const FormationDefinition *formation_def, const avt_341::msg::PoseStamped &terminal_pose,
                            std::map<std::string, avt_341::msg::Odometry> &formation_poses) override;

      void visualizeSpeedIndicators(double speed_factor, double delta_pos, const avt_341::msg::PoseStamped &target_pose,
                                    const avt_341::msg::Point &current_pos, bool heading_filter_on,
                                    bool follower_dist_break_on);
      void clearVisualization() override;

    private:
      std::shared_ptr<avt_341::node::NodeProxy> node_proxy_;
      std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Marker>> marker_pub_ = nullptr;
      bool has_visualized_ = false;
    };

    class SpeedUpFollowerFormationSpeedController : public FormationSpeedController {

    public:
      explicit SpeedUpFollowerFormationSpeedController(const std::string & veh_name, const FormationSpeedControlParams &params);

      double getSpeedFactor(const FormationDefinition *formation_def, const avt_341::msg::PoseStamped &terminal_pose,
                            std::map<std::string, avt_341::msg::Odometry> &formation_poses) override;
    };

    std::shared_ptr<FormationSpeedController>
    createFormationSpeedController(const std::string &fsc_type, const std::string &veh_name,
                                   const FormationSpeedControlParams &params,
                                   std::shared_ptr<avt_341::node::NodeProxy> node_proxy);

  }
}

#endif //AVT_341_FORMATION_SPEED_CONTROL_H
