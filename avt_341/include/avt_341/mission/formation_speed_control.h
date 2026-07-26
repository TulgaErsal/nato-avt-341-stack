#ifndef AVT_341_FORMATION_SPEED_CONTROL_H
#define AVT_341_FORMATION_SPEED_CONTROL_H

#include "avt_341_msgs/msg/follower_status.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341/node/tf_interface.h"
#include "avt_341/avt_341_utils.h"
#include "avt_341/mission/formation_utils.h"
#include "avt_341/mission/formation_definition.h"

namespace avt_341 {
  namespace mission {

    using FormationSpeedControlParams = MissionManagerParams::Fsc;

    class FormationSpeedController {

    public:
      explicit FormationSpeedController(const std::string & veh_name, const FormationSpeedControlParams &params);

      // TODO: Needs to be cleaned up. speed_setpoint just passed in for later visualization.
      // visualizeSpeedIndicators should be on base class and number of parameters needs to be reduced.
      virtual double
      getSpeedFactor(const FormationDefinition *formation_def, const geometry_msgs::msg::PoseStamped &terminal_pose,
                     std::map<std::string, nav_msgs::msg::Odometry> &formation_poses, double speed_setpoint) = 0;

      virtual void
      clearVisualization();

    protected:
      FormationSpeedControlParams fsc_params_;
      std::string my_name_;

      geometry_msgs::msg::PoseStamped
      getFollowerTargetPose(nav_msgs::msg::Odometry leader_odom, avt_341_msgs::msg::FollowerStatus status);
    };

    class NullFormationSpeedController : public FormationSpeedController {
    public:
      explicit NullFormationSpeedController(const std::string & veh_name, const FormationSpeedControlParams &params);

      double getSpeedFactor(const FormationDefinition *formation_def, const geometry_msgs::msg::PoseStamped &terminal_pose,
                            std::map<std::string, nav_msgs::msg::Odometry> &formation_poses, double speed_setpoint) override;
    };

    class SlowLeaderFormationSpeedController : public FormationSpeedController {

    public:
      SlowLeaderFormationSpeedController(const std::string &my_name, const FormationSpeedControlParams &params,
                                         rclcpp::Node::SharedPtr node,
                                         std::shared_ptr<avt_341::node::TfInterface> tf);

      double getSpeedFactor(const FormationDefinition *formation_def, const geometry_msgs::msg::PoseStamped &terminal_pose,
                            std::map<std::string, nav_msgs::msg::Odometry> &formation_poses, double speed_setpoint) override;

      void visualizeSpeedIndicators(double speed_factor, double delta_pos, const geometry_msgs::msg::PoseStamped &target_pose,
                                    const geometry_msgs::msg::Point &current_pos, bool heading_filter_on,
                                    bool follower_dist_break_on, double speed_setpoint);
      void clearVisualization() override;

    private:
      rclcpp::Node::SharedPtr node_;
      std::shared_ptr<avt_341::node::TfInterface> tf_;
      std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> marker_pub_ = nullptr;
      bool has_visualized_ = false;
    };

    class SpeedUpFollowerFormationSpeedController : public FormationSpeedController {

    public:
      explicit SpeedUpFollowerFormationSpeedController(const std::string & veh_name, const FormationSpeedControlParams &params);

      double getSpeedFactor(const FormationDefinition *formation_def, const geometry_msgs::msg::PoseStamped &terminal_pose,
                            std::map<std::string, nav_msgs::msg::Odometry> &formation_poses, double speed_setpoint) override;
    };

    std::shared_ptr<FormationSpeedController>
    createFormationSpeedController(const std::string &veh_name,
                                   const FormationSpeedControlParams &params,
                                   rclcpp::Node::SharedPtr node,
                                   std::shared_ptr<avt_341::node::TfInterface> tf);

  }
}

#endif //AVT_341_FORMATION_SPEED_CONTROL_H
