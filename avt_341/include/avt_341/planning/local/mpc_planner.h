///**
// * \class MpcPlanner
// *
// * Model Predictive Control planner. Abstraction layer for the AUMPC planner created by Dario Sirangelo.
// * AUPMC is based on the Julia implementation of the AVT-341 MPC Local Planner Plugin
// * ref: https://github.com/TulgaErsal/AVT-341-MPC
// *
// * \author Marius Thoresen
// *
// * \date 4/15/2023
//*/

#ifndef MPC_PLANNER_H
#define MPC_PLANNER_H

#include <utility>
#include "avt_341/node/ros_types.h"
#include "avt_341/avt_341_utils.h"
#include "avt_341/planning/local/mpc_planner_solver.h"

namespace avt_341 {
namespace planning {

namespace MpcUtils {

inline double quaternionMsg2Yaw(const avt_341::msg::Quaternion& orientation_msg) {
    avt_341::msg_tf::Quaternion orientation(orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w);
    // NOTE: getRPY() expects a double, hence we can cast back to float when setting the state.
    double roll, pitch, yaw;
    avt_341::msg_tf::Matrix3x3 rotation(orientation);
    rotation.getRPY(roll, pitch, yaw);
    return yaw;
}

inline avt_341::msg::Quaternion yaw2quaternionMsg(double yaw) {
    avt_341::msg_tf::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    avt_341::msg::Quaternion orientation_msg;
    orientation_msg.x = orientation.getX();
    orientation_msg.y = orientation.getY();
    orientation_msg.z = orientation.getZ();
    orientation_msg.w = orientation.getW();
    return orientation_msg;
}
}


struct Control {
    double speed_longitudinal;
    double steering_angle;
};


class MpcPlanner {
public:
    MpcPlanner() = default;
    void initialize();

    msg::Path getPlannedPathRos();

    double getControlJerk();

    double getControlSteeringRate();

    Control getControls();

    std::vector<Control> getControlsVector();

    void plan();

    bool ready() const { return initialized_ && has_state_ && has_goal_; }

    void setGoal(double goal_x, double goal_y);

    void setGoal(double goal_x, double goal_y, double goal_yaw);

    void setState(double x,
                  double y,
                  double yaw,
                  double long_speed,
                  double lateral_speed,
                  double yaw_rate,
                  double steering_angle,
                  double long_acceleration,
                  double tan_alpha_f,
                  double tan_alpha_r);

    void setObstacles(std::vector<float> x,
                      std::vector<float> y,
                      std::vector<float> r);

    void setAxleDistanceFront(double distance) { d_a_ = distance; }

    void setAxleDistanceRear(double distance) { d_b_ = distance; }

    void setBounds();

    void setFrontAxleCombinedStiffness(double stiffness) { c_alpha_f_ = stiffness; }

    void setLateralSpeedMax(double speed) { lateral_speed_max_ = speed; }

    void setLateralSpeedMin(double speed) { lateral_speed_min_ = speed; }

    void setLongAccelerationMax(double acceleration) { long_acceleration_max_ = acceleration; }

    void setLongAccelerationMin(double acceleration) { long_acceleration_min_ = acceleration; }

    void setLongJerkMax(double jerk) { long_jerk_max_ = jerk; }

    void setLongJerkMin(double jerk) { long_jerk_min_ = jerk; }

    void setLongSpeedMax(double speed) { long_speed_max_ = speed; }

    void setLongSpeedMin(double speed) { long_speed_min_ = speed; }

    void setMpcWeights();

    void setRearAxleCombinedStiffness(double stiffness) { c_alpha_r_ = stiffness; }

    void setRelaxationLength(double length) { b_ = length; }

    void setSolverMaxIterations(int iterations) { solver_max_iterations_ = iterations; }

    void setSolverMaxWallTime(double time) { solver_max_wall_time_ = time; }

    void setSolverName(std::string name) { solver_name_ = std::move(name); }

    void setSolverOptions();

    void setSolverPrintLevel(int level) { solver_print_level_ = level; }

    void setSolverTimeSpan(double span) { solver_time_span_ = span; }

    void setSolverTimeStep(double step) { solver_time_step_ = step; }

    void setSolverTolerance(double tolerance) { solver_tolerance_ = tolerance; }

    void setSteeringAngleMax(double steering_angle) { steering_angle_max_ = steering_angle; }

    void setSteeringAngleMin(double steering_angle) { steering_angle_min_ = steering_angle; }

    void setSteeringRateMax(double steering_rate) { steering_rate_max_ = steering_rate; }

    void setSteeringRateMin(double steering_rate) { steering_rate_min_ = steering_rate; }

    void setVehicleMass(double mass) { m_ = mass; }

    void setWeights(double w_goal, double w_steering_rate, double w_obstacle_term, double w_long_jerk);

    void setYawInertia(double inertia) { i_zz_ = inertia; }

    void setYawRateMax(double yaw_rate) { yaw_rate_max_ = yaw_rate; }

    void setYawRateMin(double yaw_rate) { yaw_rate_min_ = yaw_rate; }
//  Unnecessary constraints?
private:

    std::unique_ptr<avt_341::MPC> mpc_;
    bool initialized_{false};
    bool has_state_{false};
    bool has_goal_{false};

    // state
    double x_{0.0};
    double y_{0.0};
    double yaw_{0.0};
    double long_speed_{0.0};
    double lateral_speed_{0.0};
    double yaw_rate_{0.0};
    double steering_angle_{0.0};
    double long_acceleration_{0.0};

    // goal
    double goal_x_{0.0};
    double goal_y_{0.0};
    double goal_yaw_{0.0};

    //
    float d_a_{0.0};
    float d_b_{0.0};
    float m_{0.0};
    float c_alpha_f_{0.0};
    float c_alpha_r_{0.0};
    float i_zz_{0.0};
    float b_{0.0};

// Bounds
    double steering_angle_min_{0.0};
    double steering_angle_max_{0.0};
    double steering_rate_min_{0.0};
    double steering_rate_max_{0.0};
    double long_jerk_min_{0.0};
    double long_jerk_max_{0.0};
    double long_acceleration_min_{0.0};
    double long_acceleration_max_{0.0};
    double long_speed_min_{0.0};
    double long_speed_max_{0.0};
    double lateral_speed_min_{0.0};
    double lateral_speed_max_{0.0};
    double yaw_rate_min_{0.0};
    double yaw_rate_max_{0.0};

// Weights
    double w_obstacle_term_{0.0};
    double w_goal_{0.0};
    double w_long_jerk_{0.0};
    double w_steering_rate_{0.0};

// Solver options
    std::string solver_name_;
    int solver_max_iterations_{0};
    double solver_time_span_{0};
    double solver_time_step_{0};
    double solver_max_wall_time_{0};
    double solver_tolerance_{0};
    int solver_print_level_{0};
};

} // end namespace planning
} // end namespace avt_341

#endif // define MPC_PLANNER_H