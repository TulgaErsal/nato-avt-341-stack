/**
* \class PurePursuitController
*
* A pure-pursuit vehicle control that follows an input vehicle
* trajectory in 2D space. 
* 
* See "Implementation of the Pure Pursuit Path Tracking Algorithm"
* by Craig Coulter, CMU-RI-TR-92-01
* 
* and
* 
* "Automatic Steering Methods for Autonomous Automobile Path Tracking"
* by Jarrod M. Snider, CMU-RI-TR-09-08
*
* \author Chris Goodin
*
* \date 8/31/2020
*/
#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include "avt_341_nav/control/pid_controller.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "avt_341_nav/core/math_dto.hpp"

namespace avt_341_nav {
namespace control{

class PurePursuitController {
public:
	/// Create a controller
	PurePursuitController();

	/**
	* Calculate a driving command based on a trajectory
	* The first point on the trajectory must be the current
	* vehicle state.
	* \param traj The desired trajectory
	*/
	geometry_msgs::msg::Twist GetDcFromTraj(nav_msgs::msg::Path traj, core::vec2 & goal);

	/**
	* Set the wheelbase of the vehicle in meters
	* \param wb Wheelbase to set
	*/
	void SetWheelbase(double wb) { wheelbase_ = wb; }

	/**
	* Set the max steering angle of the vehicle in radians
	* \param st Max steering angle
	*/
	void SetMaxSteering(double st) { max_steering_angle_ = st; }

	/** 
	* Set the minimum look-ahead distance of the planner, in meters
	* \param min_la The minimum look-ahead distance
	*/
	void SetMinLookAhead(double min_la) { min_lookahead_ = min_la; }

	/**
	* Set the maximum look-ahead distance of the planner, in meters
	* \param max_la The maximum look-ahead distance
	*/
	void SetMaxLookAhead(double max_la) { max_lookahead_ = max_la; }

	/** 
	* Set the gain factor on the steering controller
	* \param k Desired gain factor
	* \param pursuit_k proportional gain multiplying the desired steering angle
	* \param pursuit_kp proportional gain multiplying the error
	* \param pursuit_kd derivative gain multiplying the delta error
	*/
	void SetSteeringParams(double k, double pursuit_k, double pursuit_kp, double pursuit_kd) {
		k_ = k; 
		pursuit_k_ = pursuit_k;
		pursuit_kp_ = pursuit_kp;
		pursuit_kd_ = pursuit_kd;
	}

	/**
	* Set the maximum allowed speed of the vehicle
	* Contoller will limit throttle to stay below this speed
	* \param speed Maximum desired speed in m/s
	*/
	void SetMaxStableSpeed(double speed) { max_stable_speed_ = speed; }

	/**
	* Set the desired speed of the vehicle in m/s
	* \param speed The desired speed
	*/
	void SetDesiredSpeed(double speed) {
		desired_speed_ = speed;
		speed_controller_.SetSetpoint(speed);
	}

	/**
	* Set the coefficients of the PID speed controller
	* \param kp Proportional coefficient
	* \param ki Integral coefficient
	* \param kd Derivative coefficient
	*/
	void SetSpeedControllerParams(double kp, double ki, double kd) {
		speed_controller_.SetKp(kp);
		speed_controller_.SetKi(ki);
		speed_controller_.SetKd(kd);
	}

	/**
	* Set the current vehicle position in local ENU
	* \param x Current x-coordinate in ENU
	* \param y Current y-coordinate in ENU
	*/
	void SetVehiclePosition(double x, double y) {
		veh_x_ = x;
		veh_y_ = y;
	}

	/**
	* Set the current vehicle speed in m/s
	* \param speed The current vehicle speed 
	*/
	void SetVehicleSpeed(double speed);

	/**
	* Set the current vehicle heading in radians
	* \param heading The current vehicle heading
	*/
	void SetVehicleOrientation(double heading) {
		veh_heading_ = heading;
	}

	/**
	 *  Set the vehicle position, orientation and speed
	 * \param state The vehicle state
	 */
	void SetVehicleState(nav_msgs::msg::Odometry state);

	/**
	* A scale factor for the output throttle.
	* Set to one by default. Shouldn't be changed under most circumstances.
	* The PID parameters should control the speed effectively
	*/
	void SetThrottleCoeff(double tc){ throttle_coeff_ = tc; }

	/**
	* Call this to turn skid-steering controller on or off
	* By default, skid_steered is false and ackerman steering is used
	*/
	void IsSkidSteered(bool skid_steered){ skid_steered_ = skid_steered; }

	/**
	* Set the parameters for the skid steering control model.
	* Kx = Ky = Kl
	* Ktheta = kt
	* See "A Stable Tracking Control Method for a Non-Holonomic Mobile Robot"
	*/
	void SetSkidSteerParams(double kl, double kt){
		kx_ = kl;
		ky_ = kl;
		k_theta_ = kt;
	}

	/// Get a pointer to the PID speed controller
	PidController *GetPidSpeedController(){ return &speed_controller_; }

private:
	bool skid_steered_;
	geometry_msgs::msg::Twist GetDcAckermann(double alpha, double lookahead, core::vec2 curr_dir, double target_speed);
	geometry_msgs::msg::Twist GetDcSkid(double dx, double dy, double dtheta);

	// steering parameters for the skid steered model
	double kx_;
	double ky_;
	double k_theta_;

	double wheelbase_; //meters
	double max_steering_angle_; //radians
	double min_lookahead_; //meters
	double max_lookahead_; //meters
	double k_; //unitless
	double pursuit_k_;
	double pursuit_kp_;
	double pursuit_kd_;
	double desired_speed_; // m/s
	double max_stable_speed_;
	double throttle_coeff_;
	PidController speed_controller_;

	//current vehicle state info
	double veh_x_;
	double veh_y_;
	double veh_heading_;
	double veh_speed_;
	double vx_;
	double vy_;
	double steer_cur_;
	double err_last_;
	double err_accum_;
	double current_angular_velocity_;
};

} // namespace control
} // namespace avt_341_nav

#endif
