#include "avt_341/control/pure_pursuit_controller.h"

namespace avt_341 {
namespace control{

PurePursuitController::PurePursuitController() {
	skid_steered_ = false;

	//default wheelbase and steer angle
	// set to MRZR values
	wheelbase_ = 2.731; // meters
	max_steering_angle_ = 0.69; //39.5 degrees
	// max_stable_speed_ = 35.0; //5.0;

	// tunable parameters
	min_lookahead_ = 3.0;
	max_lookahead_ = 10.0;
	k_ = 1.2;
	throttle_coeff_ = 1.0;

	//vehicle state parameters
	veh_x_ = 0.0;
	veh_y_ = 0.0;
	veh_speed_ = 0.0;
	vx_ = 0.0;
	vy_ = 0.0;

	k_theta_ = 1.0;
	kx_ = 1.0;
	ky_ = 1.0;

	steer_cur_ = 0.0;
	err_last_ = 0.0;
	err_accum_ = 0.0;
}

void PurePursuitController::SetVehicleState(avt_341::msg::Odometry state){
// Set the current state of the vehicle, which should be the first pose in the path
	veh_x_ = state.pose.pose.position.x;
	veh_y_ = state.pose.pose.position.y;
	vx_ = state.twist.twist.linear.x;
	vy_ = state.twist.twist.linear.y;
	current_angular_velocity_ = state.twist.twist.angular.z;
	veh_speed_ = sqrt(vx_*vx_ + vy_*vy_);
	veh_heading_ = utils::GetHeadingFromOrientation(state.pose.pose.orientation);
}

void PurePursuitController::SetVehicleSpeed(double speed){
	veh_speed_ = speed;
	vx_ = std::cos(veh_heading_)*veh_speed_;
	vy_ = std::sin(veh_heading_)*veh_speed_;
}


avt_341::msg::Twist PurePursuitController::GetDcFromTraj(avt_341::msg::Path traj, utils::vec2 & goal) {
	//initialize the driving command
  	avt_341::msg::Twist dc;

	//make sure the path contains some points
	int np = traj.poses.size();

	if (np < 2) return dc;

	// extract the path that the vehicle needs to follow
	std::vector<utils::vec2> path;

	//populate the desired path
	path.resize(np);
	for (int i = 0; i < np; i++) {
		path[i] = utils::vec2(traj.poses[i].pose.position.x, traj.poses[i].pose.position.y);
	}

	//calculate the lookahead distance based on current speed
	utils::vec2 currpos(veh_x_, veh_y_);
	double path_length = utils::length(path[np - 1] - currpos);
	double lookahead = k_ * veh_speed_;

	if (lookahead > max_lookahead_)lookahead = max_lookahead_;
	if (lookahead < min_lookahead_)lookahead = min_lookahead_;
	// if (lookahead > path_length)lookahead = path_length - 0.01;


	utils::vec2 lookahead_pos(veh_x_ + lookahead*std::cos(veh_heading_), veh_y_ + lookahead*std::sin(veh_heading_));
	utils::vec2 veh_pos(veh_x_, veh_y_);


	double min_dist = 1.0E9;
	int min_idx = 0;

	utils::vec2 diff_vec;
	for (int i = 0; i < np - 2; i++) {
		diff_vec = path[i] - lookahead_pos;
		double d0 = std::sqrt(utils::dot(diff_vec, diff_vec));
		if (d0 < min_dist) {
			min_dist = d0;
			min_idx = i;
		}
	}


	double min_dist2 = 1.0E9;
	int min_idx2 = 0;

	utils::vec2 diff_vec2;
	for (int i = 1; i < np - 2; i++) {
		diff_vec2 = path[i] - veh_pos;
		double d0 = std::sqrt(utils::dot(diff_vec2, diff_vec2));
		if (d0 < min_dist2) {
			min_dist2 = d0;
			min_idx2 = i;
		}
	}

	utils::vec2 dirc1;
	dirc1 = path[min_idx2] - path[min_idx2 -1];
	utils::vec2 dirc2;
	dirc2 = path[min_idx2 + 1] - path[min_idx2];


	

	double dpsi =
	    std::asin((dirc1.x * dirc2.y - dirc1.y * dirc2.x) /
	              (std::sqrt(utils::dot(dirc1, dirc1)) *
	               std::sqrt(utils::dot(dirc2, dirc2))));
	double dlength = std::sqrt(utils::dot(dirc2, dirc2));
	double desired_sa = std::atan(2.5*(dpsi/dlength));


	utils::vec2 p2l;
	utils::vec2 p2p;

	p2l = lookahead_pos - path[min_idx];
	p2p = path[min_idx+1] - path[min_idx];

	double err = p2p.x * p2l.y - p2p.y*p2l.x; //p2p x p2l

	goal = path[min_idx];

	double target_speed = desired_speed_;

	dc.linear.x = 0.0;
	dc.angular.z = 0.0;
	dc.linear.y = 0.0;

	//determine the desired normalized steering angle
	double sangle;
	double delta_angle = 0.001;
	double derr = err - err_last_;
	err_accum_ += err;
	sangle = pursuit_k_ * desired_sa - pursuit_kp_ * err - pursuit_kd_ * derr;
	err_last_ = err;

	if (fabs(sangle - steer_cur_)> delta_angle) {
		sangle = steer_cur_ + (sangle - steer_cur_)/(fabs((sangle - steer_cur_))+0.0001)*delta_angle;
	}
	steer_cur_ = sangle;

	sangle = sangle / max_steering_angle_;
	sangle = std::min(1.0, sangle);
	sangle = std::max(-1.0, sangle);
	dc.angular.z = sangle;

	//Use the speed controller to get throttle/braking
	//adjust the target speed so you back off during hard turns
	double adj_speed = target_speed * std::exp(-0.69*std::pow(std::fabs(dc.angular.z), 4.0));
	speed_controller_.SetSetpoint(adj_speed);
	double throttle = speed_controller_.GetControlVariable(veh_speed_, 0.01);
	if (throttle < 0.0) { //braking
		dc.linear.x = 0.0;
		dc.linear.y = std::max(-1.0, throttle);
	}
	else {
		dc.linear.y = 0.0;
		dc.linear.x = std::min(1.0, throttle);
	}

	dc.linear.x = throttle_coeff_*dc.linear.x;
		// dc = GetDcSkid(to_goal.x, to_goal.y, dtheta);
		// dc = GetDcAckermann(alpha, lookahead, curr_dir, target_speed);
	return dc;
}



avt_341::msg::Twist PurePursuitController::GetDcSkid(double dx, double dy, double dtheta){
	// The skid steer algorithm is taken from 
	// A Stable Tracking Control Method for a Non-Holonomic Mobile Robot
	// Yutaka Kanayam, 1991
	// Proceedings of IROS 91
	// NOTE: The output is the typical cmd_vel message for a mobile robot where
	// the velocities are true velocities, not throttle-steering-brake commands like 
	// the controller for the Ackerman vehicle
	avt_341::msg::Twist dc;
	dc.linear.x = 0.0;
	dc.linear.y = 0.0;
	dc.linear.z = 0.0;
	dc.angular.x = 0.0;
	dc.angular.y = 0.0;
	dc.angular.z = 0.0;

	double vr = desired_speed_;
	double ct = std::cos(veh_heading_);
	double st = std::sin(veh_heading_);
	double xe = ct*dx + st*dy;
	double ye = -st*dx + ct*dy;

	double v = vr*std::cos(dtheta) + kx_*xe;
	double w = current_angular_velocity_ + vr*(ky_*ye + k_theta_*std::sin(dtheta));

	dc.linear.x = v*ct;
	dc.linear.y = v*st;
	dc.angular.z = w;

	return dc;
}

avt_341::msg::Twist PurePursuitController::GetDcAckermann(double alpha, double lookahead, utils::vec2 curr_dir, double target_speed){
	avt_341::msg::Twist dc;
	dc.linear.x = 0.0;
	dc.angular.z = 0.0;
	dc.linear.y = 0.0;

	//determine the desired normalized steering angle
	double sangle = std::atan2(2 * wheelbase_*std::sin(alpha), lookahead);
	sangle = sangle / max_steering_angle_;
	sangle = std::min(1.0, sangle);
	sangle = std::max(-1.0, sangle);
	dc.angular.z = sangle;

	//Use the speed controller to get throttle/braking
	//addjust the target speed so you back off during hard turns
	double adj_speed = target_speed * std::exp(-0.69*std::pow(std::fabs(dc.angular.z), 4.0));
	speed_controller_.SetSetpoint(adj_speed);
	double vdot = vx_*curr_dir.x + vy_*curr_dir.y;
	double throttle = speed_controller_.GetControlVariable(veh_speed_, 0.01);
	// double throttle = speed_controller_.GetControlVariable(vdot, 0.01);
	if (throttle < 0.0) { //braking
		dc.linear.x = 0.0;
		dc.linear.y = 0.0; // std::max(-1.0, throttle);
	}
	else {
		dc.linear.y = 0.0;
		dc.linear.x = std::min(1.0, throttle);
	}

	dc.linear.x = throttle_coeff_*dc.linear.x;

	return dc;
} // GetDcAcerman

} // namespace control
} // namespace avt_341
