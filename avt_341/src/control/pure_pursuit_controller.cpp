#include "avt_341/control/pure_pursuit_controller.h"

namespace avt_341 {
namespace control{

PurePursuitController::PurePursuitController() {
	skid_steered_ = false;

	//default wheelbase and steer angle
	// set to MRZR values
	wheelbase_ = 2.731f; // meters
	max_steering_angle_ = 0.69f; //39.5 degrees
	// max_stable_speed_ = 35.0f; //5.0;

	// tunable parameters
	min_lookahead_ = 3.0f;
	max_lookahead_ = 10.0f;
	k_ = 1.2f;
	throttle_coeff_ = 1.0f;

	//vehicle state parameters
	veh_x_ = 0.0f;
	veh_y_ = 0.0f;
	veh_speed_ = 0.0f;
	vx_ = 0.0f;
	vy_ = 0.0f;

	k_theta_ = 1.0f;
	kx_ = 1.0f;
	ky_ = 1.0f;

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

void PurePursuitController::SetVehicleSpeed(float speed){
	veh_speed_ = speed;
	vx_ = cosf(veh_heading_)*veh_speed_;
	vy_ = sinf(veh_heading_)*veh_speed_;
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
	float path_length = utils::length(path[np - 1] - currpos);
	float lookahead = k_ * veh_speed_;

	if (lookahead > max_lookahead_)lookahead = max_lookahead_;
	if (lookahead < min_lookahead_)lookahead = min_lookahead_;
	// if (lookahead > path_length)lookahead = path_length - 0.01;


	utils::vec2 lookahead_pos(veh_x_ + lookahead*cosf(veh_heading_), veh_y_ + lookahead*sinf(veh_heading_));
	utils::vec2 veh_pos(veh_x_, veh_y_);


	float min_dist = 1.0E9f;
	int min_idx = 0;

	utils::vec2 diff_vec;
	for (int i = 0; i < np - 2; i++) {
		diff_vec = path[i] - lookahead_pos;
		float d0 = sqrt(utils::dot(diff_vec, diff_vec));
		if (d0 < min_dist) {
			min_dist = d0;
			min_idx = i;
		}
	}


	float min_dist2 = 1.0E9f;
	int min_idx2 = 0;

	utils::vec2 diff_vec2;
	for (int i = 1; i < np - 2; i++) {
		diff_vec2 = path[i] - veh_pos;
		float d0 = sqrt(utils::dot(diff_vec2, diff_vec2));
		if (d0 < min_dist2) {
			min_dist2 = d0;
			min_idx2 = i;
		}
	}

	utils::vec2 dirc1;
	dirc1 = path[min_idx2] - path[min_idx2 -1];
	utils::vec2 dirc2;
	dirc2 = path[min_idx2 + 1] - path[min_idx2];


	

 	float dpsi = asin((dirc1.x * dirc2.y - dirc1.y*dirc2.x)/(sqrt(utils::dot(dirc1, dirc1))* sqrt(utils::dot(dirc2, dirc2))));//acos((utils::dot(dirc1, dirc2))/( utils::dot(dirc1, dirc1)* utils::dot(dirc2, dirc2) ));
	float dlength = sqrt(utils::dot(dirc2, dirc2));
	float desired_sa = atan(2.5*(dpsi/dlength));


	utils::vec2 p2l;
	utils::vec2 p2p;

	p2l = lookahead_pos - path[min_idx];
	p2p = path[min_idx+1] - path[min_idx];

	float err = p2p.x * p2l.y - p2p.y*p2l.x; //p2p x p2l

	goal = path[min_idx];

	float target_speed = desired_speed_;

	dc.linear.x = 0.0;
	dc.angular.z = 0.0;
	dc.linear.y = 0.0;

	//determine the desired normalized steering angle
	float sangle;
	float delta_angle = 0.001;
	float derr = err - err_last_;
	float err_accum_ = err + err_accum_;
	sangle = pursuit_kp_ * desired_sa - pursuit_ki_ * err - pursuit_kd_ * derr;
	err_last_ = err;

	if (fabs(sangle - steer_cur_)> delta_angle) {
		sangle = steer_cur_ + (sangle - steer_cur_)/(fabs((sangle - steer_cur_))+0.0001)*delta_angle;
	}
	steer_cur_ = sangle;

	sangle = sangle / max_steering_angle_;
	sangle = std::min(1.0f, sangle);
	sangle = std::max(-1.0f, sangle);
	dc.angular.z = sangle;

	//Use the speed controller to get throttle/braking
	//adjust the target speed so you back off during hard turns
	float adj_speed = target_speed * exp(-0.69*pow(fabs(dc.angular.z), 4.0f));
	speed_controller_.SetSetpoint(adj_speed);
	float throttle = speed_controller_.GetControlVariable(veh_speed_, 0.01f);
	if (throttle < 0.0f) { //braking
		dc.linear.x = 0.0f;
		dc.linear.y = std::max(-1.0f, throttle);
	}
	else {
		dc.linear.y = 0.0f;
		dc.linear.x = std::min(1.0f, throttle);
	}

	dc.linear.x = throttle_coeff_*dc.linear.x;
		// dc = GetDcSkid(to_goal.x, to_goal.y, dtheta);
		// dc = GetDcAckermann(alpha, lookahead, curr_dir, target_speed);
	return dc;
}



avt_341::msg::Twist PurePursuitController::GetDcSkid(float dx, float dy, float dtheta){
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

	float vr = desired_speed_;
	float ct = cosf(veh_heading_);
	float st = sinf(veh_heading_);
	float xe = ct*dx + st*dy;
	float ye = -st*dx + ct*dy;

	float v = vr*cosf(dtheta) + kx_*xe;
	float w = current_angular_velocity_ + vr*(ky_*ye + k_theta_*sinf(dtheta));

	dc.linear.x = v*ct;
	dc.linear.y = v*st;
	dc.angular.z = w;

	return dc;
}

avt_341::msg::Twist PurePursuitController::GetDcAckermann(float alpha, float lookahead, utils::vec2 curr_dir, float target_speed){
	avt_341::msg::Twist dc;
	dc.linear.x = 0.0;
	dc.angular.z = 0.0;
	dc.linear.y = 0.0;

	//determine the desired normalized steering angle
	float sangle = (float)atan2(2 * wheelbase_*sin(alpha), lookahead);
	sangle = sangle / max_steering_angle_;
	sangle = std::min(1.0f, sangle);
	sangle = std::max(-1.0f, sangle);
	dc.angular.z = sangle;

	//Use the speed controller to get throttle/braking
	//addjust the target speed so you back off during hard turns
	float adj_speed = target_speed * exp(-0.69*pow(fabs(dc.angular.z), 4.0f));
	speed_controller_.SetSetpoint(adj_speed);
	float vdot = vx_*curr_dir.x + vy_*curr_dir.y;
	float throttle = speed_controller_.GetControlVariable(veh_speed_, 0.01f);
	//float throttle = speed_controller_.GetControlVariable(vdot, 0.01f);
	if (throttle < 0.0f) { //braking
		dc.linear.x = 0.0f;
		dc.linear.y = 0.0; //std::max(-1.0f, throttle);
	}
	else {
		dc.linear.y = 0.0f;
		dc.linear.x = std::min(1.0f, throttle);
	}

	dc.linear.x = throttle_coeff_*dc.linear.x;

	return dc;
} // GetDcAcerman

} // namespace control
} // namespace avt_341
