/**
 C++ implementation of the goal_point_processor.jl found in the MPC planner stack.
*/
#include <rclcpp/rclcpp.hpp>
#include "avt_341_msgs/msg/follower_status.hpp"
#include "avt_341_msgs/msg/nav_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "avt_341/avt_341_utils.h"
#include "avt_341/core/dto_conversion.h"
#include <avt_341/mpc_local_planner_params_service.hpp>
#include <memory>
// Globals
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pub_steering_angle;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pub_steering_rate;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pub_time_gap;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> pub_scenario_tag;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> pub_segment_start;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> pub_segment_end;
std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PointStamped>> pub_goalPoint;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pub_desiredHeading;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> pub_goalPointIsEnd;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pub_finalHeading;
rclcpp::Node::SharedPtr n;
nav_msgs::msg::Path global_path_input;
std_msgs::msg::Float64MultiArray veh_input;
std_msgs::msg::Float64 speedSetpoint_input;
rclcpp::Time veh_input_stamp, last_veh_stamp, init_time;
avt_341_msgs::msg::FollowerStatus follower_status_input;
double last_steer_angle = 0.0;
rclcpp::Time last_steer_time;
bool steer_initialized = false;

float speedSetpoint, desiredHeading, finalHeading;
float autoFinalHeading;
bool finalHeadingSet;
bool autoFinalHeadingSet;
bool priorUseLeader, turningAround, goal_set;
bool alwaysPubGoal;
bool useAutoFinalHeading;
int priorIndex, priorPathLength;
avt_341::utils::vec2 goal;
bool goal_is_end = false;

avt_341::params::mpc_local_planner::Params params;

void callback_global_path(nav_msgs::msg::Path::SharedPtr global_path) {
    global_path_input = *global_path;

    // Auto-compute final heading from the direction of the last two waypoints.
    if (global_path->poses.size() >= 2) {
        const auto& p1 = global_path->poses[global_path->poses.size() - 2].pose.position;
        const auto& p2 = global_path->poses[global_path->poses.size() - 1].pose.position;
        autoFinalHeading = static_cast<float>(std::atan2(p2.y - p1.y, p2.x - p1.x));
        autoFinalHeadingSet = true;
    }

    std_msgs::msg::String scenario_msg;
    scenario_msg.data = "path_update";
    pub_scenario_tag->publish(scenario_msg);

    std_msgs::msg::Bool seg_start_msg;
    seg_start_msg.data = true;
    pub_segment_start->publish(seg_start_msg);

    std_msgs::msg::Bool seg_end_msg;
    seg_end_msg.data = true;
    pub_segment_end->publish(seg_end_msg);
}

void callback_veh(std_msgs::msg::Float64MultiArray::SharedPtr veh) {
    veh_input = *veh;
    veh_input_stamp = n->now();
}

void callback_speedSetpoint(std_msgs::msg::Float64::SharedPtr ss) {
    speedSetpoint_input = *ss;
}

void callback_follower_status(avt_341_msgs::msg::FollowerStatus::SharedPtr follower_status) {
    follower_status_input = *follower_status;
}

void callback_gp_state(avt_341_msgs::msg::NavState::SharedPtr msg) {

	if (!avt_341::core::HasActiveGoal(msg)) {
		return;
	}
	finalHeading = avt_341::utils::GetHeadingFromOrientation(msg->goal.pose.orientation);
    finalHeadingSet = avt_341::utils::UseGoalOrientation(msg->goal);
}

void publishSteeringRate(double current_angle) {
    if (steer_initialized) {
        rclcpp::Duration dt = n->now() - last_steer_time;
        double seconds_since_last_update = dt.seconds();
        if (seconds_since_last_update > 0.001) {
            double steer_rate = (current_angle - last_steer_angle) / seconds_since_last_update;
            std_msgs::msg::Float64 msg;
            msg.data = steer_rate;
            pub_steering_rate->publish(msg);
        }
    }
    last_steer_angle = current_angle;
    last_steer_time = n->now();
    steer_initialized = true;
}

bool new_input_available(std_msgs::msg::Float64MultiArray veh, nav_msgs::msg::Path global_path, std_msgs::msg::Float64 ss) {
    // Check for new input
	if (veh_input_stamp == last_veh_stamp || global_path.poses.size() < 2 || veh_input_stamp == init_time) {
		return false;
	}

    // Check for speed setpoint changes
    if (speedSetpoint_input.data > 0) {
		speedSetpoint = ss.data;
    }

    last_veh_stamp = veh_input_stamp;

    const double la = params.vehicle_axle_distance_front;
    double current_time = veh.data[0];
	double yaw = veh.data[6];
	double x_veh = veh.data[1] + la*cos(yaw); // x position of front axle
	double y_veh = veh.data[2] + la*sin(yaw); // y position of front axle
	double longvel = veh.data[3];
	double latvel = veh.data[4];
	double steer_angle = veh.data[5];
	
	//Steering angle publishing; could be set in a better place
	std_msgs::msg::Float64 steer_msg;
	steer_msg.data = steer_angle;
	pub_steering_angle->publish(steer_msg);

	//Steering rate
	publishSteeringRate(steer_angle);
	double yawrate = veh.data[7];
	double longacc = veh.data[8];

	avt_341::utils::vec2 vehiclePosition(x_veh, y_veh);
	avt_341::utils::vec2 globalPoint(0.0, 0.0);

    const double T =
        params.prediction_time_horizon + params.goal_lookahead_time_padding;
    double lookahead_dist;
    if (params.use_goal_lookahead_maxspeed) {
        lookahead_dist = speedSetpoint * T;
    } else {
        const double v0 = longvel;
        const double v_sp = static_cast<double>(speedSetpoint);
        if (v0 >= v_sp) {
            lookahead_dist = v_sp * T;
        } else {
            const double t_accel = (v_sp - v0) / params.ax_max;
            if (t_accel >= T) {
                lookahead_dist =
                    v0 * T + 0.5 * params.ax_max * T * T;
            } else {
                const double d_accel =
                    v0 * t_accel +
                    0.5 * params.ax_max * t_accel * t_accel;
                const double d_cruise = v_sp * (T - t_accel);
                lookahead_dist = d_accel + d_cruise;
            }
        }
    }

    int pathStartIndex = 0;
    if (follower_status_input.use_leader) { //asked to follow a leader
		if (!priorUseLeader){
            priorUseLeader = true;
        }
        // Find nearest index on global path starting from the beginning
        float distanceToGlobalPoint = -1;
        int closestIndex = 0;
        for (int gp = 0; gp < global_path.poses.size(); gp++) {
            globalPoint.x = global_path.poses[gp].pose.position.x;
            globalPoint.y = global_path.poses[gp].pose.position.y;
            float currentDistance = (globalPoint - vehiclePosition).mag();
            if (distanceToGlobalPoint < 0 || currentDistance < distanceToGlobalPoint) {
                distanceToGlobalPoint = currentDistance;
                closestIndex = gp;
            }
        }

        // move along global path starting from closestIndex until you exceed prediction horizon
        float pathLength = 0.0f;
        int lastIndexConsidered = closestIndex;
        for (int gp = closestIndex; gp < global_path.poses.size(); gp++) {
            globalPoint.x = global_path.poses[gp].pose.position.x;
            globalPoint.y = global_path.poses[gp].pose.position.y;
            if (gp > closestIndex) {
                avt_341::utils::vec2 prevPoint(global_path.poses[gp-1].pose.position.x, global_path.poses[gp-1].pose.position.y);
                pathLength += (globalPoint - prevPoint).mag();
            }
            lastIndexConsidered = gp;
            if (pathLength > lookahead_dist) {
                break;
            }
        }
        // If the last index considered is the last index of the global path, mark goal as end
        if (lastIndexConsidered >= (int)global_path.poses.size() - 1) {
            goal_is_end = true;
        } else {
            goal_is_end = false;
        }
    }
    else {
		if (priorUseLeader) { //was follower, now starting to be independent
			priorUseLeader = false;
			priorIndex = 0;
			priorPathLength = 0;
        }
        // Find nearest index on global path to vehicle position
        float distanceToGlobalPoint = -1;
        int closestIndex = 0;
        for (int gp = 0; gp < (int)global_path.poses.size(); gp++) {
            globalPoint.x = global_path.poses[gp].pose.position.x;
            globalPoint.y = global_path.poses[gp].pose.position.y;
            float currentDistance = (globalPoint - vehiclePosition).mag();
            if (distanceToGlobalPoint < 0 || currentDistance < distanceToGlobalPoint) {
                distanceToGlobalPoint = currentDistance;
                closestIndex = gp;
            }
        }
        pathStartIndex = closestIndex;
		float pathLength = 0.0f;
		int lastIndexConsidered = closestIndex;
		for (int gp = closestIndex; gp < (int)global_path.poses.size(); gp++) {
			globalPoint.x = global_path.poses[gp].pose.position.x;
            globalPoint.y = global_path.poses[gp].pose.position.y;
			if (gp > closestIndex) {
                avt_341::utils::vec2 prevPoint(global_path.poses[gp-1].pose.position.x, global_path.poses[gp-1].pose.position.y);
                pathLength += (globalPoint - prevPoint).mag();
            }
			lastIndexConsidered = gp;
			if (pathLength > lookahead_dist){
				break;
            }
		}
        // If the last index considered is the last index of the global path, mark goal as end
		if (global_path.poses.size() > 0) {
			goal_is_end = (lastIndexConsidered >= (int)global_path.poses.size() - 1);
		} else {
			goal_is_end = false;
		}
    }

	if (!goal_set || params.always_publish_goal) {
		goal = globalPoint;
        goal_set = true;
        turningAround = false;
    }
	else {
		avt_341::utils::vec3 globalPointVector(globalPoint.x-x_veh, globalPoint.y-y_veh, 0);
		avt_341::utils::vec3 leftBoundaryVector(cos(params.front_angle_goal)*cos(yaw) + sin(params.front_angle_goal)*-sin(yaw),
                                                cos(params.front_angle_goal)*sin(yaw) + sin(params.front_angle_goal)*cos(yaw),
                                                1.0f);
		avt_341::utils::vec3 rightBoundaryVector(cos(params.front_angle_goal)*cos(yaw) - sin(params.front_angle_goal)*-sin(yaw),
                                                cos(params.front_angle_goal)*sin(yaw) - sin(params.front_angle_goal)*cos(yaw),
                                                1.0f);
		if (cross(globalPointVector,leftBoundaryVector).z < 0 || cross(globalPointVector,rightBoundaryVector).z > 0) {
			if (!turningAround) {
				turningAround = true;
				goal = globalPoint;
            }
			else {
				return true;
			}
        }
		else {
			turningAround = false;
			goal = globalPoint;
		}
    }
	float distanceToGoal = (globalPoint - vehiclePosition).mag();

	goal = globalPoint;
    avt_341::utils::vec2 heading;
	if (global_path.poses.size() > 1 && !priorUseLeader && pathStartIndex + 1 < (int)global_path.poses.size()) {
		heading.x = global_path.poses[pathStartIndex+1].pose.position.x - global_path.poses[pathStartIndex].pose.position.x;
        heading.y = global_path.poses[pathStartIndex+1].pose.position.y - global_path.poses[pathStartIndex].pose.position.y;
    }
    else {
		heading = goal - vehiclePosition;
	}

	desiredHeading = atan2(heading.y,heading.x);

	return true;

}

int main(int argc, char* argv[]) {
    // Initialize ROS node.
    rclcpp::init(argc, argv);
    n = rclcpp::Node::make_shared("goal_point_processor");
    // Crate node subscribers
    auto sub_path = n->create_subscription<nav_msgs::msg::Path>("avt_341/global_path",1,callback_global_path);
    auto sub_veh = n->create_subscription<std_msgs::msg::Float64MultiArray>("avt_341/veh",1,callback_veh);
    auto sub_speed = n->create_subscription<std_msgs::msg::Float64>("avt_341/speed_setpoint",1,callback_speedSetpoint);
    auto sub_follow = n->create_subscription<avt_341_msgs::msg::FollowerStatus>("avt_341/follower_status",1,callback_follower_status);
    auto sub_goal_pose = n->create_subscription<avt_341_msgs::msg::NavState>("avt_341/state",1,callback_gp_state);

    pub_time_gap = n->create_publisher<std_msgs::msg::Float64>("time_gap",10);
    pub_steering_angle = n->create_publisher<std_msgs::msg::Float64>("steering_angle",10);
    pub_steering_rate = n->create_publisher<std_msgs::msg::Float64>("steering_rate",10);
    pub_scenario_tag = n->create_publisher<std_msgs::msg::String>("scenario_tag",10);
    pub_segment_start = n->create_publisher<std_msgs::msg::Bool>("segment_start_tag",10);
    pub_segment_end = n->create_publisher<std_msgs::msg::Bool>("segment_end_tag",10);
    pub_goalPoint = n->create_publisher<geometry_msgs::msg::PointStamped>("avt_341/mpc_goalPoint",1);
    pub_desiredHeading = n->create_publisher<std_msgs::msg::Float64>("avt_341/mpc_desiredHeading",1);
    pub_goalPointIsEnd = n->create_publisher<std_msgs::msg::Bool>("avt_341/mpc_goalPoint_is_end_of_global_path",1);
    pub_finalHeading = n->create_publisher<std_msgs::msg::Float64>("avt_341/mpc_final_heading",1);
 
    avt_341::params::mpc_local_planner::ParamsListener param_listener(n);
    params = param_listener.get_params();

    // Initialize variables
    init_time = n->now();
    veh_input_stamp = init_time;
    last_veh_stamp = init_time;
    speedSetpoint = static_cast<float>(params.max_speed);
    priorUseLeader = false;
    priorIndex = 0;
    priorPathLength = 0;
    goal_set = false;
    turningAround = false;
    desiredHeading = 0.0f;
    finalHeading = 0.0f;
    finalHeadingSet = false;
    autoFinalHeading = 0.0f;
    autoFinalHeadingSet = false;

    rclcpp::Rate rosrate(20.0f);
    while (rclcpp::ok()) {
        if (new_input_available(veh_input, global_path_input, speedSetpoint_input)) {
            geometry_msgs::msg::PointStamped ros_goalPoint;
            ros_goalPoint.point.x = goal.x;
            ros_goalPoint.point.y = goal.y;
            ros_goalPoint.point.z = 0.0f;
            ros_goalPoint.header.frame_id = "map";
            pub_goalPoint->publish(ros_goalPoint);
            std_msgs::msg::Bool ros_goalPointIsEnd;
            ros_goalPointIsEnd.data = goal_is_end;
            pub_goalPointIsEnd->publish(ros_goalPointIsEnd);
            std_msgs::msg::Float64 ros_desiredHeading;
            ros_desiredHeading.data = desiredHeading;
            pub_desiredHeading->publish(ros_desiredHeading);
            if (goal_is_end) {
                float headingToPublish = 0.0f;
                bool shouldPublish = false;
                if (finalHeadingSet) {
                    headingToPublish = finalHeading;
                    shouldPublish = true;
                } else if (params.use_auto_final_heading &&
                           autoFinalHeadingSet) {
                    headingToPublish = autoFinalHeading;
                    shouldPublish = true;
                }
                if (shouldPublish) {
                    std_msgs::msg::Float64 ros_finalHeading;
                    ros_finalHeading.data = headingToPublish;
                    pub_finalHeading->publish(ros_finalHeading);
                }
            }
        }
        rosrate.sleep();
        rclcpp::spin_some(n);
    }
}
