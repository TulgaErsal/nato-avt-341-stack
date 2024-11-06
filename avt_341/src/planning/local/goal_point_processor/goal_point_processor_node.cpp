/**
 C++ implementation of the goal_point_processor.jl found in the MPC planner stack.
*/

#include "avt_341/node/node_proxy.h"
#include "avt_341/node/ros_types.h"
#include "avt_341/avt_341_utils.h"

// Globals
std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;
std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointStamped>> pub_goalPoint = nullptr;
std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Float64>> pub_desiredHeading = nullptr;
avt_341::msg::Path global_path_input;
avt_341::msg::Float64MultiArray veh_input;
avt_341::msg::Float64 speedSetpoint_input;
avt_341::msg::Time veh_input_stamp, last_veh_stamp, init_time;
avt_341::msg::FollowerStatus follower_status_input;
float speedSetpoint, desiredHeading;
bool priorUseLeader, turningAround, goal_set;
int priorIndex, priorPathLength;
avt_341::utils::vec2 goal;

// Params
float max_speed, la, predictionTimeHorizon, frontAngleGoal;

void callback_global_path(avt_341::msg::PathPtr global_path) {
    global_path_input = *global_path;
}

void callback_veh(avt_341::msg::Float64MultiArrayPtr veh) {
    veh_input = *veh;
    veh_input_stamp = n->get_stamp();
}

void callback_speedSetpoint(avt_341::msg::Float64Ptr ss) {
    speedSetpoint_input = *ss;
}

void callback_follower_status(avt_341::msg::FollowerStatusPtr follower_status) {
    follower_status_input = *follower_status;
}

bool new_input_available(avt_341::msg::Float64MultiArray veh, avt_341::msg::Path global_path, avt_341::msg::Float64 ss) {
    // Check for new input
	if (veh_input_stamp == last_veh_stamp || global_path.poses.size() < 2 || veh_input_stamp == init_time) {
		return false;
	}

    // Check for speed setpoint changes
    if (speedSetpoint_input.data > 0) {
		speedSetpoint = ss.data;
    }

    last_veh_stamp = veh_input_stamp;
    
    double current_time = veh.data[0];
	double yaw = veh.data[6];
	double x_veh = veh.data[1] + la*cos(yaw); // x position of front axle
	double y_veh = veh.data[2] + la*sin(yaw); // y position of front axle
	double longvel = veh.data[3];
	double latvel = veh.data[4];
	double steer_angle = veh.data[5];
	double yawrate = veh.data[7];
	double longacc = veh.data[8];

	avt_341::utils::vec2 vehiclePosition(x_veh, y_veh);
	avt_341::utils::vec2 globalPoint(0.0, 0.0);

    if (follower_status_input.use_leader) { //asked to follow a leader
		if (!priorUseLeader){
			priorUseLeader = true;
        }
        //if there is a reduction in global path length, assume the leader changed and reset the priorIndex
		if (global_path.poses.size() < priorPathLength) {
			priorIndex = 0;
        }
		priorPathLength = global_path.poses.size();

		//find nearest index on global path starting from priorIndex
		float distanceToGlobalPoint = -1;
		int closestIndex = priorIndex;
		for (int gp=priorIndex;gp<global_path.poses.size();gp++) {
			globalPoint.x = global_path.poses[gp].pose.position.x;
            globalPoint.y = global_path.poses[gp].pose.position.y;
			float currentDistance = (globalPoint-vehiclePosition).mag();
			if (distanceToGlobalPoint < 0) {
				distanceToGlobalPoint = currentDistance;
				closestIndex = gp;
				continue;
            }
			if (currentDistance < distanceToGlobalPoint) {
				distanceToGlobalPoint = currentDistance;
				closestIndex = gp;
            }
		}
		priorIndex = closestIndex;

		// move along global path starting from closestIndex until you exceed prediction horizon
		for (int gp=priorIndex;gp<global_path.poses.size();gp++) {
			globalPoint.x = global_path.poses[gp].pose.position.x;
            globalPoint.y = global_path.poses[gp].pose.position.y;
			distanceToGlobalPoint = (globalPoint-vehiclePosition).mag();
			if (distanceToGlobalPoint > (predictionTimeHorizon+0.1)*speedSetpoint) {
				break;
            }
        }
    }
    else {
		if (priorUseLeader) { //was follower, now starting to be independent
			priorUseLeader = false;
			priorIndex = 0;
			priorPathLength = 0;
        }
		for (int gp=0;gp<global_path.poses.size();gp++) {
			globalPoint.x = global_path.poses[gp].pose.position.x;
            globalPoint.y = global_path.poses[gp].pose.position.y;
			float distanceToGlobalPoint = (globalPoint-vehiclePosition).mag();
			avt_341::utils::vec2 v1(cos(yaw), sin(yaw));
			avt_341::utils::vec2 v2 = globalPoint-vehiclePosition;
			float angleToGlobalPoint = acos(dot(v1,v2)/(v1.mag()*v2.mag()));
			// Check prediction horizon
			if (distanceToGlobalPoint > (predictionTimeHorizon+0.1)*speedSetpoint){
				break;
            }
		}
    }

	if (!goal_set) {
		goal = globalPoint;
        goal_set = true;
    }
	else {
		avt_341::utils::vec3 globalPointVector(globalPoint.x-x_veh, globalPoint.y-y_veh, 0);
		avt_341::utils::vec3 leftBoundaryVector(cos(frontAngleGoal)*cos(yaw) + sin(frontAngleGoal)*-sin(yaw),
                                                cos(frontAngleGoal)*sin(yaw) + sin(frontAngleGoal)*cos(yaw),
                                                1.0f);
		avt_341::utils::vec3 rightBoundaryVector(cos(frontAngleGoal)*cos(yaw) - sin(frontAngleGoal)*-sin(yaw),
                                                cos(frontAngleGoal)*sin(yaw) - sin(frontAngleGoal)*cos(yaw),
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
	if (global_path.poses.size() > 1 && !priorUseLeader) {
		heading.x = global_path.poses[1].pose.position.x-global_path.poses[0].pose.position.x;
        heading.y = global_path.poses[1].pose.position.y-global_path.poses[0].pose.position.y;
    }
    else {
		heading = goal - vehiclePosition;
	}

	desiredHeading = atan2(heading.y,heading.x);

	return true;

}

int main(int argc, char* argv[]) {
    // Initialize ROS node.
    n = avt_341::node::init_node(argc, argv, "goal_point_processor");

    // Create node subscribers.
    auto sub_path = n->create_subscription<avt_341::msg::Path>("avt_341/global_path", 1, callback_global_path);
    auto sub_veh = n->create_subscription<avt_341::msg::Float64MultiArray>("avt_341/veh", 1, callback_veh);
    auto sub_speed = n->create_subscription<avt_341::msg::Float64>("avt_341/speed_setpoint", 1, callback_speedSetpoint);
    auto sub_follower_status = n->create_subscription<avt_341::msg::FollowerStatus>("avt_341/follower_status", 1, callback_follower_status);

    // Create node publishers.
    pub_goalPoint = n->create_publisher<avt_341::msg::PointStamped>("avt_341/mpc_goalPoint", 1);
    pub_desiredHeading = n->create_publisher<avt_341::msg::Float64>("avt_341/mpc_desiredHeading", 1);

    // Retrieve params
    n->get_parameter("~max_speed", max_speed, 5.0f);
    n->get_parameter("~vehicle_axle_distance_front", la, 1.25f);
    n->get_parameter("~prediction_time_horizon", predictionTimeHorizon, 2.0f);
    n->get_parameter("~front_angle_goal", frontAngleGoal, 1.571f);

    // Initialize variables
    init_time = n->get_stamp();
    veh_input_stamp = init_time;
    last_veh_stamp = init_time;
    speedSetpoint = max_speed;
    priorUseLeader = false;
    priorIndex = 0;
    priorPathLength = 0;
    goal_set = false;
    turningAround = false;
    desiredHeading = 0.0f;

    avt_341::node::Rate rosrate(20.0f);
    while (avt_341::node::ok()) {
        if (new_input_available(veh_input, global_path_input, speedSetpoint_input)) {
            avt_341::msg::PointStamped ros_goalPoint;
            ros_goalPoint.point.x = goal.x;
            ros_goalPoint.point.y = goal.y;
            ros_goalPoint.point.z = 0.0f;
            ros_goalPoint.header.frame_id = "map";
            pub_goalPoint->publish(ros_goalPoint);

            avt_341::msg::Float64 ros_desiredHeading;
            ros_desiredHeading.data = desiredHeading;
            pub_desiredHeading->publish(ros_desiredHeading);
        }
        rosrate.sleep();
        n->spin_some();
    }
}