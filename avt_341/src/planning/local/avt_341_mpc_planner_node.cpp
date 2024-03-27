/**
 * \file avt_341_mpc_planner_node.cpp
 * Plan a local trajectory using the MPC planner
 *
 * \author Marius Thoresen
 *
 * \contact marius.thoresen@ffi.no
 *
 * \date 4/14/2023
*/

#include <cmath>
#include "avt_341/node/node_proxy.h"
#include "avt_341/node/ros_types.h"
#include "avt_341/planning/local/mpc_planner.h"
#include "avt_341/planning/local/mpc_planner_solver.h"

using namespace avt_341::planning;

std::shared_ptr<avt_341::node::NodeProxy> node;

// Initialise ROS messages.
avt_341::msg::Odometry g_odometry;

avt_341::msg::Path g_global_path;

double g_acceleration{0.0};

double g_steering_angle{0.0};

std::vector<float> g_obstacles_x;

std::vector<float> g_obstacles_y;

std::vector<float> g_obstacles_r;

// Initialise receive flags.
bool g_received_odometry{false};

bool g_received_obstacles{false};

bool g_received_global_path{false};

bool g_received_acceleration{false};

/**
 * @brief Store the AGV odometry and mark it as received.
 *
 * @param msg_received_odom Pointer to the odometry ROS nav_msgs/Odometry message.
 */
void callbackOdometry(avt_341::msg::OdometryPtr msg_received_odometry) {
    g_odometry = *msg_received_odometry;
    g_received_odometry = true;
}

/**
 * @brief Receive and store obstacle list from dedicated obstacle processor node.
 *
 * @param msg_received_obstacles Pointer to the avt_341_msgs::Obstacles message.
 */
void callbackObstacles(avt_341::msg::ObstaclesPtr msg_received_obstacles) {
    double radius = msg_received_obstacles->obstacle_size_meters;
    g_obstacles_x.clear();
    g_obstacles_y.clear();
    g_obstacles_r.clear();
    for (size_t i = 0; i < msg_received_obstacles->data.size() / 2; i++) {
        double x = msg_received_obstacles->data[2 * i];
        double y = msg_received_obstacles->data[2 * i + 1];
        g_obstacles_x.push_back(x);
        g_obstacles_y.push_back(y);
        g_obstacles_r.push_back(radius);
    }
    g_received_obstacles = true;
}


/**
 * @brief Store the global path as received. The first point outside the MPC planning radius
 * will be selected as the goal.
 *
 * @param msg_received_path Pointer to the goal pose ROS nav_msgs/Path message.
 */
void callbackGlobalPath(avt_341::msg::PathPtr msg_received_global_path) {
    g_global_path = *msg_received_global_path;
    g_received_global_path = true;
}

/**
 * @brief Store the acceleration.
 * Can be obtained from either IMU topic or acceleration topic.
 *
 * @param msg_received_acceleration Pointer to the ROS acceleration Float64 message
 */
void callbackAcceleration(avt_341::msg::Float64Ptr msg_received_acceleration) {
    g_acceleration = msg_received_acceleration->data;
    g_received_acceleration = true;
}

void callbackImu(avt_341::msg::ImuPtr msg_received_imu) {
    g_acceleration = msg_received_imu->linear_acceleration.x;
    g_received_acceleration = true;
}

void callbackAccelStamped(avt_341::msg::AccelStampedPtr msg_received_acceleration) {
    g_acceleration = msg_received_acceleration->accel.linear.x;
    g_received_acceleration = true;
}

void updateState(avt_341::planning::MpcPlanner& planner) {
    planner.setState(g_odometry.pose.pose.position.x,
                     g_odometry.pose.pose.position.y,
                     MpcUtils::quaternionMsg2Yaw(g_odometry.pose.pose.orientation),
                     g_odometry.twist.twist.linear.x,
                     g_odometry.twist.twist.linear.y,
                     g_odometry.twist.twist.angular.z,
                     g_steering_angle,
                     g_acceleration,
                     0.000,
                     0.000);
    g_received_odometry = false;
}

double distance(const avt_341::msg::Point& a, const avt_341::msg::Point& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

void updateGoal(avt_341::planning::MpcPlanner& planner, double speed_max, double time_span) {
    auto& vehicle_position = g_odometry.pose.pose.position;
    avt_341::msg::Point best_path_point;
    for (auto& path_pose: g_global_path.poses) {
        best_path_point = path_pose.pose.position;
        if (distance(path_pose.pose.position, vehicle_position) > (time_span + 0.1) * speed_max) {
            break;
        }
    }
    planner.setGoal(best_path_point.x, best_path_point.y);
}

void updateObstacles(avt_341::planning::MpcPlanner& planner) {
    planner.setObstacles(g_obstacles_x, g_obstacles_y, g_obstacles_r);
    g_received_obstacles = false;
}

int main(int argc, char* argv[]) {
    // Initialize ROS node.
    node = avt_341::node::init_node(argc, argv, "avt_341_mpc_planner_node");

    // Create node subscribers.
    auto sub_odometry = node->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 1, callbackOdometry);
    auto sub_obstacles = node->create_subscription<avt_341::msg::Obstacles>("avt_341/obstacles", 1, callbackObstacles);
    auto sub_global_path = node->create_subscription<avt_341::msg::Path>("avt_341/global_path", 1, callbackGlobalPath);
    //auto sub_imu = node->create_subscription<avt_341::msg::Imu>("avt_341/imu", 1, callbackImu);
    auto sub_accel = node->create_subscription<avt_341::msg::AccelStamped>("avt_341/vehicle_cg/accel", 1, callbackAccelStamped);

    // Create node publishers.
    auto pub_local_path = node->create_publisher<avt_341::msg::Path>("avt_341/local_path", 1);
    auto pub_control_jerk = node->create_publisher<avt_341::msg::Float64>("avt_341/cmd_jerk", 1);
    auto pub_control_steering_rate = node->create_publisher<avt_341::msg::Float64>("avt_341/cmd_steer_rate", 1);
    auto pub_control_velocities = node->create_publisher<avt_341::msg::Twist>("avt_341/cmd_vel", 1);
    auto pub_speed = node->create_publisher<avt_341::msg::Float64>("avt_341/cmd_steer", 1);
    auto pub_steer = node->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed", 1);

    // Declare and read node parameters from the ROS parameter server.
    float rate;
    node->get_parameter("~mpc_rate", rate, 0.0f);

    float vehicle_mass;
    node->get_parameter("~mpc_vehicle_mass", vehicle_mass, 0.0f);
    float yaw_inertia;
    node->get_parameter("~mpc_vehicle_yaw_inertia", yaw_inertia, 0.0f);
    float axle_distance_front;
    node->get_parameter("~mpc_vehicle_axle_distance_front", axle_distance_front, 0.0f);
    float axle_distance_rear;
    node->get_parameter("~mpc_vehicle_axle_distance_rear", axle_distance_rear, 0.0f);
    float axle_combined_stiffness_front;
    node->get_parameter("~mpc_vehicle_axle_combined_stiffness_front", axle_combined_stiffness_front, 0.0f);
    float axle_combined_stiffness_rear;
    node->get_parameter("~mpc_vehicle_axle_combined_stiffness_rear", axle_combined_stiffness_rear, 0.0f);
    float relaxation_length;
    node->get_parameter("~mpc_vehicle_relaxation_length", relaxation_length, 0.0f);

    float steering_angle_max;
    node->get_parameter("~mpc_bounds_steering_angle_max", steering_angle_max, 0.0f);
    float steering_rate_max;
    node->get_parameter("~mpc_bounds_steering_rate_max", steering_rate_max, 0.0f);
    float yaw_rate_min;
    node->get_parameter("~mpc_bounds_yaw_rate_min", yaw_rate_min, 0.0f);
    float yaw_rate_max;
    node->get_parameter("~mpc_bounds_yaw_rate_max", yaw_rate_max, 0.0f);
    float lateral_speed_max;
    node->get_parameter("~mpc_bounds_lateral_speed_max", lateral_speed_max, 0.0f);
    float long_speed_max;
    node->get_parameter("~mpc_bounds_longitudinal_speed_max", long_speed_max, 0.0f);
    float long_speed_min;
    node->get_parameter("~mpc_bounds_longitudinal_speed_min", long_speed_min, 0.0f);
    float long_acceleration_max;
    node->get_parameter("~mpc_bounds_longitudinal_acceleration_max", long_acceleration_max, 0.0f);
    float long_acceleration_min;
    node->get_parameter("~mpc_bounds_longitudinal_acceleration_min", long_acceleration_min, 0.0f);
    float long_jerk_max;
    node->get_parameter("~mpc_bounds_longitudinal_jerk_max", long_jerk_max, 0.0f);
    float long_jerk_min;
    node->get_parameter("~mpc_bounds_longitudinal_jerk_min", long_jerk_min, 0.0f);

    float rate_control;
    node->get_parameter("~mpc_rate_control", rate_control, 0.0f);
    float rate_plan;
    node->get_parameter("~mpc_rate_plan", rate_plan, 0.0f);
    std::string solver_name;
    node->get_parameter("~mpc_solver_name", solver_name, {});
    int solver_max_iterations;
    node->get_parameter("~mpc_solver_max_iterations", solver_max_iterations, 0);
    float solver_tolerance;
    node->get_parameter("~mpc_solver_tolerance", solver_tolerance, 0.0f);
    float solver_max_wall_time;
    node->get_parameter("~mpc_solver_max_wall_time", solver_max_wall_time, 0.0f);
    float solver_time_span;
    node->get_parameter("~mpc_solver_time_span", solver_time_span, 0.0f);
    float solver_time_step;
    node->get_parameter("~mpc_solver_time_step", solver_time_step, 0.0f);
    int solver_print_level;
    node->get_parameter("~mpc_solver_print_level", solver_print_level, 0);

    float weight_goal;
    node->get_parameter("~mpc_weight_goal", weight_goal, 1.0f);
    float weight_steering_rate;
    node->get_parameter("~mpc_weight_steering_rate", weight_steering_rate, 1.0f);
    float weight_obstacle_term;
    node->get_parameter("~mpc_weight_obstacle_term", weight_obstacle_term, 1.0f);
    float weight_longitudinal_jerk;
    node->get_parameter("~mpc_weight_longitudinal_jerk", weight_longitudinal_jerk, 1.0f);

    avt_341::planning::MpcPlanner planner;
    planner.setVehicleMass(vehicle_mass);
    planner.setYawInertia(yaw_inertia);
    planner.setAxleDistanceFront(axle_distance_front);
    planner.setAxleDistanceRear(axle_distance_rear);
    planner.setFrontAxleCombinedStiffness(axle_combined_stiffness_front);
    planner.setRearAxleCombinedStiffness(axle_combined_stiffness_rear);
    planner.setRelaxationLength(relaxation_length);

    planner.setSteeringAngleMin(-steering_angle_max);
    planner.setSteeringAngleMax(steering_angle_max);
    planner.setSteeringRateMin(-steering_rate_max);
    planner.setSteeringRateMax(steering_rate_max);
    planner.setYawRateMin(yaw_rate_min);
    planner.setYawRateMax(yaw_rate_max);
    planner.setLateralSpeedMin(-lateral_speed_max);
    planner.setLateralSpeedMax(lateral_speed_max);
    planner.setLongSpeedMin(long_speed_min);
    planner.setLongSpeedMax(long_speed_max);
    planner.setLongAccelerationMin(long_acceleration_min);
    planner.setLongAccelerationMax(long_acceleration_max);
    planner.setLongJerkMin(long_jerk_min);
    planner.setLongJerkMax(long_jerk_max);

    planner.setSolverName(solver_name);
    planner.setSolverMaxIterations(solver_max_iterations);
    planner.setSolverMaxWallTime(solver_max_wall_time);
    planner.setSolverTolerance(solver_tolerance);
    planner.setSolverTimeSpan(solver_time_span);
    planner.setSolverTimeStep(solver_time_step);
    planner.setSolverPrintLevel(solver_print_level);

    planner.setWeights(weight_goal, weight_steering_rate, weight_obstacle_term, weight_longitudinal_jerk);

    planner.initialize();

    avt_341::node::Rate rosrate(rate);

    while (avt_341::node::ok()) {
        if (g_received_global_path && g_received_odometry) {
            // Update the planner with the latest information.
            updateState(planner);
            updateGoal(planner, long_speed_max, solver_time_span);
            if (g_received_obstacles) {
                updateObstacles(planner);
            }

            // Run the planning step.
            if (planner.ready()) {
                planner.plan();

                // Serialise and publish the planned path.
                auto msg_path = planner.getPlannedPathRos();
                msg_path.header.stamp = node->get_stamp();
                pub_local_path->publish(msg_path);

                // Serialise and publish the jerk.
                double jerk = planner.getControlJerk();
                avt_341::msg::Float64 msg_control_jerk;
                msg_control_jerk.data = jerk;
                pub_control_jerk->publish(msg_control_jerk);

                // Serialise and publish the target steering angle.
                double steering_rate = planner.getControlSteeringRate();
                avt_341::msg::Float64 msg_control_steering_rate;
                msg_control_steering_rate.data = steering_rate;
                pub_control_steering_rate->publish(msg_control_steering_rate);

                avt_341::msg::Float64 msg_speed;
                avt_341::msg::Float64 msg_steer;
                auto controls = planner.getControls();
                msg_speed.data = controls.speed_longitudinal;
                msg_steer.data = controls.steering_angle;

                pub_speed->publish(msg_speed);
                pub_steer->publish(msg_steer);
                avt_341::msg::Twist msg_cmd_vel;
                msg_cmd_vel.linear.x = controls.speed_longitudinal;
                msg_cmd_vel.angular.z = controls.steering_angle;
                pub_control_velocities->publish(msg_cmd_vel);

                /*
                node->log_info_throttle(1.0, "MPC Planner Output: \n\tavt_341/local_path (length) = \t\t%d\n\tavt_341/cmd_jerk = \t%lf\n\tavt_341/cmd_steer_rate = \t\t%lf\n\tavt_341/cmd_vel (linear.x)= \t%f\n\tavt_341/cmd_steer = \t%f\n\tavt_341/desired_speed = \t%f\n\t",
                    msg_path.poses.size(), jerk, steering_rate, controls.speed_longitudinal, controls.speed_longitudinal, controls.steering_angle);
                */

            } else {
                std::cout << "Planner not ready. Either missing data or not initialized" << std::endl;
            }
        }

        // Run the ROS node at the specified rate.
        node->spin_some();
        rosrate.sleep();
    }

    return EXIT_SUCCESS;
}
