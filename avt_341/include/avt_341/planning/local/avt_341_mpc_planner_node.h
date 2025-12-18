/**
 * @file avt_341_mpc_planner_node.cpp
 *
 * @brief Header for the MPC planner wrapper.
 *
 * @date 08/16/2024
 *
 * @author Dario Sirangelo (dsi@mpe.au.dk)
 *         Aarhus University (DK)
 *         Department of Mechanical and Production Engineering
 *         Section Mechatronics & Dynamics
 * @author Evan Vandermate (evanderm@mtu.edu)
 *         Keweenaw Research Center (KRC)
 */

#ifndef AVT_341_MPC_PLANNER_NODE_H
#define AVT_341_MPC_PLANNER_NODE_H

#include <iostream>
#include <mutex>

#include <avt_341/node/node_proxy.h>
#include <avt_341/node/ros_types.h>

// Julia header throws "No Target Architecture" error otherwise on Windows systems
#ifdef _WIN64
 #define _AMD64_
#endif

#include <julia.h>

// TODO: I do not have a Windows environment to test whether this declaration
// has any effect.
#ifdef _OS_WINDOWS_
__declspec(dllexport);
#endif

// ROS node functions
// ------------------

void CatchJuliaException();

// ------------------

// Thread safety
// -------------

bool has_error = false;
// -------------

// ROS members
// -----------

std::shared_ptr<avt_341::node::NodeProxy> node;
// -----------

// ROS parameters
// --------------

std::string sysimage_path;
std::string planner_module_path;
std::string parameters_module_path;
std::string models_module_path;
double rate;
std::string tire_model;
std::string linear_solver;
int num_col_points;
double prediction_time_horizon;
int max_num_obs;
int max_num_seg;
double min_speed;
double max_speed;
bool stop_on_max_solve_time;
bool use_hard_constraints;
bool use_segmentation;
double segmentation_resolution;
double w_distance_to_obstacles;
double w_distance_to_goal;
double w_deviation_in_yaw;
double w_yaw_accel;
double w_traversability_cost;
double safety_margin;
double grid_resolution;
double front_angle_goal;
double front_angle_obstacle;
double front_angle_segmentation;
double w_final_speed;
bool adaptive;
double vehicle_axle_distance_front;
bool obstacles_vizualize;
bool publish_steering_commands;
double slope_threshold;
double rms_threshold;
double speed_around_large_slopes_and_rms;
double sa_min;
double sa_max;
double sr_min;
double sr_max;
// --------------

// Globals
// --------------

bool recv_veh_input = false;
bool recv_seg_input = false;
bool is_initialized = false;
// --------------

// Julia modules
// -------------

/** @brief Pointer to the Julia function to set the path to the Julia MPC
 * module. */
jl_module_t* mpc_module = NULL;
// -------------

// Julia functions
// ---------------

/** @brief Pointer to the Julia function to setup the solver. */
jl_function_t* j_setup = NULL;

/** @brief Pointer to the Julia function to plan a path. */
jl_function_t* j_plan = NULL;

/** @brief Pointer to the Julia function to set vehicle state. */
jl_function_t* j_set_state = NULL;

/** @brief Pointer to the Julia function to set the detected obstacles. */
jl_function_t* j_set_obstacles = NULL;

/** @brief Pointer to the Julia function to set the planner goal point. */
jl_function_t* j_set_goal_point = NULL;

/** @brief Pointer to the Julia function to set the planner desired heading. */
jl_function_t* j_set_heading = NULL;

/** @brief Pointer to the Julia function to set the planner speed setpoint. */
jl_function_t* j_set_speed = NULL;

/** @brief Pointer to the Julia function to set the estimated sinkage. */
jl_function_t* j_set_sinkage = NULL;

/** @brief Pointer to the Julia function to set the segmentation grid cells. */
jl_function_t* j_set_segmentation = NULL;

/** @brief Pointer to the Julia function to set the terrain slope. */
jl_function_t* j_set_terrain_slope = NULL;

/** @brief Pointer to the Julia function to set the terrain rms. */
jl_function_t* j_set_terrain_rms = NULL;

/** @brief Pointer to the Julia function to get the MPC path */
jl_function_t* j_get_path = NULL;

/** @brief Pointer to the Julia function to get the MPC speed */
jl_function_t* j_get_speed = NULL;

/** @brief Pointer to the Julia function to get the MPC speed at the end of the prediction horizon */
jl_function_t* j_get_final_speed = NULL;

/** @brief Pointer to the Julia function to get the MPC steering */
jl_function_t* j_get_steering = NULL;

/** @brief Pointer to the Julia function to get slope limiting flag */
jl_function_t* j_get_slope_limited = NULL;

/** PARAMETER SETTERS */
jl_function_t* j_set_tire_model = NULL;
jl_function_t* j_set_num_col_points = NULL;
jl_function_t* j_set_prediction_time_horizon = NULL;
jl_function_t* j_set_max_num_obs = NULL;
jl_function_t* j_set_max_num_seg = NULL;
jl_function_t* j_set_sigma = NULL;
jl_function_t* j_set_min_speed = NULL;
jl_function_t* j_set_max_speed = NULL;
jl_function_t* j_set_stop_on_max_solve_time = NULL;
jl_function_t* j_set_use_hard_constraints = NULL;
jl_function_t* j_set_use_segmentation = NULL;
jl_function_t* j_set_w_distance_to_obstacles = NULL;
jl_function_t* j_set_w_distance_to_goal = NULL;
jl_function_t* j_set_w_deviation_in_yaw = NULL;
jl_function_t* j_set_w_yaw_accel = NULL;
jl_function_t* j_set_w_traversability_cost = NULL;
jl_function_t* j_set_safety_margin = NULL;
jl_function_t* j_set_grid_resolution = NULL;
jl_function_t* j_set_front_angle_goal = NULL;
jl_function_t* j_set_front_angle_obstacle = NULL;
jl_function_t* j_set_terrain_adaptive = NULL;
jl_function_t* j_set_veh_front_axle_dist = NULL;
jl_function_t* j_set_front_angle_segmentation = NULL;
jl_function_t* j_set_linear_solver = NULL;
jl_function_t* j_set_slope_threshold = NULL;
jl_function_t* j_set_rms_threshold = NULL;
jl_function_t* j_set_speed_around_large_slopes_and_rms = NULL;
jl_function_t* j_set_sa_min = NULL;
jl_function_t* j_set_sa_max = NULL;
jl_function_t* j_set_sr_min = NULL;
jl_function_t* j_set_sr_max = NULL;
jl_function_t* j_set_w_final_speed = NULL;
jl_function_t* j_set_leader_speed = NULL;
jl_function_t* j_set_follower_status = NULL;
jl_function_t* j_set_goal_point_is_end_of_global_path = NULL;

// ---------------

// Macros
// ------

#define CATCH_JULIA_EXCEPTION CatchJuliaException()
// ------

#endif // #define AVT_341_MPC_PLANNER_NODE_H