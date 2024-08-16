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
#include <avt_341_msgs/msg/obstacles.hpp>
#include <avt_341_msgs/msg/sinkage.hpp>
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

std::atomic<bool> has_error = false;
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
int num_col_points;
double prediction_time_horizon;
int max_num_obs;
double min_speed;
double max_speed;
bool stop_on_max_solve_time;
bool use_hard_constraints;
double w_distance_to_obstacles;
double w_distance_to_goal;
double w_deviation_in_yaw;
double safety_margin;
double grid_resolution;
double front_angle_goal;
double front_angle_obstacle;
bool adaptive;
double vehicle_axle_distance_front;
bool obstacles_vizualize;
// --------------

// Globals
// --------------

bool recv_veh_input = false;
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

/** @brief Pointer to the Julia function to get the MPC path */
jl_function_t* j_get_path = NULL;

/** @brief Pointer to the Julia function to get the MPC speed */
jl_function_t* j_get_speed = NULL;

/** @brief Pointer to the Julia function to get the MPC steering */
jl_function_t* j_get_steering = NULL;

/** PARAMETER SETTERS */
jl_function_t* j_set_tire_model = NULL;
jl_function_t* j_set_num_col_points = NULL;
jl_function_t* j_set_prediction_time_horizon = NULL;
jl_function_t* j_set_max_num_obs = NULL;
jl_function_t* j_set_min_speed = NULL;
jl_function_t* j_set_max_speed = NULL;
jl_function_t* j_set_stop_on_max_solve_time = NULL;
jl_function_t* j_set_use_hard_constraints = NULL;
jl_function_t* j_set_w_distance_to_obstacles = NULL;
jl_function_t* j_set_w_distance_to_goal = NULL;
jl_function_t* j_set_w_deviation_in_yaw = NULL;
jl_function_t* j_set_safety_margin = NULL;
jl_function_t* j_set_grid_resolution = NULL;
jl_function_t* j_set_front_angle_goal = NULL;
jl_function_t* j_set_front_angle_obstacle = NULL;
jl_function_t* j_set_terrain_adaptive = NULL;
jl_function_t* j_set_veh_front_axle_dist = NULL;

// ---------------

// Macros
// ------

#define CATCH_JULIA_EXCEPTION CatchJuliaException()
// ------

#endif // #define AVT_341_MPC_PLANNER_NODE_H