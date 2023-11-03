/**
 * @file avt_341_mpc_planner_node.cpp
 *
 * @brief Header for the MPC planner wrapper.
 *
 * @date 03/11/2023
 *
 * @author Dario Sirangelo (dsi@mpe.au.dk)
 *         Aarhus University (DK)
 *         Department of Mechanical and Production Engineering
 *         Section Mechatronics & Dynamics
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
std::mutex planner_mutex;
// -------------

// ROS members
// -----------

std::shared_ptr<avt_341::node::NodeProxy> node;
// -----------

// ROS parameters
// --------------

std::string sysimage_path;
std::string module_path;
bool use_terrain_adaptive;
double t_span;
double u_min;
double l_a;
double rate;
// --------------

// Julia modules
// -------------

/** @brief Pointer to the Julia function to set the path to the Julia MPC
 * module. */
jl_module_t* mpc_module = NULL;
// -------------

// Julia functions
// ---------------

/** @brief Pointer to the Julia function to set vehicle state. */
jl_function_t* j_set_state = NULL;

/** @brief Pointer to the Julia function to set the planner goal point. */
jl_function_t* j_set_goal_point = NULL;

/** @brief Pointer to the Julia function to set the front axle position. */
jl_function_t* j_set_front_axle_position = NULL;

/** @brief Pointer to the Julia function to set the prediction horizon. */
jl_function_t* j_set_prediction_horizon = NULL;

/** @brief Pointer to the Julia function to set the minimum speed. */
jl_function_t* j_set_minimum_speed = NULL;

/** @brief Pointer to the Julia function to set whether or not to use the
 * terrain-adaptive formulation. */
jl_function_t* j_use_terrain_adaptive = NULL;

/** @brief Pointer to the Julia function to set the detected obstacles. */
jl_function_t* j_set_obstacles = NULL;

/** @brief Pointer to the Julia function to set the estimated sinkage. */
jl_function_t* j_set_sinkage = NULL;
// ---------------

// Macros
// ------

#define CATCH_JULIA_EXCEPTION CatchJuliaException()
// ------

#endif // #define AVT_341_MPC_PLANNER_NODE_H