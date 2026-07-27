/**
 * @file avt_341_mpc_planner_node.cpp
 *
 * @brief Plan a local trajectory using the model predictive control planner.
 *        This ROS node is a wrapper to the TulgaErsal/AVT-341-MPC planner
 *        through the Julia C API.
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
#include <avt_341/planning/local/avt_341_mpc_planner_node.h>
#include <avt_341/node/node_types.h>
#include <avt_341/mpc_local_planner_params_service.hpp>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "avt_341_msgs/msg/follower_status.hpp"
#include "avt_341_msgs/msg/sinkage.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <rclcpp/rclcpp.hpp>

// This call must be included in the ROS node executable before initialising
// the Julia C bindings and is required for fast execution of wrapped Julia
// code.
JULIA_DEFINE_FAST_TLS();

jl_function_t* j_get_heading = nullptr;

void CatchJuliaException()
{
    // Catch exceptions from the Julia function call.
    if (jl_exception_occurred()) {
        const char *p = jl_string_ptr(jl_eval_string("sprint(showerror, ccall(:jl_exception_occurred, Any, ()))"));
        RCLCPP_ERROR(node->get_logger(), "Julia module has thrown an exception: %s", p);
        has_error = true;
    }
}

bool reset_called = false;

void ResetCallback(const std_msgs::msg::String::SharedPtr msg) {
    if(msg->data.find(avt_341::node::NodeType::LocalPlanner) !=
       std::string::npos) {
        reset_called = true;
       }
}


void VehicleStateCallback(std_msgs::msg::Float64MultiArray::SharedPtr f64_ma_msg)
{
    jl_value_t* array_type = jl_apply_array_type((jl_value_t*)jl_float64_type, 1);
    double* veh_data_arr = const_cast<double*>(&f64_ma_msg->data[0]);
    jl_array_t *veh_data = jl_ptr_to_array_1d(array_type, veh_data_arr, 11, 0);

    jl_call1(j_set_state, (jl_value_t*)veh_data);
    CATCH_JULIA_EXCEPTION;

    recv_veh_input = true;
}

// Returns the squared distance from point (px, py) to the segment (ax,ay)-(bx,by).
static double SegmentDistSq(double px, double py,
                             double ax, double ay,
                             double bx, double by)
{
    double dx = bx - ax, dy = by - ay;
    double len_sq = dx * dx + dy * dy;
    if (len_sq < 1e-12) {
        double ex = px - ax, ey = py - ay;
        return ex * ex + ey * ey;
    }
    double t = ((px - ax) * dx + (py - ay) * dy) / len_sq;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    double cx = ax + t * dx - px;
    double cy = ay + t * dy - py;
    return cx * cx + cy * cy;
}

// Returns the subset of obstacles (x, y, size triples) whose center lies
// within half_width of any segment of the path polyline.
static std::vector<double> CullObstaclesToCorridor(
    const std::vector<double>& obs,
    const std::vector<std::pair<double, double>>& path,
    double half_width)
{
    const double threshold_sq = half_width * half_width;
    const int num_obs = static_cast<int>(obs.size()) / 3;
    std::vector<double> culled;
    culled.reserve(obs.size());

    for (int i = 0; i < num_obs; i++) {
        const double ox = obs[3 * i];
        const double oy = obs[3 * i + 1];

        bool in_corridor = false;
        for (size_t j = 0; j + 1 < path.size(); j++) {
            if (SegmentDistSq(ox, oy,
                              path[j].first,     path[j].second,
                              path[j+1].first,   path[j+1].second) <= threshold_sq) {
                in_corridor = true;
                break;
            }
        }

        if (in_corridor) {
            culled.push_back(ox);
            culled.push_back(oy);
            culled.push_back(obs[3 * i + 2]);
        }
    }
    return culled;
}

void ObstaclesCallback(std_msgs::msg::Float64MultiArray::SharedPtr obs_msg)
{
    if (!is_initialized) return;

    const std::vector<double>* obs_to_use = &obs_msg->data;
    std::vector<double> culled;

    if (mpc_params.use_corridor_culling && mpc_path_cache.size() >= 2) {
        culled = CullObstaclesToCorridor(
            obs_msg->data, mpc_path_cache, mpc_params.corridor_half_width);
        obs_to_use = &culled;
    }

    if (mpc_params.visualize_culled_obstacles && culled_obs_marker_pub) {
        visualization_msgs::msg::MarkerArray marker_array;
        // Delete all previous markers.
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = node->now();
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        // Add one cube per obstacle cluster in the culled set.
        const int num_obs = static_cast<int>(obs_to_use->size()) / 3;
        for (int i = 0; i < num_obs; i++) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = node->now();
            m.id = i + 1; // 0 is reserved for the DELETEALL marker
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = (*obs_to_use)[3 * i];
            m.pose.position.y = (*obs_to_use)[3 * i + 1];
            m.pose.position.z = 0.0;
            m.scale.x = (*obs_to_use)[3 * i + 2];
            m.scale.y = (*obs_to_use)[3 * i + 2];
            m.scale.z = (*obs_to_use)[3 * i + 2];
            m.color.r = 1.0f;
            m.color.g = 0.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
            marker_array.markers.push_back(m);
        }
        culled_obs_marker_pub->publish(marker_array);
    }

    jl_value_t* obs_type = jl_apply_array_type((jl_value_t*)jl_float64_type, 1);
    double* obs_arr = const_cast<double*>(obs_to_use->data());
    jl_array_t *obs_arg = jl_ptr_to_array_1d(obs_type, obs_arr, obs_to_use->size(), 0);

    jl_call1(j_set_obstacles, (jl_value_t*)obs_arg);
    CATCH_JULIA_EXCEPTION;
}

void GoalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point_stamped_msg)
{
    double x = point_stamped_msg->point.x;
    double y = point_stamped_msg->point.y;

    jl_value_t *j_x = jl_box_float64(x);
    jl_value_t *j_y = jl_box_float64(y);

    jl_call2(j_set_goal_point, j_x, j_y);
    CATCH_JULIA_EXCEPTION;
}

void GoalPointEndOfGlobalPathCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool flag = msg->data;
    jl_value_t *j_flag = jl_box_bool(flag);

    jl_call1(j_set_goal_point_is_end_of_global_path, j_flag);
    CATCH_JULIA_EXCEPTION;
}

void HeadingCallback(std_msgs::msg::Float64::SharedPtr heading_msg)
{
    double psi = heading_msg->data;

    jl_value_t *j_psi = jl_box_float64(psi);

    jl_call1(j_set_heading, j_psi);
    CATCH_JULIA_EXCEPTION;
}

void FinalHeadingCallback(std_msgs::msg::Float64::SharedPtr heading_msg)
{
    double theta = heading_msg->data;

    jl_value_t *j_theta = jl_box_float64(theta);

    jl_call1(j_set_final_heading, j_theta);
    CATCH_JULIA_EXCEPTION;
}

void SpeedCallback(std_msgs::msg::Float64::SharedPtr speed_msg)
{
    double speed = speed_msg->data;

    jl_value_t *j_speed = jl_box_float64(speed);

    jl_call1(j_set_speed, j_speed);
    CATCH_JULIA_EXCEPTION;
}

void SinkageCallback(avt_341_msgs::msg::Sinkage::SharedPtr sinkage_msg)
{
    double sinkage = sinkage_msg->n;

    jl_value_t *j_sinkage = jl_box_float64(sinkage);

    jl_call1(j_set_sinkage, j_sinkage);
    CATCH_JULIA_EXCEPTION;
}

void SegCallback(std_msgs::msg::Float64MultiArray::SharedPtr seg_msg)
{
    if (!is_initialized) return;

    jl_value_t* seg_type = jl_apply_array_type((jl_value_t*)jl_float64_type, 1);
    double* seg_arr = const_cast<double*>(&seg_msg->data[0]);
    jl_array_t *seg_arg = jl_ptr_to_array_1d(seg_type, seg_arr, seg_msg->data.size(), 0);
    jl_value_t *seg_res =
        jl_box_float64(mpc_params.segmentation_resolution);

    jl_call2(j_set_segmentation, (jl_value_t*)seg_arg, seg_res);
    CATCH_JULIA_EXCEPTION;

    recv_seg_input = true;
}

void TerrainSlopeCallback(std_msgs::msg::Float64::SharedPtr terrain_slope_msg)
{
    double terrain_slope = terrain_slope_msg->data;

    jl_value_t *j_terrain_slope = jl_box_float64(terrain_slope);

    jl_call1(j_set_terrain_slope, j_terrain_slope);
    CATCH_JULIA_EXCEPTION;
}

void TerrainRMSCallback(std_msgs::msg::Float64::SharedPtr terrain_rms_msg)
{
    double terrain_rms = terrain_rms_msg->data;

    jl_value_t *j_terrain_rms = jl_box_float64(terrain_rms);

    jl_call1(j_set_terrain_rms, j_terrain_rms);
    CATCH_JULIA_EXCEPTION;
}

void LeaderOdomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    double speed = msg->twist.twist.linear.x;
    jl_value_t *j_speed = jl_box_float64(speed);
    jl_call1(j_set_leader_speed, j_speed);
    CATCH_JULIA_EXCEPTION;

    double lx = msg->pose.pose.position.x;
    double ly = msg->pose.pose.position.y;
    double qw = msg->pose.pose.orientation.w;
    double qxi = msg->pose.pose.orientation.x;
    double qyi = msg->pose.pose.orientation.y;
    double qzi = msg->pose.pose.orientation.z;
    double lyaw = std::atan2(2.0*(qw*qzi + qxi*qyi), 1.0 - 2.0*(qyi*qyi + qzi*qzi));
    double lyaw_rate = msg->twist.twist.angular.z;
    jl_value_t *pose_args[4] = {
        jl_box_float64(lx), jl_box_float64(ly),
        jl_box_float64(lyaw), jl_box_float64(lyaw_rate)
    };
    jl_call(j_set_leader_pose, pose_args, 4);
    CATCH_JULIA_EXCEPTION;
}

void FollowerStatusCallback(avt_341_msgs::msg::FollowerStatus::SharedPtr msg)
{
    jl_value_t *j_xo = jl_box_float64(msg->x_offset);
    jl_value_t *j_yo = jl_box_float64(msg->y_offset);
    jl_call2(j_set_formation_offset, j_xo, j_yo);
    CATCH_JULIA_EXCEPTION;
}

void LeaderStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool status = !(msg->data);
    jl_value_t *j_status = jl_box_bool(status);
    jl_call1(j_set_follower_status, j_status);
    CATCH_JULIA_EXCEPTION;
}

nav_msgs::msg::Path GetMPCPath()
{
    jl_array_t *j_path = (jl_array_t*)jl_call0(j_get_path);
    CATCH_JULIA_EXCEPTION;
    double *path = (double*)jl_array_data(j_path);
    size_t path_len = jl_array_dim(j_path,0);

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = node->now();
    for (int i=0; i<path_len; i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "odom";
        pose.pose.position.x = path[path_len*0 + i];
        pose.pose.position.y = path[path_len*1 + i];
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        path_msg.poses.push_back(pose);
    }
    return path_msg;
}

std_msgs::msg::Float64 GetMPCSpeed()
{
    double speed = jl_unbox_float64(jl_call0(j_get_speed));
    CATCH_JULIA_EXCEPTION;
    
    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = speed;
    return speed_msg;
}

std_msgs::msg::Float64 GetMPCFinalSpeed()
{
    double final_speed = jl_unbox_float64(jl_call0(j_get_final_speed));
    CATCH_JULIA_EXCEPTION;
    
    std_msgs::msg::Float64 final_speed_msg;
    final_speed_msg.data = final_speed;
    return final_speed_msg;
}

std_msgs::msg::Float64 GetMPCSteering()
{
    double steering = jl_unbox_float64(jl_call0(j_get_steering));
    CATCH_JULIA_EXCEPTION;
    
    std_msgs::msg::Float64 steering_msg;
    steering_msg.data = steering;
    return steering_msg;
}

ackermann_msgs::msg::AckermannDriveStamped GetMPCDrive()
{
    double speed = jl_unbox_float64(jl_call0(j_get_speed));
    double steering = jl_unbox_float64(jl_call0(j_get_steering));
    CATCH_JULIA_EXCEPTION;
    
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.frame_id = "avt_341";
    drive_msg.header.stamp = node->now();
    drive_msg.drive.speed = speed;
    drive_msg.drive.steering_angle = steering;
    return drive_msg;
}

std_msgs::msg::Float64MultiArray GetMPCHeading()
{
  
    jl_array_t *j_heading = (jl_array_t*)jl_call0(j_get_heading);
    CATCH_JULIA_EXCEPTION;

    double *heading_data = (double*)jl_array_data(j_heading);
    size_t heading_len = jl_array_dim(j_heading, 0);
    std_msgs::msg::Float64MultiArray heading_msg;
    heading_msg.data.resize(heading_len);

    for (size_t i = 0; i < heading_len; i++) {
        heading_msg.data[i] = heading_data[i];
    }
    return heading_msg;
}

std_msgs::msg::Bool GetSlopeLimited()
{
    bool slope_limited = jl_unbox_bool(jl_call0(j_get_slope_limited));
    CATCH_JULIA_EXCEPTION;
    
    std_msgs::msg::Bool slope_limited_msg;
    slope_limited_msg.data = slope_limited;
    return slope_limited_msg;
}

bool NewInputAvailable() {
    return recv_veh_input;
}

void PublishPath() {}

void Plan() {}

void InitialiseJuliaAPI()
{
    // Initialise the Julia C bindings
    // -------------------------------
    jl_options.handle_signals = JL_OPTIONS_HANDLE_SIGNALS_OFF;

    // ------------------------------------------------------
    // ----------[ Initialize Julia system image. ]----------
    if (mpc_params.sysimage_path.empty())
    {
        RCLCPP_INFO(node->get_logger(), "Loading Julia system image at %s ...", MPC_SYSIMAGE_PATH);
        jl_init_with_image(NULL, MPC_SYSIMAGE_PATH);
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Loading Julia system image at %s ...", mpc_params.sysimage_path.c_str());
        jl_init_with_image(NULL, mpc_params.sysimage_path.c_str());
    }
    CATCH_JULIA_EXCEPTION;
    // ----------[ Initialize Julia system image. ]----------
    // ------------------------------------------------------

    // ----------------------------------------------------------
    // ----------[ Load the Julia MPC planner module. ]----------
    if (!mpc_params.planner_module_path.empty())
    {
        RCLCPP_INFO(node->get_logger(), "Loading Julia module from user-defined path at: %s ...", mpc_params.planner_module_path.c_str());
    }
    else if (!strlen(MPC_PLANNER_MODULE_PATH) == 0)
    {
        RCLCPP_INFO(node->get_logger(), "No absolute path to the Julia module was defined. Reverting to "
            "CMake compile definition, defined at: %s", MPC_PLANNER_MODULE_PATH);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "No valid path to the Julia module could be found. Check your "
            "CMake build log for variable MPC_PLANNER_MODULE_PATH or define the "
            "parameter ~julia_planner_module_path manually.");
        has_error = EXIT_FAILURE;
        jl_atexit_hook(has_error);
        throw std::invalid_argument(
            "No valid path to the Julia MPC module could be found.");
    }

    RCLCPP_INFO(node->get_logger(), "Loading Julia planner module at: %s", MPC_PLANNER_MODULE_PATH);
    std::string planner_module_include_command(std::string("Base.include(Main, \"") + MPC_PLANNER_MODULE_PATH +
                                               std::string("\")"));
    jl_eval_string(planner_module_include_command.c_str());
    // ----------[ Load the Julia MPC planner module. ]----------
    // ----------------------------------------------------------

    // -------------------------------------------------------------
    // ----------[ Load the Julia MPC parameters module. ]----------
    if (!mpc_params.parameters_module_path.empty())
    {
        RCLCPP_INFO(node->get_logger(), "Loading Julia MPC parameters module from user-defined path at: %s ...", mpc_params.parameters_module_path.c_str());
    }
    else if (!strlen(MPC_PARAMETERS_MODULE_PATH) == 0)
    {
        RCLCPP_INFO(node->get_logger(), "No absolute path to the Julia MPC parameters module was defined. Reverting to "
            "CMake compile definition, defined at: %s", MPC_PARAMETERS_MODULE_PATH);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "No valid path to the Julia MPC parameters module could be found. Check your "
            "CMake build log for variable MPC_PARAMETERS_MODULE_PATH or define the "
            "parameter ~julia_parameters_module_path manually.");
        has_error = EXIT_FAILURE;
        jl_atexit_hook(has_error);
        throw std::invalid_argument(
            "No valid path to the Julia MPC parameters module could be found.");
    }

    RCLCPP_INFO(node->get_logger(), "Loading Julia MPC parameters module at: %s", MPC_PARAMETERS_MODULE_PATH);

    std::string parameters_module_include_command(std::string("Base.include(Main.MPC, \"") + MPC_PARAMETERS_MODULE_PATH +
                                                  std::string("\")"));
    jl_eval_string(parameters_module_include_command.c_str());
    // ----------[ Load the Julia MPC parameters module. ]----------
    // -------------------------------------------------------------

    // ---------------------------------------------------------
    // ----------[ Load the Julia MPC models module. ]----------
    if (!mpc_params.models_module_path.empty())
    {
        RCLCPP_INFO(node->get_logger(), "Loading Julia MPC models module from user-defined path at: %s ...", mpc_params.models_module_path.c_str());
    }
    else if (!strlen(MPC_MODELS_MODULE_PATH) == 0)
    {
        RCLCPP_INFO(node->get_logger(), "No absolute path to the Julia MPC models module was defined. Reverting to "
            "CMake compile definition, defined at: %s", MPC_MODELS_MODULE_PATH);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "No valid path to the Julia MPC models module could be found. Check your "
            "CMake build log for variable MPC_MODELS_MODULE_PATH or define the "
            "parameter ~julia_models_module_path manually.");
        has_error = EXIT_FAILURE;
        jl_atexit_hook(has_error);
        throw std::invalid_argument(
            "No valid path to the Julia MPC models module could be found.");
    }

    RCLCPP_INFO(node->get_logger(), "Loading Julia MPC models module at: %s", MPC_MODELS_MODULE_PATH);
    RCLCPP_INFO(node->get_logger(), "Using linear solver: %s", mpc_params.linear_solver.c_str());

    std::string models_module_include_command(std::string("Base.include(Main.MPC, \"") + MPC_MODELS_MODULE_PATH +
                                                  std::string("\")"));
    jl_eval_string(models_module_include_command.c_str());
    // ----------[ Load the Julia MPC models module. ]----------
    // ---------------------------------------------------------

    jl_eval_string("using Main.MPC");

    // Define the Julia module.
    mpc_module = (jl_module_t *)jl_eval_string("Main.MPC");

    // Define the Julia functions.
    j_setup = jl_get_function(mpc_module, "Setup");
    j_plan = jl_get_function(mpc_module, "Plan");
    j_set_state = jl_get_function(mpc_module, "SetState");
    j_set_obstacles = jl_get_function(mpc_module, "SetObstacles");
    j_set_goal_point = jl_get_function(mpc_module, "SetGoalPoint");
    j_set_heading = jl_get_function(mpc_module, "SetHeading");
    j_set_speed = jl_get_function(mpc_module, "SetSpeedSetpoint");
    j_set_sinkage = jl_get_function(mpc_module, "SetSinkage");
    j_set_terrain_slope = jl_get_function(mpc_module, "SetTerrainSlope");
    j_set_terrain_rms = jl_get_function(mpc_module, "SetTerrainRMS");
    j_set_segmentation = jl_get_function(mpc_module, "SetSegmentation");
    j_get_path = jl_get_function(mpc_module, "GetPath");
    j_get_speed = jl_get_function(mpc_module, "GetSpeed");
    j_get_final_speed = jl_get_function(mpc_module, "GetFinalSpeed");
    j_get_steering = jl_get_function(mpc_module, "GetSteering");
    j_get_heading = jl_get_function(mpc_module, "GetHeading");
    j_get_slope_limited = jl_get_function(mpc_module, "GetSlopeLimited");
    j_set_leader_speed = jl_get_function(mpc_module, "SetLeaderSpeed");
    j_set_follower_status = jl_get_function(mpc_module, "SetFollowerStatus");
    j_set_w_final_speed = jl_get_function(mpc_module, "SetWFinalSpeed");
    j_set_final_heading = jl_get_function(mpc_module, "SetFinalHeading");
    j_set_w_final_heading = jl_get_function(mpc_module, "SetWFinalHeading");
    j_set_goal_point_is_end_of_global_path = jl_get_function(mpc_module, "SetGoalPointIsEndOfGlobalPath");
    j_set_leader_pose = jl_get_function(mpc_module, "SetLeaderPose");
    j_set_formation_offset = jl_get_function(mpc_module, "SetFormationOffset");

    // [PARAM SETTERS]
    j_set_tire_model = jl_get_function(mpc_module, "SetTireModel");
    j_set_num_col_points = jl_get_function(mpc_module, "SetNumColPoints");
    j_set_prediction_time_horizon = jl_get_function(mpc_module, "SetPredictionTimeHorizon");
    j_set_max_num_obs = jl_get_function(mpc_module, "SetMaxNumObs");
    j_set_max_num_seg = jl_get_function(mpc_module, "SetMaxNumSeg");
    j_set_sigma = jl_get_function(mpc_module, "SetSigma");
    j_set_min_speed = jl_get_function(mpc_module, "SetMinSpeed");
    j_set_max_speed = jl_get_function(mpc_module, "SetMaxSpeed");
    j_set_use_hard_constraints = jl_get_function(mpc_module, "SetUseHardConstraints");
    j_set_use_segmentation = jl_get_function(mpc_module, "SetUseSegmentation");
    j_set_w_distance_to_obstacles = jl_get_function(mpc_module, "SetWDistanceToObstacles");
    j_set_w_distance_to_goal = jl_get_function(mpc_module, "SetWDistanceToGoal");
    j_set_w_deviation_in_yaw = jl_get_function(mpc_module, "SetWDeviationInYaw");
    j_set_w_yaw_accel = jl_get_function(mpc_module, "SetWYawAccel");
    j_set_w_traversability_cost = jl_get_function(mpc_module, "SetWTraversabilityCost");
    j_set_safety_margin = jl_get_function(mpc_module, "SetSafetyMargin");
    j_set_grid_resolution = jl_get_function(mpc_module, "SetGridResolution");
    j_set_front_angle_goal = jl_get_function(mpc_module, "SetFrontAngleGoal");
    j_set_front_angle_obstacle = jl_get_function(mpc_module, "SetFrontAngleObstacle");
    j_set_terrain_adaptive = jl_get_function(mpc_module, "SetTerrainAdaptive");
    // j_set_veh_front_axle_dist = jl_get_function(mpc_module, "SetVehFrontAxleDist");
    j_set_front_angle_segmentation = jl_get_function(mpc_module, "SetFrontAngleSeg");
    j_set_linear_solver = jl_get_function(mpc_module, "SetLinearSolver");
    j_set_slope_threshold = jl_get_function(mpc_module, "SetSlopeThreshold");
    j_set_rms_threshold = jl_get_function(mpc_module, "SetRMSThreshold");
    j_set_speed_around_large_slopes_and_rms = jl_get_function(mpc_module, "SetSpeedAroundLargeSlopesAndRMS");
    j_set_sa_min = jl_get_function(mpc_module, "SetSteeringAngleMin");
    j_set_sa_max = jl_get_function(mpc_module, "SetSteeringAngleMax");
    j_set_sr_min = jl_get_function(mpc_module, "SetSteeringRateMin");
    j_set_sr_max = jl_get_function(mpc_module, "SetSteeringRateMax");
    j_set_ax_max = jl_get_function(mpc_module, "SetAxMax");
    // -------------------------------

    // Convert params to Julia types
    jl_value_t *j_tire_model =
        jl_cstr_to_string(mpc_params.tire_model.c_str());
    jl_value_t *j_num_col_points =
        jl_box_int32(static_cast<int32_t>(mpc_params.num_col_points));
    jl_value_t *j_prediction_time_horizon =
        jl_box_float64(mpc_params.prediction_time_horizon);
    jl_value_t *j_max_num_obs =
        jl_box_int32(static_cast<int32_t>(mpc_params.max_num_obs));
    jl_value_t *j_max_num_seg =
        jl_box_int32(static_cast<int32_t>(mpc_params.max_num_seg));
    jl_value_t *j_sigma =
        jl_box_float64(1.414214 * mpc_params.grid_resolution);
    jl_value_t *j_min_speed = jl_box_float64(mpc_params.min_speed);
    jl_value_t *j_max_speed = jl_box_float64(mpc_params.max_speed);
    jl_value_t *j_use_hard_constraints =
        jl_box_int32(mpc_params.use_hard_constraints);
    jl_value_t *j_use_segmentation =
        jl_box_int32(mpc_params.use_segmentation);
    jl_value_t *j_w_distance_to_obstacles =
        jl_box_float64(mpc_params.w_distance_to_obstacles);
    jl_value_t *j_w_distance_to_goal =
        jl_box_float64(mpc_params.w_distance_to_goal);
    jl_value_t *j_w_deviation_in_yaw =
        jl_box_float64(mpc_params.w_deviation_in_yaw);
    jl_value_t *j_w_yaw_accel =
        jl_box_float64(mpc_params.w_yaw_accel);
    jl_value_t *j_w_traversability_cost =
        jl_box_float64(mpc_params.w_traversability_cost);
    jl_value_t *j_safety_margin =
        jl_box_float64(mpc_params.safety_margin);
    jl_value_t *j_grid_resolution =
        jl_box_float64(mpc_params.grid_resolution);
    jl_value_t *j_w_final_speed =
        jl_box_float64(mpc_params.w_final_speed);
    jl_value_t *j_w_final_heading =
        jl_box_float64(mpc_params.w_final_heading);
    jl_value_t *j_front_angle_goal =
        jl_box_float64(mpc_params.front_angle_goal);
    jl_value_t *j_front_angle_obstacle =
        jl_box_float64(mpc_params.front_angle_obstacle);
    jl_value_t *j_adaptive = jl_box_int32(mpc_params.adaptive);
    jl_value_t *j_front_angle_segmentation =
        jl_box_float64(mpc_params.front_angle_segmentation);
    jl_value_t *j_linear_solver =
        jl_cstr_to_string(mpc_params.linear_solver.c_str());
    jl_value_t *j_slope_threshold =
        jl_box_float64(mpc_params.slope_threshold);
    jl_value_t *j_rms_threshold =
        jl_box_float64(mpc_params.rms_threshold);
    jl_value_t *j_speed_around_large_slopes_and_rms =
        jl_box_float64(mpc_params.speed_around_large_slopes_and_rms);
    jl_value_t *j_sa_min = jl_box_float64(mpc_params.sa_min);
    jl_value_t *j_sa_max = jl_box_float64(mpc_params.sa_max);
    jl_value_t *j_sr_min = jl_box_float64(mpc_params.sr_min);
    jl_value_t *j_sr_max = jl_box_float64(mpc_params.sr_max);
    jl_value_t *j_ax_max = jl_box_float64(mpc_params.ax_max);

    // Set Julia parameters
    jl_call1(j_set_tire_model, j_tire_model);
    jl_call1(j_set_num_col_points, j_num_col_points);
    jl_call1(j_set_prediction_time_horizon, j_prediction_time_horizon);
    jl_call1(j_set_max_num_obs, j_max_num_obs);
    jl_call1(j_set_max_num_seg, j_max_num_seg);
    jl_call1(j_set_sigma, j_sigma);
    jl_call1(j_set_min_speed, j_min_speed);
    jl_call1(j_set_max_speed, j_max_speed);
    jl_call1(j_set_use_hard_constraints, j_use_hard_constraints);
    jl_call1(j_set_use_segmentation, j_use_segmentation);
    jl_call1(j_set_w_distance_to_obstacles, j_w_distance_to_obstacles);
    jl_call1(j_set_w_distance_to_goal, j_w_distance_to_goal);
    jl_call1(j_set_w_deviation_in_yaw, j_w_deviation_in_yaw);
    jl_call1(j_set_w_yaw_accel, j_w_yaw_accel);
    jl_call1(j_set_w_traversability_cost, j_w_traversability_cost);
    jl_call1(j_set_safety_margin, j_safety_margin);
    jl_call1(j_set_w_final_speed, j_w_final_speed);
    jl_call1(j_set_w_final_heading, j_w_final_heading);
    jl_call1(j_set_grid_resolution, j_grid_resolution);
    jl_call1(j_set_front_angle_goal, j_front_angle_goal);
    jl_call1(j_set_front_angle_obstacle, j_front_angle_obstacle);
    jl_call1(j_set_terrain_adaptive, j_adaptive);
    // jl_call1(j_set_veh_front_axle_dist, j_vehicle_axle_distance_front);
    jl_call1(j_set_front_angle_segmentation, j_front_angle_segmentation);
    jl_call1(j_set_linear_solver, j_linear_solver);
    jl_call1(j_set_slope_threshold, j_slope_threshold);
    jl_call1(j_set_rms_threshold, j_rms_threshold);
    jl_call1(j_set_speed_around_large_slopes_and_rms, j_speed_around_large_slopes_and_rms);
    jl_call1(j_set_sa_min, j_sa_min);
    jl_call1(j_set_sa_max, j_sa_max);
    jl_call1(j_set_sr_min, j_sr_min);
    jl_call1(j_set_sr_max, j_sr_max);
    jl_call1(j_set_ax_max, j_ax_max);
    CATCH_JULIA_EXCEPTION;
}

void InitialisePlanner()
{
    RCLCPP_INFO(node->get_logger(), "Initializing MPC planner.");

    // Initialise the planner
    // ----------------------
    jl_call0(j_setup);
    CATCH_JULIA_EXCEPTION;
    // ----------------------

    is_initialized = true;
    RCLCPP_INFO(node->get_logger(), "MPC planner initialized.");
}

void UpdateCostFnWeights(
    const avt_341::params::mpc_local_planner::Params& params) {
    mpc_params.w_distance_to_obstacles = params.w_distance_to_obstacles;
    mpc_params.w_distance_to_goal = params.w_distance_to_goal;
    mpc_params.w_deviation_in_yaw = params.w_deviation_in_yaw;
    mpc_params.w_yaw_accel = params.w_yaw_accel;
    mpc_params.w_traversability_cost = params.w_traversability_cost;
    mpc_params.w_final_speed = params.w_final_speed;
    mpc_params.w_final_heading = params.w_final_heading;

    jl_call1(j_set_w_distance_to_obstacles,
             jl_box_float64(mpc_params.w_distance_to_obstacles));
    jl_call1(j_set_w_distance_to_goal,
             jl_box_float64(mpc_params.w_distance_to_goal));
    jl_call1(j_set_w_deviation_in_yaw,
             jl_box_float64(mpc_params.w_deviation_in_yaw));
    jl_call1(j_set_w_yaw_accel,
             jl_box_float64(mpc_params.w_yaw_accel));
    jl_call1(j_set_w_traversability_cost,
             jl_box_float64(mpc_params.w_traversability_cost));
    jl_call1(j_set_w_final_speed,
             jl_box_float64(mpc_params.w_final_speed));
    jl_call1(j_set_w_final_heading,
             jl_box_float64(mpc_params.w_final_heading));
    CATCH_JULIA_EXCEPTION;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("avt_341_mpc_wrapper_node");
    avt_341::params::mpc_local_planner::ParamsListener param_listener(
        node);
    mpc_params = param_listener.get_params();

    // Initialise the Julia C API.
    InitialiseJuliaAPI();

    // Register subscriptions
    // ----------------------
    auto veh_state_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>("avt_341/veh",1,VehicleStateCallback);
    auto obs_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>("avt_341/obstacle_clusters",1,ObstaclesCallback);
    auto goal_pt_sub = node->create_subscription<geometry_msgs::msg::PointStamped>("avt_341/mpc_goalPoint",1,GoalPointCallback);
    auto goal_end_sub = node->create_subscription<std_msgs::msg::Bool>("avt_341/mpc_goalPoint_is_end_of_global_path", 1, GoalPointEndOfGlobalPathCallback);
    auto head_sub = node->create_subscription<std_msgs::msg::Float64>("avt_341/mpc_desiredHeading",1,HeadingCallback);
    auto final_head_sub = node->create_subscription<std_msgs::msg::Float64>("avt_341/mpc_final_heading",1,FinalHeadingCallback);
    auto speed_sub = node->create_subscription<std_msgs::msg::Float64>("avt_341/speed_setpoint",1,SpeedCallback);
    auto sink_sub = node->create_subscription<avt_341_msgs::msg::Sinkage>("avt_341/sinkage",1,SinkageCallback);
    auto seg_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>("avt_341/segmentation_cells",1,SegCallback);
    auto reset_sub = node->create_subscription<std_msgs::msg::String>("avt_341/reset",1,ResetCallback);
    auto terrain_slope_sub = node->create_subscription<std_msgs::msg::Float64>("avt_341/terrain_slope",1,TerrainSlopeCallback);
    auto terrain_rms_sub = node->create_subscription<std_msgs::msg::Float64>("avt_341/terrain_rms",1,TerrainRMSCallback);
    auto leader_odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("avt_341/leader_odometry",1,LeaderOdomCallback);
    auto leader_status_sub = node->create_subscription<std_msgs::msg::Bool>("avt_341/leader_status",1,LeaderStatusCallback);
    auto follower_status_sub = node->create_subscription<avt_341_msgs::msg::FollowerStatus>("avt_341/follower_status",1,FollowerStatusCallback);

    // Register publishers
    // -------------------.
    auto path_pub = node->create_publisher<nav_msgs::msg::Path>("avt_341/local_path", 1);
    auto speed_pub = node->create_publisher<std_msgs::msg::Float64>("avt_341/desired_speed",1);
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> steer_pub = nullptr;
    if (mpc_params.publish_steering_commands) {
        steer_pub = node->create_publisher<std_msgs::msg::Float64>("avt_341/cmd_steer", 1);
    }
    auto drive_pub = node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("avt_341/drive", 1);
    auto heading_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("avt_341/mpc_heading_trajectory", 1); 
    auto reset_ack_pub = node->create_publisher<std_msgs::msg::String>("avt_341/reset_ack", 1);
    auto slope_limited_pub = node->create_publisher<std_msgs::msg::Bool>("avt_341/mpc_slope_limited", 1);
    if (mpc_params.visualize_culled_obstacles) {
        culled_obs_marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("avt_341/culled_obstacle_markers", 1);
    }

    RCLCPP_INFO(node->get_logger(), "Julia API initialized. Running main loop.");

    RCLCPP_INFO(node->get_logger(), "Node running at %.2f Hz.", mpc_params.rate);

    RCLCPP_INFO(node->get_logger(), "Number of collocation points: %lld.", static_cast<long long>(mpc_params.num_col_points));

    RCLCPP_INFO(node->get_logger(), "Prediction time horizon: %.1f.", mpc_params.prediction_time_horizon);

    rclcpp::Rate node_rate(mpc_params.rate);
    while (rclcpp::ok() && !has_error)
    {
        auto updated_params = mpc_params;
        if (param_listener.try_update_params(updated_params)) {
            UpdateCostFnWeights(updated_params);
        }

        if (NewInputAvailable()) {
            if (!is_initialized) {
                InitialisePlanner();
            }

            // Update Julia MPC planner
            jl_call0(j_plan);
            CATCH_JULIA_EXCEPTION;

            // Publish MPC outputs and cache path for obstacle corridor culling
            auto mpc_path_msg = GetMPCPath();
            mpc_path_cache.clear();
            for (const auto& pose : mpc_path_msg.poses) {
                mpc_path_cache.emplace_back(pose.pose.position.x, pose.pose.position.y);
            }
            path_pub->publish(mpc_path_msg);
            speed_pub->publish(GetMPCSpeed());
            if (mpc_params.publish_steering_commands) {
                steer_pub->publish(GetMPCSteering());
            }
            drive_pub->publish(GetMPCDrive());
	        heading_pub->publish(GetMPCHeading());
            slope_limited_pub->publish(GetSlopeLimited());
        }

        if(reset_called && is_initialized) {
            // Nothing to reset currently
            RCLCPP_INFO(node->get_logger(), "Resetting MPC local planner.");
            std_msgs::msg::String reset_ack_msg;
            reset_ack_msg.data = avt_341::node::NodeType::LocalPlanner;
            reset_ack_pub->publish(reset_ack_msg);
            reset_called = false;
        }

        rclcpp::spin_some(node);
        node_rate.sleep();
    }

    // Cleanup
    // -------

    // Exit the Julia C bindings cleanly.
    jl_atexit_hook(has_error);

    // Ensure the shared pointer to the ROS node is reset to avoid exceptions
    // from ROS.
    node.reset();

    return has_error;
}
