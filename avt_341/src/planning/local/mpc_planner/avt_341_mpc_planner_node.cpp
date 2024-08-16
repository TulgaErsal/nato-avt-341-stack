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

// This call must be included in the ROS node executable before initialising
// the Julia C bindings and is required for fast execution of wrapped Julia
// code.
JULIA_DEFINE_FAST_TLS();

void CatchJuliaException()
{
    // Catch exceptions from the Julia function call.
    if (jl_exception_occurred())
    {
        node->log_error("Julia module has thrown an exception: %s",
                        jl_typeof_str(jl_exception_occurred()));
        has_error = true;
    }
}

void VehicleStateCallback(avt_341::msg::Float64MultiArrayPtr f64_ma_msg)
{
    jl_value_t* array_type = jl_apply_array_type((jl_value_t*)jl_float64_type, 1);
    jl_array_t *veh_data = jl_ptr_to_array_1d(array_type, &f64_ma_msg->data[0], 11, 0);

    jl_call1(j_set_state, (jl_value_t*)veh_data);
    CATCH_JULIA_EXCEPTION;

    recv_veh_input = true;
}

void ObstaclesCallback(avt_341::msg::Float64MultiArrayPtr obs_msg)
{
    jl_value_t* obs_type = jl_apply_array_type((jl_value_t*)jl_float64_type, 1);
    jl_array_t *obs_arg = jl_ptr_to_array_1d(obs_type, &obs_msg->data[0], obs_msg->data.size(), 0);

    jl_call1(j_set_obstacles, (jl_value_t*)obs_arg);
    CATCH_JULIA_EXCEPTION;
}

void GoalPointCallback(avt_341::msg::PointStampedPtr point_stamped_msg)
{
    double x = point_stamped_msg->point.x;
    double y = point_stamped_msg->point.y;

    jl_value_t *j_x = jl_box_float64(x);
    jl_value_t *j_y = jl_box_float64(y);

    jl_call2(j_set_goal_point, j_x, j_y);
    CATCH_JULIA_EXCEPTION;
}

void HeadingCallback(avt_341::msg::Float64Ptr heading_msg)
{
    double psi = heading_msg->data;

    jl_value_t *j_psi = jl_box_float64(psi);

    jl_call1(j_set_heading, j_psi);
    CATCH_JULIA_EXCEPTION;
}

void SpeedCallback(avt_341::msg::Float64Ptr speed_msg)
{
    double speed = speed_msg->data;

    jl_value_t *j_speed = jl_box_float64(speed);

    jl_call1(j_set_speed, j_speed);
    CATCH_JULIA_EXCEPTION;
}

void SinkageCallback(avt_341::msg::SinkagePtr sinkage_msg)
{
    double sinkage = sinkage_msg->n;

    jl_value_t *j_sinkage = jl_box_float64(sinkage);

    jl_call1(j_set_sinkage, j_sinkage);
    CATCH_JULIA_EXCEPTION;
}

avt_341::msg::Path GetMPCPath()
{
    jl_array_t *j_path = (jl_array_t*)jl_call0(j_get_path);
    CATCH_JULIA_EXCEPTION;
    double *path = (double*)jl_array_data(j_path);
    size_t path_len = jl_array_dim(j_path,0);

    avt_341::msg::Path path_msg;
    path_msg.header.frame_id = "odom";
    for (int i=0; i<path_len; i++) {
        avt_341::msg::PoseStamped pose;
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

avt_341::msg::Float64 GetMPCSpeed()
{
    double speed = jl_unbox_float64(jl_call0(j_get_speed));
    CATCH_JULIA_EXCEPTION;
    
    avt_341::msg::Float64 speed_msg;
    speed_msg.data = speed;
    return speed_msg;
}

avt_341::msg::Float64 GetMPCSteering()
{
    double steering = jl_unbox_float64(jl_call0(j_get_steering));
    CATCH_JULIA_EXCEPTION;
    
    avt_341::msg::Float64 steering_msg;
    steering_msg.data = steering;
    return steering_msg;
}

bool NewInputAvailable() {
    return recv_veh_input;
}

void PublishPath() {}

void Plan() {}

void DeclareParameters()
{
    node->get_parameter("~sysimage_path", sysimage_path, std::string());
    node->get_parameter("~julia_planner_planner_module_path", planner_module_path, std::string());
    node->get_parameter("~julia_parameters_module_path", parameters_module_path, std::string());
    node->get_parameter("~julia_models_module_path", models_module_path, std::string());
    node->get_parameter("~rate", rate, 10.0);
    node->get_parameter("~tire_model", tire_model, std::string("L"));
    node->get_parameter("~num_col_points", num_col_points, 10);
    node->get_parameter("~prediction_time_horizon", prediction_time_horizon, 2.0);
    node->get_parameter("~max_num_obs", max_num_obs, 500);
    node->get_parameter("~min_speed", min_speed, 0.5);
    node->get_parameter("~max_speed", max_speed, 3.5);
    node->get_parameter("~stop_on_max_solve_time", stop_on_max_solve_time, true);
    node->get_parameter("~use_hard_constraints", use_hard_constraints, false);
    node->get_parameter("~w_distance_to_obstacles", w_distance_to_obstacles, 5.);
    node->get_parameter("~w_distance_to_goal", w_distance_to_goal, 100.);
    node->get_parameter("~w_deviation_in_yaw", w_deviation_in_yaw, 1.);
    node->get_parameter("~safety_margin", safety_margin, 0.0);
    node->get_parameter("~grid_resolution", grid_resolution, 0.25);
    node->get_parameter("~front_angle_goal", front_angle_goal, 1.571);
    node->get_parameter("~front_angle_obstacle", front_angle_obstacle, 1.571);
    node->get_parameter("~adaptive", adaptive, false);
    node->get_parameter("~vehicle_axle_distance_front", vehicle_axle_distance_front, 1.38599 );
}

void InitialiseJuliaAPI()
{
    // Initialise the Julia C bindings
    // -------------------------------
    jl_options.handle_signals = JL_OPTIONS_HANDLE_SIGNALS_OFF;

    // ------------------------------------------------------
    // ----------[ Initialize Julia system image. ]----------
    if (sysimage_path.empty())
    {
        node->log_info("Loading Julia system image at %s ...", MPC_SYSIMAGE_PATH);
        jl_init_with_image(NULL, MPC_SYSIMAGE_PATH);
    }
    else
    {
        node->log_info("Loading Julia system image at %s ...", sysimage_path);
        jl_init_with_image(NULL, sysimage_path.c_str());
    }
    CATCH_JULIA_EXCEPTION;
    node->log_info("Julia C API is now initialised.");
    // ----------[ Initialize Julia system image. ]----------
    // ------------------------------------------------------
    
    // ----------------------------------------------------------
    // ----------[ Load the Julia MPC planner module. ]----------
    if (!planner_module_path.empty())
    {
        node->log_info("Loading Julia module from user-defined path at: %s ...",
                       planner_module_path);
    }
    else if (!strlen(MPC_PLANNER_MODULE_PATH) == 0)
    {
        node->log_warning(
            "No absolute path to the Julia module was defined. Reverting to "
            "CMake compile definition, defined at: %s",
            MPC_PLANNER_MODULE_PATH);
    }
    else
    {
        node->log_error(
            "No valid path to the Julia module could be found. Check your "
            "CMake build log for variable MPC_PLANNER_MODULE_PATH or define the "
            "parameter ~julia_planner_module_path manually.");
        has_error = EXIT_FAILURE;
        jl_atexit_hook(has_error);
        throw std::invalid_argument(
            "No valid path to the Julia MPC module could be found.");
    }

    node->log_info("Loading Julia planner module at: %s", MPC_PLANNER_MODULE_PATH);
    std::string planner_module_include_command(std::string("Base.include(Main, \"") + MPC_PLANNER_MODULE_PATH +
                                               std::string("\")"));
    jl_eval_string(planner_module_include_command.c_str());
    // ----------[ Load the Julia MPC planner module. ]----------
    // ----------------------------------------------------------

    // -------------------------------------------------------------
    // ----------[ Load the Julia MPC parameters module. ]----------
    if (!parameters_module_path.empty())
    {
        node->log_info("Loading Julia MPC parameters module from user-defined path at: %s ...",
                       parameters_module_path);
    }
    else if (!strlen(MPC_PARAMETERS_MODULE_PATH) == 0)
    {
        node->log_warning(
            "No absolute path to the Julia MPC parameters module was defined. Reverting to "
            "CMake compile definition, defined at: %s",
            MPC_PARAMETERS_MODULE_PATH);
    }
    else
    {
        node->log_error(
            "No valid path to the Julia MPC parameters module could be found. Check your "
            "CMake build log for variable MPC_PARAMETERS_MODULE_PATH or define the "
            "parameter ~julia_parameters_module_path manually.");
        has_error = EXIT_FAILURE;
        jl_atexit_hook(has_error);
        throw std::invalid_argument(
            "No valid path to the Julia MPC parameters module could be found.");
    }

    node->log_info("Loading Julia MPC parameters module at: %s", MPC_PARAMETERS_MODULE_PATH);

    std::string parameters_module_include_command(std::string("Base.include(Main.MPC, \"") + MPC_PARAMETERS_MODULE_PATH +
                                                  std::string("\")"));
    jl_eval_string(parameters_module_include_command.c_str());
    // ----------[ Load the Julia MPC parameters module. ]----------
    // -------------------------------------------------------------

    // ---------------------------------------------------------
    // ----------[ Load the Julia MPC models module. ]----------
    if (!models_module_path.empty())
    {
        node->log_info("Loading Julia MPC models module from user-defined path at: %s ...",
                       models_module_path);
    }
    else if (!strlen(MPC_MODELS_MODULE_PATH) == 0)
    {
        node->log_warning(
            "No absolute path to the Julia MPC models module was defined. Reverting to "
            "CMake compile definition, defined at: %s",
            MPC_MODELS_MODULE_PATH);
    }
    else
    {
        node->log_error(
            "No valid path to the Julia MPC models module could be found. Check your "
            "CMake build log for variable MPC_MODELS_MODULE_PATH or define the "
            "parameter ~julia_models_module_path manually.");
        has_error = EXIT_FAILURE;
        jl_atexit_hook(has_error);
        throw std::invalid_argument(
            "No valid path to the Julia MPC models module could be found.");
    }

    node->log_info("Loading Julia MPC models module at: %s", MPC_MODELS_MODULE_PATH);

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
    j_get_path = jl_get_function(mpc_module, "GetPath");
    j_get_speed = jl_get_function(mpc_module, "GetSpeed");
    j_get_steering = jl_get_function(mpc_module, "GetSteering");
    // [PARAM SETTERS]
    j_set_tire_model = jl_get_function(mpc_module, "SetTireModel");
    j_set_num_col_points = jl_get_function(mpc_module, "SetNumColPoints");
    j_set_prediction_time_horizon = jl_get_function(mpc_module, "SetPredictionTimeHorizon");
    j_set_max_num_obs = jl_get_function(mpc_module, "SetMaxNumObs");
    j_set_min_speed = jl_get_function(mpc_module, "SetMinSpeed");
    j_set_max_speed = jl_get_function(mpc_module, "SetMaxSpeed");
    j_set_stop_on_max_solve_time = jl_get_function(mpc_module, "SetStopOnMaxSolveTime");
    j_set_use_hard_constraints = jl_get_function(mpc_module, "SetUseHardConstraints");
    j_set_w_distance_to_obstacles = jl_get_function(mpc_module, "SetWDistanceToObstacles");
    j_set_w_distance_to_goal = jl_get_function(mpc_module, "SetWDistanceToGoal");
    j_set_w_deviation_in_yaw = jl_get_function(mpc_module, "SetWDeviationInYaw");
    j_set_safety_margin = jl_get_function(mpc_module, "SetSafetyMargin");
    j_set_grid_resolution = jl_get_function(mpc_module, "SetGridResolution");
    j_set_front_angle_goal = jl_get_function(mpc_module, "SetFrontAngleGoal");
    j_set_front_angle_obstacle = jl_get_function(mpc_module, "SetFrontAngleObstacle");
    j_set_terrain_adaptive = jl_get_function(mpc_module, "SetTerrainAdaptive");
    j_set_veh_front_axle_dist = jl_get_function(mpc_module, "SetVehFrontAxleDist");
    // -------------------------------
}

void InitialisePlanner()
{
    // Convert params to Julia types
    jl_value_t *j_tire_model = jl_cstr_to_string(tire_model.c_str());
    jl_value_t *j_num_col_points = jl_box_int32(num_col_points);
    jl_value_t *j_prediction_time_horizon = jl_box_float64(prediction_time_horizon);
    jl_value_t *j_max_num_obs = jl_box_int32(max_num_obs);
    jl_value_t *j_min_speed = jl_box_float64(min_speed);
    jl_value_t *j_max_speed = jl_box_float64(max_speed);
    jl_value_t *j_stop_on_max_solve_time = jl_box_int32(stop_on_max_solve_time);
    jl_value_t *j_use_hard_constraints = jl_box_int32(use_hard_constraints);
    jl_value_t *j_w_distance_to_obstacles = jl_box_float64(w_distance_to_obstacles);
    jl_value_t *j_w_distance_to_goal = jl_box_float64(w_distance_to_goal);
    jl_value_t *j_w_deviation_in_yaw = jl_box_float64(w_deviation_in_yaw);
    jl_value_t *j_safety_margin = jl_box_float64(safety_margin);
    jl_value_t *j_grid_resolution = jl_box_float64(grid_resolution);
    jl_value_t *j_front_angle_goal = jl_box_float64(front_angle_goal);
    jl_value_t *j_front_angle_obstacle = jl_box_float64(front_angle_obstacle);
    jl_value_t *j_adaptive = jl_box_int32(adaptive);
    jl_value_t *j_vehicle_axle_distance_front = jl_box_float64(vehicle_axle_distance_front);

    // Set Julia parameters
    jl_call1(j_set_tire_model, j_tire_model);
    jl_call1(j_set_num_col_points, j_num_col_points);
    jl_call1(j_set_prediction_time_horizon, j_prediction_time_horizon);
    jl_call1(j_set_max_num_obs, j_max_num_obs);
    jl_call1(j_set_min_speed, j_min_speed);
    jl_call1(j_set_max_speed, j_max_speed);
    jl_call1(j_set_stop_on_max_solve_time, j_stop_on_max_solve_time);
    jl_call1(j_set_use_hard_constraints, j_use_hard_constraints);
    jl_call1(j_set_w_distance_to_obstacles, j_w_distance_to_obstacles);
    jl_call1(j_set_w_distance_to_goal, j_w_distance_to_goal);
    jl_call1(j_set_w_deviation_in_yaw, j_w_deviation_in_yaw);
    jl_call1(j_set_safety_margin, j_safety_margin);
    jl_call1(j_set_grid_resolution, j_grid_resolution);
    jl_call1(j_set_front_angle_goal, j_front_angle_goal);
    jl_call1(j_set_front_angle_obstacle, j_front_angle_obstacle);
    jl_call1(j_set_terrain_adaptive, j_adaptive);
    jl_call1(j_set_veh_front_axle_dist, j_vehicle_axle_distance_front);
    CATCH_JULIA_EXCEPTION;

    // Initialise the planner
    // ----------------------
    jl_call0(j_setup);
    CATCH_JULIA_EXCEPTION;
    // ----------------------

    is_initialized = true;
}

int main(int argc, char *argv[])
{
    node = avt_341::node::init_node(argc, argv, "avt_341_mpc_wrapper_node");

    // Declare parameters on the ROS parameter server.
    DeclareParameters();

    // Initialise the Julia C API.
    InitialiseJuliaAPI();

    // Register subscriptions
    // ----------------------
    auto veh_state_sub = node->create_subscription<avt_341::msg::Float64MultiArray>("avt_341/veh",1,VehicleStateCallback);
    auto obs_sub = node->create_subscription<avt_341::msg::Float64MultiArray>("avt_341/obstacle_clusters",1,ObstaclesCallback);
    auto goal_pt_sub = node->create_subscription<avt_341::msg::PointStamped>("avt_341/mpc_goalPoint",1,GoalPointCallback);
    auto head_sub = node->create_subscription<avt_341::msg::Float64>("avt_341/mpc_desiredHeading",1,HeadingCallback);
    auto speed_sub = node->create_subscription<avt_341::msg::Float64>("avt_341/speed_setpoint",1,SpeedCallback);
    if (adaptive) {
        auto sink_sub = node->create_subscription<avt_341::msg::Sinkage>("avt_341/sinkage",1,SinkageCallback);
    }
    // Register publishers
    // -------------------.
    auto path_pub = node->create_publisher<avt_341::msg::Path>("avt_341/local_path", 1);
    auto speed_pub = node->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed",1);
    auto steer_pub = node->create_publisher<avt_341::msg::Float64>("avt_341/cmd_steer", 1);

    avt_341::node::Rate node_rate(rate);
    while (avt_341::node::ok() && !has_error)
    {
        if (NewInputAvailable()) {
            if (!is_initialized) {
                InitialisePlanner();
            }

            // Update Julia MPC planner
            jl_call0(j_plan);
            CATCH_JULIA_EXCEPTION;

            // Publish MPC outputs
            path_pub->publish(GetMPCPath());
            speed_pub->publish(GetMPCSpeed());
            steer_pub->publish(GetMPCSteering());
        }

        node->spin_some();
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