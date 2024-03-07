/**
 * @file avt_341_mpc_planner_node.cpp
 *
 * @brief Plan a local trajectory using the model predictive control planner.
 *        This ROS node is a wrapper to the TulgaErsal/AVT-341-MPC planner
 *        through the Julia C API.
 *
 * @date 03/11/2023
 *
 * @author Dario Sirangelo (dsi@mpe.au.dk)
 *         Aarhus University (DK)
 *         Department of Mechanical and Production Engineering
 *         Section Mechatronics & Dynamics
 */

// TODO: The subscribers should really be defined outside of the local scope.
// TODO: Split some of the warning message for better readability.
// TODO: The lock guards should not exclude each other, only the planner update block!

#include <avt_341/planning/local/avt_341_mpc_planner_node.h>

// This call must be included in the ROS node executable before initialising
// the Julia C bindings and is required for fast execution of wrapped Julia
// code.
JULIA_DEFINE_FAST_TLS();

void SinkageCallback(avt_341_msgs::msg::Sinkage::SharedPtr sinkage_msg)
{
    std::lock_guard<std::mutex> guard(planner_mutex);

    double n = sinkage_msg->n;

    jl_value_t *j_n = jl_box_float64(n);

    jl_call1(j_set_sinkage, j_n);
    CATCH_JULIA_EXCEPTION;

    node->log_info("Received sinkage %f", n);
}

void OccupancyGridCallback(avt_341::msg::OccupancyGridPtr occgrid_msg)
{
    std::lock_guard<std::mutex> guard(planner_mutex);
}

void PathCallback(avt_341::msg::PathPtr path_msg)
{
    std::lock_guard<std::mutex> guard(planner_mutex);
}

void ObstaclesCallback(avt_341_msgs::msg::Obstacles::SharedPtr obs_msg)
{
    std::lock_guard<std::mutex> guard(planner_mutex);

    int64_t id = obs_msg->id;
    double obstacle_size_meters = obs_msg->obstacle_size_meters;
    std::vector<double> data = obs_msg->data;

    jl_value_t *j_id = jl_box_int64(id);
    jl_value_t *j_obstacle_size_meters = jl_box_float64(obstacle_size_meters);
    // TODO: Copy the array here.
    jl_value_t *j_data;

    jl_call3(j_set_obstacles, j_id, j_obstacle_size_meters, j_data);
    CATCH_JULIA_EXCEPTION;

    node->log_info("Received %i obstacles with size %f",
                   data.size(),
                   obstacle_size_meters);
}

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

void HeadingCallback(avt_341::msg::Float64Ptr heading_msg)
{
    std::lock_guard<std::mutex> guard(planner_mutex);

    double psi = heading_msg->data;

    jl_value_t *j_psi = jl_box_float64(psi);

    jl_call1(j_set_heading, j_psi);
    CATCH_JULIA_EXCEPTION;

    node->log_info("Updated MPC desired heading %f", psi);
}

void GoalPointCallback(avt_341::msg::PointStampedPtr point_stamped_msg)
{
    std::lock_guard<std::mutex> guard(planner_mutex);

    double x = point_stamped_msg->point.x;
    double y = point_stamped_msg->point.y;

    jl_value_t *j_x = jl_box_float64(x);
    jl_value_t *j_y = jl_box_float64(y);

    jl_call2(j_set_goal_point, j_x, j_y);
    CATCH_JULIA_EXCEPTION;

    node->log_info("Updated MPC goal point %f %f", x, y);
}

void VehicleStateCallback(avt_341::msg::Float64MultiArrayPtr f64_ma_msg)
{
    std::lock_guard<std::mutex> guard(planner_mutex);

    double x = f64_ma_msg->data[1] + l_a * std::cos(f64_ma_msg->data[6]);
    double y = f64_ma_msg->data[2] + l_a * std::sin(f64_ma_msg->data[6]);
    double u = f64_ma_msg->data[3];
    double v = f64_ma_msg->data[4];
    double delta = f64_ma_msg->data[5];
    double psi = f64_ma_msg->data[6];
    double r = f64_ma_msg->data[7];
    double a = f64_ma_msg->data[6];

    jl_value_t **j_args;
    j_args[0] = jl_box_float64(x);
    j_args[1] = jl_box_float64(y);
    j_args[2] = jl_box_float64(u);
    j_args[3] = jl_box_float64(v);
    j_args[4] = jl_box_float64(delta);
    j_args[5] = jl_box_float64(psi);
    j_args[6] = jl_box_float64(r);
    j_args[7] = jl_box_float64(a);

    jl_call(j_set_state, j_args, 8);
    CATCH_JULIA_EXCEPTION;

    node->log_info("Updated vehicle state\nX: %f\nY: %f\nU: %f\nV: %f\nDELTA: "
                   "%f\nPSI: %f\nR: %f\nA: %f",
                   x,
                   y,
                   u,
                   v,
                   delta,
                   psi,
                   r,
                   a);
}
void PublishPath() {}

void Plan() {}

void UpdateState() {}

void DeclareParameters()
{
    // Declare the Julia SysImage path parameter.
    node->get_parameter("~mpc_rate", rate, 10.0);

    // Declare the Julia SysImage path parameter.
    node->get_parameter("~sysimage_path", sysimage_path, std::string());

    // Declare the Julia planner module path parameter.
    node->get_parameter("~julia_planner_planner_module_path", planner_module_path, std::string());

    // Declare the Julia parameters module path parameter.
    node->get_parameter("~julia_parameters_module_path", parameters_module_path, std::string());

    // Declare the Julia parameters module path parameter.
    node->get_parameter("~julia_models_module_path", models_module_path, std::string());

    // Set the minimum speed.
    node->get_parameter("~front_axle_distance", l_a, 0.5);

    // Set the minimum speed.
    node->get_parameter("~speed_min", u_min, 0.5);

    // Set the prediction horizon.
    node->get_parameter("~prediction_horizon", t_span, 5.0);

    // Set the prediction horizon.
    node->get_parameter("~use_terrain_adaptive", use_terrain_adaptive, false);
}
void InitialiseJuliaAPI()
{
    // Initialise the Julia C bindings
    // -------------------------------

    jl_options.handle_signals = JL_OPTIONS_HANDLE_SIGNALS_OFF;

    // If the user provided a Julia system image, use it during the
    // initialisation procedure.
    if (sysimage_path.empty())
    {
        node->log_warning("No Julia system image specified!");
        node->log_info("Initialising Julia C API with no system image.");
        jl_init();
    }
    else
    {
        node->log_info("Loading Julia system image at %s ...", sysimage_path);
        jl_init_with_image(NULL, sysimage_path.c_str());
    }
    CATCH_JULIA_EXCEPTION;
    node->log_info("Julia C API is now initialised.");

    // TODO: Lots of repetition here for the time being - I will wrap this in a
    // function later.
    
    // Load the Julia MPC planner module.
    // First, look for a user-specified absolute path from the ROS parameter
    // server. If none is provided, revert to the path defined during the
    // original CMake build. If no compile definition for the path is provided,
    // terminate the Julia C API and throw an exception.
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
    jl_eval_string("using Main.MPC");

    // Load the Julia MPC parameters module. Similar considerations as for the
    // Julia MPC planner module apply.
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

    // Load the Julia MPC models module. Similar considerations as for the
    // Julia MPC planner module apply.
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

    // Define the Julia module.
    mpc_module = (jl_module_t *)jl_eval_string("Main.MPC");

    // Define the Julia functions.
    j_plan = jl_get_function(mpc_module, "Plan");
    j_set_front_axle_position =
        jl_get_function(mpc_module, "SetFrontAxlePosition");
    j_set_goal_point = jl_get_function(mpc_module, "SetGoalPoint");
    j_set_heading = jl_get_function(mpc_module, "SetHeading");
    j_set_minimum_speed = jl_get_function(mpc_module, "SetMinimumSpeed");
    j_set_prediction_horizon =
        jl_get_function(mpc_module, "SetPredictionHorizon");
    j_use_terrain_adaptive =
        jl_get_function(mpc_module, "SetUseTerrainAdaptive");
    j_set_sinkage = jl_get_function(mpc_module, "SetSinkage");
    j_set_state = jl_get_function(mpc_module, "SetState");
    // -------------------------------
}

void InitialisePlanner()
{
    // Initialise the planner
    // ----------------------

    // Set the prediction horizon in the planner.
    jl_value_t *j_t_span = jl_box_float64(t_span);
    jl_call1(j_set_prediction_horizon, j_t_span);

    // Set the minimum speed in the planner.
    jl_value_t *j_u_min = jl_box_float64(u_min);
    jl_call1(j_set_minimum_speed, j_u_min);

    // Set the position of the front axle in the planner.
    jl_value_t *j_l_a = jl_box_float64(l_a);
    jl_call1(j_set_front_axle_position, j_l_a);
    // ----------------------
}

int main(int argc, char *argv[])
{
    node = avt_341::node::init_node(argc, argv, "avt_341_mpc_wrapper_node");

    // Declare parameters on the ROS parameter server.
    DeclareParameters();

    // Initialise the Julia C API.
    InitialiseJuliaAPI();

    // Initialise planner with the user-specified parameters.
    InitialisePlanner();

    // Register subscriptions
    // ----------------------

    // Register a subscription for the vehicle state.
    auto veh_state_sub =
        node->create_subscription<avt_341::msg::Float64MultiArray>(
            "avt_341/veh",
            1,
            VehicleStateCallback);

    // Register a subscription for the detected obstacles.
    auto obs_sub = node->create_subscription<avt_341_msgs::msg::Obstacles>(
        "avt_341/obstacles",
        1,
        ObstaclesCallback);

    // TODO: This subscription odes not fit the case convention...
    // Register a subscription for the planner goal point.
    auto goal_pt_sub =
        node->create_subscription<geometry_msgs::msg::PointStamped>(
            "avt_341/mpc_goalPoint",
            1,
            GoalPointCallback);

    // TODO: This subscription odes not fit the case convention...
    // Register a subscription for the desired heading.
    auto head_sub =
        node->create_subscription<avt_341::msg::Float64>(
            "avt_341/mpc_desiredHeading",
            1,
            HeadingCallback);

    // Register a subscription for the estimated sinkage..
    if (use_terrain_adaptive)
    {
        auto sink_sub = node->create_subscription<avt_341_msgs::msg::Sinkage>(
            "avt_341/sinkage",
            1,
            SinkageCallback);
    }
    // ----------------------

    // Register publishers
    // -------------------

    // Register a publisher for the optimal path.
    auto path_pub =
        node->create_publisher<avt_341::msg::Path>("avt_341/local_path", 1);

    // Register a publisher for the desired speed.
    auto speed_pub =
        node->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed",
                                                      1);

    // Register a publisher for the desired steering angle.
    auto steer_pub =
        node->create_publisher<avt_341::msg::Float64>("avt_341/cmd_steer", 1);

    // ----------------------

    avt_341::node::Rate node_rate(rate);
    while (avt_341::node::ok() && !has_error)
    {
        jl_call0(j_plan);
        CATCH_JULIA_EXCEPTION;

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