
#include <gtest/gtest.h>
#include <string>
#include <iostream>
#include <vector>

// Julia header throws "No Target Architecture" error otherwise on Windows systems
#ifdef _WIN64
 #define _AMD64_
#endif
#include <julia.h>

// Definitions that should come from CMake
#ifndef MPC_PLANNER_MODULE_PATH
#define MPC_PLANNER_MODULE_PATH ""
#endif
#ifndef MPC_PARAMETERS_MODULE_PATH
#define MPC_PARAMETERS_MODULE_PATH ""
#endif
#ifndef MPC_MODELS_MODULE_PATH
#define MPC_MODELS_MODULE_PATH ""
#endif
#ifndef MPC_SYSIMAGE_PATH
#define MPC_SYSIMAGE_PATH ""
#endif

// Helper to check for Julia exceptions
bool HasJuliaException() {
    if (jl_exception_occurred()) {
        const char *p = jl_string_ptr(jl_eval_string("sprint(showerror, ccall(:jl_exception_occurred, Any, ()))"));
        std::cerr << "Julia Exception: " << p << std::endl;
        return true;
    }
    return false;
}

class JuliaTestEnvironment : public ::testing::Environment {
public:
    void SetUp() override {
        if (!jl_is_initialized()) {
            jl_options.handle_signals = JL_OPTIONS_HANDLE_SIGNALS_OFF;
            if (std::string(MPC_SYSIMAGE_PATH).empty()) {
                jl_init();
            } else {
                jl_init_with_image(NULL, MPC_SYSIMAGE_PATH);
            }
        }
        
        // 1. Load the planner module into Main
        std::string planner_path = MPC_PLANNER_MODULE_PATH;
        if (!planner_path.empty()) {
            std::string command = "Base.include(Main, \"" + planner_path + "\")";
            jl_eval_string(command.c_str());
            if (HasJuliaException()) return;
        }

        // 2. Load the parameters module into Main.MPC
        std::string params_path = MPC_PARAMETERS_MODULE_PATH;
        if (!params_path.empty()) {
            std::string command = "Base.include(Main.MPC, \"" + params_path + "\")";
            jl_eval_string(command.c_str());
            if (HasJuliaException()) return;
        }

        // 3. Load the models module into Main.MPC
        std::string models_path = MPC_MODELS_MODULE_PATH;
        if (!models_path.empty()) {
            std::string command = "Base.include(Main.MPC, \"" + models_path + "\")";
            jl_eval_string(command.c_str());
            if (HasJuliaException()) return;
        }

        jl_eval_string("using Main.MPC");
        HasJuliaException();
    }
};

void SetDefaultMPCParameters(jl_module_t* mpc_module) {
    auto set_int = [&](const char* func_name, int val) {
        jl_function_t* f = jl_get_function(mpc_module, func_name);
        if (f) jl_call1(f, jl_box_int32(val));
    };
    auto set_float = [&](const char* func_name, double val) {
        jl_function_t* f = jl_get_function(mpc_module, func_name);
        if (f) jl_call1(f, jl_box_float64(val));
    };
    auto set_string = [&](const char* func_name, const char* val) {
        jl_function_t* f = jl_get_function(mpc_module, func_name);
        if (f) jl_call1(f, jl_cstr_to_string(val));
    };

    set_string("SetTireModel", "L");
    set_int("SetNumColPoints", 10);
    set_float("SetPredictionTimeHorizon", 2.0);
    set_int("SetMaxNumObs", 50);
    set_int("SetMaxNumSeg", 50);
    set_float("SetSigma", 0.35);
    set_float("SetMinSpeed", 0.5);
    set_float("SetMaxSpeed", 5.0);
    set_int("SetUseHardConstraints", 0);
    set_int("SetUseSegmentation", 0);
    set_float("SetWDistanceToObstacles", 5.0);
    set_float("SetWDistanceToGoal", 100.0);
    set_float("SetWDeviationInYaw", 1.0);
    set_float("SetWYawAccel", 1.0);
    set_float("SetWTraversabilityCost", 0.1);
    set_float("SetSafetyMargin", 0.5);
    set_float("SetGridResolution", 0.25);
    set_float("SetWFinalSpeed", 10.0);
    set_float("SetFrontAngleGoal", 1.57);
    set_float("SetFrontAngleObstacle", 1.57);
    set_int("SetTerrainAdaptive", 0);
    set_float("SetFrontAngleSeg", 1.57);
    set_string("SetLinearSolver", "mumps");
    set_float("SetSlopeThreshold", 0.2);
    set_float("SetRMSThreshold", 0.05);
    set_float("SetSpeedAroundLargeSlopesAndRMS", 2.0);
    set_float("SetSteeringAngleMin", -0.5);
    set_float("SetSteeringAngleMax", 0.5);
    set_float("SetSteeringRateMin", -0.5);
    set_float("SetSteeringRateMax", 0.5);
    set_int("SetUseAdaptivePredictionHorizon", 0);
    set_float("SetMinPredictionHorizonDistance", 8.0);
    set_float("SetPredictionHorizonTimeMax", 10.0);
    set_float("SetWPredictionHorizonAnchor", 5.0);

    HasJuliaException();
}

TEST(MPCPlannerTest, PlanPath) {
    jl_module_t* mpc_module = (jl_module_t *)jl_eval_string("Main.MPC");
    ASSERT_NE(mpc_module, nullptr) << "Main.MPC module not found";

    SetDefaultMPCParameters(mpc_module);

    jl_function_t* j_setup = jl_get_function(mpc_module, "Setup");
    jl_function_t* j_plan = jl_get_function(mpc_module, "Plan");
    jl_function_t* j_set_state = jl_get_function(mpc_module, "SetState");
    jl_function_t* j_set_goal_point = jl_get_function(mpc_module, "SetGoalPoint");
    jl_function_t* j_get_path = jl_get_function(mpc_module, "GetPath");

    ASSERT_NE(j_setup, nullptr) << "Setup function not found";
    ASSERT_NE(j_plan, nullptr) << "Plan function not found";
    ASSERT_NE(j_set_state, nullptr) << "SetState function not found";

    jl_call0(j_setup);
    ASSERT_FALSE(HasJuliaException()) << "MPC Setup failed";

    std::vector<double> state_data = {0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    jl_value_t* array_type = jl_apply_array_type((jl_value_t*)jl_float64_type, 1);
    jl_array_t *jl_state = jl_ptr_to_array_1d(array_type, state_data.data(), state_data.size(), 0);
    jl_call1(j_set_state, (jl_value_t*)jl_state);
    ASSERT_FALSE(HasJuliaException());

    jl_call2(j_set_goal_point, jl_box_float64(10.0), jl_box_float64(0.0));
    ASSERT_FALSE(HasJuliaException());

    jl_call0(j_plan);
    ASSERT_FALSE(HasJuliaException()) << "MPC Plan failed";

    jl_array_t *j_path = (jl_array_t*)jl_call0(j_get_path);
    ASSERT_NE(j_path, nullptr);
    size_t path_len = jl_array_dim(j_path, 0);
    EXPECT_GT(path_len, 0);

    if (path_len > 0) {
        double *path_data = (double*)jl_array_data(j_path);
        double last_x = path_data[path_len*0 + path_len - 1];
        EXPECT_GT(last_x, 0.0);
        std::cout << "Path generated with " << path_len << " points. Final X: " << last_x << std::endl;
    }
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new JuliaTestEnvironment);
    int result = RUN_ALL_TESTS();
    if (jl_is_initialized()) jl_atexit_hook(0);
    return result;
}
