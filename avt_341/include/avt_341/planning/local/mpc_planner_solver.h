#pragma once

#include <boost/filesystem.hpp>
#include <casadi/casadi.hpp>

using namespace casadi;

namespace avt_341 {
class MPC {
  public:
    MPC();
    ~MPC();
    void AddObstacle(float x, float y, float r) {
        obs_x_.push_back(x);
        obs_y_.push_back(y);
        obs_r_.push_back(r);
    }
    std::vector<float> GetCtrl(size_t i);
    std::vector<float> GetSolInt(float t);
    std::vector<float> GetVar(size_t i);
    std::vector<float> GetNextCtrl();
    void UseJIT(bool use_jit) { use_jit_ = use_jit; };
    void UseCodeGen(bool use_codegen) { use_codegen_ = use_codegen; }
    void UseCleanBuild(bool use_clean_build) {
        use_clean_build_ = use_clean_build;
    }
    void UseParallel(bool use_parallel) { use_parallel_ = use_parallel; }
    Function IntegratorUser(float t);

    void ClearObstacles() {
        obs_x_.clear();
        obs_y_.clear();
        obs_r_.clear();
    }
    void AddObstacles(std::vector<float> x, std::vector<float> y,
                      std::vector<float> r) {
        obs_x_ = x;
        obs_y_ = y;
        obs_r_ = r;
    }
    void Plan();
    void Setup();
    void SetTimeStep(float dt) { dt_ = dt; }
    void SetGoal(float g_x, float g_y, float g_psi) {
        g_x_ = g_x;
        g_y_ = g_y;
        g_psi_ = g_psi;
    }
    void SetGoalX(float g_x) { g_x_ = g_x; }
    void SetGoalY(float g_y) { g_y_ = g_y; }
    void SetGoalPsi(float g_psi) { g_psi_ = g_psi; }
    void SetMaxX(float x_up) { x_up_ = x_up; }
    void SetMinX(float x_low) { x_low_ = x_low; }
    void SetMaxY(float y_up) { y_up_ = y_up; }
    void SetMinY(float y_low) { y_low_ = y_low; }
    void SetMaxSpeed(float u_up) { u_up_ = u_up; }
    void SetMinSpeed(float u_low) { u_low_ = u_low; }
    void SetMaxAcc(float a_up) { a_up_ = a_up; }
    void SetMinAcc(float a_low) { a_low_ = a_low; }
    void SetMaxJerk(float j_up) { j_up_ = j_up; }
    void SetMinJerk(float j_low) { j_low_ = j_low; }
    void SetMaxSteer(float delta_up) { delta_up_ = delta_up; }
    void SetMinSteer(float delta_low) { delta_low_ = delta_low; }
    void SetMaxSteerRate(float deltadot_up) { deltadot_up_ = deltadot_up; }
    void SetMinSteerRate(float deltadot_low) { deltadot_low_ = deltadot_low; }
    void SetHorizon(float t) { t_ = t; }

    bool HasPlan() { return nlp_has_solution_; }
    void Initialise();
    void SetCornStiffnessFront(float c_alpha_f) { c_alpha_f_ = c_alpha_f; }
    void SetCornStiffnessRear(float c_alpha_r) { c_alpha_r_ = c_alpha_r; }
    void SetDisAxleFront(float d_a) { d_a_ = d_a; }
    void SetDisAxleRear(float d_b) { d_b_ = d_b; }
    void SetInertia(float i_zz) { i_zz_ = i_zz; }
    void SetInitAcc(float a_init) { a_init_ = a_init; }
    void SetInitLatVel(float v_init) { v_init_ = v_init; }
    void SetInitSpeed(float u_init) { u_init_ = u_init; }
    void SetInitSteer(float delta_init) { delta_init_ = delta_init; }
    void SetInitTanSlipFront(float tan_alpha_f_init) {
        tan_alpha_f_init_ = tan_alpha_f_init;
    }
    void SetInitTanSlipRear(float tan_alpha_r_init) {
        tan_alpha_r_init_ = tan_alpha_r_init;
    }
    void SetInitX(float x_init) { x_init_ = x_init; }
    void SetInitY(float y_init) { y_init_ = y_init; }
    void SetInitYaw(float psi_init) { psi_init_ = psi_init; }
    void SetInitYawRate(float r_init) { r_init_ = r_init; }
    void SetMass(float m) { m_ = m; }
    void SetMaxCPUTime(float cpu_time) { cpu_time_ = cpu_time; }
    void SetMaxIters(int max_iters) { max_iters_ = max_iters; }
    void SetMaxLatVel(float v_up) { v_up_ = v_up; }
    void SetMaxYawRate(float r_up) { r_up_ = r_up; }
    void SetMinLatVel(float v_low) { v_low_ = v_low; }
    void SetMinYawRate(float r_low) { r_low_ = r_low; }
    void SetPrintLevel(int print_level) { print_level_ = print_level; }
    void SetRelaxLength(float b) { b_ = b; }
    void SetSafetyMargin(float eta_r) { eta_r_ = eta_r; }
    void SetSolver(std::string solver_name) { solver_name_ = solver_name; }
    void SetTolerance(float tol) { tol_ = tol; }
    void SetWeightGoal(float w_g) { w_g_ = w_g; }
    void SetWeightGoalYaw(float w_g_psi) { w_g_psi_ = w_g_psi; }
    void SetWeightJerk(float w_j) { w_j_ = w_j; }
    void SetWeightObstacle(float w_obs) { w_obs_ = w_obs; }
    void SetWeightSteerRate(float w_deltadot) { w_deltadot_ = w_deltadot; }

  private:
    float a_low_ = -std::numeric_limits<float>::infinity();
    float a_up_ = std::numeric_limits<float>::infinity();
    std::vector<casadi::SX> controls_;
    float delta_low_ = -std::numeric_limits<float>::infinity();
    float delta_up_ = std::numeric_limits<float>::infinity();
    float deltadot_low_ = -std::numeric_limits<float>::infinity();
    float deltadot_up_ = std::numeric_limits<float>::infinity();
    float dt_ = 0.5;
    casadi::SX equations_;
    casadi::Function f_;
    casadi::Function s_;
    float g_x_ = 3.0;
    float g_y_ = 3.0;
    float g_psi_ = 0.0;
    float j_low_ = -std::numeric_limits<float>::infinity();
    float j_up_ = std::numeric_limits<float>::infinity();
    float last_execution_time_ = -1.0;
    size_t n_;
    casadi::SX nlp_f_;
    std::vector<casadi::SX> nlp_g_;
    bool nlp_has_solution_ = false;
    std::vector<float> nlp_lam_x0_;
    std::vector<float> nlp_lam_g0_;
    std::vector<float> nlp_lbg_;
    std::vector<float> nlp_lbw_;
    casadi::Dict nlp_opts_;
    std::vector<float> nlp_ubg_;
    std::vector<float> nlp_ubw_;
    std::vector<casadi::SX> nlp_w_;
    std::vector<float> nlp_w0_;
    std::vector<float> nlp_x0_;
    float psi_low_ = -std::numeric_limits<float>::infinity();
    float psi_up_ = std::numeric_limits<float>::infinity();
    std::vector<float> q_u_;
    casadi::Function solver_;
    std::vector<casadi::SX> states_;
    float t_ = 3.0;
    float tan_alpha_f_low_ = -std::numeric_limits<float>::infinity();
    float tan_alpha_f_up_ = std::numeric_limits<float>::infinity();
    float tan_alpha_r_low_ = -std::numeric_limits<float>::infinity();
    float tan_alpha_r_up_ = std::numeric_limits<float>::infinity();
    std::vector<float> t_span_;
    casadi::SX u_;
    float u_low_ = -std::numeric_limits<float>::infinity();
    float u_up_ = std::numeric_limits<float>::infinity();
    std::vector<float> u0_;
    casadi::SX x_;
    std::vector<float> x0_;
    casadi::SX xdot_;

    casadi::SX GetLateralForce(SX alpha, float gamma, float f_z);
    Function Integrator();

    float a_init_ = 0.0;
    float b_;
    float c_alpha_f_;
    float c_alpha_r_;
    std::string compiler_ = "gcc";
    float cpu_time_ = -1.0;
    float d_a_;
    float d_b_;
    float delta_init_ = 0.0;
    float eta_r_ = 0.1;
    float i_zz_;
    float m_;
    int max_iters_ = 100;
    std::vector<float> obs_x_;
    std::vector<float> obs_y_;
    std::vector<float> obs_r_;
    int print_level_ = 1;
    float psi_init_ = 0.0;
    float r_init_ = 0.0;
    float r_low_ = -std::numeric_limits<float>::infinity();
    float r_up_ = std::numeric_limits<float>::infinity();
    std::string solver_name_ = "mumps";
    float tan_alpha_f_init_ = 0.0;
    float tan_alpha_r_init_ = 0.0;
    float tol_ = 0.001;
    float u_init_ = 0.0;
    bool use_clean_build_ = false;
    bool use_codegen_ = false;
    bool use_hard_constraints_ = false;
    bool use_jit_ = false;
    bool use_parallel_ = false;
    float v_init_ = 0.0;
    float v_low_ = -std::numeric_limits<float>::infinity();
    float v_up_ = std::numeric_limits<float>::infinity();
    float w_deltadot_ = 1.0;
    float w_g_ = 1.0;
    float w_g_psi_ = 1.0;
    float w_j_ = 1.0;
    float w_obs_ = 1.0;
    float x_init_ = 0.0;
    float x_low_ = -std::numeric_limits<float>::infinity();
    float x_up_ = std::numeric_limits<float>::infinity();
    float y_init_ = 0.0;
    float y_low_ = -std::numeric_limits<float>::infinity();
    float y_up_ = std::numeric_limits<float>::infinity();
};

} // namespace avt_341