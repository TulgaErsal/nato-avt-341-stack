#include <avt_341/planning/local/mpc_planner_solver.h>

using namespace casadi;

namespace avt_341 {

MPC::MPC() {
    x_ = vertcat(SXVector({casadi::SX::sym("X_0"), casadi::SX::sym("X_1"),
                           casadi::SX::sym("X_2"), casadi::SX::sym("X_3"),
                           casadi::SX::sym("X_4"), casadi::SX::sym("X_5"),
                           casadi::SX::sym("X_6"), casadi::SX::sym("X_7"),
                           casadi::SX::sym("X_8"), casadi::SX::sym("X_9")}));

    u_ = vertcat(SXVector({casadi::SX::sym("U_0"), casadi::SX::sym("U_1")}));
}

MPC::~MPC() {}

Function MPC::IntegratorUser(float t) {
    auto dt = t / 4.0;
    auto x_0 = SX::sym("X", x_.size());
    auto u_0 = SX::sym("U", u_.size());
    auto x = x_0;
    for (size_t i = 0; i < 4; ++i) {
        auto k_1 = f_(SXVector{x, u_0}).at(0) * dt;
        auto k_2 = f_(SXVector{x + 0.5 * k_1, u_0}).at(0) * dt;
        auto k_3 = f_(SXVector{x + 0.5 * k_2, u_0}).at(0) * dt;
        auto k_4 = f_(SXVector{x + k_3, u_0}).at(0) * dt;
        x += (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4) / 6.0;
    }
    return Function("f", {x_0, u_0}, {x});
}

Function MPC::Integrator() { return IntegratorUser(dt_); }

void MPC::Plan() {
    auto start_time = std::chrono::steady_clock::now();
    std::map<std::string, DM> solver_args{{"lbx", nlp_lbw_},
                                          {"ubx", nlp_ubw_},
                                          {"lbg", nlp_lbg_},
                                          {"ubg", nlp_ubg_},
                                          {"x0", nlp_w0_}};
    if (nlp_has_solution_) {
        for (size_t i = 0; i < x0_.size(); ++i) {
            nlp_x0_[i] = x0_[i];
        }
        solver_args["x0"] = nlp_x0_;
        solver_args["lam_x0"] = nlp_lam_x0_;
        solver_args["lam_g0"] = nlp_lam_g0_;
    }

    auto solver_result = solver_(solver_args);
    nlp_x0_ = std::vector<float>(solver_result.at("x"));
    nlp_lam_x0_ = std::vector<float>(solver_result.at("lam_x"));
    nlp_lam_g0_ = std::vector<float>(solver_result.at("lam_g"));
    nlp_has_solution_ = true;

    auto end_time = std::chrono::steady_clock::now();
    last_execution_time_ =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                              start_time)
            .count() /
        1000.0;
}

void MPC::Initialise() {
    auto f_y_f = - 2.0 * c_alpha_f_ * atan(x_(8));
    auto f_y_r = - 2.0 * c_alpha_r_ * atan(x_(9));

    xdot_ = vertcat(SXVector{{
        /* SAE
        (x_(6) * cos(x_(4)) - x_(2) * sin(x_(4))),
        x_(6) * sin(x_(4)) + x_(2) * cos(x_(4)),
        (f_y_f + f_y_r) / m_ - x_(3) * x_(6),
        (d_a_ * f_y_f - d_b_ * f_y_r) / i_zz_,
        x_(3),
        u_(1),
        x_(7),
        u_(0),
        x_(2) / b_ - fabs(x_(6)) / b_ * x_(8) - x_(5),
        x_(2) / b_ - fabs(x_(6)) / b_ * x_(9),
        */
        (x_(6) * cos(-x_(4)) - x_(2) * sin(-x_(4))),
        x_(6) * sin(-x_(4)) + x_(2) * cos(-x_(4)),
        -((f_y_f + f_y_r) / m_ - (-x_(3)) * x_(6)),
        -((d_a_ * f_y_f - d_b_ * f_y_r) / i_zz_),
        x_(3),
        u_(1),
        x_(7),
        u_(0),
        (-x_(2)) / b_ - fabs(x_(6)) / b_ * x_(8) - (-x_(5)),
        (-x_(2)) / b_ - fabs(x_(6)) / b_ * x_(9),
    }});

    x0_ = std::vector<float>(x_.size1(), 0.0);
    u0_ = std::vector<float>(u_.size1(), 0.0);

    nlp_opts_ = casadi::Dict({{"ipopt.hessian_approximation", "limited-memory"},
                              {"ipopt.linear_solver", solver_name_},
                              {"ipopt.max_iter", max_iters_},
                              {"ipopt.max_cpu_time", cpu_time_},
                              {"ipopt.print_level", 0},
                              {"ipopt.sb", "yes"},
                              {"ipopt.tol", tol_},
                              {"print_time", 0}});

    cpu_time_ = (cpu_time_ < 0.0) ? t_ : cpu_time_;
    nlp_opts_["ipopt.max_cpu_time"] = cpu_time_;
}

void MPC::Setup() {

    nlp_w_.clear();
    nlp_w0_.clear();
    nlp_lbw_.clear();
    nlp_ubw_.clear(),
    nlp_g_.clear();
    nlp_lbg_.clear();
    nlp_ubg_.clear();

    x0_[0] = x_init_;
    x0_[1] = y_init_;
    x0_[2] = v_init_;
    x0_[3] = r_init_;
    x0_[4] = psi_init_;
    x0_[5] = delta_init_;
    x0_[6] = u_init_;
    x0_[7] = a_init_;
    x0_[8] = tan_alpha_f_init_;
    x0_[9] = tan_alpha_r_init_;

    n_ = (size_t)(t_ / dt_);
    for (float x = 0.0; x < n_ * dt_; x += dt_)
        t_span_.push_back(x);

    std::vector<float> lbx{x_low_,           y_low_,          v_low_, r_low_,
                            psi_low_,         delta_low_,      u_low_, a_low_,
                            tan_alpha_f_low_, tan_alpha_f_low_};
    std::vector<float> lbu{j_low_, deltadot_low_};
    std::vector<float> ubx{x_up_,           y_up_,          v_up_, r_up_,
                            psi_up_,         delta_up_,      u_up_, a_up_,
                            tan_alpha_f_up_, tan_alpha_f_up_};
    std::vector<float> ubu{j_up_, deltadot_up_};
    std::vector<float> null(x_.size1(), 0.0);

    f_ = Function("f", SXVector{x_, u_}, SXVector{xdot_},
                  std::vector<std::string>{"x", "u"},
                  std::vector<std::string>{"ode"});
    s_ = f_.factory("integ", {"x", "u"}, {"ode"});

    std::string processing = (use_parallel_) ? "openmp" : "serial";
    auto f_map = Integrator().map(n_, processing);

    auto X = SX::sym("X", x_.size1(), n_ + 1);
    auto U = SX::sym("U", u_.size1(), n_);

    auto X_next = f_map(SXVector{X(Slice(), Slice(0, (int)n_)), U}).at(0);

    auto w_u = SX::sym("W_U", Sparsity::diag(u_.size1()));
    w_u(0, 0) = w_j_;
    w_u(1, 1) = w_deltadot_;
    auto u = SX::sym("u", u_.size1(), 1);
    auto cost_u = mtimes(u.T(), mtimes(w_u, u));
    auto f_cost_u = Function("FWU", {u}, {cost_u});

    nlp_w_.insert(nlp_w_.end(), X(Slice(), 0));
    nlp_w0_.insert(nlp_w0_.end(), x0_.begin(), x0_.end());
    nlp_lbw_.insert(nlp_lbw_.end(), nlp_w0_.begin(), nlp_w0_.end());
    nlp_ubw_.insert(nlp_ubw_.end(), nlp_w0_.begin(), nlp_w0_.end());

    nlp_f_ = 0.0;
    for (size_t k = 0; k < n_; ++k) {
        nlp_w_.insert(nlp_w_.end(), U(Slice(), k));
        nlp_w0_.insert(nlp_w0_.end(), u0_.begin(), u0_.end());
        nlp_lbw_.insert(nlp_lbw_.end(), lbu.begin(), lbu.end());
        nlp_ubw_.insert(nlp_ubw_.end(), ubu.begin(), ubu.end());

        auto cost_u_k = f_cost_u(U(Slice(), k)).at(0);
        auto cost_g_k =
            w_g_ *
            (pow((X(0, k) - g_x_), 2.0) + (pow((X(1, k) - g_y_), 2.0)));
        auto cost_g_psi_k = w_g_psi_ * pow(X(4, k) - g_psi_, 2.0);

        SX cost_obs = 0.0;
        for (size_t i = 0; i < obs_x_.size(); ++i) {
            float rho_ks = 2.5 * obs_r_[i];
            cost_obs += exp(rho_ks * (-pow(X(0, k) - obs_x_[i], 2.0) -
                                      pow(X(1, k) - obs_y_[i], 2.0) +
                                      pow(obs_r_[i] + eta_r_, 2.0)));
        }
        nlp_f_ += w_obs_ * cost_obs;
        nlp_f_ += cost_u_k + cost_g_k + cost_g_psi_k;

        if (use_hard_constraints_) {
            nlp_g_.insert(nlp_g_.end(), cost_obs);
            nlp_lbg_.insert(nlp_lbg_.end(),
                            -std::numeric_limits<float>::infinity());
            nlp_ubg_.insert(nlp_ubg_.end(), 1.0);
        }

        nlp_w_.insert(nlp_w_.end(), X(Slice(), k + 1));
        nlp_w0_.insert(nlp_w0_.end(), x0_.begin(), x0_.end());
        nlp_lbw_.insert(nlp_lbw_.end(), lbx.begin(), lbx.end());
        nlp_ubw_.insert(nlp_ubw_.end(), ubx.begin(), ubx.end());
        nlp_g_.insert(nlp_g_.end(), X_next(Slice(), k) - X(Slice(), k + 1));
        nlp_lbg_.insert(nlp_lbg_.end(), null.begin(), null.end());
        nlp_ubg_.insert(nlp_ubg_.end(), null.begin(), null.end());
    }

    SXDict nlp = {
        {"x", vertcat(nlp_w_)}, {"f", nlp_f_}, {"g", vertcat(nlp_g_)}};
    solver_ = nlpsol("solver", "ipopt", nlp, nlp_opts_);
    if (use_codegen_) {
        if (!boost::filesystem::exists("avt_341_mpc.c") || use_clean_build_) {
            solver_.generate_dependencies("avt_341_mpc.c");
        }

        if (use_jit_) {
            solver_ = nlpsol("solver", "ipopt", "avt_341_mpc.c");
        } else {
            if (!boost::filesystem::exists("nlp.so") || use_clean_build_) {
                int flag = system(
                    (compiler_ +
                     " -fPIC -shared -O3 avt_341_mpc.c -o avt_341_mpc.so")
                        .c_str());
                casadi_assert(flag == 0, "Compilation failed");
            }
            solver_ = nlpsol("solver", "ipopt", "nlp.so", nlp_opts_);
        }
    }
}

std::vector<float> MPC::GetSolInt(float t) {
    //auto s = IntegratorUser(t);
    //auto sol = s(SXVector{x0_, GetNextCtrl()});
    //return std::vector<float>(sol.at(0));
    return std::vector<float>();
}

std::vector<float> MPC::GetVar(size_t i) {
    std::vector<float> sol;
    sol.insert(sol.end(), nlp_x0_.begin() + i, nlp_x0_.begin() + i + 1);
    for (size_t j = 0; j < n_; ++j) {
        sol.insert(sol.end(),
                   nlp_x0_.begin() + x_.size1() + u_.size1() + i +
                       j * (x_.size1() + 2),
                   nlp_x0_.begin() + x_.size1() + u_.size1() + i +
                       j * (x_.size1() + 2) + 1);
    }
    return sol;
}

std::vector<float> MPC::GetCtrl(size_t i) {
    std::vector<float> ctrl;
    for (size_t j = 0; j < n_; ++j) {
        ctrl.insert(ctrl.end(),
                    nlp_x0_.begin() + (j + 1) * x_.size1() + j * u_.size1() + i,
                    nlp_x0_.begin() + (j + 1) * x_.size1() + j * u_.size1() +
                        i + 1);
    }
    return ctrl;
}

std::vector<float> MPC::GetNextCtrl() {
    std::vector<float> ctrl;
    ctrl.insert(ctrl.end(), nlp_x0_.begin() + x_.size1(),
                nlp_x0_.begin() + x_.size1() + 2);
    return ctrl;
}

} // namespace avt_341
