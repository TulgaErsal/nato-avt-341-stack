/**
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +  AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles  +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

* @file      imm_filter.hpp
* @brief     Interacting Multiple Model (IMM) filter combining:
*              - Model 0: Constant Velocity (CV), 6-state linear KF
*                         state: [x, vx, y, vy, z, vz]
*              - Model 1: Constant Turn Rate (CTR), 5-state EKF
*                         state: [x, vx, y, vy, omega]
*              - Model 2: No Motion (NM), 2-state linear KF
*                         state: [x, y]
*
* Because CV and CTR share the same Cartesian state layout for the first four
* states [x, vx, y, vy], the IMM mixing step requires NO Jacobian
* transformations between these models.  Cross-model covariance projections
* are trivially obtained by submatrix extraction, and the standard
* spread-of-means mixing formula (Bar-Shalom et al., 2001) is applied
* directly.  This guarantees positive-semidefinite (PSD) mixed covariances at
* every step, eliminating the negative variance issue.
*
* IMM algorithm per time step:
*   1. Mixing  — compute mixed initial conditions (state + covariance) for
*                each sub-filter using the proper PSD formula.
*   2. Predict — each sub-filter runs its own prediction step.
*   3. Update  — each sub-filter updates with the shared position measurement.
*   4. Combine — fuse model outputs weighted by likelihood-updated model
*                probabilities.
*
* @copyright MIT License, NATO AVT-341 Autonomy Stack (2026)
*/

#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <Eigen/Dense>

#include <avt_341/perception/filtering/cv_filter.hpp>
#include <avt_341/perception/filtering/ctr_filter.hpp>
#include <avt_341/perception/filtering/nm_filter.hpp>

namespace avt_341 {
namespace perception {
namespace filtering {

/**
 * @brief IMM filter: CV (model 0) + CTR (model 1) + NM (model 2).
 *
 * External interface mirrors the previous CAFilter API used in ObjectTrackingNode.
 */
class IMMFilter {
  public:
    static constexpr int kNumModels = 3;

    using Vec3 = Eigen::Vector3d;
    using Vec2 = Eigen::Vector2d;
    using MeasurementVector3D = Eigen::Matrix<double, 3, 1>;
    using MeasurementVector2D = Eigen::Matrix<double, 2, 1>;

    // Uniform prior variances for states not tracked by a given model.
    // Used when projecting a lower-dimensional model's covariance into a
    // higher-dimensional model's state space.
    double sigma2_v_uniform     = (20.0 * 20.0) / 12.0;   // +-20 m/s uniform
    double sigma2_omega_uniform = (1.5 * 1.5)   / 3.0;    // +-1.5 rad/s uniform

    /**
     * @param dt                   Estimator time step [s].
     * @param process_variance     Shared process noise scale for all sub-filters.
     * @param measurement_variance Shared measurement noise scale for all sub-filters.
     * @param cv_init_prob         Initial probability for the CV model.
     * @param ctr_init_prob        Initial probability for the CTR model.
     * @param nm_init_prob         Initial probability for the NM model.
     * @param persistence_prob      Diagonal entry of the Markov transition matrix
     *                             (probability of staying in the same model).
     */
    IMMFilter(const double dt,
              const double process_variance,
              const double measurement_variance,
              const double cv_init_prob    = 0.33,
              const double ctr_init_prob   = 0.33,
              const double nm_init_prob    = 0.33,
              const double persistence_prob = 0.9)
        : cv_ (dt, process_variance, measurement_variance),
          ctr_(dt, process_variance, measurement_variance),
          nm_ (dt, process_variance, measurement_variance),
          dt_(dt)
    {
        const double sum = cv_init_prob + ctr_init_prob + nm_init_prob;
        mu_[0] = cv_init_prob   / sum;
        mu_[1] = ctr_init_prob  / sum;
        mu_[2] = nm_init_prob   / sum;

        const double p_stay   = std::max(0.01, std::min(0.99, persistence_prob));
        const double p_switch = (1.0 - p_stay) / (kNumModels - 1);
        for (int i = 0; i < kNumModels; ++i)
            for (int j = 0; j < kNumModels; ++j)
                pi_[i][j] = (i == j) ? p_stay : p_switch;

        fused_position_.setZero();
        fused_velocity_.setZero();
        fused_yaw_ = 0.0;
        sigma_ = measurement_variance; // variance here is actually std dev
    }

    // -----------------------------------------------------------------------
    // Initialization helpers
    // -----------------------------------------------------------------------

    void SetInitialPosition(const Vec3& pos) {
        cv_ .SetInitialPosition(pos);
        ctr_.SetInitialPosition(pos);
        nm_ .SetInitialPosition(pos);
        fused_position_ = pos;
    }

    void SetInitialVelocity(const Vec3& vel) {
        cv_ .SetInitialVelocity(vel);
        ctr_.SetInitialVelocity(vel);
        nm_ .SetInitialVelocity(vel);
    }

    // -----------------------------------------------------------------------
    // IMM Predict
    // -----------------------------------------------------------------------

    /**
     * @brief Full IMM predict step:
     *   1. Compute mixing probabilities.
     *   2. Mix initial conditions (state + covariance, guaranteed PSD).
     *   3. Run each sub-filter's predict step.
     */
    void Predict() {
        // --- Step 1: Mixing probabilities ---------------------------------
        // c_bar[j] = sum_i mu[i] * pi[i][j]  (predicted model probability)
        std::array<double, kNumModels> c_bar;
        for (int j = 0; j < kNumModels; ++j) {
            c_bar[j] = 0.0;
            for (int i = 0; i < kNumModels; ++i)
                c_bar[j] += mu_[i] * pi_[i][j];
        }

        // mu_mix[i][j] = mu[i] * pi[i][j] / c_bar[j]
        double mu_mix[kNumModels][kNumModels];
        for (int j = 0; j < kNumModels; ++j)
            for (int i = 0; i < kNumModels; ++i)
                mu_mix[i][j] = (c_bar[j] > 1e-12)
                    ? (mu_[i] * pi_[i][j] / c_bar[j])
                    : (1.0 / kNumModels);

        // --- Step 2: State and covariance mixing ---------------------------
        // All three models share the same Cartesian layout for [x, vx, y, vy]:
        //   CV  indices: 0=x, 1=vx, 2=y, 3=vy  (6D: also 4=z, 5=vz)
        //   CTR indices: 0=x, 1=vx, 2=y, 3=vy  (5D: also 4=omega)
        //   NM  indices: 0=x, 1=y               (2D, position only)
        //
        // No Jacobians are needed for the shared subspace.

        const auto& x_cv  = cv_ .GetState();     // 6D [x,vx,y,vy,z,vz]
        const auto& x_ctr = ctr_.GetState();     // 5D [x,vx,y,vy,omega]
        const auto& x_nm  = nm_ .GetState();     // 2D [x,y]

        const auto& P_cv  = cv_ .GetCovariance();  // 6x6
        const auto& P_ctr = ctr_.GetCovariance();  // 5x5
        const auto& P_nm  = nm_ .GetCovariance();  // 2x2

        // ----- Mix state for CV (model 0) -----
        Eigen::Matrix<double, 6, 1> x_cv_mixed = x_cv;
        x_cv_mixed(0) = mu_mix[0][0]*x_cv(0) + mu_mix[1][0]*x_ctr(0) + mu_mix[2][0]*x_nm(0);
        x_cv_mixed(1) = mu_mix[0][0]*x_cv(1) + mu_mix[1][0]*x_ctr(1) + mu_mix[2][0]*0.0;
        x_cv_mixed(2) = mu_mix[0][0]*x_cv(2) + mu_mix[1][0]*x_ctr(2) + mu_mix[2][0]*x_nm(1);
        x_cv_mixed(3) = mu_mix[0][0]*x_cv(3) + mu_mix[1][0]*x_ctr(3) + mu_mix[2][0]*0.0;
        // z, vz: only from CV (indices 4,5 unchanged)

        // ----- Mix state for CTR (model 1) -----
        Eigen::Matrix<double, 5, 1> x_ctr_mixed = x_ctr;
        x_ctr_mixed(0) = mu_mix[0][1]*x_cv(0) + mu_mix[1][1]*x_ctr(0) + mu_mix[2][1]*x_nm(0);
        x_ctr_mixed(1) = mu_mix[0][1]*x_cv(1) + mu_mix[1][1]*x_ctr(1) + mu_mix[2][1]*0.0;
        x_ctr_mixed(2) = mu_mix[0][1]*x_cv(2) + mu_mix[1][1]*x_ctr(2) + mu_mix[2][1]*x_nm(1);
        x_ctr_mixed(3) = mu_mix[0][1]*x_cv(3) + mu_mix[1][1]*x_ctr(3) + mu_mix[2][1]*0.0;
        x_ctr_mixed(4) = mu_mix[1][1]*x_ctr(4); // omega: 0 from CV and NM

        // ----- Mix state for NM (model 2) -----
        Eigen::Matrix<double, 2, 1> x_nm_mixed;
        x_nm_mixed(0) = mu_mix[0][2]*x_cv(0) + mu_mix[1][2]*x_ctr(0) + mu_mix[2][2]*x_nm(0);
        x_nm_mixed(1) = mu_mix[0][2]*x_cv(2) + mu_mix[1][2]*x_ctr(2) + mu_mix[2][2]*x_nm(1);

        // ----- Covariance mixing (proper spread-of-means formula) -----
        //
        // P_j^0 = sum_i { mu_ij * [ P_i^(j) + d_ij * d_ij^T ] }
        //
        // where d_ij = x_i^(j) - x_j^0  (difference in model-j state space)
        // and   P_i^(j) is model i's covariance projected into model j's space.
        //
        // This formula is guaranteed PSD.

        // === Mix covariance for CV (6D) ===
        {
            // Projection of CV into CV: identity
            Eigen::Matrix<double, 6, 1> d0 = x_cv - x_cv_mixed;
            Eigen::Matrix<double, 6, 6> T0 = P_cv + d0 * d0.transpose();

            // Projection of CTR into CV: shared [x,vx,y,vy] block; keep CV's z,vz
            Eigen::Matrix<double, 6, 1> x_ctr_in_cv;
            x_ctr_in_cv << x_ctr(0), x_ctr(1), x_ctr(2), x_ctr(3), x_cv(4), x_cv(5);
            Eigen::Matrix<double, 6, 6> P_ctr_in_cv = Eigen::Matrix<double, 6, 6>::Zero();
            P_ctr_in_cv.block<4, 4>(0, 0) = P_ctr.block<4, 4>(0, 0);
            P_ctr_in_cv(4, 4) = P_cv(4, 4);
            P_ctr_in_cv(5, 5) = P_cv(5, 5);
            Eigen::Matrix<double, 6, 1> d1 = x_ctr_in_cv - x_cv_mixed;
            Eigen::Matrix<double, 6, 6> T1 = P_ctr_in_cv + d1 * d1.transpose();

            // Projection of NM into CV: position entries, uniform prior for velocities
            Eigen::Matrix<double, 6, 1> x_nm_in_cv;
            x_nm_in_cv << x_nm(0), 0.0, x_nm(1), 0.0, x_cv(4), x_cv(5);
            Eigen::Matrix<double, 6, 6> P_nm_in_cv = Eigen::Matrix<double, 6, 6>::Zero();
            P_nm_in_cv(0, 0) = P_nm(0, 0);
            P_nm_in_cv(0, 2) = P_nm(0, 1);
            P_nm_in_cv(2, 0) = P_nm(1, 0);
            P_nm_in_cv(2, 2) = P_nm(1, 1);
            P_nm_in_cv(1, 1) = sigma2_v_uniform;
            P_nm_in_cv(3, 3) = sigma2_v_uniform;
            P_nm_in_cv(4, 4) = P_cv(4, 4);
            P_nm_in_cv(5, 5) = P_cv(5, 5);
            Eigen::Matrix<double, 6, 1> d2 = x_nm_in_cv - x_cv_mixed;
            Eigen::Matrix<double, 6, 6> T2 = P_nm_in_cv + d2 * d2.transpose();

            Eigen::Matrix<double, 6, 6> P_cv_new =
                mu_mix[0][0]*T0 + mu_mix[1][0]*T1 + mu_mix[2][0]*T2;
            P_cv_new = (P_cv_new + P_cv_new.transpose()) * 0.5;
            cv_.SetState(x_cv_mixed);
            cv_.SetCovariance(P_cv_new);
        }

        // === Mix covariance for CTR (5D) ===
        {
            // Projection of CV into CTR: shared [x,vx,y,vy] block; uniform prior for omega
            Eigen::Matrix<double, 5, 1> x_cv_in_ctr;
            x_cv_in_ctr << x_cv(0), x_cv(1), x_cv(2), x_cv(3), 0.0;
            Eigen::Matrix<double, 5, 5> P_cv_in_ctr = Eigen::Matrix<double, 5, 5>::Zero();
            P_cv_in_ctr.block<4, 4>(0, 0) = P_cv.block<4, 4>(0, 0);
            P_cv_in_ctr(4, 4) = sigma2_omega_uniform;
            Eigen::Matrix<double, 5, 1> d0 = x_cv_in_ctr - x_ctr_mixed;
            Eigen::Matrix<double, 5, 5> T0 = P_cv_in_ctr + d0 * d0.transpose();

            // Projection of CTR into CTR: identity
            Eigen::Matrix<double, 5, 1> d1 = x_ctr - x_ctr_mixed;
            Eigen::Matrix<double, 5, 5> T1 = P_ctr + d1 * d1.transpose();

            // Projection of NM into CTR: position entries, uniform prior for velocities/omega
            Eigen::Matrix<double, 5, 1> x_nm_in_ctr;
            x_nm_in_ctr << x_nm(0), 0.0, x_nm(1), 0.0, 0.0;
            Eigen::Matrix<double, 5, 5> P_nm_in_ctr = Eigen::Matrix<double, 5, 5>::Zero();
            P_nm_in_ctr(0, 0) = P_nm(0, 0);
            P_nm_in_ctr(0, 2) = P_nm(0, 1);
            P_nm_in_ctr(2, 0) = P_nm(1, 0);
            P_nm_in_ctr(2, 2) = P_nm(1, 1);
            P_nm_in_ctr(1, 1) = sigma2_v_uniform;
            P_nm_in_ctr(3, 3) = sigma2_v_uniform;
            P_nm_in_ctr(4, 4) = sigma2_omega_uniform;
            Eigen::Matrix<double, 5, 1> d2 = x_nm_in_ctr - x_ctr_mixed;
            Eigen::Matrix<double, 5, 5> T2 = P_nm_in_ctr + d2 * d2.transpose();

            Eigen::Matrix<double, 5, 5> P_ctr_new =
                mu_mix[0][1]*T0 + mu_mix[1][1]*T1 + mu_mix[2][1]*T2;
            P_ctr_new = (P_ctr_new + P_ctr_new.transpose()) * 0.5;
            ctr_.SetState(x_ctr_mixed);
            ctr_.SetCovariance(P_ctr_new);
        }

        // === Mix covariance for NM (2D) ===
        {
            // Projection of CV into NM: extract [x, y] = indices [0, 2]
            Eigen::Matrix<double, 2, 1> x_cv_in_nm;
            x_cv_in_nm << x_cv(0), x_cv(2);
            Eigen::Matrix<double, 2, 2> P_cv_in_nm;
            P_cv_in_nm(0, 0) = P_cv(0, 0); P_cv_in_nm(0, 1) = P_cv(0, 2);
            P_cv_in_nm(1, 0) = P_cv(2, 0); P_cv_in_nm(1, 1) = P_cv(2, 2);
            Eigen::Matrix<double, 2, 1> d0 = x_cv_in_nm - x_nm_mixed;
            Eigen::Matrix<double, 2, 2> T0 = P_cv_in_nm + d0 * d0.transpose();

            // Projection of CTR into NM: extract [x, y] = indices [0, 2]
            Eigen::Matrix<double, 2, 1> x_ctr_in_nm;
            x_ctr_in_nm << x_ctr(0), x_ctr(2);
            Eigen::Matrix<double, 2, 2> P_ctr_in_nm;
            P_ctr_in_nm(0, 0) = P_ctr(0, 0); P_ctr_in_nm(0, 1) = P_ctr(0, 2);
            P_ctr_in_nm(1, 0) = P_ctr(2, 0); P_ctr_in_nm(1, 1) = P_ctr(2, 2);
            Eigen::Matrix<double, 2, 1> d1 = x_ctr_in_nm - x_nm_mixed;
            Eigen::Matrix<double, 2, 2> T1 = P_ctr_in_nm + d1 * d1.transpose();

            // NM into NM: identity
            Eigen::Matrix<double, 2, 1> d2 = x_nm - x_nm_mixed;
            Eigen::Matrix<double, 2, 2> T2 = P_nm + d2 * d2.transpose();

            Eigen::Matrix<double, 2, 2> P_nm_new =
                mu_mix[0][2]*T0 + mu_mix[1][2]*T1 + mu_mix[2][2]*T2;
            P_nm_new = (P_nm_new + P_nm_new.transpose()) * 0.5;
            nm_.SetState(x_nm_mixed);
            nm_.SetCovariance(P_nm_new);
        }

        // --- Step 3: Individual prediction steps --------------------------
        cv_ .Predict();
        ctr_.Predict();
        nm_ .Predict();

        // --- Step 4: Update fused outputs from predicted sub-filter states -
        // This keeps GetState() / GetPosition3D() / GetVelocity3D() consistent
        // with the latest predicted states, not just the last update step.
        const auto& x_cv_pred  = cv_ .GetState();
        const Vec2  p_ctr_pred = ctr_.GetPosition2D();
        const Vec2  p_nm_pred  = nm_ .GetPosition2D();

        fused_position_.x() = mu_[0]*x_cv_pred(0) + mu_[1]*p_ctr_pred(0) + mu_[2]*p_nm_pred(0);
        fused_position_.y() = mu_[0]*x_cv_pred(2) + mu_[1]*p_ctr_pred(1) + mu_[2]*p_nm_pred(1);
        fused_position_.z() = x_cv_pred(4);

        fused_velocity_.x() = mu_[0]*x_cv_pred(1) + mu_[1]*ctr_.GetVx();
        fused_velocity_.y() = mu_[0]*x_cv_pred(3) + mu_[1]*ctr_.GetVy();
        fused_velocity_.z() = x_cv_pred(5);
    }

    // -----------------------------------------------------------------------
    // IMM Update
    // -----------------------------------------------------------------------

    /**
     * @brief Update the IMM with a 3D position measurement [x, y, z].
     *
     * CV receives the full 3D measurement; CTR and NM receive the 2D [x,y]
     * component.  An optional per-call measurement noise covariance overrides
     * the static R built at construction time.
     */
    void Update(const MeasurementVector3D& z,
                const std::optional<Eigen::Matrix3d>& R_override = std::nullopt) {
        const MeasurementVector2D z2d = z.head<2>();

        if (R_override) {
            const Eigen::Matrix2d R2d = R_override->topLeftCorner<2, 2>();
            cv_.Update(z, R_override.value());
            const auto S_ctr = ctr_.Update(z2d, R2d);
            const auto S_nm  = nm_ .Update(z2d, R2d);
            const double ll_cv  = ComputeCVLikelihood(z, &R_override.value());
            const double ll_ctr = ctr_.LogLikelihood(z2d, S_ctr);
            const double ll_nm  = nm_ .LogLikelihood(z2d, S_nm);
            chi2_.x() = ComputeCVChi2(z, &R_override.value());
            chi2_.y() = ctr_.chi2(z2d, S_ctr);
            chi2_.z() = nm_ .chi2(z2d, S_nm);
            UpdateModelProbabilities(ll_cv, ll_ctr, ll_nm);
        } else {
            cv_.Update(z);
            const auto S_ctr = ctr_.Update(z2d);
            const auto S_nm  = nm_ .Update(z2d);
            const double ll_cv  = ComputeCVLikelihood(z, nullptr);
            const double ll_ctr = ctr_.LogLikelihood(z2d, S_ctr);
            const double ll_nm  = nm_ .LogLikelihood(z2d, S_nm);
            chi2_.x() = ComputeCVChi2(z, nullptr);
            chi2_.y() = ctr_.chi2(z2d, S_ctr);
            chi2_.z() = nm_ .chi2(z2d, S_nm);
            UpdateModelProbabilities(ll_cv, ll_ctr, ll_nm);
        }

        // --- Fuse outputs (weighted combination) --------------------------
        const auto& x_cv  = cv_ .GetState();
        const Vec2  p_ctr = ctr_.GetPosition2D();
        const Vec2  p_nm  = nm_ .GetPosition2D();

        fused_position_.x() = mu_[0]*x_cv(0) + mu_[1]*p_ctr(0) + mu_[2]*p_nm(0);
        fused_position_.y() = mu_[0]*x_cv(2) + mu_[1]*p_ctr(1) + mu_[2]*p_nm(1);
        fused_position_.z() = x_cv(4);  // z only from CV

        const double vx_ctr = ctr_.GetVx();
        const double vy_ctr = ctr_.GetVy();
        fused_velocity_.x() = mu_[0]*x_cv(1) + mu_[1]*vx_ctr;
        fused_velocity_.y() = mu_[0]*x_cv(3) + mu_[1]*vy_ctr;
        fused_velocity_.z() = x_cv(5);

        // Fuse heading: derive yaw from Cartesian velocities.
        // NM has no velocity state so it contributes the last known fused_yaw_.
        const double kSpeedThreshold = 0.2;
        const double speed_cv  = std::hypot(x_cv(1), x_cv(3));
        const double speed_ctr = ctr_.GetSpeed();

        const double yaw_cv  = (speed_cv  > kSpeedThreshold)
            ? std::atan2(x_cv(3), x_cv(1)) : fused_yaw_;
        const double yaw_ctr = (speed_ctr > kSpeedThreshold)
            ? ctr_.GetYaw() : fused_yaw_;

        fused_yaw_ = mu_[0]*yaw_cv + mu_[1]*yaw_ctr + mu_[2]*fused_yaw_;
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /**
     * @brief Returns a 9-element synthetic state vector compatible with
     *        existing tracker code reading indices (0),(3),(6) for x,y,z
     *        and (1),(4),(7) for vx,vy,vz.
     */
    Eigen::Matrix<double, 9, 1> GetState() const {
        const auto& x_cv = cv_.GetState();
        Eigen::Matrix<double, 9, 1> s;
        s(0) = fused_position_.x();
        s(1) = fused_velocity_.x();
        s(2) = 0.0;        // ax (not tracked)
        s(3) = fused_position_.y();
        s(4) = fused_velocity_.y();
        s(5) = 0.0;        // ay (not tracked)
        s(6) = fused_position_.z();
        s(7) = fused_velocity_.z();
        s(8) = 0.0;        // az (not tracked)
        return s;
    }

    /**
     * @brief Fused 6x6 pose covariance: [x, y, z, roll, pitch, yaw].
     *
     * Position rows/columns come from the IMM-fused position covariance.
     * Yaw variance is propagated from the velocity covariance via the
     * linearized Jacobian of yaw = atan2(vy, vx).
     */
    Eigen::Matrix<double, 6, 6> GetPoseCovariance() const {
        const auto& P_nm = nm_.GetCovariance();
        const auto& P_cv = cv_.GetCovariance();
        const auto& P_ctr = ctr_.GetCovariance();

        Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();

        // Fused position covariance: weighted sum of [x,y] blocks from CV and CTR
        P(0, 0) = mu_[0] * P_cv(0,0) + mu_[1] * P_ctr(0,0) + mu_[2] * P_nm(0,0);
        P(0, 1) = mu_[0] * P_cv(0,2) + mu_[1] * P_ctr(0,2) + mu_[2] * P_nm(0,1);
        P(1, 0) = P(0, 1);
        P(1, 1) = mu_[0] * P_cv(2,2) + mu_[1] * P_ctr(2,2) + mu_[2] * P_nm(1,1);
        P(2, 2) = P_cv(4, 4);   // z from CV only

        // Roll and pitch: no information
        P(3, 3) = sigma2_omega_uniform;
        P(4, 4) = sigma2_omega_uniform;

        // Yaw variance via linearized Jacobian
        P(5, 5) = GetFusedYawVariance();

        return P;
    }

    /**
     * @brief Yaw variance propagated from the velocity covariance of each model.
     *
     * Uses the linearized Jacobian of yaw = atan2(vy, vx) w.r.t. [vx, vy].
     * Falls back to sigma2_omega_uniform when nearly stationary.
     */
    double GetFusedYawVariance() const {
        const auto& x_cv  = cv_ .GetState();
        const auto& P_cv  = cv_ .GetCovariance();
        const auto& P_ctr = ctr_.GetCovariance();

        auto YawVarianceFromVelocity = [&](double vx, double vy,
                                           double pVxVx, double pVxVy,
                                           double pVyVy) -> double {
            const double v2 = vx*vx + vy*vy;
            if (v2 < 0.01) return sigma2_omega_uniform;
            // J = [-vy/v^2, vx/v^2]
            const double jx = -vy / v2;
            const double jy =  vx / v2;
            return jx*jx*pVxVx + 2.0*jx*jy*pVxVy + jy*jy*pVyVy;
        };

        const double s2_cv  = YawVarianceFromVelocity(
            x_cv(1), x_cv(3), P_cv(1,1), P_cv(1,3), P_cv(3,3));
        const double s2_ctr = YawVarianceFromVelocity(
            ctr_.GetVx(), ctr_.GetVy(),
            P_ctr(1,1), P_ctr(1,3), P_ctr(3,3));

        return mu_[0]*s2_cv + mu_[1]*s2_ctr + mu_[2]*sigma2_omega_uniform;
    }

    /** @brief Fused 3D world-frame position. */
    Vec3   GetPosition3D()  const { return fused_position_; }

    /** @brief Fused heading [rad], derived from Cartesian velocities. */
    double GetYaw()         const { return fused_yaw_; }

    /** @brief Fused 3D velocity. */
    Vec3   GetVelocity3D()  const { return fused_velocity_; }

    /** @brief Model probabilities: [0]=CV, [1]=CTR, [2]=NM. */
    const std::array<double, kNumModels>& GetModelProbabilities() const { return mu_; }

    /** @brief CTR speed [m/s]. */
    double GetCTRSpeed()    const { return ctr_.GetSpeed(); }

    /** @brief CTR heading [rad], derived from CTR velocity. */
    double GetCTRYaw()      const { return ctr_.GetYaw(); }

    /** @brief CTR turn rate [rad/s]. */
    double GetCTROmega()    const { return ctr_.GetOmega(); }

    /** @brief CTR state covariance (5x5). */
    CTRFilter::StateMatrix GetCTRCovariance() const { return ctr_.GetCovariance(); }

    CVFilter<3>& GetCVFilter()  { return cv_; }
    CTRFilter&   GetCTRFilter() { return ctr_; }
    NMFilter&    GetNMFilter()  { return nm_; }

    Vec3 GetAllChi2() { return chi2_; }

    /**  @brief Get a chi2 estimate for the current mixed model and a new measurement only in plane(x,y) */
    double GetChi2IMM2D(const MeasurementVector3D& z, 
        const std::optional<Eigen::Matrix3d>& R_override = std::nullopt) {
        const auto& x_imm = GetState();
        const auto  Pimm = GetPoseCovariance();

        Eigen::Matrix<double, 2, 1> y;
        y(0) = z(0) - (x_imm(0)); // should also use prediction w. velocity* dt 
        y(1) = z(1) - (x_imm(3)); // should also use prediction w. velocity* dt 
        Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
        H(0, 0) = 1.0; H(1, 1) = 1.0; // pose covariance: [x, y, z, roll, pitch, yaw]
        Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
        if (R_override) {
            const Eigen::Matrix2d R2d = R_override->topLeftCorner<2, 2>();
            S = H * Pimm * H.transpose() + R2d;
        }
        else {
            const double r_diag = sigma_ * sigma_;
            S = H * Pimm * H.transpose() + Eigen::Matrix2d::Identity() * r_diag;
        }
        if (S.determinant() <= 0.0)
            return -1e9;
        else
            return (y.transpose() * S.inverse() * y)(0, 0);

    }

    /** @brief NM state covariance (2x2). */
    Eigen::Matrix<double, 2, 2> GetnmStatecovariance() { return nm_.GetS(); }

  private:
    void UpdateModelProbabilities(double ll_cv, double ll_ctr, double ll_nm) {
        const double max_ll = std::max({ll_cv, ll_ctr, ll_nm});
        const std::array<double, kNumModels> l = {
            std::exp(ll_cv  - max_ll),
            std::exp(ll_ctr - max_ll),
            std::exp(ll_nm  - max_ll)
        };

        std::array<double, kNumModels> c_bar;
        for (int j = 0; j < kNumModels; ++j) {
            c_bar[j] = 0.0;
            for (int i = 0; i < kNumModels; ++i)
                c_bar[j] += mu_[i] * pi_[i][j];
        }

        double norm = 0.0;
        for (int j = 0; j < kNumModels; ++j) {
            mu_[j] = l[j] * c_bar[j];
            norm   += mu_[j];
        }
        if (norm < 1e-12)
            for (auto& m : mu_) m = 1.0 / kNumModels;
        else
            for (auto& m : mu_) m /= norm;
    }

    double ComputeCVLikelihood(const Eigen::Matrix<double, 3, 1>& z,
                               const Eigen::Matrix3d* R3) const {
        const auto& x_cv = cv_.GetState();
        const auto  Pcv  = cv_.GetStateUncertainty();

        Eigen::Matrix<double, 3, 1> y;
        y(0) = z(0) - x_cv(0);
        y(1) = z(1) - x_cv(2);
        y(2) = z(2) - x_cv(4);

        Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
        H(0, 0) = 1.0; H(1, 2) = 1.0; H(2, 4) = 1.0;

        const double r_diag = R3 ? R3->coeff(0, 0) : sigma_ * sigma_;
        const Eigen::Matrix3d S =
            H * Pcv * H.transpose() + Eigen::Matrix3d::Identity() * r_diag;
        const double det = S.determinant();
        if (det <= 0.0) return -1e9;
        return -0.5 * (y.transpose() * S.inverse() * y)(0, 0)
               - 0.5 * std::log(det)
               - 1.5 * std::log(2.0 * M_PI);
    }

    double ComputeCVChi2(const Eigen::Matrix<double, 3, 1>& z,
                         const Eigen::Matrix3d* R3) const {
        const auto& x_cv = cv_.GetState();
        const auto  Pcv  = cv_.GetStateUncertainty();

        Eigen::Matrix<double, 3, 1> y;
        y(0) = z(0) - x_cv(0);
        y(1) = z(1) - x_cv(2);
        y(2) = z(2) - x_cv(4);

        Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
        H(0, 0) = 1.0; H(1, 2) = 1.0; H(2, 4) = 1.0;

        const double r_diag = R3 ? R3->coeff(0, 0) : sigma_ * sigma_;
        const Eigen::Matrix3d S =
            H * Pcv * H.transpose() + Eigen::Matrix3d::Identity() * r_diag;
        if (S.determinant() <= 0.0) return -1e9;
        return (y.transpose() * S.inverse() * y)(0, 0);
    }

    CVFilter<3>  cv_;
    CTRFilter    ctr_;
    NMFilter     nm_;
    double       dt_;

    std::array<double, kNumModels> mu_;
    double pi_[kNumModels][kNumModels];

    Vec3   fused_position_;
    Vec3   fused_velocity_;
    double fused_yaw_ = 0.0;
    double sigma_;
    Vec3   chi2_;

    static constexpr double cv_measurement_variance_approx_ = 0.01;
};

} // namespace filtering
} // namespace perception
} // namespace avt_341
