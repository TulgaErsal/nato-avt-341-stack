/**
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +  AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles  +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

* @file      imm_filter.hpp
* @brief     Interacting Multiple Model (IMM) filter combining:
*              - Model 0: Constant Velocity (CV), 6-state linear KF
*              - Model 1: Constant Turn Rate and Acceleration (CTRA), 6-state EKF
*              - Model 2: No Motion (NM), 2-state linear KF
*
* The IMM algorithm (Bar-Shalom et al.) runs at every time step:
*   1. Mixing  — compute mixed initial conditions for each model.
*   2. Predict — each model runs its individual prediction step.
*   3. Update  — each model updates with the shared position measurement.
*   4. Combine — fuse model outputs weighted by their likelihood-updated
*                model probabilities.
*
* All models observe 2D position (x, y) from the same LiDAR measurement.
* The z-axis estimate is taken from the CV filter only, since CTRA and NM are
* 2D horizontal-plane models.
*
* Parameters exposed as ROS node parameters:
*   filters_kalman_process       — shared process noise scale
*   filters_kalman_measurement   — shared measurement noise scale
*   filters_imm_cv_init_prob     — initial probability for the CV model  (default 0.33)
*   filters_imm_ctra_init_prob   — initial probability for the CTRA model (default 0.33)
*   filters_imm_nm_init_prob     — initial probability for the NM model  (default 0.33)
*   filters_imm_transition_prob  — Markov model-transition probability   (default 0.9)
*
* @copyright MIT License, NATO AVT-341 Autonomy Stack (2026)
*/

#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <Eigen/Dense>

#include <avt_341/perception/filtering/cv_filter.hpp>
#include <avt_341/perception/filtering/ctra_filter.hpp>
#include <avt_341/perception/filtering/nm_filter.hpp>

namespace avt_341 {
namespace perception {
namespace filtering {

/**
 * @brief IMM filter: CV (model 0) + CTRA (model 1) + NM (model 2).
 *
 * External interface mirrors the CAFilter API used in ObjectTrackingNode,
 * so the node needs only minimal changes:
 *   - Replace `std::shared_ptr<CAFilter<3>> filter_` with
 *     `std::shared_ptr<IMMFilter>  filter_`
 *   - Pass 2D position measurement to Update() (same as before)
 *   - Read position from GetPosition3D() and velocity from GetVelocity3D()
 */
class IMMFilter {
  public:
    static constexpr int kNumModels = 3;
    // 3D position + velocity vectors returned to the tracker
    using Vec3 = Eigen::Vector3d;
    using Vec2 = Eigen::Vector2d;
    using MeasurementVector3D = Eigen::Matrix<double, 3, 1>;
    using MeasurementVector2D = Eigen::Matrix<double, 2, 1>;

    /**
     * @param dt                   Estimator time step [s].
     * @param process_variance     Process noise scale (shared between models).
     * @param measurement_variance Measurement noise scale (shared).
     * @param cv_init_prob         Initial model probability for CV model.
     * @param ctra_init_prob       Initial model probability for CTRA model.
     * @param nm_init_prob         Initial model probability for NM model.
     * @param transition_prob      Diagonal entry of the Markov transition matrix
     *                             (probability of staying in the same model).
     */
    IMMFilter(const double dt,
              const double process_variance,
              const double measurement_variance,
              const double cv_init_prob   = 0.33,
              const double ctra_init_prob = 0.33,
              const double nm_init_prob   = 0.33,
              const double transition_prob = 0.9)
        : cv_(dt, process_variance, measurement_variance),
          ctra_(dt, process_variance, measurement_variance),
          nm_(dt, process_variance, measurement_variance),
          dt_(dt)
    {
        // Normalize initial model probabilities
        const double sum = cv_init_prob + ctra_init_prob + nm_init_prob;
        mu_[0] = cv_init_prob   / sum;
        mu_[1] = ctra_init_prob / sum;
        mu_[2] = nm_init_prob   / sum;

 

        // Markov transition matrix: pi_[i][j] = P(switching to j | currently in i)
        const double p_stay   = std::max(0.01, std::min(0.99, transition_prob));
        const double p_switch = (1.0 - p_stay) / (kNumModels - 1);
        for (int i = 0; i < kNumModels; ++i) {
            for (int j = 0; j < kNumModels; ++j) {
                pi_[i][j] = (i == j) ? p_stay : p_switch;
            }
        }

        // Fused state output
        fused_position_.setZero();
        fused_velocity_.setZero();
        fused_yaw_ = 0;
    }

    // -----------------------------------------------------------------------
    // Initialization helpers  (match CAFilter API)
    // -----------------------------------------------------------------------

    /**
     * @brief Set global initial position for all sub-filters.
     * @param pos 3D global position vector [x, y, z].
     */
    void SetInitialPosition(const Vec3& pos) {
        cv_.SetInitialPosition(pos);
        ctra_.SetInitialPosition(pos);
        nm_.SetInitialPosition(pos);
        fused_position_ = pos;
    }

    /**
     * @brief Set initial velocity for sub-filters that track velocity.
     *        NM has no velocity state so it ignores this call.
     * @param vel 3D velocity [vx, vy, vz].
     */
    void SetInitialVelocity(const Vec3& vel) {
        cv_.SetInitialVelocity(vel);
        ctra_.SetInitialVelocity(vel);
        nm_.SetInitialVelocity(vel);
    }

    // -----------------------------------------------------------------------
    // IMM Predict
    // -----------------------------------------------------------------------

    /**
     * @brief Run one full IMM predict cycle:
     *          1. Compute mixing probabilities
     *          2. Compute mixed initial conditions for each filter
     *          3. Run each filter's predict step
     */
    void Predict() {
        // --- Step 1: Mixing probabilities  c_ij = mu_[i] * pi_[i][j] ---
        // c_j = sum_i( mu_[i] * pi_[i][j] )   (predicted model probability)
        std::array<double, kNumModels> c_bar;
        for (int j = 0; j < kNumModels; ++j) {
            c_bar[j] = 0.0;
            for (int i = 0; i < kNumModels; ++i) {
                c_bar[j] += mu_[i] * pi_[i][j];
            }
        }
        // Mixing weights: mu_ij = mu_[i] * pi_[i][j] / c_bar[j]
        double mu_mix[kNumModels][kNumModels];
        for (int j = 0; j < kNumModels; ++j) {
            for (int i = 0; i < kNumModels; ++i) {
                mu_mix[i][j] = (c_bar[j] > 1e-12)
                    ? (mu_[i] * pi_[i][j] / c_bar[j])
                    : (1.0 / kNumModels);
            }
        }

        // --- Step 2: Mix initial conditions ---
        // All three models share the 2D position (x, y) as the common subspace.
        // Only the position entries are mixed across models; each model's own
        // velocity / heading states are left intact to avoid cross-contamination
        // of incompatible state representations.
        //
        // CV state layout (stride = 2):  [x, vx, y, vy, z, vz]
        //   x pos → index 0,  y pos → index 2,  z pos → index 4
        {
            const auto& x_cv    = cv_.GetState();          // 6D  [x,vx, y,vy, z,vz]
            const Vec2  pos_ctra = ctra_.GetPosition2D();  // 2D  [x, y]
            const Vec2  pos_nm   = nm_.GetPosition2D();    // 2D  [x, y]

            // --- Mix into CV (model 0) ---
            Eigen::Matrix<double, 6, 1> x_cv_mixed = x_cv;
            x_cv_mixed(0) = mu_mix[0][0] * x_cv(0)       // x from CV
                          + mu_mix[1][0] * pos_ctra(0)    // x from CTRA
                          + mu_mix[2][0] * pos_nm(0);     // x from NM
            x_cv_mixed(2) = mu_mix[0][0] * x_cv(2)       // y from CV
                          + mu_mix[1][0] * pos_ctra(1)    // y from CTRA
                          + mu_mix[2][0] * pos_nm(1);     // y from NM
            cv_.SetState(x_cv_mixed);

            // --- Mix into CTRA (model 1) ---
            const auto& x_ctra = ctra_.GetState();
            CTRAFilter::StateVector x_ctra_mixed = x_ctra;
            x_ctra_mixed(0) = mu_mix[0][1] * x_cv(0)
                            + mu_mix[1][1] * x_ctra(0)
                            + mu_mix[2][1] * pos_nm(0);
            x_ctra_mixed(1) = mu_mix[0][1] * x_cv(2)
                            + mu_mix[1][1] * x_ctra(1)
                            + mu_mix[2][1] * pos_nm(1);
            ctra_.SetState(x_ctra_mixed);

            // --- Mix into NM (model 2) ---
            const auto& x_nm = nm_.GetState();
            NMFilter::StateVector x_nm_mixed;
            x_nm_mixed(0) = mu_mix[0][2] * x_cv(0)
                          + mu_mix[1][2] * x_ctra(0)
                          + mu_mix[2][2] * x_nm(0);
            x_nm_mixed(1) = mu_mix[0][2] * x_cv(2)
                          + mu_mix[1][2] * x_ctra(1)
                          + mu_mix[2][2] * x_nm(1);
            nm_.SetState(x_nm_mixed);
        }

        // --- Step 3: Individual predictions ---
        cv_.Predict();
        ctra_.Predict();
        nm_.Predict();
    }

    // -----------------------------------------------------------------------
    // IMM Update
    // -----------------------------------------------------------------------

    /**
     * @brief Update the IMM with a 3D position measurement.
     *
     * The z component is used only by the CV filter; CTRA and NM receive [x, y].
     *
     * @param z          3D position measurement [x, y, z] in the world frame.
     * @param R_override Optional 3×3 measurement noise covariance for this
     *                   step.  When supplied it overrides the static R built at
     *                   construction time for all sub-filters:
     *                     - CV    receives the full 3×3 matrix.
     *                     - CTRA and NM receive the top-left 2×2 block.
     *                   When std::nullopt (default), each sub-filter uses its
     *                   own internally stored R.
     */
    void Update(const MeasurementVector3D& z,
                const std::optional<Eigen::Matrix3d>& R_override = std::nullopt) {
        const MeasurementVector2D z2d = z.head<2>();

        // --- Update each filter and collect innovation covariances ---
        if (R_override) {
            const Eigen::Matrix2d R2d = R_override->topLeftCorner<2, 2>();
			

            cv_.Update(z, R_override.value());
            CTRAFilter::MeasurementCovariance S_ctra = ctra_.Update(z2d, R2d);
            NMFilter::MeasurementCovariance   S_nm   = nm_.Update(z2d, R2d);

            // --- Compute log-likelihoods using the supplied R ---
            const double lambda_cv   = ComputeCVLikelihood(z, &R_override.value());
            const double lambda_ctra = ctra_.LogLikelihood(z2d, S_ctra);
            const double lambda_nm   = nm_.LogLikelihood(z2d, S_nm);

            // --- Compute chi2 ---
            const double chi2_cv   = ComputeCVChi2(z, nullptr);
            const double chi2_ctra = ctra_.chi2(z2d, S_ctra);
            const double chi2_nm   = nm_.chi2(z2d, S_nm);
            // collect chi2
            chi2_.x() = chi2_cv;
            chi2_.y() = chi2_ctra;
            chi2_.z() = chi2_nm;
            UpdateModelProbabilities(lambda_cv, lambda_ctra, lambda_nm);
        } else {
            cv_.Update(z);

            CTRAFilter::MeasurementCovariance S_ctra = ctra_.Update(z2d);
            NMFilter::MeasurementCovariance   S_nm   = nm_.Update(z2d);

            // --- Compute log-likelihoods ---
            const double lambda_cv   = ComputeCVLikelihood(z, nullptr);
            const double lambda_ctra = ctra_.LogLikelihood(z2d, S_ctra);
            const double lambda_nm   = nm_.LogLikelihood(z2d, S_nm);

            // --- Compute chi2 ---
            const double chi2_cv   = ComputeCVChi2(z, nullptr);
            const double chi2_ctra = ctra_.chi2(z2d, S_ctra);
            const double chi2_nm   = nm_.chi2(z2d, S_nm);
            // collect chi2 Vec3 used for convenience
            chi2_.x() = chi2_cv;
            chi2_.y() = chi2_ctra;
            chi2_.z() = chi2_nm;
            UpdateModelProbabilities(lambda_cv, lambda_ctra, lambda_nm);
        }

        // --- Fuse outputs (weighted combination of all three models) ---
        // CV state layout (stride = 2):  [x, vx, y, vy, z, vz]
        const auto& x_cv   = cv_.GetState();
        const Vec2  p_ctra = ctra_.GetPosition2D();
        double v_ctra = ctra_.GetSpeed();
        double yaw_ctra = ctra_.GetYaw();
         
        const Vec2  p_nm   = nm_.GetPosition2D();

        fused_position_.x() = mu_[0] * x_cv(0) + mu_[1] * p_ctra(0) + mu_[2] * p_nm(0);
        fused_position_.y() = mu_[0] * x_cv(2) + mu_[1] * p_ctra(1) + mu_[2] * p_nm(1);
        // z only from CV (CTRA and NM are 2D)
        fused_position_.z() = x_cv(4);

        if (v_ctra > 0.2 || sqrt(std::pow(x_cv(1), 2) + std::pow(x_cv(3), 2)) > 0.2) { 
            //when sufficient speed the CV model velocity have information of yaw
            // NM gives no information of orientation
            double yaw_cv = atan2(x_cv(1), x_cv(3));
            //if (abs(yaw_cv- yaw_ctra)<3/12)
                //fused_yaw_ = mu_[0] * yaw_cv + mu_[1] * yaw_ctra;
            //else 
               fused_yaw_ = yaw_ctra;

        }
        else {
            fused_yaw_ = yaw_ctra; // ctra may have memory of yaw even if speed is zero

        }

            

        // Velocity from CV filter and ctra.
        // When NM dominates (mu_[2] ≈ 1), the CV velocity naturally tends to
        // zero as the CV filter observes no motion.
        fused_velocity_.x() = mu_[0] * x_cv(1) + mu_[1] * v_ctra * cos(yaw_ctra);
        fused_velocity_.y() = mu_[0] * x_cv(3) + mu_[1] * v_ctra * sin(yaw_ctra);
        fused_velocity_.z() = x_cv(5);


    }

    // -----------------------------------------------------------------------
    // Accessors mirroring the CAFilter interface expected by the tracker node
    // -----------------------------------------------------------------------

    /**
     * @brief Returns a synthetic 9-element state vector compatible with existing
     *        tracker code that reads state_filtered(0), (3), (6) for x,y,z and
     *        (1), (4), (7) for vx, vy, vz.  Acceleration entries (2), (5), (8)
     *        are set to zero since the CV model does not track acceleration.
     */
    Eigen::Matrix<double, 9, 1> GetState() const {
        const auto& x_cv = cv_.GetState();
        Eigen::Matrix<double, 9, 1> s;
        s(0) = fused_position_.x();   // x pos (fused)
        s(1) = x_cv(1);               // vx
        s(2) = 0.0;                   // ax (not tracked by CV)
        s(3) = fused_position_.y();   // y pos (fused)
        s(4) = x_cv(3);               // vy
        s(5) = 0.0;                   // ay (not tracked by CV)
        s(6) = fused_position_.z();   // z pos (from CV)
        s(7) = x_cv(5);               // vz
        s(8) = 0.0;                   // az (not tracked by CV)
        return s;
    }

    /** @brief Fused 3D world-frame position. */
    Vec3 GetPosition3D() const { return fused_position_; }

    /** @brief Fused yaw. */
    double GetYaw() const { return fused_yaw_; }

    /** @brief 3D velocity estimate (from CV model). */
    Vec3 GetVelocity3D() const { return fused_velocity_; }

    /** @brief Probability of each model: [0]=CV, [1]=CTRA, [2]=NM. */
    const std::array<double, kNumModels>& GetModelProbabilities() const { return mu_; }

    /** @brief CTRA speed estimate [m/s]. */
    double GetCTRASpeed() const { return ctra_.GetSpeed(); }

    /** @brief CTRA heading estimate [rad]. */
    double GetCTRAYaw() const { return ctra_.GetYaw(); }

    /** @brief CTRA turn rate estimate [rad/s]. */
    double GetCTRAOmega() const { return ctra_.GetOmega(); }

    /** @brief CTRA turn state covariance matrix */
    CTRAFilter::StateMatrix GetCTRACovariance() const { return ctra_.GetCovariance(); }


    /** @brief Access underlying CV filter (for state injection by tracker). */
    CVFilter<3>& GetCVFilter() { return cv_; }

    /** @brief Access underlying CTRA filter. */
    CTRAFilter& GetCTRAFilter() { return ctra_; }

    /** @brief Access underlying NM filter. */
    NMFilter& GetNMFilter() { return nm_; }
    
    /** @brief Access underlying chi2 results */
    Vec3  GetAllChi2() { return chi2_; }

    Eigen::Matrix<double, 2, 2> GetnmStatecovariance() { return nm_.GetS(); };

  private:
    // ---- Update model probabilities from three log-likelihoods ----
    void UpdateModelProbabilities(const double log_l_cv,
                                  const double log_l_ctra,
                                  const double log_l_nm) {
        // Convert log-likelihoods to non-negative weights (numerically stable)
        const double max_ll = std::max({log_l_cv, log_l_ctra, log_l_nm});
        const std::array<double, kNumModels> l = {
            std::exp(log_l_cv   - max_ll),
            std::exp(log_l_ctra - max_ll),
            std::exp(log_l_nm   - max_ll)
        };

        // Predicted model probabilities  c_bar = sum_i( mu_[i] * pi_[i][j] )
        std::array<double, kNumModels> c_bar;
        for (int j = 0; j < kNumModels; ++j) {
            c_bar[j] = 0.0;
            for (int i = 0; i < kNumModels; ++i) {
                c_bar[j] += mu_[i] * pi_[i][j];
            }
        }

        // mu_j ∝ lambda_j * c_bar_j
        double norm = 0.0;
        for (int j = 0; j < kNumModels; ++j) {
            mu_[j] = l[j] * c_bar[j];
            norm   += mu_[j];
        }
        if (norm < 1e-12) {
            for (auto& m : mu_) m = 1.0 / kNumModels;
        } else {
            for (auto& m : mu_) m /= norm;
        }
    }

    // ---- Compute log-likelihood for the CV filter given measurement z ----
    // When R3 is non-null the supplied matrix is used; otherwise the scalar
    // cv_measurement_variance_approx_ approximation is used.
    double ComputeCVLikelihood(const Eigen::Matrix<double, 3, 1>& z,
                               const Eigen::Matrix3d* R3) const {
        const auto& x_cv = cv_.GetState();
        Eigen::Matrix<double, 6, 6> Pcv = cv_.GetStateUncertainty();
        Eigen::Matrix<double, 3, 1> y;
        y(0) = z(0) - x_cv(0);   // x residual
        y(1) = z(1) - x_cv(2);   // y residual (CV stride=2, so y pos at index 2)
        y(2) = z(2) - x_cv(4);   // z residual (z pos at index 4)
		Eigen::Matrix<double, 3, 6> H; // Hardcoded H assuming cv order [x xdot y ydot ...
		H(0,0) = 1;
		H(1,2) = 1;
		H(2,4) = 1;

        if (R3) {
            const Eigen::Matrix<double, 3, 3> I = Eigen::Matrix<double, 3, 3>::Identity();
            Eigen::Matrix<double, 3, 3> S = H * Pcv * H.transpose();
            S = S + I * R3->coeff(0,0);
            const double det = S.determinant();
            if (det <= 0.0) return -1e9;
            return -0.5 * (y.transpose() * S.inverse() * y)(0, 0)
                   - 0.5 * std::log(det)
                   - 1.5 * std::log(2.0 * M_PI);
        } else {
			const Eigen::Matrix<double, 3, 3> I = Eigen::Matrix<double, 3, 3>::Identity();
			const Eigen::Matrix<double, 3, 3> S = H * Pcv * H.transpose() + cv_measurement_variance_approx_ * I;

            const double det_S = S.determinant();
            if (det_S <= 0.0) return -1e9;
            return -0.5 * (y.transpose() * S.inverse() * y)(0, 0)
                   - 0.5 * std::log(det_S)
                   - 1.5 * std::log(2.0 * M_PI);
        }
    }

    // ---- Compute log-likelihood for the CV filter given measurement z ----
    // When R3 is non-null the supplied matrix is used; otherwise the scalar
    // cv_measurement_variance_approx_ approximation is used.
    double ComputeCVChi2(const Eigen::Matrix<double, 3, 1>& z,
                               const Eigen::Matrix3d* R3) const {
        const auto& x_cv = cv_.GetState();
        Eigen::Matrix<double, 6, 6> Pcv = cv_.GetStateUncertainty();
        Eigen::Matrix<double, 3, 1> y;
        y(0) = z(0) - x_cv(0);   // x residual
        y(1) = z(1) - x_cv(2);   // y residual (CV stride=2, so y pos at index 2)
        y(2) = z(2) - x_cv(4);   // z residual (z pos at index 4)
		Eigen::Matrix<double, 3, 6> H; // Hardcoded H assuming cv order [x xdot y ydot ...
									   // and z as position [x,y,z]
		H(0,0) = 1; // pos_x
		H(1,2) = 1; // pos_y
		H(2,4) = 1; // pos_z
        if (R3) {
            const Eigen::Matrix<double, 3, 3> I = Eigen::Matrix<double, 3, 3>::Identity();
            Eigen::Matrix<double, 3, 3> S = H * Pcv * H.transpose();
            S = S + I * R3->coeff(0, 0);
			const double det = S.determinant();
            if (det <= 0.0) return -1e9;
            return (y.transpose() * S.inverse() * y)(0, 0);
  
        } else {
            const Eigen::Matrix<double, 3, 3> I = Eigen::Matrix<double, 3, 3>::Identity();
            const Eigen::Matrix<double, 3, 3> S = H * Pcv * H.transpose() + cv_measurement_variance_approx_ * I;
			const double det = S.determinant();
			if (det <= 0.0) return -1e9;
            return  (y.transpose() * S.inverse() * y)(0, 0);
                 
        }
    }
    CVFilter<3>  cv_;
    CTRAFilter   ctra_;
    NMFilter     nm_;
    double       dt_;

    // Model probabilities μ_j
    std::array<double, kNumModels> mu_;

    // Markov transition matrix π[i][j]
    double pi_[kNumModels][kNumModels];

    // Fused output
    Vec3 fused_position_;
    Vec3 fused_velocity_;
    double fused_yaw_;
    // chi2 measures
    Vec3 chi2_;

    // Approximate CV measurement variance used in likelihood computation.
    static constexpr double cv_measurement_variance_approx_ = 0.01;  // (0.1m σ)²
};

} // namespace filtering
} // namespace perception
} // namespace avt_341
