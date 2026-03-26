/**
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +  AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles  +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

* @file      ctr_filter.hpp
* @brief     Extended Kalman Filter with a Constant Turn Rate (CTR) motion
*            model and position-only measurement (no acceleration state).
*
* State vector (5D): [x, y, v, yaw, yaw_rate]
*   x        - position along global X axis         [m]
*   y        - position along global Y axis         [m]
*   v        - longitudinal speed                   [m/s]
*   yaw      - heading angle (0 = +X axis, CCW+)   [rad]
*   yaw_rate - turn rate (omega, CCW+)              [rad/s]
*
* Measurement vector (2D): [x, y]  — position only.
*
* This CTR model follows the recommendation in:
*   Granstrom, K., Willett, P., & Bar-Shalom, Y. (2016).
*   Systematic approach to IMM mixing for unequal dimension states.
*   IEEE Transactions on Aerospace and Electronic Systems, 51(4), 2975-2986.
*
* Dropping the acceleration state eliminates the Jacobian singularity near
* zero speed that CTRA suffers from, and removes the ill-conditioned
* dv'/da = dt coupling in the state transition matrix.
*
* @copyright MIT License, NATO AVT-341 Autonomy Stack (2026)
*/

#pragma once

#include <cmath>
#include <Eigen/Dense>
#include "angles/angles.h"

namespace avt_341 {
namespace perception {
namespace filtering {

/**
 * @brief EKF implementation of the CTR (Constant Turn Rate) motion model
 *        with 2D position-only observations.
 *
 * The state transition F is linearized (Jacobian) around the current state at
 * each predict step, making this a first-order EKF.
 *
 * State vector x = [x, y, v, yaw, omega]^T  (5-dimensional)
 * Measurement  z = [x, y]^T                 (2-dimensional)
 */
class CTRFilter {
  public:
    // Dimensions
    static constexpr int kStateDim       = 5;
    static constexpr int kMeasurementDim = 2;

    using StateVector       = Eigen::Matrix<double, kStateDim, 1>;
    using MeasurementVector = Eigen::Matrix<double, kMeasurementDim, 1>;
    using StateMatrix       = Eigen::Matrix<double, kStateDim, kStateDim>;
    using MeasurementMatrix = Eigen::Matrix<double, kMeasurementDim, kStateDim>;
    using MeasurementCovariance = Eigen::Matrix<double, kMeasurementDim, kMeasurementDim>;

    /**
     * @param dt                Time step [s].
     * @param process_variance  Scaling of the process noise covariance Q.
     * @param measurement_variance  Scaling of the measurement noise covariance R.
     */
    CTRFilter(const double dt, const double process_variance,
               const double measurement_variance)
        : dt_(dt) {
        x_.setZero();
        P_ = StateMatrix::Identity() * 100;

        BuildQ(process_variance);
        BuildR(measurement_variance);
        BuildH();
    }

    // -----------------------------------------------------------------------
    // Initialization helpers
    // -----------------------------------------------------------------------

    /** @brief Set the initial 2D position from an (x,y,z) Eigen vector (z ignored). */
    void SetInitialPosition(const Eigen::Vector3d& pos) {
        x_(0) = pos.x();
        x_(1) = pos.y();
    }

    /** @brief Zero all velocity / turn-rate states. */
    void SetInitialVelocity(const Eigen::Vector3d& /*vel*/) {
        x_(2) = 0.0;  // v
        x_(3) = 0.0;  // yaw
        x_(4) = 0.0;  // omega
    }

    // -----------------------------------------------------------------------
    // EKF Predict
    // -----------------------------------------------------------------------

    /**
     * @brief Propagate state through the CTR motion model and update P.
     *
     * The nonlinear prediction function f(x) and its Jacobian F_j are
     * computed around the current state estimate.
     */
    void Predict() {
        const double v     = x_(2);
        const double yaw   = x_(3);
        const double omega = x_(4);
        const double dt    = dt_;

        // ---- Nonlinear state prediction f(x) ----
        StateVector x_pred;
        if (std::abs(omega) < kOmegaEps) {
            // Straight-line degenerate case (omega ~ 0)
            x_pred(0) = x_(0) + v * dt * std::cos(yaw);
            x_pred(1) = x_(1) + v * dt * std::sin(yaw);
        } else {
            // General CTR prediction
            const double yaw1 = yaw + omega * dt;
            const double r    = v / omega;
            x_pred(0) = x_(0) + r * (std::sin(yaw1) - std::sin(yaw));
            x_pred(1) = x_(1) + r * (std::cos(yaw) - std::cos(yaw1));
        }
        x_pred(2) = v;
        x_pred(3) = yaw + omega * dt;
        x_pred(4) = omega;

        // Enforce non-negative speed: flip direction if v < 0
        if (x_pred(2) < 0) {
            x_pred(2) = -x_pred(2);
            x_pred(3) = angles::normalize_angle(x_pred(3) + M_PI);
        }

        // ---- Jacobian of f w.r.t. x (linearized F) ----
        StateMatrix F_j = StateMatrix::Identity();

        if (std::abs(omega) < kOmegaEps) {
            const double cos_y = std::cos(yaw);
            const double sin_y = std::sin(yaw);
            // df/dv
            F_j(0, 2) = dt * cos_y;
            F_j(1, 2) = dt * sin_y;
            // df/dyaw
            F_j(0, 3) = -v * dt * sin_y;
            F_j(1, 3) =  v * dt * cos_y;
            // dyaw/domega
            F_j(3, 4) = dt;
        } else {
            const double yaw1   = yaw + omega * dt;
            const double cos_y  = std::cos(yaw);
            const double sin_y  = std::sin(yaw);
            const double cos_y1 = std::cos(yaw1);
            const double sin_y1 = std::sin(yaw1);
            const double oo     = omega * omega;
            const double r      = v / omega;

            // df/dv
            F_j(0, 2) =  (sin_y1 - sin_y) / omega;
            F_j(1, 2) =  (cos_y - cos_y1) / omega;
            // df/dyaw
            F_j(0, 3) =  r * (cos_y1 - cos_y);
            F_j(1, 3) =  r * (sin_y1 - sin_y);
            // df/domega
            F_j(0, 4) = -v / oo * (sin_y1 - sin_y) + v * dt / omega * cos_y1;
            F_j(1, 4) = -v / oo * (cos_y - cos_y1) + v * dt / omega * sin_y1;
            // dyaw/domega
            F_j(3, 4) = dt;
        }

        x_ = x_pred;
        P_ = F_j * P_ * F_j.transpose() + Q_;
    }

    // -----------------------------------------------------------------------
    // EKF Update (linear observation h(x) = H*x)
    // -----------------------------------------------------------------------

    /**
     * @brief Update with a 2D position measurement [x, y].
     *
     * @return The innovation covariance S = H*P*H^T + R, used by the IMM.
     */
    MeasurementCovariance Update(const MeasurementVector& z) {
        return UpdateWithR(z, R_);
    }

    /**
     * @brief Update with a 2D position measurement and a per-call measurement
     *        noise covariance.  The stored R_ is not modified.
     *
     * @param z  2D position measurement [x, y].
     * @param R  Measurement noise covariance to use for this update only.
     * @return   The innovation covariance S = H*P*H^T + R.
     */
    MeasurementCovariance Update(const MeasurementVector& z,
                                 const MeasurementCovariance& R) {
        return UpdateWithR(z, R);
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /** @brief 2D position [x, y] extracted from the state. */
    Eigen::Vector2d GetPosition2D() const { return x_.head<2>(); }

    /** @brief Speed (longitudinal) [m/s]. */
    double GetSpeed()  const { return x_(2); }

    /** @brief Heading angle [rad]. */
    double GetYaw()    const { return x_(3); }

    /** @brief Turn rate [rad/s]. */
    double GetOmega()  const { return x_(4); }

    const StateVector& GetState()      const { return x_; }
    const StateMatrix& GetCovariance() const { return P_; }

    /** @brief Compute the (scalar) log-likelihood of measurement z given the
     *         current predicted distribution N(H*x, S). */
    double LogLikelihood(const MeasurementVector& z,
                         const MeasurementCovariance& S) const {
        const MeasurementVector y = z - H_ * x_;
        const double det = S.determinant();
        if (det <= 0.0) return -1e9;
        return -0.5 * (y.transpose() * S.inverse() * y)(0, 0)
               - 0.5 * std::log(det)
               - static_cast<double>(kMeasurementDim) / 2.0 * std::log(2.0 * M_PI);
    }

    /** @brief Compute the (scalar) chi2 (NIS) measure given measurement z. */
    double chi2(const MeasurementVector& z,
                const MeasurementCovariance& S) const {
        const MeasurementVector y = z - H_ * x_;
        const double det = S.determinant();
        if (det <= 0.0) return -1e9;
        return (y.transpose() * S.inverse() * y)(0, 0);
    }

    /** @brief Access the measurement matrix H (for IMM mixing). */
    const MeasurementMatrix& GetH() const { return H_; }

    /** @brief Set state and covariance directly (used by IMM mixing). */
    void SetState(const StateVector& x)      { x_ = x; }
    void SetCovariance(const StateMatrix& P) { P_ = P; }

  private:
    static constexpr double kOmegaEps = 1e-4;  ///< Threshold for straight-line approximation.

    MeasurementCovariance UpdateWithR(const MeasurementVector& z,
                                      const MeasurementCovariance& R) {
        const MeasurementVector y = z - H_ * x_;
        const MeasurementCovariance S = H_ * P_ * H_.transpose() + R;
        const Eigen::Matrix<double, kStateDim, kMeasurementDim> K =
            P_ * H_.transpose() * S.inverse();
        x_ += K * y;
        // Joseph form for numeric stability
        auto I_KH = (StateMatrix::Identity() - K * H_);
        P_ = (I_KH * P_) * I_KH.transpose() + (K * R) * K.transpose();
        P_ = (P_ + P_.transpose()) / 2;  // ensure symmetry
        if (x_(2) < 0) {
            x_(2) = -x_(2);
            x_(3) = angles::normalize_angle(x_(3) + M_PI);
        }
        return S;
    }

    void BuildQ(const double sigma) {
        Q_.setZero();
        const double dt2 = dt_ * dt_;
        const double dt3 = dt2 * dt_;
        const double dt4 = dt3 * dt_;
        const double s2  = sigma * sigma;
        const double s2o = sigma * sigma * 1600;
        // Position/speed block
        Q_(0, 0) = 0.25 * dt4 * s2;
        Q_(1, 1) = 0.25 * dt4 * s2;
        Q_(0, 2) = Q_(2, 0) = 0.5 * dt3 * s2;
        Q_(2, 2) = dt2 * s2;
        // Yaw/omega block
        Q_(3, 3) = 0.25 * dt4 * s2o;
        Q_(4, 4) = dt2 * s2o;
        Q_(3, 4) = Q_(4, 3) = 0.5 * dt3 * s2o;
    }

    void BuildR(const double sigma) {
        R_ = MeasurementCovariance::Identity() * sigma * sigma;
    }

    void BuildH() {
        H_.setZero();
        H_(0, 0) = 1.0;   // z(0) = x
        H_(1, 1) = 1.0;   // z(1) = y
    }

    double dt_;

    StateVector          x_;
    StateMatrix          P_;
    StateMatrix          Q_;
    MeasurementCovariance R_;
    MeasurementMatrix    H_;
};

} // namespace filtering
} // namespace perception
} // namespace avt_341
