/**
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +  AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles  +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

* @file      nm_filter.hpp
* @brief     Kalman filter with a No Motion (NM) model and 2D position-only
*            measurement.
*
* State vector (2D): [x, y]  — position only.
*   x  - position along global X axis  [m]
*   y  - position along global Y axis  [m]
*
* Motion model:  x_{k+1} = x_k  (identity transition, zero velocity assumed).
* Process noise: small isotropic covariance that allows the uncertainty to
*                grow when the true object starts moving, which causes the IMM
*                to shift probability to the CV or CTR model.
*
* Measurement vector (2D): [x, y]  — position only.
*
* @copyright MIT License, NATO AVT-341 Autonomy Stack (2026)
*/

#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace avt_341 {
namespace perception {
namespace filtering {

/**
 * @brief Linear Kalman filter implementing the No Motion (NM) model.
 *
 * State transition F = I (stationary assumption).
 * Observation     H = I (position is observed directly).
 *
 * State vector x = [x, y]^T  (2-dimensional)
 * Measurement  z = [x, y]^T  (2-dimensional)
 */
class NMFilter {
  public:
    static constexpr int kStateDim       = 2;
    static constexpr int kMeasurementDim = 2;

    using StateVector           = Eigen::Matrix<double, kStateDim, 1>;
    using StateMatrix           = Eigen::Matrix<double, kStateDim, kStateDim>;
    using MeasurementVector     = Eigen::Matrix<double, kMeasurementDim, 1>;
    using MeasurementCovariance = Eigen::Matrix<double, kMeasurementDim, kMeasurementDim>;

    /**
     * @param dt                  Time step [s] (kept for API symmetry; F is identity).
     * @param process_variance    Standard deviation of the position-level process
     *                            noise.  Q = process_variance² · I.  Keep small
     *                            (e.g. 0.01–0.1 m) so the model stays confident
     *                            when the object is truly stationary, but loses
     *                            probability quickly once motion is detected.
     * @param measurement_variance Standard deviation of the position measurement.
     *                            R = measurement_variance² · I.
     */
    NMFilter(const double /*dt*/,
             const double process_variance,
             const double measurement_variance) {
        x_.setZero();
        P_ = StateMatrix::Identity() * 100;
        Q_ = StateMatrix::Identity() * (process_variance * process_variance);
        R_ = MeasurementCovariance::Identity() * (measurement_variance * measurement_variance);
    }

    // -----------------------------------------------------------------------
    // Initialization helpers  (mirror CTRFilter API)
    // -----------------------------------------------------------------------

    /** @brief Set the initial 2D position from an (x,y,z) Eigen vector (z ignored). */
    void SetInitialPosition(const Eigen::Vector3d& pos) {
        x_(0) = pos.x();
        x_(1) = pos.y();
    }

    /** @brief No velocity state exists in the NM model; this is a no-op. */
    void SetInitialVelocity(const Eigen::Vector3d& /*vel*/) {}

    // -----------------------------------------------------------------------
    // Predict  (identity transition)
    // -----------------------------------------------------------------------

    /**
     * @brief Propagate uncertainty: x is unchanged, P grows by Q.
     */
    void Predict() {
        // F = I  →  x_pred = x,  P_pred = P + Q
        P_ += Q_;
    }

    // -----------------------------------------------------------------------
    // Update  (linear, H = I)
    // -----------------------------------------------------------------------

    /**
     * @brief Update with a 2D position measurement [x, y].
     *
     * @return The innovation covariance S = P + R, used by the IMM for
     *         likelihood computation.
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
     * @return   The innovation covariance S = P + R.
     */
    MeasurementCovariance Update(const MeasurementVector& z,
                                 const MeasurementCovariance& R) {
        return UpdateWithR(z, R);
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /** @brief 2D position [x, y] extracted from the state. */
    Eigen::Vector2d GetPosition2D() const { return x_; }

    const StateVector& GetState()      const { return x_; }
    const StateMatrix& GetCovariance() const { return P_; }
    StateMatrix GetS() const { return P_ + R_; }

    /** @brief Compute the scalar log-likelihood of measurement z given N(x, S). */
    double LogLikelihood(const MeasurementVector& z,
                         const MeasurementCovariance& S) const {
        const MeasurementVector y = z - x_;
        const double det = S.determinant();
        if (det <= 0.0) return -1e9;
        return -0.5 * (y.transpose() * S.inverse() * y)(0, 0)
               - 0.5 * std::log(det)
               - static_cast<double>(kMeasurementDim) / 2.0 * std::log(2.0 * M_PI);
    }
     /** @brief Compute the scalar chi2 of measurement z given N(x, S). */
    double chi2(const MeasurementVector& z,
                         const MeasurementCovariance& S) const {
        const MeasurementVector y = z - x_;
        const double det = S.determinant();
        if (det <= 0.0) return -1e9;
        return (y.transpose() * S.inverse() * y)(0, 0);
    }
    /** @brief Set state and covariance directly (used by IMM mixing). */
    void SetState(const StateVector& x)      { x_ = x; }
    void SetCovariance(const StateMatrix& P) { P_ = P; }

  private:
    MeasurementCovariance UpdateWithR(const MeasurementVector& z,
                                      const MeasurementCovariance& R) {
        // H = I  →  y = z - x,  S = P + R,  K = P * S^{-1}
        const MeasurementVector y = z - x_;
        const MeasurementCovariance S = P_ + R;
        const StateMatrix K = P_ * S.inverse();
        x_ += K * y;
        // Swap to Joseph form for numeric stability
        auto I_KH = (StateMatrix::Identity() - K);  // H identity
        P_ = (I_KH * P_) * I_KH.transpose() + (K * R) * K.transpose();
        // P_ = (StateMatrix::Identity() - K) * P_;
        return S;
    }

    StateVector           x_;
    StateMatrix           P_;
    StateMatrix           Q_;
    MeasurementCovariance R_;
};

} // namespace filtering
} // namespace perception
} // namespace avt_341
