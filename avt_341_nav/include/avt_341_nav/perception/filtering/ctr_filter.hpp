/**
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +  AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles  +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

* @file      ctr_filter.hpp
* @brief     Extended Kalman Filter with a Constant Turn Rate (CTR) motion
*            model in Cartesian coordinates with position-only measurement.
*
* State vector (5D): [x, vx, y, vy, omega]
*   x     - position along global X axis        [m]
*   vx    - velocity along global X axis        [m/s]
*   y     - position along global Y axis        [m]
*   vy    - velocity along global Y axis        [m/s]
*   omega - turn rate (CCW+)                   [rad/s]
*
* Measurement vector (2D): [x, y]  — position only.
*
* Using the same Cartesian state space as the CV filter ([x,vx,y,vy,...])
* eliminates all Jacobian transformations during IMM mixing: the first four
* states map directly between the two models without any nonlinear projection.
*
* The velocity direction is evolved by rotating [vx, vy] by omega*dt each
* step.  Position is updated via trapezoidal integration.  The Jacobian
* has a closed-form expression that is singularity-free at all speeds and
* turn rates.
*
* @copyright MIT License, NATO AVT-341 Autonomy Stack (2026)
*/

#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace avt_341_nav {
namespace perception {
namespace filtering {

/**
 * @brief EKF implementing the CTR model in Cartesian velocity coordinates.
 *
 * State x = [x, vx, y, vy, omega]^T  (5D)
 * Measurement z = [x, y]^T            (2D, position only)
 *
 * Velocity update (exact rotation):
 *   vx' = vx*cos(omega*dt) - vy*sin(omega*dt)
 *   vy' = vx*sin(omega*dt) + vy*cos(omega*dt)
 *
 * Position update (trapezoidal integration):
 *   x'  = x + (vx + vx')/2 * dt
 *   y'  = y + (vy + vy')/2 * dt
 */
class CTRFilter {
  public:
    static constexpr int kStateDim       = 5;
    static constexpr int kMeasurementDim = 2;

    using StateVector           = Eigen::Matrix<double, kStateDim, 1>;
    using StateMatrix           = Eigen::Matrix<double, kStateDim, kStateDim>;
    using MeasurementVector     = Eigen::Matrix<double, kMeasurementDim, 1>;
    using MeasurementMatrix     = Eigen::Matrix<double, kMeasurementDim, kStateDim>;
    using MeasurementCovariance = Eigen::Matrix<double, kMeasurementDim, kMeasurementDim>;

    /**
     * @param dt                  Time step [s].
     * @param process_variance    Acceleration noise std-dev [m/s^2] used for
     *                            the DWNA process noise of position/velocity.
     *                            The same value is also used as the turn-rate
     *                            noise std-dev [rad/s^2].
     * @param measurement_variance Position measurement noise std-dev [m].
     */
    CTRFilter(const double dt,
              const double process_variance,
              const double measurement_variance)
        : dt_(dt) {
        x_.setZero();
        P_ = StateMatrix::Identity() * 100.0;
        BuildQ(process_variance);
        BuildR(measurement_variance);
        BuildH();
    }

    // -----------------------------------------------------------------------
    // Initialization helpers  (match CVFilter API)
    // -----------------------------------------------------------------------

    /** @brief Set the initial 2D position (z component is ignored). */
    void SetInitialPosition(const Eigen::Vector3d& pos) {
        x_(0) = pos.x();
        x_(2) = pos.y();
    }

    /** @brief Set the initial Cartesian velocity (z component is ignored). */
    void SetInitialVelocity(const Eigen::Vector3d& vel) {
        x_(1) = vel.x();   // vx
        x_(3) = vel.y();   // vy
        x_(4) = 0.0;       // omega
    }

    // -----------------------------------------------------------------------
    // EKF Predict
    // -----------------------------------------------------------------------

    /**
     * @brief Propagate state through the CTR model and linearize.
     *
     * Velocity is rotated by omega*dt; position is integrated trapezoidally.
     * The resulting Jacobian is closed-form and singularity-free.
     */
    void Predict() {
        const double vx    = x_(1);
        const double vy    = x_(3);
        const double omega = x_(4);
        const double c     = std::cos(omega * dt_);
        const double s     = std::sin(omega * dt_);

        // Rotate velocity vector
        const double vx_new = vx * c - vy * s;
        const double vy_new = vx * s + vy * c;

        // Predict state
        StateVector x_pred;
        x_pred(0) = x_(0) + (vx + vx_new) * 0.5 * dt_;   // x (trapezoidal)
        x_pred(1) = vx_new;
        x_pred(2) = x_(2) + (vy + vy_new) * 0.5 * dt_;   // y (trapezoidal)
        x_pred(3) = vy_new;
        x_pred(4) = omega;

        // Jacobian F = df/d[x, vx, y, vy, omega]
        //
        // dx'/domega  = -dt^2/2 * vy_new
        // dvx'/domega = -dt * vy_new     (since vy_new = vx*s + vy*c)
        // dy'/domega  =  dt^2/2 * vx_new
        // dvy'/domega =  dt * vx_new     (since vx_new = vx*c - vy*s)
        StateMatrix F = StateMatrix::Zero();
        const double dt_half = 0.5 * dt_;

        // x row
        F(0, 0) = 1.0;
        F(0, 1) = dt_half * (1.0 + c);
        F(0, 3) = dt_half * (-s);
        F(0, 4) = -dt_ * dt_half * vy_new;

        // vx row
        F(1, 1) = c;
        F(1, 3) = -s;
        F(1, 4) = -dt_ * vy_new;

        // y row
        F(2, 1) = dt_half * s;
        F(2, 2) = 1.0;
        F(2, 3) = dt_half * (1.0 + c);
        F(2, 4) = dt_ * dt_half * vx_new;

        // vy row
        F(3, 1) = s;
        F(3, 3) = c;
        F(3, 4) = dt_ * vx_new;

        // omega row
        F(4, 4) = 1.0;

        x_ = x_pred;
        P_ = F * P_ * F.transpose() + Q_;
        P_ = (P_ + P_.transpose()) * 0.5;  // enforce symmetry
    }

    // -----------------------------------------------------------------------
    // EKF Update  (linear observation H*x)
    // -----------------------------------------------------------------------

    /**
     * @brief Update with a 2D position measurement [x, y].
     * @return Innovation covariance S = H*P*H^T + R (used by IMM for likelihoods).
     */
    MeasurementCovariance Update(const MeasurementVector& z) {
        return UpdateWithR(z, R_);
    }

    /**
     * @brief Update with an explicit measurement noise covariance.
     *        The stored R_ is not modified.
     */
    MeasurementCovariance Update(const MeasurementVector& z,
                                 const MeasurementCovariance& R) {
        return UpdateWithR(z, R);
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /** @brief 2D position [x, y]. */
    Eigen::Vector2d GetPosition2D() const { return {x_(0), x_(2)}; }

    /** @brief Cartesian velocity [vx, vy]. */
    Eigen::Vector2d GetVelocity2D() const { return {x_(1), x_(3)}; }

    /** @brief vx component. */
    double GetVx()    const { return x_(1); }

    /** @brief vy component. */
    double GetVy()    const { return x_(3); }

    /** @brief Speed = sqrt(vx^2 + vy^2) [m/s]. */
    double GetSpeed() const { return std::hypot(x_(1), x_(3)); }

    /** @brief Heading derived from velocity [rad].  Returns 0 when stationary. */
    double GetYaw()   const { return std::atan2(x_(3), x_(1)); }

    /** @brief Turn rate [rad/s]. */
    double GetOmega() const { return x_(4); }

    const StateVector& GetState()      const { return x_; }
    const StateMatrix& GetCovariance() const { return P_; }

    /** @brief Log-likelihood of measurement z under the predicted distribution. */
    double LogLikelihood(const MeasurementVector& z,
                         const MeasurementCovariance& S) const {
        const MeasurementVector y = z - H_ * x_;
        const double det = S.determinant();
        if (det <= 0.0) return -1e9;
        return -0.5 * (y.transpose() * S.inverse() * y)(0, 0)
               - 0.5 * std::log(det)
               - static_cast<double>(kMeasurementDim) * 0.5 * std::log(2.0 * M_PI);
    }

    /** @brief Chi-squared (NIS) statistic for measurement z. */
    double chi2(const MeasurementVector& z,
                const MeasurementCovariance& S) const {
        const MeasurementVector y = z - H_ * x_;
        if (S.determinant() <= 0.0) return -1e9;
        return (y.transpose() * S.inverse() * y)(0, 0);
    }

    const MeasurementMatrix& GetH() const { return H_; }

    void SetState(const StateVector& x)      { x_ = x; }
    void SetCovariance(const StateMatrix& P) { P_ = P; }

  private:
    // -----------------------------------------------------------------------
    // Joseph-form KF update
    // -----------------------------------------------------------------------
    MeasurementCovariance UpdateWithR(const MeasurementVector& z,
                                      const MeasurementCovariance& R) {
        const MeasurementVector     y = z - H_ * x_;
        const MeasurementCovariance S = H_ * P_ * H_.transpose() + R;
        const Eigen::Matrix<double, kStateDim, kMeasurementDim> K =
            P_ * H_.transpose() * S.inverse();
        x_ += K * y;
        const StateMatrix I_KH = StateMatrix::Identity() - K * H_;
        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
        P_ = (P_ + P_.transpose()) * 0.5;  // enforce symmetry
        return S;
    }

    // -----------------------------------------------------------------------
    // Matrix builders
    // -----------------------------------------------------------------------

    /**
     * @brief Discrete white-noise acceleration (DWNA) process noise.
     *
     * Same structure as CVFilter: per-dimension [[dt^4/4, dt^3/2],[dt^3/2, dt^2]]
     * for [x,vx] and [y,vy] blocks.  An independent term dt^2 * sigma^2 is
     * added for omega.
     */
    void BuildQ(const double sigma) {
        Q_.setZero();
        const double s2   = sigma * sigma;
        const double dt2  = dt_ * dt_;
        const double dt3  = dt2 * dt_;
        const double dt4  = dt3 * dt_;

        // [x, vx] block (indices 0,1)
        Q_(0, 0) = 0.25 * dt4 * s2;
        Q_(0, 1) = Q_(1, 0) = 0.5 * dt3 * s2;
        Q_(1, 1) = dt2 * s2;

        // [y, vy] block (indices 2,3)
        Q_(2, 2) = 0.25 * dt4 * s2;
        Q_(2, 3) = Q_(3, 2) = 0.5 * dt3 * s2;
        Q_(3, 3) = dt2 * s2;

        // omega (index 4): independent angular acceleration noise
        Q_(4, 4) = dt2 * s2;
    }

    void BuildR(const double sigma) {
        R_ = MeasurementCovariance::Identity() * (sigma * sigma);
    }

    void BuildH() {
        H_.setZero();
        H_(0, 0) = 1.0;   // z(0) = x,  at state index 0
        H_(1, 2) = 1.0;   // z(1) = y,  at state index 2
    }

    double           dt_;
    StateVector      x_;
    StateMatrix      P_;
    StateMatrix      Q_;
    MeasurementCovariance R_;
    MeasurementMatrix     H_;
};

} // namespace filtering
} // namespace perception
} // namespace avt_341_nav
