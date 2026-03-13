/**
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                      _    _    _    _    _    _    _                      +
 +                     / \  / \  / \  / \  / \  / \  / \                     +
 +                    ( A )( V )( T )( - )( 3 )( 4 )( 1 )                    +
 +                     \_/  \_/  \_/  \_/  \_/  \_/  \_/                     +
 +       _    _    _    _    _    _    _    _     _    _    _    _    _      +
 +      / \  / \  / \  / \  / \  / \  / \  / \   / \  / \  / \  / \  / \     +
 +     ( A )( U )( T )( O )( N )( O )( M )( Y ) ( S )( T )( A )( C )( K )    +
 +      \_/  \_/  \_/  \_/  \_/  \_/  \_/  \_/   \_/  \_/  \_/  \_/  \_/     +
 +                                                                           +
 +  AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles  +
 +                                                                           +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

* @file      kinematic_kalman_filter.hpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Header file for a linear Kalman filter for state estimation.
* @copyright MIT License

             NATO AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles
             Copyright (c) 2024 Dario Sirangelo (dsi@aarhusrobotics.com).

             NOTE: The above copyright only applies to the contents of this file. The source code contained in this file
             is a direct port from the GitHub repository aarhus-robotics/navi, released by the copyright holder under
             the MIT license.

             Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
             associated documentation files (the "Software"), to deal in the Software without restriction, including
             without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
             copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
             following conditions:

             The above copyright notice and this permission notice shall be included in all copies or substantial
             portions of the Software.

             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
             LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
             EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
             IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
             THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <cassert>

#include <Eigen/Dense>

namespace avt_341 {
namespace perception {
namespace filtering {

template <int state_vector_size, int measurement_vector_size>
class KalmanFilter {
  public:
    typedef Eigen::Vector<double, state_vector_size> StateVector;
    typedef Eigen::Vector<double, measurement_vector_size> MeasurementVector;
    typedef Eigen::Matrix<double, state_vector_size, measurement_vector_size> SystemMatrix;
    typedef Eigen::Matrix<double, state_vector_size, state_vector_size> StateMatrix;
    typedef Eigen::Matrix<double, measurement_vector_size, state_vector_size> MeasurementMatrix;
    typedef Eigen::Matrix<double, measurement_vector_size, measurement_vector_size> PureMeasurementMatrix;

    /**
     * @brief Construct a new Kalman filter.
     */
    KalmanFilter() {
        // Initialise the system vectors.
        x_ = StateVector::Zero();
        y_ = MeasurementVector::Zero();
        z_ = MeasurementVector::Zero();

        // Initialise system matrices
        F_ = StateMatrix::Identity();
        H_ = MeasurementMatrix::Zero();
        I_ = StateMatrix::Identity();
        K_ = SystemMatrix::Zero();
        M_ = SystemMatrix::Zero();
        P_ = StateMatrix::Identity();
        Q_ = StateMatrix::Identity();
        R_ = PureMeasurementMatrix::Identity();
        S_ = PureMeasurementMatrix::Zero();
    }

    void Predict() {
        x_ = F_ * x_;
        P_ = (F_ * P_) * F_.transpose() + Q_;
    }

    /**
     * @brief Feed a new measurement to the Kalman filter.
     *
     * @param z Constant reference to the measurement vector.
     */
    void Update(const MeasurementVector& z) {
        // Update the stored measurement vector (for serialization).
        z_ = z;

        // Compute the residual between measurement and prediction.
        y_ = z - H_ * x_;

        // Project the system uncertainty onto the measurement space.
        auto PHT = P_ * H_.transpose();
        S_ = H_ * PHT + R_;

        // Map the system uncertainty onto the Kalman gain.
        K_ = PHT * S_.inverse();

        // Update the state through the residual scaled by the Kalman gain.
        x_ += K_ * y_;

        // Note: This approach is slower than the one traditionally found in the
        // literature, however, it is numerically stable.
        auto I_KH = I_ - K_ * H_;
        P_ = (I_KH * P_) * I_KH.transpose() + (K_ * R_) * K_.transpose();
    }

    /**
     * @brief Feed a new measurement with a per-call measurement noise
     *        covariance matrix.  The stored R_ is not modified; R is used
     *        only for this update step.
     *
     * @param z Measurement vector.
     * @param R Measurement noise covariance for this step (same size as R_).
     */
    void Update(const MeasurementVector& z, const PureMeasurementMatrix& R) {
        z_ = z;
        y_ = z - H_ * x_;

        auto PHT = P_ * H_.transpose();
        S_ = H_ * PHT + R;

        K_ = PHT * S_.inverse();
        x_ += K_ * y_;

        auto I_KH = I_ - K_ * H_;
        P_ = (I_KH * P_) * I_KH.transpose() + (K_ * R) * K_.transpose();
    }

    /**
     * @brief Set the measurement function matrix $H$.
     *
     * @param H Constant reference to the measurement function matrix.
     */
    void SetMeasurementFunction(const MeasurementMatrix& H) { H_ = H; }

    const MeasurementMatrix& GetMeasurementFunction() const { return H_; }

    /**
     * @brief Set the measurement uncertainty matrix $R$
     *
     * @param R Constant reference to the measurement uncertainty matrix $R$.
     */
    void SetMeasurementUncertainty(const PureMeasurementMatrix& R) { R_ = R; }

    /**
     * @brief Set the process uncertainty matrix $Q$.
     *
     * @param Q Constant reference to the process uncertainty matrix.
     */
    void SetProcessUncertainty(const StateMatrix& Q) { Q_ = Q; }

    const StateMatrix& GetProcessUncertainty() const { return Q_; }

    /**
    * @brief Set the state uncertainty matrix $P$.
    *
    * @param Q Constant reference to the process uncertainty matrix.
    */
    void SetStateUncertainty(const StateMatrix& P) { P_ = P; }

    const StateMatrix& GetStateUncertainty() const { return P_; }

    /**
     * @brief Set the state vector.
     *
     * @param x Constant reference to the state vector.
     */
    void SetState(const StateVector& x) { x_ = x; }

    /**
     * @brief Set the state transition matrix.
     *
     * @param F Constant reference to the state transition matrix.
     */
    void SetStateTransition(const StateMatrix& F) { F_ = F; }

    const StateMatrix& GetStateTransition() const { return F_; }

    /**
     * @brief Get the state vector
     *
     * @return const Eigen::VectorXd& Constant reference to the state vector.
     */
    const StateVector& GetState() const { return x_; }

    /**
     * @brief Get the size of the state vector.
     *
     * @return const int& Constant reference to the size of the state vector.
     */
    const int& GetStateSize() const { return state_vector_size; }

    const MeasurementVector& GetMeasurement() const { return z_; }

    const int& GetMeasurementSize() const { return measurement_vector_size; }

  protected:
    /** @brief State transition matrix. */
    StateMatrix F_;

    /** @brief Measurement function. */
    MeasurementMatrix H_;

    /** @brief Identity matrix. */
    StateMatrix I_;

    /** @brief Kalman gain. */
    SystemMatrix K_;

    /** @brief Process-measurement cross correlation. */
    SystemMatrix M_;

    /** @brief Uncertainty covariance matrix. */
    StateMatrix P_;

    /** @brief Process uncertainty matrix. */
    StateMatrix Q_;

    /** @brief Measurement uncertainty matrix. */
    PureMeasurementMatrix R_;

    /** @brief System uncertainty. */
    PureMeasurementMatrix S_;

    StateVector x_;

    /** @brief Filter error defined as the residual between the measurement and
     * the prediction. */
    MeasurementVector y_;

    MeasurementVector z_;
};

} // namespace filtering
} // namespace perception
} // namespace avt_341
