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

* @file      ca_filter.hpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Header file for a Kalman filter with a constant acceleration
*            Brownian motion model and position-only measurement.
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

#include <avt_341/perception/filtering/kalman_filter.hpp>
#include <avt_341/perception/filtering/kinematic_kalman_filter.hpp>
#include <avt_341/perception/filtering/process_covariance.hpp>

namespace avt_341 {
namespace perception {
namespace filtering {

template <int state_size>
/**
 * @brief Templated class for a constant acceleration Kalman filter with
 *        position-only measurement.
 *
 * @details This filter tracks position, velocity, and acceleration for each
 * spatial dimension independently (constant acceleration / "CA" model, order=2).
 * The observation model is position-only: H is a (state_size × state_size*3)
 * matrix that selects only the position entries from the full state vector.
 * This correctly avoids the phantom velocity/acceleration measurements that
 * arise when using the full (state_size*3 × state_size*3) H matrix with a
 * position-only measurement vector.
 *
 * State vector layout (stride = order+1 = 3 per spatial dimension):
 *   [x_pos, x_vel, x_acc, y_pos, y_vel, y_acc, z_pos, z_vel, z_acc]
 *    idx 0   idx 1   idx 2  idx 3  idx 4  idx 5  idx 6  idx 7  idx 8
 *
 * Measurement vector: [x_pos, y_pos, z_pos]  (3-dimensional, position only)
 */
class CAFilter : public KalmanFilter<state_size * 3, state_size> {
  public:
    // Model order (constant acceleration).
    static constexpr int kOrder = 2;
    // Total state vector size: [pos, vel, acc] per dimension.
    static constexpr int kFullStateSize = state_size * (kOrder + 1);  // = state_size * 3

    typedef KalmanFilter<kFullStateSize, state_size> Base;

    // Position component of the state (input to SetInitialPosition / SetInitialVelocity).
    typedef Eigen::Vector<double, state_size> PositionVector;

    // Measurement vector is position-only (state_size dimensional).
    typedef Eigen::Vector<double, state_size>  MeasurementVector;
    typedef Eigen::Matrix<double, state_size, state_size> MeasurementUncertaintyMatrix;

    CAFilter(const double& time_step, const double& process_variance, const double& measurement_variance)
        : Base(), process_variance_(process_variance) {
        InitializeStateTransition(time_step);
        InitializeObservationMatrix();
        InitializeMeasurementUncertainty(measurement_variance);
        InitializeProcessUncertainty(time_step);
    }

    /**
     * @brief Build the block-diagonal state transition matrix F (kFullStateSize × kFullStateSize).
     *        Each diagonal block is the scalar constant-acceleration transition matrix for one
     *        spatial dimension.
     */
    void InitializeStateTransition(const double& time_step) {
        auto F_block = CreateStateTransitionMatrix<kOrder>(time_step);
        Base::F_ = Eigen::Matrix<double, kFullStateSize, kFullStateSize>::Zero();
        for (int i = 0; i < state_size; ++i) {
            Base::F_.template block<kOrder + 1, kOrder + 1>(
                i * (kOrder + 1), i * (kOrder + 1)) = F_block;
        }
    }

    /**
     * @brief Build the position-only observation matrix H (state_size × kFullStateSize).
     *        H(i, i*(kOrder+1)) = 1 selects the position entry for each spatial dimension i.
     *        All other entries are zero.
     */
    void InitializeObservationMatrix() {
        Base::H_.setZero();
        for (int i = 0; i < state_size; ++i) {
            Base::H_(i, i * (kOrder + 1)) = 1.0;
        }
    }

    /**
     * @brief Set the measurement uncertainty matrix R (state_size × state_size).
     *        R = measurement_variance² · I  (covariance, not std-dev).
     */
    void InitializeMeasurementUncertainty(const double& measurement_variance) {
        auto R = MeasurementUncertaintyMatrix::Identity() * std::pow(measurement_variance, 2.0);
        Base::SetMeasurementUncertainty(R);
    }

    /**
     * @brief Set the process uncertainty matrix Q (kFullStateSize × kFullStateSize) using the
     *        discrete white-noise covariance formula.
     *        Note: GetDiscreteWhiteNoise internally applies another pow(2), so the effective
     *        Q scaling is process_variance^4.  Tune accordingly.
     */
    void InitializeProcessUncertainty(const double& time_step) {
        auto Q = avt_341::perception::filtering::ProcessCovariance<state_size, kOrder>::GetDiscreteWhiteNoise(
            time_step, std::pow(process_variance_, 2.0));
        Base::SetProcessUncertainty(Q);
    }

    /**
     * @brief Set the initial position components of the state vector.
     *        Stride between dimensions in the full state is (kOrder+1) = 3.
     */
    void SetInitialPosition(const PositionVector& initial_position) {
        for (int i = 0; i < state_size; ++i) {
            Base::x_(i * (kOrder + 1)) = initial_position(i);
        }
    }

    /**
     * @brief Set the initial velocity components of the state vector.
     *        Velocity offset within each block is 1.
     */
    void SetInitialVelocity(const PositionVector& initial_velocity) {
        for (int i = 0; i < state_size; ++i) {
            Base::x_(i * (kOrder + 1) + 1) = initial_velocity(i);
        }
    }

    /**
     * @brief Run the measurement update step with a position-only measurement vector.
     *
     * @param measurement  3-dimensional position measurement [x, y, z].
     */
    void Update(const MeasurementVector& measurement) { Base::Update(measurement); }

    /**
     * @brief Run the measurement update step with a per-call measurement noise
     *        covariance.  The stored R is not modified.
     *
     * @param measurement  3-dimensional position measurement [x, y, z].
     * @param R            Measurement noise covariance (state_size × state_size)
     *                     to use for this update only.
     */
    void Update(const MeasurementVector& measurement,
                const MeasurementUncertaintyMatrix& R) {
        Base::Update(measurement, R);
    }

  private:
    double process_variance_;
};

} // namespace filtering
} // namespace perception
} // namespace avt_341
