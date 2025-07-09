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

* @file      cv_filter.hpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Header file for a Kalman filter with a constant velocity Brownian motion model.
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

#include <avt_341/perception/filtering/kinematic_kalman_filter.hpp>
#include <avt_341/perception/filtering/process_covariance.hpp>

namespace avt_341 {
namespace perception {
namespace filtering {

template <int state_size>
/**
 * @brief Templated class for a constant velocity Kalman filter.
 *
 * @details This is a Kalman filter with first-order (i.e. position and velocity
 * estimation) Netwonian particle kinematic model. The class provides
 * convenience function for providing measurements for position alone or for
 * position and velocity.
 */
class CVFilter : public KinematicKalmanFilter<state_size, 2, state_size> {
  public:
    typedef KinematicKalmanFilter<state_size, 2, state_size> KinematicFilter;
    typedef Eigen::Vector<double, state_size> StateVector;
    typedef Eigen::Vector<double, 3 * state_size> MeasurementVector;
    typedef Eigen::Matrix<double, 3 * state_size, 3 * state_size> MeasurementUncertaintyMatrix;

    CVFilter(const double& time_step, const double& process_variance, const double& measurement_variance)
        : KinematicFilter(time_step),
          process_variance_(process_variance) {
        InitializeMeasurementUncertainty(measurement_variance);
        InitializeProcessUncertainty(time_step);
    }

    void InitializeMeasurementUncertainty(const double& measurement_variance) {
        // Set measurement uncertainty.
        auto measurement_uncertainty = MeasurementUncertaintyMatrix::Identity() * std::pow(measurement_variance, 2.0);
        KinematicFilter::SetMeasurementUncertainty(measurement_uncertainty);
    }

    void InitializeProcessUncertainty(const double& time_step) {
        auto process_uncertainty =
            avt_341::perception::filtering::ProcessCovariance<state_size, 2>::GetDiscreteWhiteNoise(
                time_step,
                std::pow(process_variance_, 2.0));
        KinematicFilter::SetProcessUncertainty(process_uncertainty);
    }

    void SetInitialState(const StateVector& initial_state) { KinematicFilter::x_ = initial_state; }

    void SetInitialPosition(const StateVector& initial_position) {
        for(int i = 0; i < 2; ++i) { KinematicFilter::x_(i * state_size) = initial_position(i); }
    }

    void SetInitialVelocity(const StateVector& initial_velocity) {
        for(int i = 0; i < 2; ++i) { KinematicFilter::x_(i * state_size + 1) = initial_velocity(i); }
    }

    void Update(const MeasurementVector& measurement) { KinematicFilter::Update(measurement); }

  private:
    double process_variance_;
};

} // namespace filtering
} // namespace perception
} // namespace avt_341
