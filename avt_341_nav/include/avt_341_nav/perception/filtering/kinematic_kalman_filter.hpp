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
* @brief     Header file for templated linear Kalman filters for Brownian motion models.
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

#include <memory>

#include <Eigen/Dense>
#include <boost/math/special_functions/factorials.hpp>

#include <avt_341_nav/perception/filtering/kalman_filter.hpp>

using avt_341_nav::perception::filtering::KalmanFilter;

namespace avt_341_nav {
namespace perception {
namespace filtering {

template <int order>
Eigen::Matrix<double, (order + 1), (order + 1)> CreateStateTransitionMatrix(double time_step) {
    // Initialise the state transition matrix
    Eigen::Matrix<double, (order + 1), (order + 1)> F = Eigen::Matrix<double, (order + 1), (order + 1)>::Zero();

    for(int i = 0; i < (order + 1); ++i) { F(0, i) = std::pow(time_step, i) / boost::math::factorial<double>(i); }

    for(int i = 1; i < (order + 1); ++i) { F(i, Eigen::seq(i, Eigen::last)) = F(0, Eigen::seq(0, Eigen::last - i)); }

    return F;
}

template <int state_size, int order, int measurement_vector_size>
class KinematicKalmanFilter : public KalmanFilter<state_size*(order + 1), measurement_vector_size*(order + 1)> {
  public:
    KinematicKalmanFilter(double time_step)
        : KalmanFilter<state_size*(order + 1), measurement_vector_size*(order + 1)>() {
        const int size_x = state_size * (order + 1);

        // The state transition matrix by default is the identity matrix. Here
        // we set it to a null matrix instead.
        KalmanFilter<state_size*(order + 1), measurement_vector_size*(order + 1)>::F_ =
            Eigen::Matrix<double, size_x, size_x>::Zero();

        auto F = CreateStateTransitionMatrix<order>(time_step);

        int size = state_size;

        for(int i = 0; i < order + 1; ++i) {
            KalmanFilter<state_size*(order + 1), measurement_vector_size*(order + 1)>::F_
                .template block<(order + 1), (order + 1)>((order + 1) * i, (order + 1) * i) = F;
        }

        KalmanFilter<state_size*(order + 1), measurement_vector_size*(order + 1)>::H_.setZero();

        for(int i = 0; i < state_size; ++i) {
            KalmanFilter<state_size*(order + 1), measurement_vector_size*(order + 1)>::H_(i * (order + 1),
                                                                                          i * (order + 1)) = 1.0;
        }
    }
};

} // namespace filtering
} // namespace perception
} // namespace avt_341_nav
