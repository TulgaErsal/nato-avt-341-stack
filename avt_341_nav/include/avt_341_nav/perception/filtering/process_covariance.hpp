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

* @file      process_covariance.hpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Header file for templated process covariance matrices for common noise profiles
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

#include <Eigen/Dense>

namespace avt_341_nav {
namespace perception {
namespace filtering {

template <int state_size, int order>
class ProcessCovariance {
  public:
    typedef Eigen::Matrix<double, state_size*(order + 1), state_size*(order + 1)> ProcessCovarianceMatrix;

    static ProcessCovarianceMatrix GetDiscreteWhiteNoise(double time_step, double process_variance) {
        ProcessCovarianceMatrix Q;
        Q = ProcessCovarianceMatrix::Zero();

        Eigen::Matrix<double, (order + 1), (order + 1)> block_q;

        block_q = Eigen::Matrix<double, (order + 1), (order + 1)>::Zero();
        block_q(0, 0) = 0.25 * std::pow(time_step, 4);
        block_q(0, 1) = 0.5 * std::pow(time_step, 3);
        block_q(0, 2) = 0.5 * std::pow(time_step, 2);
        block_q(1, 0) = 0.5 * std::pow(time_step, 3);
        block_q(1, 1) = std::pow(time_step, 2);
        block_q(1, 2) = time_step;
        block_q(2, 0) = 0.5 * std::pow(time_step, 2);
        block_q(2, 1) = time_step;
        block_q(2, 2) = 1;

        for(int block_index = 0; block_index < state_size; ++block_index) {
            Q.template block<(order + 1), (order + 1)>(block_index * (order + 1), block_index * (order + 1)) = block_q;
        }

        Q *= std::pow(process_variance, 2.0);
        return Q;
    }
};

} // namespace filtering
} // namespace perception
} // namespace avt_341_nav