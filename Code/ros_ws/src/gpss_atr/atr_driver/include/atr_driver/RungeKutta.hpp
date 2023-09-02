/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 13.06.2022
 *
 *
 * #### License
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  Adapted from https://github.com/ros2-realtime-demo/pendulum.git
 */

#ifndef ATR_DRIVER__RUNGE_KUTTA_HPP_
#define ATR_DRIVER__RUNGE_KUTTA_HPP_

/*! \file ATRController.hpp
 *  \brief This file provides an implementation for a Runge-Kutta method to solve ordinary differential equations (ODE)
 *
 *  Provides the following functionalities:
 *      - Calculates the time integral of the state using Runge-Kutta 4th order
 */

#include <vector>
#include <stdexcept>
#include <functional>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace atr
{
namespace atr_driver
{
// Dean_Note: Function implementation: https://en.cppreference.com/w/cpp/utility/functional/function
// using derivativeF = std::function<double(const std::vector<double>&, double, size_t)>;
using derivativeF = std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>;

/// \class This class implements a classic 4th order
/// <a href="https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods"> Runge Kutta method</a>
/// compilation
/// This method is based on the trapezoidal rule and it just allocates memory at initialization.
class RungeKutta
{
public:
  /**
   * @brief Construct a new Runge Kutta object
   *
   * @param dimension state dimension
   */
  explicit RungeKutta(size_t dimension);

  /// \brief Time step using 4th-order Runge Kutta and trapezoidal rule
  /// \param[in] df Derivative function pointing to the ODE equations to solve
  /// \param[in,out] y Status vector with the previous status at input and next state at output.
  /// \param[out] yp Status vector with the time derivative of the state at output.
  /// \param[in] h Time step.
  /// \param[in] u Single input in the equations.
  /// \throw std::invalid_argument If the state vector doesn't have wrong dimensions.
  void step(const derivativeF& df, Eigen::VectorXd& y, Eigen::VectorXd& yp, double h, Eigen::VectorXd& u);

private:
  const size_t N;                  ///< state dimension (not used)
  Eigen::VectorXd state;           ///< state space vector: state[0]: x , state[1]: y, state[2]: \theta_z
  Eigen::VectorXd k1, k2, k3, k4;  ///< Runge-kutta increments

};  // namespace atr_driver
}  // namespace atr_driver
}  // namespace atr

#endif  // ATR_DRIVER__RUNGE_KUTTA_HPP_
