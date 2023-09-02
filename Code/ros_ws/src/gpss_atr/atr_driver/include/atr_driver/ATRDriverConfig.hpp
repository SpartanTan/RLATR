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

//

/// \file
/// \brief

#ifndef ATR_DRIVER_CONFIG_HPP_
#define ATR_DRIVER_CONFIG_HPP_

/*! \file ATRController.hpp
 *  \brief This file provides the Configuration class to allocate the ATR parameters.
 *
 *  Provides the following functionalities:
 *      - Access functions for the ATR parameters
 */

#include "atr_driver/visibility_control.hpp"
#include <chrono>

namespace atr
{
namespace atr_driver
{
class ATR_DRIVER_PUBLIC ATRDriverConfig
{
private:
  double wheel_length = 0.48;                       ///< Distance between Wheels
  double wheel_radius = 0.125;                      ///< Wheels' radius
  double damping_coefficient = 20.0;                ///< Viscous friction
  double gravity = -9.8;                            ///< gravity, non const, we may want to change it
  double max_wheel_speed = 100;                     ///< maximum allowed wheel speed
  double noise_level = 1.0;                         ///< Noise level used for simulation
  std::chrono::microseconds physics_update_period;  ///< physics simulation update period

public:
  /// \brief Constructor
  /// \param[in] wheel_length distance between wheels
  /// \param[in] wheel_radius wheel radius
  /// \param[in] damping_coefficient damping coefficient
  /// \param[in] gravity gravity
  /// \param[in] max_cart_force maximum cart force
  /// \param[in] physics_update_period physics simulation update period
  ATRDriverConfig(const double wheel_length, const double wheel_radius, const double damping_coefficient,
                  const double gravity, const double max_wheel_speed, const double noise_level,
                  std::chrono::microseconds physics_update_period);

  /// \brief Gets the distance between atr wheels
  /// \return distance in m
  double get_wheel_length() const;

  /// \brief Gets the atr wheel radius
  /// \return wheel radius in m
  double get_wheel_radius() const;

  /// \brief Gets the atr damping coefficient
  /// \return Damping coefficient
  double get_damping_coefficient() const;

  /// \brief Gets the gravity
  /// \return Gravity in m/s^2
  double get_gravity() const;

  /// \brief Gets the maximum allowed cart force
  /// \return Maximum cart force
  double get_max_cart_force() const;

  /// \brief Gets the simulated noise level
  /// \return Noise level
  double get_noise_level() const;

  /// \brief Gets the physics simulation update period
  /// \return physics simulation update period
  std::chrono::microseconds get_physics_update_period() const;
};

}  // namespace atr_driver
}  // namespace atr
#endif  // ATR_DRIVER_CONFIG_HPP_