/**
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 13.06.2022
 *
 * \copyright Copyright 2022 Chalmers
 *
 * #### Licence
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
 *  Modified from https://github.com/ros2-realtime-demo/pendulum.git
 */

#include <atr_driver/ATRDriverConfig.hpp>

#include <array>

namespace atr
{
namespace atr_driver
{
ATRDriverConfig::ATRDriverConfig(const double wheel_length, const double wheel_radius, const double damping_coefficient,
                                 const double gravity, const double max_wheel_speed, const double noise_level,
                                 std::chrono::microseconds physics_update_period)
  : wheel_length{ wheel_length }
  , wheel_radius{ wheel_radius }
  , damping_coefficient{ damping_coefficient }
  , gravity{ gravity }
  , max_wheel_speed{ max_wheel_speed }
  , noise_level{ noise_level }
  , physics_update_period{ physics_update_period }
{
}

double ATRDriverConfig::get_wheel_length() const
{
  return wheel_length;
}

double ATRDriverConfig::get_wheel_radius() const
{
  return wheel_radius;
}

double ATRDriverConfig::get_damping_coefficient() const
{
  return damping_coefficient;
}

double ATRDriverConfig::get_gravity() const
{
  return gravity;
}

double ATRDriverConfig::get_max_cart_force() const
{
  return max_wheel_speed;
}

double ATRDriverConfig::get_noise_level() const
{
  return noise_level;
}

std::chrono::microseconds ATRDriverConfig::get_physics_update_period() const
{
  return physics_update_period;
}

}  // namespace atr_driver
}  // namespace atr
