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
/// \brief This file provides an implementation for the kinematic simulation of the atr.

#ifndef ATR_DRIVER_HPP_
#define ATR_DRIVER_HPP_

/*! \file ATRDriver.hpp
 *  \brief This file provides an implementation for the kinematic simulation of the atr.
 *
 *  Provides the following functionalities:
 *      - Calculates the new ATR state based on the commanded wheel velocities (from ATRController)
 */

#include <cmath>
#include <chrono>
#include <vector>
#include <random>

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "atr_state_msgs/msg/atr_joint_command.hpp"
// NOTE: The atr state to represent the pose (state) of the atr for the driver and the controller will be Twist
// As communication with the other modules, we will use ATR_SATE

#include "atr_driver/RungeKutta.hpp"
#include "atr_driver/ATRDriverConfig.hpp"

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace atr
{
namespace atr_driver
{
/**
 *  @class This class implements a kinematic simulation for the ATR.
 *
 */

class ATR_DRIVER_PUBLIC ATRDriver
{
public:
  static constexpr std::size_t STATE_DIM = 3U;  ///< dimension of the space state array

  struct ATRState  ///< Struct representing the dynamic/kinematic state of the atr.
  {
    Eigen::Vector3d pose = Eigen::Vector3d::Zero();       ///< ATR 2D pose (x,y,theta_z)
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();        ///< ATR 2D velocity (v_x,v_y,omega_z)
    Eigen::Vector2d wheel_pos = Eigen::Vector2d::Zero();  ///< Wheel positions
    Eigen::Vector2d wheel_vel = Eigen::Vector2d::Zero();  ///< Wheel velocities
  };

private:
  const ATRDriverConfig cfg_;  ///< configuration parameters class. ATR simulation configuration parameters
  double dt_;                  ///< controller time step in seconds
  ATRState state_;             ///< atr state struct
  ATRState initial_state_;     ///< atr state struct
  RungeKutta ode_solver_;      ///< integrator

  Eigen::VectorXd controller_vel_;  ///< wheel's velocities applied by the controller. This can be considered as the
                                    ///< angular velocities applied by the wheel's motors

  Eigen::VectorXd disturbance_speed_;  ///< external disturbance. this can be considered as something pushing the cart

  std::random_device rd;                              ///< utils to generate random noise
  std::mt19937 rand_gen_;                             ///< utils to generate random noise
  std::uniform_real_distribution<double> noise_gen_;  ///< utils to generate random noise

  derivativeF derivative_function_;  ///< pointer to the derivative motion functions (ODE)

public:
  explicit ATRDriver(const ATRDriverConfig& config);

  /// \brief Set the atr state data
  /// \param[in] atr_pos atr position in m
  /// \param[in] atr_vel atr velocity in m/s
  /// \param[in] wheel_pos wheel position in radians
  /// \param[in] wheel_vel wheel velocity in radians/s
  void set_state(Eigen::Vector3d& atr_pos, Eigen::Vector3d& atr_vel, Eigen::Vector2d& wheel_pos,
                 Eigen::Vector2d& wheel_vel);

  /// \brief Set the atr state data
  /// \param[in] new_state atr full state
  void set_state(ATRState new_state);

  /// \brief Set the atr initial state data
  /// \param[in] atr_pos atr position in m
  void set_initial_state(Eigen::Vector3d& atr_pos);

  /// \brief Sets the applied wheel velocities by the controller motor
  /// \param[in] cmd_wheel_vel wheel velocities to set in rad/s.
  void set_controller_wheel_vel(const Eigen::Vector2d& cmd_wheel_vel);

  /// \brief Sets the applied disturbance
  /// \param[in] disturbance to set in rad/s.
  void set_disturbance(const Eigen::Vector2d& disturbance);

  /// \brief Get atr state
  /// \return State data
  const ATRState& get_state() const;

  /// \brief Get atr initial state
  /// \return State data
  const ATRState& get_initial_state() const;

  /// \brief Gets the applied wheel velocities by the controller motor to the cart
  /// \return controller cart applied wheel velocities in rad/s
  Eigen::Vector2d get_controller_wheel_velocity() const;

  /// \brief Gets the applied disturbance
  /// \return atr disturbance wheel speed in rad/s.
  Eigen::Vector2d get_disturbance() const;

  /// \brief Updates the driver simulation.
  void update();

  /// \brief Reset the driver simulation.
  void reset();
};
}  // namespace atr_driver
}  // namespace atr
#endif  // ATR_DRIVER_HPP_
