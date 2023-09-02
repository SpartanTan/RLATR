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

#ifndef ATR_CONTROLLER_HPP_
#define ATR_CONTROLLER_HPP_

/*! \file ATRController.hpp
 *  \brief This file provides an implementation for the kinematic control of the atr.
 *
 *  Provides the following functionalities:
 *      - Calculates the desired ATR's pose based on a Twist command
 *      - Calculates the wheel commanded velocities to move the ATR
 */

#include <cmath>
#include <vector>
#include <mutex>

#include "atr_controller/ATRControllerConfig.hpp"

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace atr
{
namespace atr_controller
{
/**
 *  @class This class implements a <a href="https://en.wikipedia.org/wiki/Full_state_feedback">
 *        Full State Feedback controller (FSF)</a>
 *
 *  The FSF uses the full internal state of the system to control the system in a closed loop.
 */

class ATR_CONTROLLER_PUBLIC ATRController
{
  // Private member variables
private:
  ATRControllerConfig cfg_;                ///< Controller configuration parameters
  std::vector<double> state_;              ///< Holds the atr full state variables
  std::vector<double> desired_state_;      ///< This is the state we will update with the teleop velocities (Twist)
  std::vector<double> teleop_ref_;         ///< Twist received from subscriber
  std::vector<double> reference_;          ///< Reference state (pose and velocities) for the controller
                                           ///< (x,y,\theta_z,v_x,v_y,\omega_z)
  std::vector<double> wheel_vel_command_;  ///< wheel velocities command to control the ATR
  bool got_state_flag;                     ///< to monitor when the atr state is updated in the controller
  std::vector<std::array<double, 3>> control_data_;  ///< control data storage
  Eigen::VectorXd xd_w_;                             ///< ATR 2D pose in the world cf
  Eigen::VectorXd xd_old_w_;                         ///< ATR desired 2D pose in t-1
  Eigen::VectorXd x_old_w_;                          ///< ATR 2D pose in t-1
  bool zero_flag_;                                   ///< flag to control the linear velocities or the angular velocity
  std::mutex state_mutex_;                           ///< mutex to protect W/R ATR state
  std::mutex teleop_mutex_;                          ///< mutex to protect W/R Twist message
  double current_time_;                              ///< running time in seconds
  double old_time_;                                  ///< time in the previous iteration (used to compute Dt)
  double ti_;                                        ///< initial time in seconds
  bool init_ctrl_flag;                               ///< flag to control when we set the initial conditions

  bool world_cf_flag_;

  // Public member methods
public:
  /**
   * @brief Controller constructor
   *
   * @param config ATR config object with control gains
   */
  explicit ATRController(const ATRControllerConfig& config);

  /**
   * @brief reset function to set the state to the initial conditions
   *
   */
  void reset();

  /**
   * @brief Update controller output command
   *
   * @param current_time current time in seconds
   */
  void update(double current_time);

  /**
   * @brief Updates the teleop data when a teleoperation message arrives.
   *
   * @param ref desired atr Twist in the ATR coordinate frame [v_x,v_y,\omega_z]_d_atr
   */
  void set_teleop(std::vector<double> ref);
  void set_teleop(std::vector<double> ref, bool world_cf);
  void set_teleop(Eigen::Vector3d& ref);

  /// \brief computes the desired state based on the ref teleop twist

  /**
   * @brief Computes the desired state based on the ref teleop twist
   *
   * @param dt Control sample time
   * @return std::vector<double> desired ATR state in the world c.f. [x,y,\theta_z,v_x,v_y,\omega_z]
   */
  std::vector<double> calculate_desired_state(double dt);

  /**
   * @brief Updates the sensor data when a status message arrives.
   *
   * @param state new atr state from ATR driver (Odometry)
   */
  void set_state(std::vector<double> state);

  /// \brief
  /// \param[in] msg Command force in Newton.

  /**
   * @brief Updates the command data from the controller before publishing.
   *
   * @param wheel_vel commanded wheel velocities
   */
  void set_wheel_vel_command(std::vector<double> wheel_vel);

  /// \brief Get atr teleoperation data
  /// \return Teleoperation data

  /**
   * @brief Get the target Twist velocity
   *
   * @return const std::vector<double>&  reference to the target twist velocity used in the controller
   */
  const std::vector<double>& get_teleop();

  /// \brief Get atr state
  /// \return State data

  /**
   * @brief Get the current ATR state
   *
   * @return const std::vector<double>& reference to the ATR state used inside the controler
   */
  const std::vector<double>& get_state();

  /// \brief Get wheel velocities command data
  /// \return Wheel velocities command in rad/s

  /**
   * @brief Get the wheel vel command
   *
   * @return const std::vector<double>& reference to the commanded wheel velocities
   */
  const std::vector<double>& get_wheel_vel_command() const;

  /**
   * @brief Get the control data
   *
   * @return const std::vector<std::array<double, 3>>& Control data used for debugging purposes. (0)[x_w], (1)[xp_w]
   *          (2)[xd_w], (3)[xpd_w], (4)[DX_w], (5)[DXp_w], (6)[Wheel_velocities]
   */
  const std::vector<std::array<double, 3>>& get_control_data() const;

  /// \brief Get the state flag
  /// \return True if the atr state has been updated

  /**
   * @brief Get the state flag
   *
   * @return true The controller has received a ATR state message (from ATR driver)
   * @return false The ATR state message has not been published yet
   */
  bool get_state_flag() const;

  /**
   * @brief Set the feedback matrix for the controller
   *
   * @param feedback_matrix vector with control gains [Kp_x,Kp_y,Kp_o,Kd_x,Kd_y,Kd_o]
   */
  void set_feedback_matrix(std::vector<double> feedback_matrix);

  /**
   * @brief Get the feedback matrix used in the controller
   *
   * @return const std::vector<double>& vector with control gains [Kp_x,Kp_y,Kp_o,Kd_x,Kd_y,Kd_o]
   */
  const std::vector<double>& get_feedback_matrix() const;

  //

  /**
   * @brief Set the initial time for the controller
   *
   * @param ti initial time in seconds, usually from a rclcpp::Time object
   */
  void set_initial_time(double ti);

  // Private member methods
private:
  // Calculate the control output based on the current state and the desired reference

  /**
   * @brief calculates the wheel velocity command based on the current ATR state and the target state
   *
   * @param current_time time in seconds since the controller started
   */
  ATR_CONTROLLER_LOCAL void calculate(double current_time);
};

}  // namespace atr_controller
}  // namespace atr

#endif  // ATR_CONTROLLER_HPP_
