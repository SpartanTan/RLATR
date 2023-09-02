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

#include "atr_driver/ATRDriver.hpp"
#include <vector>
#include "rcppmath/clamp.hpp"

#include <iostream>

namespace atr
{
namespace atr_driver
{
ATRDriver::ATRDriver(const ATRDriverConfig& config)
  : cfg_(config)
  , ode_solver_(STATE_DIM)
  , controller_vel_(Eigen::VectorXd::Zero(2, 1))
  , disturbance_speed_(Eigen::VectorXd::Zero(2, 1))
  , rand_gen_(rd())
  , noise_gen_(std::uniform_real_distribution<double>(-config.get_noise_level(), config.get_noise_level()))
{
  // Calculate the controller timestep (for discrete differentiation/integration).
  dt_ = cfg_.get_physics_update_period().count() / (1e6);
  if (std::isnan(dt_) || dt_ == 0)
  {
    throw std::runtime_error("Invalid dt_ calculated in ATRController constructor");
  }

  // x: atr state [pos_x,pos_y,angle_z]
  // u: control input [wheel_vel_r, wheel_vel_l]
  derivative_function_ = [this](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
    const double wheel_l = cfg_.get_wheel_length();
    const double wheel_r = cfg_.get_wheel_radius();
    // const double d = cfg_.get_damping_coefficient();
    // const double g = cfg_.get_gravity();

    // Motion coefficients
    double R2 = wheel_r / 2.0;
    double RL = wheel_r / wheel_l;

    // Sin funct
    double st = sin(x(2));
    double ct = cos(x(2));

    // ATR Jacobian

    Eigen::MatrixXd J(3, 2);

    J.setZero();

    J << R2 * ct, R2 * ct, R2 * st, R2 * st, RL, -RL;

    Eigen::Vector3d Xp = J * u;

    return Xp;
  };
}

void ATRDriver::set_state(Eigen::Vector3d& atr_pos, Eigen::Vector3d& atr_vel, Eigen::Vector2d& wheel_pos,
                          Eigen::Vector2d& wheel_vel)
{
  state_.pose = atr_pos;
  state_.vel = atr_vel;
  state_.wheel_pos = wheel_pos;
  state_.wheel_vel = wheel_vel;
}

void ATRDriver::set_state(ATRState new_state)
{
  state_.pose = new_state.pose;
  state_.vel = new_state.vel;
  state_.wheel_pos = new_state.wheel_pos;
  state_.wheel_vel = new_state.wheel_vel;
}

void ATRDriver::set_initial_state(Eigen::Vector3d& atr_pos)
{
  initial_state_.pose = atr_pos;
  initial_state_.vel.setZero();
  initial_state_.wheel_pos.setZero();
  initial_state_.wheel_vel.setZero();
}

void ATRDriver::set_controller_wheel_vel(const Eigen::Vector2d& cmd_wheel_vel)
{
  // Saturation function
  controller_vel_ << rcppmath::clamp(cmd_wheel_vel(0), -cfg_.get_max_cart_force(), cfg_.get_max_cart_force()),
      rcppmath::clamp(cmd_wheel_vel(1), -cfg_.get_max_cart_force(), cfg_.get_max_cart_force());
}

void ATRDriver::set_disturbance(const Eigen::Vector2d& disturbance)
{
  disturbance_speed_ = disturbance;
}

const ATRDriver::ATRState& ATRDriver::get_state() const
{
  return state_;
}

const ATRDriver::ATRState& ATRDriver::get_initial_state() const
{
  return initial_state_;
}

Eigen::Vector2d ATRDriver::get_controller_wheel_velocity() const
{
  // return Eigen::Vector2d(controller_vel_);
  return controller_vel_;
}

Eigen::Vector2d ATRDriver::get_disturbance() const
{
  return disturbance_speed_;
}

void ATRDriver::update()
{
  Eigen::VectorXd cmd_cart_speed(2, 1);

  // std::cout << "controller_vel_: " << controller_vel_ << std::endl;

  cmd_cart_speed = disturbance_speed_ + controller_vel_;

  // std::cout << "cmd_cart_speed: " << cmd_cart_speed << std::endl;

  Eigen::VectorXd X = state_.pose;
  Eigen::VectorXd Xp = state_.vel;

  // This is the simualted robot
  ode_solver_.step(derivative_function_, X, Xp, dt_, cmd_cart_speed);

  // TODO: add the real robot here
  // Send the wheel commands to the hardware 12ms
  // get the encoder positions                6ms Get the hardware info in a different thread???
  // get the odometry
  // send the odometry + encoder info to the controller
  // around 18ms

  state_.pose = X;
  state_.vel = Xp;

  state_.wheel_pos = state_.wheel_pos + dt_ * cmd_cart_speed;

  state_.wheel_vel = cmd_cart_speed;
}

void ATRDriver::reset()
{
  Eigen::Vector2d z2 = Eigen::Vector2d::Zero();

  // initial_position
  set_state(initial_state_);
  set_disturbance(z2);
  set_controller_wheel_vel(z2);
}

}  // namespace atr_driver
}  // namespace atr
