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

#include "atr_controller/ATRController.hpp"
#include "atr_utils/atr_utils.hpp"

#include <utility>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <cmath>

namespace atr
{
namespace atr_controller
{
// TODO: define the initial conditions (state) with the received atr_state constructor and reset()
ATRController::ATRController(const ATRControllerConfig& config)
  : cfg_(config)
  , state_{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
  , reference_{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
  , wheel_vel_command_{ 0.0, 0.0 }
  , got_state_flag(false)
  , xd_w_(Eigen::VectorXd::Zero(3, 1))
  , xd_old_w_(Eigen::VectorXd::Zero(3, 1))
  , x_old_w_(Eigen::VectorXd::Zero(3, 1))
  , zero_flag_(true)
  , current_time_(0.0)
  , old_time_(0.0)
  , init_ctrl_flag(true)
  , world_cf_flag_(false)
{
  // control data has 7 vectors, pose (3X1), vel (3X1), desired_pose (3x1), desired_vel (3X1)pose_error (3x1),
  // vel_error(3x1), and cmd_vel (6x1)
  control_data_.resize(7);
}

void ATRController::reset()
{
  // We reset the controller status to an up atr position by default
  // TODO: get the real state from publisher (e.g. photogrammetry)

  std::vector<double> zero_state = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // [x,y,theta_z,v_x,v_y,omega_z]

  // TODO: get the initial state from the camera in the ATRDriver class
  set_state(zero_state);

  // set zero reference velocities for the controller
  set_teleop(zero_state);

  got_state_flag = false;
}

void ATRController::update(double current_time)
{
  // Only when we get the state at least once, we can start calculating the desired state and the wheel cmd
  if (got_state_flag)
    calculate(current_time);
  else
    wheel_vel_command_ = { 0.0, 0.0 };
}

void ATRController::set_teleop(std::vector<double> ref)
{
  std::lock_guard<std::mutex> guard(teleop_mutex_);
  teleop_ref_ = ref;
}

void ATRController::set_teleop(std::vector<double> ref, bool world_cf)
{
  world_cf_flag_ = world_cf;
  std::lock_guard<std::mutex> guard(teleop_mutex_);
  teleop_ref_ = ref;
}

void ATRController::set_teleop(Eigen::Vector3d& ref)
{
  std::vector<double> aux;

  for (long int i = 0; i < ref.size(); i++)
  {
    aux.push_back(ref(i));
  }

  std::lock_guard<std::mutex> guard(teleop_mutex_);
  teleop_ref_ = aux;
}

std::vector<double> ATRController::calculate_desired_state(double dt)
{
  std::vector<double> teleop_ref = get_teleop();

  zero_flag_ = true;

  // If there's no teleop linear velocity, keep the old desired position and change only the orientation based on Twist
  // angular velocity.
  // We prioritze linear velocity over angular velocity. To change only the orientation, we need to set the lienar
  // velocity to zero and define a desired angular velocity
  if ((teleop_ref.at(0) != 0.0) || (teleop_ref.at(1) != 0.0))
  {
    zero_flag_ = false;
  }

  // Reference ATR position wrt to world cf
  Eigen::Vector3d ref_pos_w;

  // Reference linear velocity wrt world cf.
  Eigen::Vector3d ref_vel_w;

  if (zero_flag_)
  {
    // Keep position
    ref_pos_w(0) = desired_state_.at(0);
    ref_pos_w(1) = desired_state_.at(1);
    ref_vel_w(0) = 0.0;
    ref_vel_w(1) = 0.0;

    // Change orientation
    ref_vel_w(2) = teleop_ref.at(2);
    ref_pos_w(2) = desired_state_.at(2) + ref_vel_w(2) * dt;
  }
  else
  {
    Eigen::Affine3d Tm_w;

    // std::cout << "world_cf_flag_: " << world_cf_flag_ << std::endl;

    if (world_cf_flag_)
    {
      Tm_w.setIdentity();
    }
    else
    {
      Tm_w.linear() = atr::utils::Rz(desired_state_.at(2));
    }

    Tm_w.translation() << desired_state_.at(0), desired_state_.at(1), 0;

    // std::cout << "Tm_w: \n" << Tm_w.matrix() << std::endl;

    // Reference ATR position wrt to the ATR cf
    Eigen::Vector3d ref_pos_m(dt * teleop_ref.at(0), dt * teleop_ref.at(1), 0);

    // std::cout << "ref_pos_m: " << ref_pos_m.transpose() << std::endl;

    ref_pos_w = Tm_w * ref_pos_m.homogeneous();

    // std::cout << "---------------------------------------------" << std::endl;
    // std::cout << "ref_pos_w: " << ref_pos_w.transpose() << std::endl;
    // clang-format off
    // std::cout << "desired_state_: " << desired_state_.at(0) << ", " 
                                    // << desired_state_.at(1) << ", "
                                    // << desired_state_.at(2) << std::endl;
    // clang-format on

    // Reference linear velocity wrt ATR cf. Taken from the first two elements of the Twist vector
    Eigen::Vector3d ref_vel_m(teleop_ref.at(0), teleop_ref.at(1), 0);

    ref_vel_w = Tm_w.linear() * ref_vel_m;

    // std::cout << "ref_vel_w_T: " << ref_vel_w(2) << std::endl;

    // Calculate the desired orientation based on the desired position
    double Dx_w = ref_pos_w(0) - desired_state_.at(0);
    double Dy_w = ref_pos_w(1) - desired_state_.at(1);

    // std::cout << "D_w: " << Dy_w << ", " << Dx_w << std::endl;

    ref_pos_w(2) = atan2(Dy_w, Dx_w);

    // std::cout << "ref_pos_w(2)_atan: " << ref_pos_w(2) << std::endl;

    // Heuristic to find the minimum angle and direction to define the Delta orientation

    // How many 360 DEG turns have the robot taken (absolute orientation)
    double theta_factor = std::floor(std::abs(desired_state_.at(2)) / (2 * M_PI));

    // std::cout << "theta_factor: " << theta_factor << std::endl;

    // Calculate the relative orientation (0-360)
    double ss2 = (desired_state_.at(2) >= 0) ? 1 : -1;
    double real_state = desired_state_.at(2) - ss2 * theta_factor * 2 * M_PI;

    // Find the minimum angle and direction
    double thetadp_w = (2 * M_PI) + ref_pos_w(2);
    double sp_w = (2 * M_PI) + real_state;

    double Do = ref_pos_w(2) - real_state;
    double Dp = thetadp_w - real_state;
    double Di = ref_pos_w(2) - sp_w;

    Eigen::Vector3d Ds(Do, Dp, Di);

    // std::cout << "Ds: " << Ds.transpose() << std::endl;

    Eigen::Vector3d aDs = Ds.cwiseAbs();

    if (aDs(0) > aDs(1))
    {
      if (aDs(1) > aDs(2))
        ref_pos_w(2) = Ds(2);
      else
        ref_pos_w(2) = Ds(1);
    }
    else
    {
      if (aDs(0) > aDs(2))
        ref_pos_w(2) = Ds(2);
      else
        ref_pos_w(2) = Ds(0);
    }

    // std::cout << "ref_pos_w(2)_if: " << ref_pos_w(2) << std::endl;

    ref_vel_w(2) = ref_pos_w(2) / dt;
    // std::cout << "ref_vel_w(2): " << ref_vel_w(2) << std::endl;

    // At this level, we need to add the desired theta state since it will be substracted later, and ref_pos_w(2) is
    // already the error with correct sign and minimum magnitude
    ref_pos_w(2) += desired_state_.at(2);

    // std::cout << "desired_state_.at(2): " << desired_state_.at(2) << std::endl;
    // std::cout << "ref_pos_w(2)_+d: " << ref_pos_w(2) << std::endl;
  }

  // clang-format off
    desired_state_= { ref_pos_w(0),
                      ref_pos_w(1),
                      ref_pos_w(2),
                      ref_vel_w(0),
                      ref_vel_w(1),
                      ref_vel_w(2),
                    };

  // clang-format on

  return desired_state_;
}

void ATRController::set_state(std::vector<double> state)
{
  // std::lock_guard<std::mutex> guard(state_mutex_);
  state_mutex_.lock();
  state_ = state;
  state_mutex_.unlock();

  if (!got_state_flag)
  {
    got_state_flag = true;
    // Initialize the desired state with the current state. The desired_state_ represents the desired position/vel of
    // the ATR given a teleop twist cmd
    desired_state_ = state_;

    // Zero teleop linear and angular velocities
    set_teleop({ 0.0, 0.0, 0.0 });
  }
}

void ATRController::set_wheel_vel_command(std::vector<double> wheel_vel)
{
  wheel_vel_command_ = wheel_vel;
}

const std::vector<double>& ATRController::get_teleop()
{
  std::lock_guard<std::mutex> guard(teleop_mutex_);
  return teleop_ref_;
}

const std::vector<double>& ATRController::get_state()
{
  std::lock_guard<std::mutex> guard(state_mutex_);
  return state_;
}

const std::vector<double>& ATRController::get_wheel_vel_command() const
{
  return wheel_vel_command_;
}

const std::vector<std::array<double, 3>>& ATRController::get_control_data() const
{
  return control_data_;
}

bool ATRController::get_state_flag() const
{
  return got_state_flag;
}
void ATRController::set_feedback_matrix(std::vector<double> feedback_matrix)
{
  cfg_.set_feedback_matrix(feedback_matrix);
  // std::cout << __FILE__ << ":" << __LINE__ << std::endl;

  std::stringstream ss;
  ss << "[";
  for (auto&& i : cfg_.get_feedback_matrix())
  {
    ss << i << ", ";
  }
  ss << "]";
  // std::cout << __FILE__ << ":" << __LINE__ << ": " << ss.str() << std::endl;
}

const std::vector<double>& ATRController::get_feedback_matrix() const
{
  return cfg_.get_feedback_matrix();
}

void ATRController::calculate(double current_time)
{
  // Calculate the DT
  if (init_ctrl_flag)
  {
    old_time_ = current_time;
    // init_ctrl_flag = false;
  }
  double dt = current_time - old_time_;

  // Delta time cannot be zero! We assume 10ms period
  dt = (dt < 0.000001) ? 0.01 : dt;

  old_time_ = current_time;

  // std::cout << __FILE__ << ":" << __LINE__ << " Dt: " << dt << std::endl;

  // Local copy of shared variable (Mutex protected)
  std::vector<double> state = get_state();
  // TODO: change this to calculate_desired_sate(dt)
  std::vector<double> reference = calculate_desired_state(dt);

  wheel_vel_command_ = { 0.0, 0.0 };

  double st = sin(state[2]);
  double ct = cos(state[2]);

  // TODO: get this constants from parameter file
  double robotLenght = 0.48;
  double wheelRadius = 0.125;

  double R2 = wheelRadius / 2.0;
  double RL = wheelRadius / robotLenght;

  Eigen::VectorXd X_w(6, 1), Xd_w(6, 1);

  // Full state vectors

  X_w = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(state.data(), state.size());
  Xd_w = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(reference.data(), reference.size());

  // Position vectors
  Eigen::VectorXd x_w(3, 1), xd_w(3, 1);
  Eigen::VectorXd xp_w(3, 1), xpd_w(3, 1);
  x_w = X_w.block(0, 0, 3, 1);

  // Update desired position and keep desired Orientation
  // TODO: for the moment the desired orientation is initialized in 0, we need to use the initial state orientation
  xd_w = Xd_w.block(0, 0, 3, 1);

  if (init_ctrl_flag)
  {
    init_ctrl_flag = false;
    xd_old_w_ = xd_w;
    x_old_w_ = x_w;
  }

  // Velocity vectors
  xp_w = X_w.block(3, 0, 3, 1);
  xpd_w = Xd_w.block(3, 0, 3, 1);

  xp_w = (x_w - x_old_w_) / dt;
  xpd_w = (xd_w - xd_old_w_) / dt;

  // std::cout << __FILE__ << ":" << __LINE__ << " xpd_w: " << xpd_w.transpose() << std::endl;
  // std::cout << __FILE__ << ":" << __LINE__ << " xd_w: " << xd_w.transpose() << std::endl;
  // std::cout << __FILE__ << ":" << __LINE__ << " xd_old_w_: " << xd_old_w_.transpose() << std::endl;
  // std::cout << __FILE__ << ":" << __LINE__ << " dt: " << dt << std::endl;

  // double sxpd2 = (xpd_w(2) >= 0) ? 1 : -1;

  // double axpd2 = std::abs(xpd_w(2));

  // if omega_z > 30 DEG/s saturate it
  // xpd_w(2) = (axpd2 > 52.5) ? sxpd2 * 52.5 : xpd_w(2);

  // xpd_w = Xd_w.block(3, 0, 3, 1);

  // Set history
  x_old_w_ = x_w;
  xd_old_w_ = xd_w;

  // Position and velocity errors with the estimated desired orientation
  Eigen::Vector3d DX_w, DXp_w;
  DX_w = xd_w - x_w;
  DXp_w = xpd_w - xp_w;

  // dead zone to avoid orientation singularity
  double ep = 0.001 * 0.001;

  if ((DX_w(0) * DX_w(0) < ep) && (DX_w(1) * DX_w(1) < ep))
  {
    DX_w.block(0, 0, 2, 0).setZero();
    DXp_w.block(0, 0, 2, 0).setZero();
  }

  // Control Gains
  // clang-format off
  // We assume same gain for x and y
  Eigen::DiagonalMatrix<double, 3> Kp(cfg_.get_feedback_matrix()[0], 
                                      cfg_.get_feedback_matrix()[1],
                                      cfg_.get_feedback_matrix()[2]);
  Eigen::DiagonalMatrix<double, 3> Kd(cfg_.get_feedback_matrix()[3], 
                                      cfg_.get_feedback_matrix()[4],
                                      cfg_.get_feedback_matrix()[5]);
  // clang-format on

  // Inverse Jacobian Matrix, obtained from J=[R2*ct R2*ct;R2*st R2*st;RL -RL];

  Eigen::MatrixXd iJ(2, 3);

  iJ << ct / (2.0 * R2), st / (2.0 * R2), 1.0 / (2.0 * RL), ct / (2.0 * R2), st / (2.0 * R2), -1.0 / (2.0 * RL);

  // std::cout << "Kd: \n" << Kd.toDenseMatrix() << std::endl;

  // std::cout << "DXp_w: " << DXp_w.transpose() << std::endl;

  // Eigen::Vector3d KDv = Kd * DXp_w;

  // std::cout << "KDv: " << KDv.transpose() << std::endl;

  Eigen::Vector2d tau = iJ * ((Kp * DX_w) + (Kd * DXp_w));

  wheel_vel_command_.at(0) = tau(0);
  wheel_vel_command_.at(1) = tau(1);

  control_data_.at(0) = { x_w(0), x_w(1), x_w(2) };
  control_data_.at(1) = { xp_w(0), xp_w(1), xp_w(2) };
  control_data_.at(2) = { xd_w(0), xd_w(1), xd_w(2) };
  control_data_.at(3) = { xpd_w(0), xpd_w(1), xpd_w(2) };
  control_data_.at(4) = { DX_w(0), DX_w(1), DX_w(2) };
  control_data_.at(5) = { DXp_w(0), DXp_w(1), DXp_w(2) };
  control_data_.at(6) = { wheel_vel_command_.at(0), wheel_vel_command_.at(1), 0.0 };
}

void ATRController::set_initial_time(double ti)
{
  ti_ = ti;
}

}  // namespace atr_controller
}  // namespace atr
