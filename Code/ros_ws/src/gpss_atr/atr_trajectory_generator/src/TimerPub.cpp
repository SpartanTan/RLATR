/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 15.03.2021
 *
 * \copyright Copyright 2021 Chalmers
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
 *  This project has received financial  support  from  Chalmers  AI  Re-search Centre
 *  (CHAIR) and AB Volvo (Project ViMCoR).
 */

#include <stdio.h>

#include <atr_trajectory_generator/TimerPub.h>

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <atr_utils/convert_extended.h>

namespace atr_trajectory_generator
{
// clang-format off
TimerPub::TimerPub(rclcpp::Node::SharedPtr node, int atr_id, std::string pub_name, int
period_ms, std::string frame_id)
  : node_(node)
  , atr_id_(atr_id)
  , pub_name_(pub_name)
  , start_flag_(false)
  , period_ms_(period_ms)
  , period_s_(period_ms_*0.001)
  , logger_name_("TimerPub_" + std::to_string(atr_id_))
  , n_poses_(0)
  , frame_id_(frame_id)
  , counter_(0)
// clang-format on
{
  std::string s;
  pub_twist_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(pub_name_, 10);
  s = "atr_progress_" + std::to_string(atr_id);
  pub_progress_ = node_->create_publisher<std_msgs::msg::UInt8>(s, 10);

  // This timer triggers the publisher of the Path to Twist cmd message
  timer_ =
      node_->create_wall_timer(std::chrono::milliseconds(period_ms_), std::bind(&TimerPub::timer_pub_callback, this));
}

TimerPub::~TimerPub()
{
}

void TimerPub::timer_pub_callback()
{
  if (start_flag_)
  {
    // static int count = 0;

    geometry_msgs::msg::TwistStamped msg;

    // Once we send the last pose, we should set the velocities to zero

    Eigen::Vector2d Delta;

    if (counter_ > n_poses_ - 1)
    {
      Delta.setZero();

      RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "Stopped!");
      start_flag_ = false;
    }
    else if (counter_ == n_poses_ - 1)
    {
      Delta = (P_.block(0, n_poses_ - 1, 2, 1) - P_.block(0, n_poses_ - 2, 2, 1)) / period_s_;
      counter_++;
    }
    else
    {
      // RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), "-------------------------------------");
      // RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), P_.block(0, counter_ + 1, 2, 1).transpose());
      // RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), P_.block(0, counter_, 2, 1).transpose());

      // RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), "p_ms: " << period_ms_ << ", p_s: " << period_s_);

      Delta = (P_.block(0, counter_ + 1, 2, 1) - P_.block(0, counter_, 2, 1)) / period_s_;

      // RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), "Delta(" << counter_ << "): " << Delta.transpose());

      counter_++;
    }

    msg.header.stamp = node_->now();
    msg.header.frame_id = frame_id_;  //"atr_" + std::to_string(atr_id_) + "_base_link";
    msg.twist.linear.x = Delta(0);
    msg.twist.linear.y = Delta(1);
    msg.twist.angular.z = 0.0;

    // We need to calculate the Twist command based on the desired atr pose (path)
    pub_twist_->publish(msg);
  }

  // Publish a progress metric.
  static int count = 0;
  if(++count % 10 == 0) {
      std_msgs::msg::UInt8 progress_msg;
      progress_msg.data = (unsigned char)(100.0 * ((double)counter_ / (double)n_poses_));
      pub_progress_->publish(progress_msg);
  }
}

void TimerPub::start()
{
  start_flag_ = true;
}

void TimerPub::set_path(atr_path_msgs::msg::ATRPath& path)
{
  path_ = path;
  start_flag_ = true;
  counter_ = 0;
  RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), "New path set!");
}

void TimerPub::set_poses_w_time(v_PoseWDTime& poses)
{
  poses_ = poses;

  // Eigen::MatrixXd P(3, poses_.size());

  P_.resize(3, poses_.size());

  n_poses_ = P_.cols();

  int j = 0;
  for (auto&& i : poses_)
  {
    P_.col(j) << i.pose.position.x, i.pose.position.y, i.delta_time;
    // std::cout << "Poses[" << atr_id_ << "](" << j << "): \n" << P.col(j).transpose() << std::endl;
    j++;
  }

  // RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "Poses[" << atr_id_ << "]: \n" << P_);
  // std::cout << "Poses[" << atr_id_ << "]: \n" << P << std::endl;

  start_flag_ = true;
  counter_ = 0;
  RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), "New path set!");
}

const atr_path_msgs::msg::ATRPath& TimerPub::get_path() const
{
  return path_;
}

void TimerPub::set_atr_state(atr_state_msgs::msg::ATRState& state)
{
  atr_state_ = state;
}

const atr_state_msgs::msg::ATRState& TimerPub::get_atr_state() const
{
  return atr_state_;
}

}  // namespace atr_trajectory_generator
