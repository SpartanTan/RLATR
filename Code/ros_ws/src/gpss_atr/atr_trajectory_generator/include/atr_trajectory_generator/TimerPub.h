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

#ifndef TIMER_PUB_H
#define TIMER_PUB_H

/*! \file TimerPub.h
 *  \brief Provides dummy trajectory for the ATRs.
 *
 *  Provides the following functionalities:
 *      - vector of ATR Paths to send target Path to each ATR
 *      - Subscribers
 *          ATRState List (to get the current ATRs pose)
 *      - Clients
 *          ATRFormation
 *      - Services
 *          Update ATRPath List (to get the ATR Paths from ATRTrajectoryGenerator)
 *      - Publishers
 *          ATRPathList, publishes all the target ATR Paths as a List (to visualize them in rviz)
 */
#include <stdio.h>
#include <memory>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "rclcpp/rclcpp.hpp"

#include "atr_path_msgs/msg/atr_path.hpp"
#include "atr_state_msgs/msg/atr_state.hpp"

// #include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/StdVector>

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_trajectory_generator
{
using PublisherUpdateATRTwist = rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr;
using PublisherUpdateATRProgress = rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr;
using v_PoseWDTime = std::vector<atr_path_msgs::msg::PoseWithDTime, std::allocator<atr_path_msgs::msg::PoseWithDTime>>;

class TimerPub
{
private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string node_name_;
  int atr_id_;
  std::string pub_name_;
  rclcpp::TimerBase::SharedPtr timer_;
  PublisherUpdateATRTwist pub_twist_;
  PublisherUpdateATRProgress pub_progress_;

  atr_path_msgs::msg::ATRPath path_;
  v_PoseWDTime poses_;

  bool start_flag_;
  int period_ms_;
  double period_s_;
  std::mutex atr_state_mutex;
  atr_state_msgs::msg::ATRState atr_state_;
  std::string logger_name_;
  Eigen::MatrixXd P_;
  int n_poses_;
  std::string frame_id_;
  int counter_;

public:
  TimerPub(rclcpp::Node::SharedPtr node, int atr_id, std::string pub_name, int period_ms, std::string frame_id);
  ~TimerPub();

  void start();

  void timer_pub_callback();

  void set_path(atr_path_msgs::msg::ATRPath& path);

  void set_poses_w_time(v_PoseWDTime& poses);

  const atr_path_msgs::msg::ATRPath& get_path() const;

  void set_atr_state(atr_state_msgs::msg::ATRState& state);

  const atr_state_msgs::msg::ATRState& get_atr_state() const;
};

}  // namespace atr_trajectory_generator

#endif
