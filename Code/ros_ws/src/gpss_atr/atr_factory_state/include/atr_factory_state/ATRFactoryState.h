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

#ifndef ATR_FACTORY_STATE_H
#define ATR_FACTORY_STATE_H

/*! \file ATRFactoryState.h
 *  \brief Subscribes to all the ATRStateStamped topics (generated by the ATRs) and generates Optom pose data.
 *
 *  Provides the following functionalities:
 *      - ATRStateList topic publisher
 *      - Dynamic generation of ATRStateStamped subscribers
 *      - Generates dummy Optom pose for each ATRState message
 */

// Standard
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2
#include "atr_utils/AuxTools.h"
#include "atr_utils/NodeDescriptions.h"
#include "atr_state_msgs/msg/atr_state_list_stamped.hpp"
#include "atr_state_msgs/msg/atr_state_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

namespace atr_factory_state
{
class ATRFactoryState : public rclcpp::Node, public atr::utils::AuxTools, public atr::utils::NodeDescriptions
{
  // Public member variables
public:
  using ATRStateSubscriber = rclcpp::Subscription<atr_state_msgs::msg::ATRStateStamped>::SharedPtr;

private:
  rclcpp::TimerBase::SharedPtr timer_;           ///< defines the frequency of the publisher
  rclcpp::TimerBase::SharedPtr watchdog_timer_;  ///< defines the frequency of the watchdog
  rclcpp::Publisher<atr_state_msgs::msg::ATRStateListStamped>::SharedPtr publisher_;  ///< ros publisher
  tf2_ros::TransformBroadcaster tf_broadcaster_;  ///< to publish tfs and visualize them in rviz

  std::vector<ATRStateSubscriber> v_subscribers_;  ///< Array of subscribers

  std::vector<atr_state_msgs::msg::ATRState> v_atr_states_;  ///< Array of atrs

  atr_state_msgs::msg::ATRStateListStamped atr_states_msg_;  //< message container to publish the list of ATR states

  std::string list_topic_name_;       ///< Topic name to publish the list of ATRs
  std::string list_frame_id_;         ///< name of the reference coordinate frame
  size_t atr_number_;                 ///< Number of ATRs in operation
  std::vector<int64_t> atr_id_list_;  ///< Array with the ATR ids in operation

  double atr_watchdog_time;  ///< Time to reset full_state flag

  int atr_period_ms_;  ///< sample rate for the publisher (ATRStateList and Tfs)

  /**
   * @brief Map to connect Formation ID to index in the local memory
   * (map[id]=index)
   */
  std::unordered_map<int, int> map_id_index_;  ///< Map from ATR ID to Index
  std::unordered_map<int, int> map_index_id_;  ///< Map from Index to ATR ID

  std::mutex atr_states_mutex_;                 ///< Mutex to protect the shared atr data
  std::vector<std::mutex> v_atr_states_mutex_;  ///< Mutex to protect the shared atr data

  std::vector<rclcpp::Time> v_tc_;  ///< vector of Time variables to control the watchdog function

  // Public member methods
public:
  ATRFactoryState();
  /**
   * @brief Object initialization
   *          Reads parameters from yaml file
   */
  void init();

  // Private member methods
private:
  /**
   * @brief Publisher callback function to generate Optom data and append it to the Odom data
   *
   */
  void timer_callback();

  /**
   * @brief watchdog function to reset the full_pose flag after Odom data has not been received for some time
   *
   */
  void watchdog_callback();

  /**
   * @brief ATRState topic callback function to receive the Odom data from each ATR
   *
   * @param msg state information of an ATR
   */
  void topic_callback(const atr_state_msgs::msg::ATRStateStamped::SharedPtr msg);
};
}  // namespace atr_factory_state

#endif
