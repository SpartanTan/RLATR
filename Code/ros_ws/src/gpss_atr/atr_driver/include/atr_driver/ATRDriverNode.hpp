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

/// \file
/// \brief

#ifndef ATR_DRIVER__ATR_DRIVER_NODE_HPP_
#define ATR_DRIVER__ATR_DRIVER_NODE_HPP_

/*! \file ATRController.hpp
 *  \brief This file provides a ROS 2 interface to implement the inverted atr driver.
 *
 *  Provides the following functionalities:
 *      - ATR state pubisher
 *      - JointState publisher (to visualize the ATR in rviz using a robotState publisher)
 *      - Commanded wheel velocities subscriber
 *      - lifecycle states
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "atr_driver/ATRDriver.hpp"
#include "atr_driver/visibility_control.hpp"
#include "atr_state_msgs/msg/atr_joint_command.hpp"
#include "atr_state_msgs/msg/atr_joint_state.hpp"
#include "atr_state_msgs/msg/atr_state_stamped.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "atr_utils/math_defs.h"

#include <std_srvs/srv/trigger.hpp>

namespace atr
{
namespace atr_driver
{
/**
 *  @class This class implements a node containing a simulated atr or the drivers for a real one.
 */

class ATRDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using SubATRJointCmdShPt = std::shared_ptr<rclcpp::Subscription<atr_state_msgs::msg::ATRJointCommand>>;
  using LifeCycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using PubJointState = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>>;
  using PubATRJointState = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<atr_state_msgs::msg::ATRJointState>>;
  using PubATRState = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<atr_state_msgs::msg::ATRStateStamped>>;
  // using SrvReset = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_srvs::srv::Trigger>>;
  using SrvReset = rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr;

private:
  const int atr_id_;                                      ///< atr's ID
  const std::string atr_frame_id_;                        ///< reference frame for the atr_state message
  const std::string joint_state_topic_name_;              ///< topic to publish the joint_state
  const std::string joint_state_prefix_;                  ///< prefix for the joint state publisher
  const std::string state_topic_name_;                    ///< topic to publish the state (for the control)
  const std::string atr_state_topic_name_;                ///< topic to publish the atr state (for the Factory)
  const std::string command_topic_name_;                  ///< topic to receive the commanded wheel velocities
  const std::string disturbance_topic_name_;              ///< topic to receive the disturbances (not used)
  const std::string cart_base_joint_name_;                ///< name of the ATR base tf
  std::chrono::microseconds state_publish_period_;        ///< period in seconds for the driver thread
  bool enable_topic_stats_;                               ///< flag to publish stats
  const std::string topic_stats_topic_name_;              ///< topic to publish the stats
  std::chrono::milliseconds topic_stats_publish_period_;  ///< period to publish the stats
  std::chrono::milliseconds deadline_duration_;           ///< deadline time to detect missed messages
  std::vector<double> initial_state_;                     ///< initial atr's state
  const std::string factory_state_topic_name_;            ///< topic to receive the factory state
  ATRDriver driver_;                                      ///< ATR driver object
  SubATRJointCmdShPt command_sub_;                        ///< subscriber to receive the commanded wheel velocities
  SubATRJointCmdShPt disturbance_sub_;                    ///< subscriber to receive the disturbances
  PubJointState joint_state_pub_;                         ///< publisher for the joint states (for control)
  PubATRJointState state_pub_;                            ///< publisher for the ATR state (for rviz)
  PubATRState atr_state_pub_;                             ///< ATRState publisher
  rclcpp::TimerBase::SharedPtr state_timer_;              ///< main thread

  SrvReset reset_srv_;                                    ///< reset service

  sensor_msgs::msg::JointState::SharedPtr joint_state_message_;  ///< msg with the joint state
  atr_state_msgs::msg::ATRJointState::SharedPtr state_message_;  ///< msg with the ATR state

  uint32_t num_missed_deadlines_pub_;        ///< missed msg counter joint state
  uint32_t num_missed_deadlines_pub_state_;  ///< missed msg counter ATR state
  uint32_t num_missed_deadlines_sub_;        ///< missed msg counter cmd wheel velocities

public:
  /// \brief Default constructor, needed for node composition
  /// \param[in] options Node options for rclcpp internals
  ATR_DRIVER_PUBLIC explicit ATRDriverNode(const rclcpp::NodeOptions& options);

  /// \brief Parameter file constructor
  /// \param[in] node_name Name of this node
  /// \param[in] options Node options for rclcpp internals
  ATR_DRIVER_PUBLIC explicit ATRDriverNode(const std::string& node_name,
                                           const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /// \brief Initialize state message
  void init_state_message();

  /// \brief Create command subscription
  void create_command_subscription();

  /// \brief Create disturbance subscription
  void create_disturbance_subscription();

  /// \brief Create state publisher
  void create_state_publisher();

  /**
   * @brief Create a atr state msg object
   *
   * @param message atr_state_stamped message with the current atr state
   */
  void create_atr_state_msg(atr_state_msgs::msg::ATRStateStamped& message);

  /// \brief Create timer callback
  void create_state_timer_callback();

  /// \brief Log atr driver state
  void log_driver_state();

  /// \brief create dummy callback for reset srv
  void create_dummy_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response);


  /// \brief create reset service
  void create_reset_srv();

  /// \brief Transition callback for state configuring
  /// \param[in] lifecycle node state

  LifeCycleCallbackReturn on_configure(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state activating
  /// \param[in] lifecycle node state
  LifeCycleCallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state deactivating
  /// \param[in] lifecycle node state
  LifeCycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state cleaningup
  /// \param[in] lifecycle node state
  LifeCycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state shutting down
  /// \param[in] lifecycle node state
  LifeCycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
};
}  // namespace atr_driver
}  // namespace atr

#endif  // ATR_DRIVER__ATR_DRIVER_NODE_HPP_
