/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 13.06.2022
 *
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

/// \file
/// \brief This file provides a ROS 2 interface to implement the atr controller.

#ifndef ATR_CONTROLLER__ATR_CONTROLLER_NODE_HPP_
#define ATR_CONTROLLER__ATR_CONTROLLER_NODE_HPP_

/*! \file ATRControllerNode.hpp
 *  \brief This file provides a ROS 2 node to implement the atr controller.
 *
 *  Provides the following functionalities:
 *      - ATR State subscriber
 *      - Twist Subscriber
 *      - Wheel velocity commands publisher
 *      - Control Data publisher (for debugging)
 *      - lifecycle states
 */

#include <string>
#include <climits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "atr_controller/ATRController.hpp"
#include "atr_controller/visibility_control.hpp"
#include "atr_state_msgs/msg/atr_joint_command.hpp"
#include "atr_state_msgs/msg/atr_joint_state.hpp"
#include "atr_state_msgs/msg/atr_control_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "atr_srvs/srv/reload_gains.hpp"

#include "atr_utils/atr_utils.hpp"

namespace atr
{
namespace atr_controller
{
/**
 * @class This class implements a ros2 node containing a controller for the ATR.
 *
 */
class ATRControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using SrvReloadGainsATR = rclcpp::Service<atr_srvs::srv::ReloadGains>::SharedPtr;
  using SubATRJointState = std::shared_ptr<rclcpp::Subscription<atr_state_msgs::msg::ATRJointState>>;
  using SubTwist = std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>>;
  using PubLifeATRJointCmd =
      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<atr_state_msgs::msg::ATRJointCommand>>;
  using PubATRCtrlData = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<atr_state_msgs::msg::ATRControlData>>;

private:
  const int atr_id_;                                      ///< atr's id
  const std::string state_topic_name_;                    ///< topic name to receive the current ATR state (form Driver)
  const std::string command_topic_name_;                  ///< topic name to publish the wheel commanded velocities
  const std::string teleop_topic_name_;                   ///< topic name to receive the target Twist velocity
  bool enable_topic_stats_;                               ///< controls if the stats should be collected
  const std::string topic_stats_topic_name_;              ///< topic name to publish the stats
  std::chrono::milliseconds topic_stats_publish_period_;  ///< period to publish the stats
  std::chrono::milliseconds deadline_duration_;           ///< deadline time to detect missed messages
  ATRController controller_;                              ///< ATR controller object

  SubATRJointState state_sub_;  ///< subscriber to the ATR state message
  SubTwist teleop_sub_;         ///< subscriber to the Twist message

  PubLifeATRJointCmd command_pub_;  ///< publisher for the wheel commanded velocities

  SrvReloadGainsATR atr_reload_gains_service_;  ///<  Service to reload the control gains from the ros parameter server

  PubATRCtrlData ctrl_data_pub_;  ///< Publisher for the control data (debugging)

  // TODO: Change to shared ptr
  atr_state_msgs::msg::ATRControlData ctrl_data_msg_;     ///< message with the control data
  atr_state_msgs::msg::ATRJointCommand command_message_;  ///< message with the wheel cmd velocities
  uint32_t num_missed_deadlines_pub_;                     ///< counter of number of missed msgs
  uint32_t num_missed_deadlines_sub_;                     ///< counter of number of missed msgs
  rclcpp::TimerBase::SharedPtr control_timer_;            ///< main thread for the controller
  std::chrono::microseconds control_publish_period_;      ///< period for the main control thread
  bool got_state_flag;                                    ///< controls if we have received a new state
  bool set_init_time;                                     ///< controls when we have initialized the timer
  rclcpp::Time ti_;                                       ///< initial time
  double tc_;                                             ///< curren time in seconds
  std::vector<double> teleop_factors_;                    ///< gain factors for the twist message

public:
  /// \brief
  /// \param[in] options

  /**
   * @brief Default constructor, needed for node composition
   *
   * @param options  Node options for rclcpp internals
   */
  ATR_CONTROLLER_PUBLIC
  explicit ATRControllerNode(const rclcpp::NodeOptions& options);

  /**
   * @brief ATRControllerNode constructor
   *
   * @param node_name  Name of this node
   * @param options ode options for rclcpp internals
   */
  ATR_CONTROLLER_PUBLIC explicit ATRControllerNode(const std::string& node_name,
                                                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /// \brief Create teleoperation subscription
  void create_teleoperation_subscription();

  /// \brief Re-loads the control gains from the parameter file
  ///  You need to change the parameters from terminal, e.g.
  ///  ros2 param set /atr_controller controller.feedback_matrix "[3.7, 1.7, 0.0, 0.0]"
  ///  and call this function to update the member variables
  void reload_control_gains(const std::shared_ptr<atr_srvs::srv::ReloadGains::Request> request,
                            std::shared_ptr<atr_srvs::srv::ReloadGains::Response> response);

  /// \brief Create state subscription
  void create_state_subscription();

  /// \brief Create command publisher and the main control thread (Timer)
  void create_command_publisher();

  /// \brief Log atr controller state
  void log_controller_state();

  /// \brief Transition callback for state configuring
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state activating
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state deactivating
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state cleaning up
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&) override;

  /// \brief Transition callback for state shutting down
  /// \param[in] lifecycle node state
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for the main thread. Computes the wheel velocities based on the current and desired ATR
   * state
   *
   */
  void state_timer_callback();
};
}  // namespace atr_controller
}  // namespace atr

#endif  // ATR_CONTROLLER__ATR_CONTROLLER_NODE_HPP_
