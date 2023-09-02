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

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "atr_controller/ATRControllerNode.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr
{
namespace atr_controller
{
ATRControllerNode::ATRControllerNode(const rclcpp::NodeOptions& options)
  : ATRControllerNode("atr_controller_1", options)
{
}

// clang-format off
ATRControllerNode::ATRControllerNode(const std::string& node_name, const rclcpp::NodeOptions& options)
  : LifecycleNode(node_name, options)
  , atr_id_(declare_parameter<int>("atr_id", 1))
  , state_topic_name_(declare_parameter<std::string>("state_topic_name", "atr_joint_states")+ "_" +
                            std::to_string(atr_id_))
  , command_topic_name_(declare_parameter<std::string>("command_topic_name", "joint_command")+ "_" +
                            std::to_string(atr_id_))
  , teleop_topic_name_(declare_parameter<std::string>("teleop_topic_name", "teleop")+ "_" +
                            std::to_string(atr_id_))
  , enable_topic_stats_(declare_parameter<bool>("enable_topic_stats", false))
  , topic_stats_topic_name_{ declare_parameter<std::string>("topic_stats_topic_name", "controller_stats")+ "_" +
                            std::to_string(atr_id_) }
  , topic_stats_publish_period_{ std::chrono::milliseconds{
        declare_parameter<std::uint16_t>("topic_stats_publish_period_ms", 1000U) } }
  , deadline_duration_{ std::chrono::milliseconds{ declare_parameter<std::uint16_t>("deadline_duration_ms", 0U) } }
  , controller_(ATRControllerConfig(declare_parameter<std::vector<double>>("controller.feedback_matrix",
                                                          { 1.0, 1.0, 1.0, 0.01, 0.01, 0.0050 })))
  , num_missed_deadlines_pub_{ 0U }
  , num_missed_deadlines_sub_{ 0U }
  , control_publish_period_(
        std::chrono::microseconds{ declare_parameter<std::uint16_t>("state_publish_period_us", 1000U) })
  , got_state_flag(false)
  , set_init_time(true)
  , teleop_factors_(declare_parameter<std::vector<double>>("controller.teleop_factors",
                                                          { 0.0, 0.0, 0.0 }))
{
  RCLCPP_INFO_STREAM(get_logger(), "creating teleop_subs");
  create_teleoperation_subscription();

  RCLCPP_INFO_STREAM(get_logger(), "creating state_subs");
  create_state_subscription();

  RCLCPP_INFO_STREAM(get_logger(), "creating command_pub");
  create_command_publisher();

  std::stringstream sst;

  sst.str("");

  for (auto&& i : controller_.get_feedback_matrix())
  {
    sst << i << ",";
  }
  RCLCPP_WARN_STREAM(get_logger(), "Control Gains: ["<< sst.str()<<"]");

}

// clang-format on

void ATRControllerNode::create_teleoperation_subscription()
{
  // Callback function
  // See: https://docs.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-170

  auto on_atr_teleop = [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (got_state_flag)
    {
      // static double theta_old = controller_.get_state().at(2);

      // Eigen::Vector3d aux_w(msg->twist.linear.x, msg->twist.linear.y, 0);

      // Eigen::Vector3d aux_m;

      // RCLCPP_INFO_STREAM(get_logger(), "-----------------------");
      // RCLCPP_INFO_STREAM(get_logger(), "[" << atr_id_ << "] aux_w: " << aux_w.transpose());

      // // Assumption: if the frame_id is not "world", then it is atr_(atr_id)_base_link
      // // Then, we need the transformation between the atr_id frame and the world c.f.
      // if (msg->header.frame_id == "world")
      // {
      //   double theta_c = atan2(aux_w(1), aux_w(0));
      //   double Delta_theta = theta_c - theta_old;
      //   double n_aux_w = aux_w.norm();
      //   aux_m << n_aux_w * cos(Delta_theta), n_aux_w * sin(Delta_theta), 0.0;
      //   theta_old = theta_c;
      // }
      // else
      // {
      //   aux_m = aux_w;
      // }

      // RCLCPP_INFO_STREAM(get_logger(), "[" << atr_id_ << "] aux_m: " << aux_m.transpose());
      bool wcf = false;
      (msg->header.frame_id == "world") ? wcf = true : wcf = false;
      // clang-format off
      // The angular velocity doesn't need mapping
      std::vector<double> aux = { teleop_factors_.at(0) * msg->twist.linear.x, 
                                  teleop_factors_.at(1) * msg->twist.linear.y,
                                  teleop_factors_.at(2) * msg->twist.angular.z };
      // clang-format on

      if (((aux.at(0) < 0) || (std::abs(aux.at(1)) > std::abs(aux.at(0)))) && (!wcf))  //
      {
        aux.at(0) = 0;
        aux.at(1) = 0;
      }

      controller_.set_teleop(aux, wcf);
    }
    else
      RCLCPP_WARN_STREAM(get_logger(), "ATR state not yet received");
  };

  // Create subscription to the teleop topic
  // clang-format off
  teleop_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
                      teleop_topic_name_, 
                      rclcpp::QoS(10), 
                      on_atr_teleop);
  // clang-format on
}

void ATRControllerNode::reload_control_gains(const std::shared_ptr<atr_srvs::srv::ReloadGains::Request> request,
                                             std::shared_ptr<atr_srvs::srv::ReloadGains::Response> response)
{
  if (request)
  {
  }

  rclcpp::Parameter matrix = this->get_parameter("controller.feedback_matrix");

  controller_.set_feedback_matrix(matrix.as_double_array());

  std::stringstream ss1;

  for (auto&& i : controller_.get_feedback_matrix())
  {
    ss1 << i << ", ";
  }

  RCLCPP_WARN_STREAM(get_logger(), __FILE__ << ":" << __LINE__ << " New controller Gains: "
                                            << "[" << ss1.str() << "]");
  response->success = true;
}

void ATRControllerNode::create_state_subscription()
{
  // Pre-allocates message in a pool
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

  // Dean_Note: to pre-allocate a message, it must have a static size, e.g. JoinState doesn't work since the variable
  // position is defined as float64[]. On the other hand, float64[6] full_state does work!
  auto state_msg_strategy = std::make_shared<MessagePoolMemoryStrategy<atr_state_msgs::msg::ATRJointState, 1>>();

  rclcpp::SubscriptionOptions state_subscription_options;
  state_subscription_options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo&) -> void {
    num_missed_deadlines_sub_++;
  };
  if (enable_topic_stats_)
  {
    state_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    state_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name_;
    state_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period_;
  }
  auto on_sensor_message = [this](const atr_state_msgs::msg::ATRJointState::SharedPtr msg) {
    // update atr state

    int n = sizeof(msg->full_state) / sizeof(msg->full_state[0]);

    std::vector<double> current_state(6, 0);

    memcpy(&current_state[0], &msg->full_state[0], n * sizeof(double));

    // Mutex protected operation
    controller_.set_state(current_state);

    got_state_flag = true;
  };
  state_sub_ = this->create_subscription<atr_state_msgs::msg::ATRJointState>(
      state_topic_name_, rclcpp::QoS(10).deadline(deadline_duration_), on_sensor_message, state_subscription_options,
      state_msg_strategy);
}

void ATRControllerNode::create_command_publisher()
{
  // Service to trigger an ATR error
  // TODO: get the service name from parameter
  atr_reload_gains_service_ = create_service<atr_srvs::srv::ReloadGains>(
      "~/reload_gains", std::bind(&ATRControllerNode::reload_control_gains, this, _1, _2));

  rclcpp::PublisherOptions command_publisher_options;

  command_publisher_options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo&) -> void {
    num_missed_deadlines_pub_++;
  };
  command_pub_ = this->LifecycleNode::create_publisher<atr_state_msgs::msg::ATRJointCommand>(
      command_topic_name_, rclcpp::QoS(10).deadline(deadline_duration_), command_publisher_options);

  ctrl_data_pub_ = this->LifecycleNode::create_publisher<atr_state_msgs::msg::ATRControlData>("~/ctrl_data", 10);

  // With labda function
  // control_timer_ = this->create_wall_timer(state_publish_period_, state_timer_callback);

  // Binding a member function
  control_timer_ =
      this->create_wall_timer(control_publish_period_, std::bind(&ATRControllerNode::state_timer_callback, this));
}

void ATRControllerNode::log_controller_state()
{
  const auto state = controller_.get_state();
  const auto teleoperation_command = controller_.get_teleop();
  const auto wheel_command = controller_.get_wheel_vel_command();

  RCLCPP_INFO(get_logger(), "Cart position = %lf", state.at(0));
  RCLCPP_INFO(get_logger(), "Cart velocity = %lf", state.at(1));
  RCLCPP_INFO(get_logger(), "Pole angle = %lf", state.at(2));
  RCLCPP_INFO(get_logger(), "Pole angular velocity = %lf", state.at(3));
  RCLCPP_INFO(get_logger(), "Teleoperation cart position = %lf", teleoperation_command.at(0));
  RCLCPP_INFO(get_logger(), "Teleoperation cart velocity = %lf", teleoperation_command.at(1));
  RCLCPP_INFO(get_logger(), "Wheel command R= %lf", wheel_command.at(0));
  RCLCPP_INFO(get_logger(), "Wheel command L= %lf", wheel_command.at(1));
  RCLCPP_INFO(get_logger(), "Publisher missed deadlines = %u", num_missed_deadlines_pub_);
  RCLCPP_INFO(get_logger(), "Subscription missed deadlines = %u", num_missed_deadlines_sub_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ATRControllerNode::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  // reset internal state of the controller for a clean start
  controller_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ATRControllerNode::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Activating");
  command_pub_->on_activate();
  ctrl_data_pub_->on_activate();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ATRControllerNode::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  command_pub_->on_deactivate();
  ctrl_data_pub_->on_deactivate();
  // log the status to introspect the result
  log_controller_state();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ATRControllerNode::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ATRControllerNode::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ATRControllerNode::state_timer_callback()
{
  if (got_state_flag)
  {
    if (set_init_time)
    {
      set_init_time = false;
      ti_ = now();
    }

    tc_ = (now() - ti_).nanoseconds() * 1E-9;

    controller_.set_initial_time(ti_.nanoseconds() * 1E-9);

    // The time tc_ is used to calculate the dt inside the controller
    controller_.update(tc_);
  }
  else
  {
    controller_.set_wheel_vel_command({ 0, 0 });
  }

  // Publish the wheel velocity commands

  std::vector<double> wheel_cmd = { 0, 0 };

  if (controller_.get_state_flag())
    wheel_cmd = controller_.get_wheel_vel_command();

  atr_state_msgs::msg::ATRControlData ctrl_msg;

  std::vector<std::array<double, 3>> control_data = controller_.get_control_data();

  ctrl_msg.pose = control_data.at(0);
  ctrl_msg.velocity = control_data.at(1);
  ctrl_msg.desired_pose = control_data.at(2);
  ctrl_msg.desired_velocity = control_data.at(3);
  ctrl_msg.pose_error = control_data.at(4);
  ctrl_msg.velocity_error = control_data.at(5);
  ctrl_msg.cmd_vel = control_data.at(6);

  command_message_.wheel_velocity = { wheel_cmd.at(0), wheel_cmd.at(1) };
  command_pub_->publish(command_message_);
  ctrl_data_pub_->publish(ctrl_msg);
}

}  // namespace atr_controller
}  // namespace atr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(atr::atr_controller::ATRControllerNode)
