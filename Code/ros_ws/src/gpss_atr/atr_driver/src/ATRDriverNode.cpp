/**
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 13.06.2022
 *
 * \copyright Copyright 2022 Chalmers
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

#include <string>
#include <memory>

#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "atr_driver/ATRDriverNode.hpp"
#include "atr_utils/AuxTools.h"

namespace atr
{
namespace atr_driver
{
ATRDriverNode::ATRDriverNode(const rclcpp::NodeOptions& options) : ATRDriverNode("atr_driver", options)
{
}

//clang-format off
ATRDriverNode::ATRDriverNode(const std::string& node_name, const rclcpp::NodeOptions& options)
  : LifecycleNode(node_name, options)
  , atr_id_(declare_parameter<int>("atr_id", 1))
  , atr_frame_id_(declare_parameter<std::string>("frame_id", "map"))
  , joint_state_topic_name_(declare_parameter<std::string>("joint_state_topic_name", "atr_joint_states") + "_" +
                            std::to_string(atr_id_))
  , joint_state_prefix_("atr_" + std::to_string(atr_id_) + "_")
  , state_topic_name_(declare_parameter<std::string>("state_topic_name", "atr_states") + "_" + std::to_string(atr_id_))
  , atr_state_topic_name_(declare_parameter<std::string>("atr_state_topic_name", "atr_states") + "_" +
                          std::to_string(atr_id_))
  , command_topic_name_(declare_parameter<std::string>("command_topic_name", "joint_command") + "_" +
                        std::to_string(atr_id_))
  , disturbance_topic_name_(declare_parameter<std::string>("disturbance_topic_name", "disturbance") + "_" +
                            std::to_string(atr_id_))
  , cart_base_joint_name_(declare_parameter<std::string>("cart_base_joint_name", "cart_base_joint") + "_" +
                          std::to_string(atr_id_))
  , state_publish_period_(
        std::chrono::microseconds{ declare_parameter<std::uint16_t>("state_publish_period_us", 1000U) })
  , enable_topic_stats_(declare_parameter<bool>("enable_topic_stats", false))
  , topic_stats_topic_name_{ declare_parameter<std::string>("topic_stats_topic_name", "driver_stats") + "_" +
                             std::to_string(atr_id_) }
  , topic_stats_publish_period_{ std::chrono::milliseconds{
        declare_parameter<std::uint16_t>("topic_stats_publish_period_ms", 1000U) } }
  , deadline_duration_{ std::chrono::milliseconds{ declare_parameter<std::uint16_t>("deadline_duration_ms", 0U) } }
  , initial_state_(declare_parameter<std::vector<double>>("driver.initial_state", { 0.0, 0.0, 0.0 }))
  , factory_state_topic_name_(declare_parameter<std::string>("factory_state_topic_name", ""))
  , driver_(ATRDriverConfig(
        declare_parameter<double>("driver.wheel_length", 0.48), declare_parameter<double>("driver.wheel_radius", 0.125),
        declare_parameter<double>("driver.damping_coefficient", 20.0),
        declare_parameter<double>("driver.gravity", -9.8), declare_parameter<double>("driver.max_wheel_speed", 1000.0),
        declare_parameter<double>("driver.noise_level", 1.0), std::chrono::microseconds{ state_publish_period_ }))
  , num_missed_deadlines_pub_{ 0U }
  , num_missed_deadlines_pub_state_{ 0U }
  , num_missed_deadlines_sub_{ 0U }
// clang-format on
{
  init_state_message();

  create_state_publisher();

  create_command_subscription();

  create_disturbance_subscription();

  create_state_timer_callback();

  // create_dummy_callback();

  create_reset_srv();

}

void ATRDriverNode::init_state_message()
{
  joint_state_message_ = std::make_shared<sensor_msgs::msg::JointState>();
  state_message_ = std::make_shared<atr_state_msgs::msg::ATRJointState>();

  // TODO: change this with parameters
  std::string atr_frame_id_ = "world";

  joint_state_message_->header.frame_id = atr_frame_id_;

  // clang-format off
  joint_state_message_->name = {  joint_state_prefix_ + "base_link_axis_x_joint",
                                  joint_state_prefix_ + "base_link_axis_y_joint",
                                  joint_state_prefix_ + "base_link_axis_z_joint",
                                  joint_state_prefix_ + "back_right_wheel_joint",
                                  joint_state_prefix_ + "back_left_wheel_joint",
                                  joint_state_prefix_ + "sw_arm_right_connect_1",
                                  joint_state_prefix_ + "front_right_swivel_wheel_joint",
                                  joint_state_prefix_ + "sw_arm_left_connect_1",
                                  joint_state_prefix_ + "front_left_swivel_wheel_joint"
                        };
  // clang-format on

  joint_state_message_->position.resize(joint_state_message_->name.size());
  joint_state_message_->velocity.resize(joint_state_message_->name.size());
  joint_state_message_->effort.resize(joint_state_message_->name.size());

  std::fill(joint_state_message_->position.begin(), joint_state_message_->position.end(), 0.0);
  std::fill(joint_state_message_->velocity.begin(), joint_state_message_->velocity.end(), 0.0);
  std::fill(joint_state_message_->effort.begin(), joint_state_message_->effort.end(), 0.0);

  Eigen::Vector3d aux =
      Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(initial_state_.data(), initial_state_.size());

  // The parameter file defines the initial orientation in DEG. We need to change it to RADS before we use it.

  aux(2) = DEG2RAD(aux(2));

  // If there's no factory state publisher, we use the initial state defined in the parameter.yaml file
  // TODO: analize which is option is better, get the initial state from optometry as a topic (factory_state) or as
  // a service. In that case, the factory_state module will receive the info from the ar_tag_tracker and send the
  // initial state as a request to the  atr_driver service.
  if (factory_state_topic_name_ == "")
  {
    driver_.set_initial_state(aux);
    driver_.set_state(driver_.get_initial_state());
  }
  // TODO: Else wait until we receive a state from the factory state and populate the initial and current atr state with
  // that information

}  // namespace atr_driver

void ATRDriverNode::create_state_publisher()
{
  rclcpp::PublisherOptions sensor_publisher_options;
  sensor_publisher_options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo&) -> void {
    num_missed_deadlines_pub_++;
  };

  rclcpp::PublisherOptions sensor_state_publisher_options;

  sensor_state_publisher_options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo&) -> void {
    num_missed_deadlines_pub_state_++;
  };

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic_name_, rclcpp::QoS(10).deadline(deadline_duration_), sensor_publisher_options);

  state_pub_ = this->create_publisher<atr_state_msgs::msg::ATRJointState>(
      state_topic_name_, rclcpp::QoS(10).deadline(deadline_duration_), sensor_state_publisher_options);

  // Publisher for the ATR state topic
  atr_state_pub_ = this->create_publisher<atr_state_msgs::msg::ATRStateStamped>(atr_state_topic_name_, 10);
}

void ATRDriverNode::create_command_subscription()
{
  // Pre-allocates message in a pool
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
  auto command_msg_strategy = std::make_shared<MessagePoolMemoryStrategy<atr_state_msgs::msg::ATRJointCommand, 1>>();

  rclcpp::SubscriptionOptions command_subscription_options;
  command_subscription_options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo&) -> void {
    num_missed_deadlines_sub_++;
  };
  if (enable_topic_stats_)
  {
    command_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    command_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name_;
    command_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period_;
  }
  auto on_command_received = [this](atr_state_msgs::msg::ATRJointCommand::SharedPtr msg) {
    // RCLCPP_WARN_STREAM(get_logger(),
    //                    "cmd wheel vel: " << msg->wheel_velocity.at(0) << "," << msg->wheel_velocity.at(1));
    // TODO: protect this W/R with mutex
    driver_.set_controller_wheel_vel(Eigen::Vector2d(msg->wheel_velocity.data()));
  };
  command_sub_ = this->create_subscription<atr_state_msgs::msg::ATRJointCommand>(
      command_topic_name_, rclcpp::QoS(10).deadline(deadline_duration_), on_command_received,
      command_subscription_options, command_msg_strategy);
}

void ATRDriverNode::create_disturbance_subscription()
{
  auto on_disturbance_received = [this](atr_state_msgs::msg::ATRJointCommand::SharedPtr msg) {
    driver_.set_disturbance(Eigen::Vector2d(msg->wheel_velocity.data()));
  };
  disturbance_sub_ = this->create_subscription<atr_state_msgs::msg::ATRJointCommand>(
      disturbance_topic_name_, rclcpp::QoS(10), on_disturbance_received);
}

void ATRDriverNode::create_atr_state_msg(atr_state_msgs::msg::ATRStateStamped& message)
{
  message.header.frame_id = atr_frame_id_;
  message.header.stamp = now();
  message.state.atr_id = atr_id_;

  message.state.pose.type.id = atr_state_msgs::msg::ATRPoseType::IDLE;

  ATRDriver::ATRState current_state = driver_.get_state();

  // Basic Rotation in \theta_z
  tf2::Quaternion q;
  q.setRPY(0, 0, current_state.pose(2));
  q.normalize();

  message.state.pose.fused_odom.position.x = current_state.pose(0);
  message.state.pose.fused_odom.position.y = current_state.pose(1);
  message.state.pose.fused_odom.position.z = 0.0;

  message.state.pose.fused_odom.orientation.x = q.x();
  message.state.pose.fused_odom.orientation.y = q.y();
  message.state.pose.fused_odom.orientation.z = q.z();
  message.state.pose.fused_odom.orientation.w = q.w();

  // for the moment, we assume that odom and fused odom are the same
  message.state.pose.odom = message.state.pose.fused_odom;

  // Time derivative of the fused_odom
  message.state.vel.fused_odom.linear.x = current_state.vel(0);
  message.state.vel.fused_odom.linear.y = current_state.vel(1);
  message.state.vel.fused_odom.linear.z = 0.0;
  message.state.vel.fused_odom.angular.x = 0.0;
  message.state.vel.fused_odom.angular.y = 0.0;
  message.state.vel.fused_odom.angular.z = current_state.vel(2);

  // message.state.full_state = false; Not needed since is set to false by default
  message.state.pose_source = atr_state_msgs::msg::ATRState::ODOM;

  // Over all state of the ATR
  message.state.overall.status = atr_state_msgs::msg::ATRStateOverall::AVAILABLE;

  // ATR mission
  message.state.mission.status = atr_state_msgs::msg::ATRStateMission::NOT_IN_USE;

  // ATR Load status
  message.state.load.status = atr_state_msgs::msg::ATRStateLoad::UNLOADED;

  // ATR Signals (which signals should be activated in the ATR)
  message.state.signal.types.push_back(atr_state_msgs::msg::ATRStateSignals::CHARGING);

  // ATR Actuator
  message.state.actuator.type = atr_state_msgs::msg::ATRStateActuator::LINEAR;
  message.state.actuator.status = atr_state_msgs::msg::ATRStateActuator::CLOSED;
  message.state.actuator.value = 0.0;

  // ATR Emergency stopped?
  message.state.emerg_stop = false;

  // ATR Error
  message.state.error.id = atr_error_msgs::msg::ATRError::NO_ERROR;
  message.state.error.message = "";

  // ATR Battery
  message.state.battery.status = atr_state_msgs::msg::ATRBatteryState::FULL;
  // Battery charge (100%)
  message.state.battery.charge_state = 1.0;
  // Battery duration (5 hrs)
  message.state.battery.life_time = 5.0;
  // Battery capacity, in this case, we assume the battery has 100% capacity
  message.state.battery.health_state = 1.0;

  // Collision state

  // Collision sensors. We assume two type of sensors (two ultrasonic and one bumper)
  atr_state_msgs::msg::ATRCollisionState collision_state;

  // No collision
  collision_state.status = atr_state_msgs::msg::ATRCollisionState::NONE;
  // minimal distance to an obstacle (no collision we use the maximum value)
  collision_state.distance = 1000;
  collision_state.description = "Collision Free";

  atr_state_msgs::msg::ATRCollisionSensor aux_sensor;

  // Ultrasonic 1
  aux_sensor.id = 0;
  aux_sensor.type = atr_state_msgs::msg::ATRCollisionSensor::ULTRA_SONIC;
  aux_sensor.data = 1000.0;
  collision_state.sensors.push_back(aux_sensor);

  // Ultrasonic 2
  aux_sensor.id = 1;
  aux_sensor.type = atr_state_msgs::msg::ATRCollisionSensor::ULTRA_SONIC;
  aux_sensor.data = 1000.0;
  collision_state.sensors.push_back(aux_sensor);

  message.state.collisions.push_back(collision_state);

  // Bumper (populating the message without aux instances)
  // add a new collision state
  message.state.collisions.push_back(atr_state_msgs::msg::ATRCollisionState());
  // Fill in the variables
  message.state.collisions[1].status = atr_state_msgs::msg::ATRCollisionState::NONE;
  message.state.collisions[1].distance = 1000;
  message.state.collisions[1].description = "Bumper off";

  // add a new sensor for this collision state
  message.state.collisions[1].sensors.push_back(atr_state_msgs::msg::ATRCollisionSensor());
  // Fill in the parameters of the new sensor
  message.state.collisions[1].sensors[0].type = atr_state_msgs::msg::ATRCollisionSensor::BUMPER;
  message.state.collisions[1].sensors[0].id = 3;
  message.state.collisions[1].sensors[0].data = 0;
}

void ATRDriverNode::create_state_timer_callback()
{
  auto state_timer_callback = [this]() {
    driver_.update();
    const auto state = driver_.get_state();

    joint_state_message_->position.at(0) = state.pose(0);
    joint_state_message_->position.at(1) = state.pose(1);
    joint_state_message_->position.at(2) = state.pose(2);

    joint_state_message_->velocity.at(0) = state.vel(0);
    joint_state_message_->velocity.at(1) = state.vel(1);
    joint_state_message_->velocity.at(2) = state.vel(2);

    joint_state_message_->effort.at(0) = state.wheel_vel(0);
    joint_state_message_->effort.at(1) = state.wheel_vel(1);
    joint_state_message_->header.stamp = now();

    state_message_->full_state = {
      state.pose(0), state.pose(1), state.pose(2), state.vel(0), state.vel(1), state.vel(2)
    };

    atr_state_msgs::msg::ATRStateStamped atr_state_message;
    create_atr_state_msg(atr_state_message);

    // TODO: create another thread for the joint_state publisher. We don't need the joint_states at the same freq. as
    // the atr_state for the control
    joint_state_pub_->publish(*joint_state_message_.get());
    state_pub_->publish(*state_message_.get());

    atr_state_pub_->publish(atr_state_message);
  };
  state_timer_ = this->create_wall_timer(state_publish_period_, state_timer_callback);

  // cancel immediately to prevent triggering it in this state
  state_timer_->cancel();
}

using std::placeholders::_1;
using std::placeholders::_2;

void  ATRDriverNode::create_dummy_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response){

  // timer_callback();

  driver_.reset();

  response->success = true;
  response->message = "resetting !";

  // dummy if to avoid unused variable warning
  // clang-format off
  if (request){}
  // clang-format on

}



void ATRDriverNode::create_reset_srv(){

  //mall
  // path_list_srv_ = create_service<std_srvs::srv::Trigger>(
  //     trigger_service_name_, std::bind(&ATRPathGenerator::triggerPathListCB, this, _1, _2));

  reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/reset_1", std::bind(&ATRDriverNode::create_dummy_callback, this, _1,_2));

  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service =
  //   atr_driver->create_service<std_srvs::srv::Trigger>("/reset_1", &create_dummy_callback);


}

void ATRDriverNode::log_driver_state()
{
  const ATRDriver::ATRState state = driver_.get_state();
  const Eigen::Vector2d disturbance_wheel_vel = driver_.get_disturbance();
  const Eigen::Vector2d controller_wheel_vel_command = driver_.get_controller_wheel_velocity();

  RCLCPP_INFO_STREAM(get_logger(), "Cart position = " << state.pose.transpose());
  RCLCPP_INFO_STREAM(get_logger(), "Cart velocity = " << state.vel.transpose());
  RCLCPP_INFO_STREAM(get_logger(), "Controller wheel vel command = " << controller_wheel_vel_command);
  RCLCPP_INFO_STREAM(get_logger(), "Disturbance vel = " << disturbance_wheel_vel);
  RCLCPP_INFO_STREAM(get_logger(), "Publisher missed deadlines = " << num_missed_deadlines_pub_);
  RCLCPP_INFO_STREAM(get_logger(), "Subscription missed deadlines = " << num_missed_deadlines_sub_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ATRDriverNode::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  // reset internal state of the driver for a clean start
  driver_.reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ATRDriverNode::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Activating");
  joint_state_pub_->on_activate();
  state_pub_->on_activate();
  atr_state_pub_->on_activate();

  // reset_srv_->on_activate();

  state_timer_->reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ATRDriverNode::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  state_timer_->cancel();
  joint_state_pub_->on_deactivate();
  state_pub_->on_deactivate();
  atr_state_pub_->on_deactivate();

  // reset_srv_->on_deactivate();

  // log the status to introspect the result
  log_driver_state();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ATRDriverNode::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ATRDriverNode::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
}  // namespace atr_driver
}  // namespace atr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point,
// allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(atr::atr_driver::ATRDriverNode)
