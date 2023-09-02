/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 11.01.2022
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
#ifndef JOY2TWIST_H
#define JOY2TWIST_H

/*! \file Joy2Twist.h
 *  \brief Transforms joy message to Twist message type
 *
 *  Provides the following functionalities:
 *      - Subscriber to Joy message
 *      - Publisher to publish the Joy message transformed into Twist message
 */

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/joy.hpp>
// #include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <string>
#include <vector>
#include <math.h>

#include "time.h"
#include "stdlib.h"
#include <memory>

/*********************************************************************
 * EIGEN INCLUDES
 ********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace Eigen;

namespace atr_joy
{
// Joystick configuration (NOTE: This configuration is for DirectInput mode, you need to put the gamepad in DirectInput
// mode, marked “D” on the gamepad)

// TODO: ADD service to activate rumble
// ros2 topic pub --rate 1 /joy/set_feedback sensor_msgs/msg/JoyFeedback "{type: 1, id: 0, intensity: 1.0}"
// If the joystick doesn't rumble, switch to "X" and back to "D"

// TODO: add service to remap the topic name during execution, we need this to control multiple ATRs with one joystick
// The topic remapping can be done when launching the node but it cannot be changed during execution

enum F710_BUTTONS
{
  F710_BUTTON_X = 0,
  F710_BUTTON_A,
  F710_BUTTON_B,
  F710_BUTTON_Y,
  F710_BUTTON_LB,
  F710_BUTTON_RB,
  F710_BUTTON_LT,
  F710_BUTTON_RT,
  F710_BUTTON_BACK,
  F710_BUTTON_START,
  F710_BUTTON_JOY_LEFT_DOWN,
  F710_BUTTON_JOY_RIGHT_DOWN
};

enum F710_AXIS
{
  F710_AXIS_STICK_LEFT_LEFTWARDS = 0,
  F710_AXIS_STICK_LEFT_UPWARDS,
  F710_AXIS_STICK_RIGHT_LEFTWARDS,
  F710_AXIS_STICK_RIGHT_UPWARDS,
  F710_AXIS_BUTTON_CROSS_LEFTWARDS,
  F710_AXIS_BUTTON_CROSS_UPWARDS
};

/**
 *  @class This class maps Joy messages to Twist messages
 */
class Joy2Twist : public rclcpp::Node
{
  // Private variables
private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr f710Subs_;  ///< Subscriber to get joytick information
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr f710TwistPublisher_;  ///< Publisher to publish the
                                                                                       ///< Joystick information
                                                                                       ///< transformed into Twist
                                                                                       ///< message

  Vector3d m_gains_;                          ///< gain factor to change joystick data [-1,1] to max velocity
  geometry_msgs::msg::TwistStamped m_twMsg_;  ///< Twist message
  std::string m_joyTopicName_;                ///< Topic name where the joy information will be published
  std::string m_cmdTwistName_;                ///< Topic name where the transformed Twist information will be published
  int atr_id_;

  bool m_status_;  ///< Flag to detect when the gains have been loaded successfully

  // public methods
public:
  /**
   * @brief Construct a new Joy 2 Twist object
   *
   */
  Joy2Twist();

  /**
   * @brief Destroy the Joy 2 Twist object
   *
   */
  ~Joy2Twist();

  /**
   * @brief Set the Gains object
   *
   * @param gains sets new gains to map joy data to twist data
   */
  void setGains(const Vector3d& gains);

  /**
   * @brief loads parameters from the ros parameter server
   *
   * @return true all the parameters were loaded
   * @return false something was wrong
   */
  bool loadGains();

  /**
   * @brief Gets a flag to verify when the parameters have been loaded
   *
   * @return true all parameters are loaded
   * @return false  somethins was wrong
   */
  bool getStatus();
  // private methods
private:
  /**
   * @brief Joy message callback function to receive new joy messages
   *
   * @param msg Joy message data
   */
  void f710SubsCallBack(const sensor_msgs::msg::Joy::SharedPtr msg);
};

}  // namespace atr_joy

#endif  // JOY2TWIST_H
