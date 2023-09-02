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

#include "atr_controller/ATRControllerConfig.hpp"
#include <utility>
#include <vector>
#include <iostream>
#include <sstream>

namespace atr
{
namespace atr_controller
{
ATRControllerConfig::ATRControllerConfig(std::vector<double> feedback_matrix)
  : feedback_matrix_{ std::move(feedback_matrix) }
{
}

const std::vector<double>& ATRControllerConfig::get_feedback_matrix() const
{
  return feedback_matrix_;
}

void ATRControllerConfig::set_feedback_matrix(std::vector<double> feedback_matrix)
{
  feedback_matrix_ = feedback_matrix;
}

}  // namespace atr_controller
}  // namespace atr