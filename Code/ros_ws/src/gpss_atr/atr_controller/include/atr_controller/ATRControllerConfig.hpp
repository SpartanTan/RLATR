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

//

/*! \file ATRController.hpp
 *  \brief This file provides the Configuration class to allocate the ATR parameters.
 *
 *  Provides the following functionalities:
 *      - control gains access functions
 */

#ifndef ATR_CONTROLLER_CONFIG_HPP_
#define ATR_CONTROLLER_CONFIG_HPP_

#include <cmath>
#include <vector>
#include "atr_controller/visibility_control.hpp"

namespace atr
{
namespace atr_controller
{
/*! \class This class implements a container for the control parameters
 */
class ATR_CONTROLLER_PUBLIC ATRControllerConfig
{
public:
  /**
   * @brief Construct a new ATRControllerConfig object
   *
   * @param feedback_matrix control gains vector [Kp_x,Kp_y,Kp_o,Kd_x,Kd_y,Kd_o]
   */
  explicit ATRControllerConfig(std::vector<double> feedback_matrix);

  /**
   * @brief Get the feedback matrix object
   *
   * @return const std::vector<double>& reference to the control gain vector
   */
  const std::vector<double>& get_feedback_matrix() const;

  /**
   * @brief Set the feedback matrix vector
   *
   * @param feedback_matrix desired control feedback gains vector [Kp_x,Kp_y,Kp_o,Kd_x,Kd_y,Kd_o]
   */
  void set_feedback_matrix(std::vector<double> feedback_matrix);

private:
  std::vector<double> feedback_matrix_;  ///< feedback_matrix Feedback matrix values
};

}  // namespace atr_controller
}  // namespace atr
#endif  // ATR_CONTROLLER_CONFIG_HPP_