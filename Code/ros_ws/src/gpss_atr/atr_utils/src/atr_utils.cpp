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
 */

#include "atr_utils/atr_utils.hpp"

namespace atr
{
namespace utils
{
Eigen::Matrix3d Rz(double theta_rad)
{
  Eigen::Matrix3d R;

  R << cos(theta_rad), -sin(theta_rad), 0, sin(theta_rad), cos(theta_rad), 0, 0, 0, 1;
  return R;
}

void printVector(std::vector<double> in, std::string name)
{
  std::stringstream ss;

  for (auto&& i : in)
  {
    ss << i << ",";
  }

  std::cout << __FILE__ << ":" << __LINE__ << " Got " << name << ": (" << ss.str() << ")" << std::endl;
}

}  // namespace utils
}  // namespace atr