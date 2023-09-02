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

#include <atr_driver/RungeKutta.hpp>

namespace atr
{
namespace atr_driver
{
RungeKutta::RungeKutta(size_t dimension)
  : N(dimension)
  , state(Eigen::VectorXd::Zero(dimension, 1))
  , k1(Eigen::VectorXd::Zero(dimension, 1))
  , k2(Eigen::VectorXd::Zero(dimension, 1))
  , k3(Eigen::VectorXd::Zero(dimension, 1))
  , k4(Eigen::VectorXd::Zero(dimension, 1))
{
  // k1.resize(dimension);
  // k2.resize(dimension);
  // k3.resize(dimension);
  // k4.resize(dimension);
  // state.resize(dimension);
}

void RungeKutta::step(const derivativeF& df, Eigen::VectorXd& y, Eigen::VectorXd& yp, double h, Eigen::VectorXd& u)
{
  // std::size_t i = 0U;

  yp.setZero();

  // stage 1
  yp = k1 = df(y, u);

  // stage 2
  state = y + h * 0.5 * k1;
  k2 = df(state, u);

  // stage 3
  state = y + h * (0.5 * k2);
  k3 = df(state, u);

  // stage 4
  state = y + h * (1.0 * k3);
  k4 = df(state, u);

  // update next step
  y = y + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
}
}  // namespace atr_driver
}  // namespace atr