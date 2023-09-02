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

#ifndef ATR_UTILS_HPP_
#define ATR_UTILS_HPP_

/*! \file atr_utils.hpp
 *  \brief This file provides basic tools needed for the ATR implementation.
 *
 *  Provides the following functionalities:
 *      - Print function for std::vector
 *      - Basic Rotation Matrices
 */

#include <cmath>
#include <vector>
#include <mutex>
#include <iostream>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace atr
{
namespace utils
{
/**
 * @brief Auxiliary function to compute a basic Rotation matrix around z
 *
 * @param theta_rad Rotation angle in rad
 * @return Eigen::Matrix3d SO(3) Rotation Matrix
 */
Eigen::Matrix3d Rz(double theta_rad);

// TODO: Make it a template function
/**
 * @brief prints a std:vector
 *
 * @param in vector
 * @param name of the variable
 */
void printVector(std::vector<double> in, std::string name);

}  // namespace utils
}  // namespace atr

#endif