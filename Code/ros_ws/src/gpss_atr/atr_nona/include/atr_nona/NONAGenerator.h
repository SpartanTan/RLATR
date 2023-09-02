/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 15.03.2021
 *
 * \copyright Copyright 2021 Chalmers
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
 *  This project has received financial  support  from  Chalmers  AI  Re-search Centre
 *  (CHAIR) and AB Volvo (Project ViMCoR).
 */

#ifndef NONA_GENERATOR_H
#define NONA_GENERATOR_H

/*! \file NonaGenerator.h
 *  \brief Generates dummy objects and publish them as an NONAObjectList message.
 *
 *  Provides the following functionalities:
 *    - NONA Object List topic publisher (List of NONA Objects as Line Markers)
 *    - Generates objects (dummy objects)
 *    - GetPredictedObjectList service (to provide nona object list to the ATRTrajectoryGenerator module)
 *
 */

// Standard
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "atr_object_msgs/msg/object_list_stamped.hpp"
#include "atr_error_msgs/msg/atr_error.hpp"
#include "atr_srvs/srv/get_object_list_stamped.hpp"

#include <atr_utils/atr_utils.hpp>
#include <atr_utils/AuxTools.h>

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// Additional Libraries
// #include <rapidjson/document.h>
// #include <rapidjson/filereadstream.h>
// #include <rapidjson/stringbuffer.h>
// #include <rapidjson/writer.h>

#include <nlohmann/json.hpp>

using namespace std::chrono_literals;

namespace atr_nona
{
struct NONADescription
{
  std::vector<Eigen::MatrixXd> v_Points;
  std::vector<int8_t> v_classes;
  std::vector<int8_t> v_types;
};

class NonaGenerator : public rclcpp::Node, public atr::utils::AuxTools
{
  // Public member variables
public:
  using SrvObjectList = rclcpp::Service<atr_srvs::srv::GetObjectListStamped>::SharedPtr;
  using VString = std::vector<std::string>;
  using json = nlohmann::json;
  // Private member variables
private:
  rclcpp::TimerBase::SharedPtr timer_;                                               ///< Timer to trigger the publisher
  rclcpp::Publisher<atr_object_msgs::msg::ObjectListStamped>::SharedPtr publisher_;  ///< ObjectList publisher

  rclcpp::TimerBase::SharedPtr update_timer_;  ///< timer to trigger new object list

  MarkerArrayPublisher v_marker_publisher_;  ///< Marker array pub. Publishes the NONA polygons as line Markers

  SrvObjectList service_;

  std::string topic_name_;            ///< topic name for the nona object list message
  std::string marker_topic_name_;     ///< topic name for the marker array
  std::string frame_id_;              ///< reference frame for all the objects, usually "map"
  std::string get_o_list_srv_name_;   ///< service name to provide the NONA object list
  std::string description_filename_;  ///< filename to load the NONA description (JSON file)
  VString error_message_;             ///< error message to describe the type of error

  bool error_flag_;  ///< error flag, true in case of error

  NONADescription nona_description_;                   ///< container of the NONA data
  atr_object_msgs::msg::ObjectListStamped nona_list_;  ///< list of nona objects (shared variable between publisher and
                                                       ///< service)
  std::mutex data_mutex_;  ///< Mutex to protect write/read access between the service and the publisher

  // Public member methods
public:
  /**
   * @brief Construct a new Object List Publisher object
   *
   */
  NonaGenerator();

  /**
   * @brief Parameter loading and initialization using yaml file
   *
   */
  void init();

  /**
   * @brief Get the NONA Descriptions from JSON file
   *          It loads the node's description from a json file defined by the configuration yaml file, see launch file.
   */
  void getNONADescriptionsJSON();

  /**
   * @brief Get the NONA Descriptions from JSON file
   *          It loads the node's description from a json file defined by the configuration yaml file, see launch file.
   *          This function uses rapidJSON, but it is NOT completely implemented.
   */
  // void getNONADescriptionsRapidJSON();

  // Private member methods
private:
  /**
   * @brief Callback function for the service to provide NONA Obj list
   *
   * @param request (empty)
   * @param response List of NONA objects and success flag with error message describing the failure
   */
  void getObjectListCB(const std::shared_ptr<atr_srvs::srv::GetObjectListStamped::Request> request,
                       std::shared_ptr<atr_srvs::srv::GetObjectListStamped::Response> response);

  /**
   * @brief callback function for the timer.
   *
   * In this case, this function creates the objects and publish them.
   *
   */
  void timer_callback();
};
}  // namespace atr_nona

#endif
