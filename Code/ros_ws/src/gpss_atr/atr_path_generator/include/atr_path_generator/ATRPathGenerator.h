/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 15.03.2021
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

#ifndef ATR_PATH_GENERATOR_H
#define ATR_PATH_GENERATOR_H

/*! \file ATRPathGenerator.h
 *  \brief Provides dummy trajectory for the ATRs.
 *
 *  Provides the following functionalities:
 *      - List of paths for the ATRs, as publisher and client (to send info to the Fleet Ctrl)
 *      - Subscribers
 *          ATRState List
 *          Object List
 *      - Clients
 *          ATRFormation
 *          Path(not really implemented)
 *          NONA Object List
 *      - Service
            Update Predicted Object List
 */

// Standard
#include <memory>

// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>

#include "atr_job_msgs/msg/atr_job_list.hpp"
#include "atr_path_msgs/msg/atr_path_list.hpp"
#include "atr_state_msgs/msg/atr_state_list_stamped.hpp"
#include "atr_object_msgs/msg/object_list.hpp"
#include "atr_srvs/srv/get_atr_formation.hpp"
#include "atr_srvs/srv/get_object_list_stamped.hpp"
#include "atr_srvs/srv/update_atr_path_list.hpp"
#include "atr_srvs/srv/update_job_list.hpp"
#include "atr_srvs/srv/update_predicted_object_list.hpp"
#include "atr_utils/AuxTools.h"
#include "atr_utils/NodeDescriptions.h"
#include "atr_utils/math_defs.h"

using std::placeholders::_1;

namespace atr_path_generator
{
class ATRPathGenerator : public rclcpp::Node, public atr::utils::AuxTools, public atr::utils::NodeDescriptions
{
public:
  using SrvRespFutureFormation = rclcpp::Client<atr_srvs::srv::GetATRFormation>::SharedFuture;
  using ClientGetATRFormation = rclcpp::Client<atr_srvs::srv::GetATRFormation>::SharedPtr;
  using SubsATRStateListStamped = rclcpp::Subscription<atr_state_msgs::msg::ATRStateListStamped>::SharedPtr;
  using SubsObjectList = rclcpp::Subscription<atr_object_msgs::msg::ObjectList>::SharedPtr;
  using ClientGetObjectList = rclcpp::Client<atr_srvs::srv::GetObjectListStamped>::SharedPtr;
  using SrvUpdatePredObjList = rclcpp::Service<atr_srvs::srv::UpdatePredictedObjectList>::SharedPtr;
  using SrvUpdateJobList = rclcpp::Service<atr_srvs::srv::UpdateJobList>::SharedPtr;
  using SrvTriggerUpdateATRPathList = rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr;
  using PubATRPathList = rclcpp::Publisher<atr_path_msgs::msg::ATRPathList>::SharedPtr;
  using ClientUpdateATRPathList = rclcpp::Client<atr_srvs::srv::UpdateATRPathList>::SharedPtr;

  using ATRFormationListShPt = atr_formation_msgs::msg::ATRFormationList::SharedPtr;
  using ATRStateListShPt = atr_state_msgs::msg::ATRStateListStamped::SharedPtr;
  using ObjectListShPt = atr_object_msgs::msg::ObjectList::SharedPtr;
  using PredObjEllipseListShPt = atr_predicted_object_msgs::msg::PredictedObjectEllipseList::SharedPtr;
  using ATRPathListShPt = atr_path_msgs::msg::ATRPathList::SharedPtr;
  using ReqUpdateATRPathList = atr_srvs::srv::UpdateATRPathList::Request::SharedPtr;
  using SrvRespFutureUpdateATRPathList = rclcpp::Client<atr_srvs::srv::UpdateATRPathList>::SharedFuture;
  using JobListShPt = atr_job_msgs::msg::ATRJobList::SharedPtr;

private:
  rclcpp::TimerBase::SharedPtr timer_;            ///< Timer to trigger the publisher
  ClientGetATRFormation formation_list_client_;   ///< Client to request the ATR Formation list
  SubsATRStateListStamped atr_list_subs_;         ///< Subscriber to the ATRStateList topic
  SubsObjectList obj_list_subs_;                  ///< Subscriber for the dyn and static object list
  ClientGetObjectList nona_list_client_;          ///< Client to request the ATR Formation list
  SrvUpdatePredObjList service_;                  ///< Service to update the internal predicted object list
  SrvUpdateJobList job_service_;                  ///< Service to update the job list
  SrvTriggerUpdateATRPathList path_list_srv_;     ///< Service to send the path list
  PubATRPathList atr_path_list_pub_;              ///< Publisher for the ATR Path list
  ClientUpdateATRPathList atr_path_list_client_;  ///< Client to update the ATR Path List

  std::vector<std::string> v_data_names_;      ///< Vector of data names to identify the different input signals
  std::unordered_map<int, int> map_id_index_;  ///< Map to connect Object ID to index in the local memory map[id] =
                                               ///< index)
  std::map<std::string, int> map_data_index_;  ///< Map to connect received data type with index (vector of flags and
                                               ///< mutex)

  std::vector<bool> v_data_flags;  ///< Flags to control when the data is available

  std::string atr_formation_service_name_;  ///< Name of the service that provides the ATR Formation List
  std::string nona_service_name_;           ///< Name of the service that provides the NONA Object List

  std::string obj_topic_name_;         ///< Object list topic name for the subscriber
  std::string atr_topic_name_;         ///< ATR list topic name for the subscriber
  std::string atr_path_topic_name_;    ///< ATR Path list topic name for the publisher
  std::string service_name_;           ///< Service name to update the local predicted object list
  int atr_period_ms_;                  ///< Sample time for the publisher in ms
  double atr_period_;                  ///< Sample time for the publisher in s
  std::string frame_id_;               ///< Reference frame for the paths
  std::string atr_path_service_name_;  ///< Name for the service to update the ATR Path List
  std::string job_service_name_;       ///< Name for the service to update the job list
  std::string trigger_service_name_;   ///< Name for the trigger service

  ATRFormationListShPt atr_formation_list_;  ///< Shared variable to allocate the formation list
  ATRStateListShPt atr_state_list_;          ///< Shared variable to allocate the atr state list
  ObjectListShPt obj_list_;                  ///< Shared variable to allocate the obj list
  ObjectListShPt nona_list_;                 ///< Shared variable to allocate the NONA obj list
  PredObjEllipseListShPt pred_obj_list_;     ///< Shared variable to allocate the pred obj list
  ATRPathListShPt atr_path_list_;            ///< Shared variable to allocate the atr path list (publish var)
  JobListShPt job_list_;                     ///< Shared variable to allocate the job list

  std::mutex data_mutex_;      ///< Mutex to protect write/read access between subscribers/server and main process
  std::mutex atr_path_mutex_;  ///< Mutex to protect write/read access between publisher and client
  std::mutex job_list_mutex_;  ///< Mutex to protect write/read access to the shared job list
  int job_number_;             ///< Number of requested Jobs

public:
  /**
   * @brief Standard constructor
   *
   */
  ATRPathGenerator(/* args */);
  ~ATRPathGenerator();

  /**
   * @brief Initialize the object
   *          It load the parameters from a yaml file
   */
  void init();

  /**
   * @brief Get the Nodes Descriptions from JSON file
   *          It loads the node's description from a json file defined by the configuration yaml file, see launch file.
   */
  // void get_node_descriptions_json();

private:
  /**
   * @brief callback function for the Obj List subscriber
   *
   * @param msg Object list
   */
  void obj_subs_callback(const atr_object_msgs::msg::ObjectList::SharedPtr msg);

  /**
   * @brief callback function for the ATRState List subscriber
   *
   * @param msg ATRState list
   */
  void atr_subs_callback(const atr_state_msgs::msg::ATRStateListStamped::SharedPtr msg);

  /**
   * @brief updates the job list to generate the trajectories
   *
   * @param request new job list
   * @param response success or flag with error message describing the failure
   */
  void updateJobListCB(const std::shared_ptr<atr_srvs::srv::UpdateJobList::Request> request,
                       std::shared_ptr<atr_srvs::srv::UpdateJobList::Response> response);

  /**
   * @brief update predicted object list callback function
   *
   * @param request new predicted object list
   * @param response success flag with error message describing the failure
   */
  void updatePredObjectListCB(const std::shared_ptr<atr_srvs::srv::UpdatePredictedObjectList::Request> request,
                              std::shared_ptr<atr_srvs::srv::UpdatePredictedObjectList::Response> response);

  /**
   * @brief Provides a service to request a new path list and send it to the trajectory generator
   *
   * @param request unused
   * @param response success = true or error if it fails
   */
  void triggerPathListCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief callback function triggered by the timer. It controls the publisher
   *
   */
  void timer_callback();

  /**
   * @brief Get the ATR Formation list. Requests the Formation list from the server
   *
   * @return true the formation list was received
   * @return false error
   */
  bool getFormation();

  /**
   * @brief Get the NONA object list. Requests the NONA list from the server
   *
   * @return true list of NONA objects received
   * @return false error
   */
  bool getNONA();

  /**
   * @brief Waits until the Formation List and the NONA list are received
   *
   * @return true received both lists
   * @return false error
   */
  bool getStaticData();

  atr::math::VDouble get_min_max_distance(atr_path_msgs::msg::PathWithDTime& path);

  //atr_path_msgs::msg::PathWithDTime insert_path_segments(atr_path_msgs::msg::PathWithDTime& path);
};
}  // namespace atr_path_generator

#endif