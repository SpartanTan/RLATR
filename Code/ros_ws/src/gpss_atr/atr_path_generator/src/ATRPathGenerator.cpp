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

#include "atr_path_generator/ATRPathGenerator.h"

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include "atr_utils/convert_extended.h"

// Standard
#include <fstream>
#include <iomanip>

// JSON
#include <nlohmann/json.hpp>

using json = nlohmann::json;

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_path_generator
{
ATRPathGenerator::ATRPathGenerator() : Node("object_list_subscriber"), NodeDescriptions("ATRPathGenerator")
{
  // Init parameters
  init();

  // Parse JSON file with the Nodes and Paths
  parse_json_file(nodes_filename_);

  // Get the node and path descriptions
  get_node_descriptions_json();

  // Shared objects
  atr_formation_list_ = std::make_shared<atr_formation_msgs::msg::ATRFormationList>();
  atr_state_list_ = std::make_shared<atr_state_msgs::msg::ATRStateListStamped>();
  obj_list_ = std::make_shared<atr_object_msgs::msg::ObjectList>();
  nona_list_ = std::make_shared<atr_object_msgs::msg::ObjectList>();
  pred_obj_list_ = std::make_shared<atr_predicted_object_msgs::msg::PredictedObjectEllipseList>();
  atr_path_list_ = std::make_shared<atr_path_msgs::msg::ATRPathList>();

  // Create Map and initialize data flags
  // v_data_names_ = { "formation", "atr_state", "objects", "nona", "pred_obj" };
  v_data_names_ = { "atr_state", "nona", "jobs" };

  int idx = 0;
  for (auto&& nam : v_data_names_)
  {
    map_data_index_[nam] = idx;
    v_data_flags.push_back(false);
    idx++;
  }

  // Formation List client (Static Info)
  // formation_list_client_ = this->create_client<atr_srvs::srv::GetATRFormation>(atr_formation_service_name_);

  // Subscription to ATR list topic
  atr_list_subs_ = create_subscription<atr_state_msgs::msg::ATRStateListStamped>(
      atr_topic_name_, 10, std::bind(&ATRPathGenerator::atr_subs_callback, this, _1));

  // Subscription to Object list topic
  obj_list_subs_ = create_subscription<atr_object_msgs::msg::ObjectList>(
      obj_topic_name_, 10, std::bind(&ATRPathGenerator::obj_subs_callback, this, _1));

  // NONA List client (Static Info)
  nona_list_client_ = this->create_client<atr_srvs::srv::GetObjectListStamped>(nona_service_name_);

  // Update Local Predicted Object List
  service_ = create_service<atr_srvs::srv::UpdatePredictedObjectList>(
      service_name_, std::bind(&ATRPathGenerator::updatePredObjectListCB, this, _1, _2));

  // Trigger the function to send a new path list to the trajectory generator module
  path_list_srv_ = create_service<std_srvs::srv::Trigger>(
      trigger_service_name_, std::bind(&ATRPathGenerator::triggerPathListCB, this, _1, _2));

  // Update Job List
  // job_service_ = create_service<atr_srvs::srv::UpdateJobList>(
  //     job_service_name_, std::bind(&ATRPathGenerator::updateJobListCB, this, _1, _2));

  // Gets the nona data
  if (!getStaticData())
  {
    RCLCPP_ERROR(get_logger(), "Error getting static data");
  }

  // ATR Path List publisher
  atr_path_list_pub_ = this->create_publisher<atr_path_msgs::msg::ATRPathList>(atr_path_topic_name_, 10);

  // UpdateATRPath client To send the new ATR Path List as a request to the service
  // provided by ATR Fleet Control
  atr_path_list_client_ = this->create_client<atr_srvs::srv::UpdateATRPathList>(atr_path_service_name_);

  // This timer triggers the publisher of the ObjectList message
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(atr_period_ms_),
  //                                  std::bind(&ATRPathGenerator::timer_callback, this));

  RCLCPP_INFO(get_logger(), "Ready to generate trajectories!");
}

ATRPathGenerator::~ATRPathGenerator()
{
}

void ATRPathGenerator::init()
{
  std::vector<std::string> param_names = { "formation_list_service_name",
                                           "atr_list_topic_name",
                                           "object_list_topic_name",
                                           "nona_list_service_name",
                                           "predicted_obj_list_service_name",
                                           "atr_path_topic_name",
                                           "period_ms",
                                           "frame_id",
                                           "path_list_service_name",
                                           "nodes_filename",
                                           "job_list_service_name",
                                           "path_list_trigger_service_name" };

  declare_parameter<std::string>(param_names.at(0));
  declare_parameter<std::string>(param_names.at(1));
  declare_parameter<std::string>(param_names.at(2));
  declare_parameter<std::string>(param_names.at(3));
  declare_parameter<std::string>(param_names.at(4));
  declare_parameter<std::string>(param_names.at(5));
  declare_parameter<int>(param_names.at(6));
  declare_parameter<std::string>(param_names.at(7));
  declare_parameter<std::string>(param_names.at(8));
  declare_parameter<std::string>(param_names.at(9));
  declare_parameter<std::string>(param_names.at(10));
  declare_parameter<std::string>(param_names.at(11));

  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  atr_formation_service_name_ = params.at(0).as_string();
  atr_topic_name_ = params.at(1).as_string();
  obj_topic_name_ = params.at(2).as_string();
  nona_service_name_ = params.at(3).as_string();
  service_name_ = params.at(4).as_string();
  atr_path_topic_name_ = params.at(5).as_string();
  atr_period_ms_ = params.at(6).as_int();
  atr_period_ = atr_period_ms_ * 1000.0;
  frame_id_ = params.at(7).as_string();
  atr_path_service_name_ = params.at(8).as_string();
  nodes_filename_ = params.at(9).as_string();
  job_service_name_ = params.at(10).as_string();
  trigger_service_name_ = params.at(11).as_string();
}

void ATRPathGenerator::atr_subs_callback(const atr_state_msgs::msg::ATRStateListStamped::SharedPtr msg)
{
  size_t msg_size = msg->atr_states.size();

  // RCLCPP_INFO_STREAM(get_logger(), __FILE__ << ":" << __LINE__);

  if (msg_size > 0)
  {
    // TODO: verify that full_state = true or pose_source = FUSED_ODOM before making the local copy
    // This is to verify that the trajectory generator uses the fused_odom to compute the paths
    data_mutex_.lock();
    atr_state_list_->atr_states = msg->atr_states;
    data_mutex_.unlock();

    // Set the ATR data flag to true
    v_data_flags[map_data_index_["atr_state"]] = true;

    // TODO: Change this with the real job number
    // For the dummy test, we use the number if atrs as the number of jobs
    // In this dummy demo, we are not using the job info, only the number of jobs
    job_number_ = atr_state_list_->atr_states.size();
    // Set the Jobs List data flag to true
    v_data_flags[map_data_index_["jobs"]] = true;
  }
  else
  {
    std::string error_message = "The ATR List is empty [" + std::to_string(msg_size) + "]";
    RCLCPP_ERROR_STREAM(get_logger(), error_message);
  }
}

void ATRPathGenerator::obj_subs_callback(const atr_object_msgs::msg::ObjectList::SharedPtr msg)
{
  size_t msg_size = msg->objects.size();

  if (msg_size > 0)
  {
    data_mutex_.lock();
    obj_list_->objects = msg->objects;
    data_mutex_.unlock();

    // Set the Object List data flag to true
    v_data_flags[map_data_index_["objects"]] = true;

    // RCLCPP_INFO_STREAM(get_logger(), "Got new Object List with [" << obj_list_->objects.size() << "] objects");
  }
  else
  {
    std::string error_message = "The Object List is empty [" + std::to_string(msg_size) + "]";
    RCLCPP_ERROR_STREAM(get_logger(), error_message);
  }
}

void ATRPathGenerator::updateJobListCB(const std::shared_ptr<atr_srvs::srv::UpdateJobList::Request> request,
                                       std::shared_ptr<atr_srvs::srv::UpdateJobList::Response> response)
{
  size_t data_size = request->list.jobs.size();
  RCLCPP_INFO_STREAM(get_logger(), "Number of received jobs: " << data_size);

  if (data_size)
  {
    // job_list_mutex_.lock();
    // job_list_->jobs = request->list.jobs;
    // job_list_mutex_.unlock();
    // auto joblist = request->list.jobs;

    job_number_ = request->list.jobs.size();

    RCLCPP_INFO_STREAM(get_logger(), "Requested Jobs: " << job_number_);

    for (auto&& i : request->list.jobs)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Job_id: " << (int)i.job_id);

      for (auto&& j : i.tasks)
      {
        RCLCPP_INFO_STREAM(get_logger(), "task_id: " << (int)j.task_id);
        RCLCPP_INFO_STREAM(get_logger(), "task_location: " << (int)j.task_location);

        for (auto&& k : j.precedence)
        {
          RCLCPP_INFO_STREAM(get_logger(), "prec: " << (int)k);
        }
        for (auto&& k : j.time_window)
        {
          RCLCPP_INFO_STREAM(get_logger(), "time_w: " << (float)k);
        }
      }
    }
  }
  else
  {
    response->success = false;
    response->error.id = atr_error_msgs::msg::ATRError::DATA_NOT_AVAILABLE;
    response->error.message = "The Job list is empty";
  }

  // Set the Jobs List data flag to true
  v_data_flags[map_data_index_["jobs"]] = true;

  response->success = true;
}

void ATRPathGenerator::updatePredObjectListCB(
    const std::shared_ptr<atr_srvs::srv::UpdatePredictedObjectList::Request> request,
    std::shared_ptr<atr_srvs::srv::UpdatePredictedObjectList::Response> response)
{
  size_t data_size = request->p_list.p_object_ellipses.size();
  if (data_size > 0)
  {
    data_mutex_.lock();
    pred_obj_list_->p_object_ellipses = request->p_list.p_object_ellipses;
    data_mutex_.unlock();
    response->success = true;

    // Set the Predicted Object List data flag to true
    v_data_flags[map_data_index_["pred_obj"]] = true;

    // RCLCPP_INFO_STREAM(get_logger(),
    //                    "Got new Predicted Object List with [" << pred_obj_list_->p_object_ellipses.size()
    //                                                           << "] objects");
  }
  else
  {
    std::string error_message = "The Predicted Object List is empty [" + std::to_string(data_size) + "]";
    RCLCPP_ERROR_STREAM(get_logger(), error_message);
    response->success = false;
    response->error.id = atr_error_msgs::msg::ATRError::DATA_NOT_AVAILABLE;
    response->error.message = error_message;
  }
}

void ATRPathGenerator::triggerPathListCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  timer_callback();

  response->success = true;
  response->message = "All good!";

  // dummy if to avoid unused variable warning
  // clang-format off
  if (request){}
  // clang-format on

}  // namespace atr_path_generator

void ATRPathGenerator::timer_callback()
{
  // TODO: change data_ready to false, and add the OPCUA client condition!
  bool data_ready = true;

  // Verify that all the needed data is available at least once
  int idx = 0;

  for (auto&& flag : v_data_flags)
  {
    if (!flag)
    {
      data_ready = false;
      RCLCPP_WARN_STREAM(get_logger(), "(" << idx << ")Data [" << v_data_names_[idx] << "] missing, waiting ...");
      // is enough if one data type is missing
      break;
    }
    idx++;
  }

  if (data_ready)
  {
    atr_path_msgs::msg::ATRPathList atr_path_msg;

    auto local_atr_formation_list = std::make_shared<atr_formation_msgs::msg::ATRFormationList>();
    auto local_atr_state_list = std::make_shared<atr_state_msgs::msg::ATRStateListStamped>();
    auto local_obj_list = std::make_shared<atr_object_msgs::msg::ObjectList>();
    auto local_nona_list = std::make_shared<atr_object_msgs::msg::ObjectList>();
    auto local_pred_obj_list = std::make_shared<atr_predicted_object_msgs::msg::PredictedObjectEllipseList>();

    // Collect Data from all the other threads
    // TODO: use one mutex for each thread to avoid bottleneck issues
    data_mutex_.lock();
    // local_atr_formation_list = atr_formation_list_;
    local_atr_state_list = atr_state_list_;
    // local_obj_list = obj_list_;
    local_nona_list = nona_list_;
    // local_pred_obj_list = pred_obj_list_;
    data_mutex_.unlock();

    size_t atr_number = local_atr_state_list->atr_states.size();

    // For the moment the ATR paths are manually defined in the json file.
    // We need to verify that we have enough paths for all the ATRs
    if (atr_number > v_paths_.size())
    {
      RCLCPP_ERROR_STREAM(get_logger(), " Not enough paths: ATRs (" << atr_number << ") "
                                                                    << "Paths: " << v_paths_.size());
      exit(EXIT_FAILURE);
    }

    if (job_number_ > (int)v_paths_.size())
    {
      RCLCPP_ERROR_STREAM(get_logger(), " Not enough paths: Requested Jobs (" << job_number_ << ") "
                                                                              << "Paths: " << v_paths_.size());
      exit(EXIT_FAILURE);
    }

    // RCLCPP_INFO_STREAM(get_logger(), "atr_number: "<<atr_number);

    // Populate the PathList
    atr_path_msgs::msg::ATRPathList atr_path_list_msg;

    // The number of pats should be the number of ATRs
    // atr_path_list_msg->paths.resize(atr_number);
    atr_path_list_msg.paths.resize(job_number_);

    // Get the current time to stamp the messages
    rclcpp::Time aux_time = now();

    atr::math::real theta = 0.0;
    tf2::Transform tf;
    geometry_msgs::msg::Transform ts;
    tf2::Quaternion q, q_z_pi2, qt;

    int id = 0;
    int path_id = rand() % (v_paths_.size());
    for (auto&& paths : atr_path_list_msg.paths)
    {
      paths.header.frame_id = frame_id_;
      paths.header.stamp = aux_time;
      paths.atr_path.atr_id = local_atr_state_list->atr_states[id].atr_id;
      // TODO: define priority 1=max??
      paths.atr_path.priority = 1;

      // Get the number of nodes (Paths - graph) for each path
      // RCLCPP_INFO_STREAM(get_logger(),"graph["<<id+1<<"] size: "<< map_path_id_graph_[id + 1].size());
      paths.atr_path.path_w_time.poses.resize(v_paths_.at(path_id).graph.size());

      // Get the poses for each node (Nodes - position)
      int count = 0;

      Eigen::Vector3d init_pose(0, 0, 0);

      for (auto&& poses : paths.atr_path.path_w_time.poses)
      {
        // The delta time should be defined by the trajectory planner
        poses.delta_time = 0.1;  // in [sec]

        // Gets the path with pathID = id+1, extracting the NodeID of each (count) node in the graph.
        // With this Node ID, we extract the position of the node from v_nodes_
        Eigen::Vector3d aux_pos =
            v_nodes_.at(map_node_id_idx_[v_paths_[path_id].graph.at(count)]).position;

        // RCLCPP_INFO_STREAM(get_logger(), "aux_pos["<<count<<"]: "<<aux_pos.transpose());

        tf.setOrigin(tf2::convert(aux_pos));

        // Calculate the orientation based on the current and next positions
        Eigen::Vector3d DX = aux_pos - init_pose;
        theta = atan2(DX(1), DX(0));
        q.setRPY(0, 0, theta);  // Basic Rotation in z
        q.normalize();
        tf.setRotation(qt);

        ts = tf2::toMsg(tf);
        // Assigning the aux_pose to atr_path_list_msg.paths.atr_path.path_w_time.poses.pose
        // we use the conversion of tf to geometry_msgs::msgs::Pose
        tf2::convert(ts, poses.pose);

        // Set history and next point
        init_pose = aux_pos;
        count++;
      }

      // Increase the id to get the next ATR info
      id += 1;
    }

    // int path_id = 0;
    // for (auto& paths : atr_path_list_msg.paths)
    // {
    //   int pose_id = 0;
    //   for (auto&& poses : paths.atr_path.path_w_time.poses)
    //   {
    //     RCLCPP_INFO_STREAM(get_logger(), "Pose{" << path_id << "}[" << pose_id << "]: (" << poses.pose.position.x
    //                                              << ", " << poses.pose.position.y << ", " << poses.pose.position.z
    //                                              << ")");
    //     pose_id++;
    //   }

    //   path_id++;
    // }

    // Insert additional nodes  in each path to make quasi-equidistant poses.
    // int path_id = 0;
    // for (auto& paths : atr_path_list_msg.paths)
    // {
      // RCLCPP_INFO_STREAM(get_logger(), "-------Path: " << path_id);

      // We need to insert nodes to create equidistant poses.
      // This is important for the trajectory generator using BSplines
      // paths.atr_path.path_w_time = insert_path_segments(paths.atr_path.path_w_time);
    //   path_id++;
    // }

    // Copy data to share it with the client
    atr_path_mutex_.lock();
    atr_path_list_->paths = atr_path_list_msg.paths;
    atr_path_mutex_.unlock();

    // Generate homogeneous nodes

    // Publish data
    atr_path_list_pub_->publish(*atr_path_list_.get());

    // Send the new ATR Path list to the server
    if (atr_path_list_client_->service_is_ready())
    {
      ReqUpdateATRPathList request = std::make_shared<atr_srvs::srv::UpdateATRPathList::Request>();
      // Copy the new atr path list as a request
      request->list = atr_path_list_msg;
      // Call the service (update the atr path list in the server)
      // We use the non blocking version
      auto future_result = atr_path_list_client_->async_send_request(
          request, std::bind(&ATRPathGenerator::wait_for_srv_response<SrvRespFutureUpdateATRPathList>, this, _1));

      // For testing purposes, the timer will be shot one time and then stop.
      // timer_->cancel();
    }
    else
    {
      RCLCPP_WARN_STREAM(get_logger(), "Service: " + atr_path_service_name_ + " not ready yet, skipping data");
    }

  }  // if data ready
}

bool ATRPathGenerator::getFormation()
{
  // Get the ATR Formation list
  rclcpp::Rate loop_rate(2);
  while (!formation_list_client_->service_is_ready())
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service ATR Formation List. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "service  ATR Formation List not available, waiting again...");
    loop_rate.sleep();
  }

  auto formation_req = std::make_shared<atr_srvs::srv::GetATRFormation::Request>();

  // Using -1 as formation id means that we request all the available formations
  formation_req->formation_ids.push_back(-1);

  // Call the Formation List service
  auto result = formation_list_client_->async_send_request(formation_req);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      atr_formation_list_->formations = result.get()->atr_formation_list.formations;
      // Set the Formation List data flag to true
      v_data_flags[map_data_index_["formation"]] = true;
      return true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Error[" << result.get()->error.id
                                                 << "]: getting formation list: " << result.get()->error.message);
    }
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Failed to call service ATR Formation List");
  }

  return false;
}

bool ATRPathGenerator::getNONA()
{
  // Get the NONA Object list
  rclcpp::Rate loop_rate(2);
  while (!nona_list_client_->service_is_ready())
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service NONA Object List. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "service  NONA Object List not available, waiting again...");
    loop_rate.sleep();
  }

  auto nona_req = std::make_shared<atr_srvs::srv::GetObjectListStamped::Request>();

  // Call the Formation List service
  auto result = nona_list_client_->async_send_request(nona_req);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      nona_list_->objects = result.get()->list.objects;

      RCLCPP_INFO_STREAM(get_logger(), "Got NONA objects!");

      // Set the NONA Object List data flag to true
      v_data_flags[map_data_index_["nona"]] = true;
      return true;
    }
    else
    {
      RCLCPP_WARN_STREAM(get_logger(), "Error[" << static_cast<int>(result.get()->error.id)
                                                << "]: getting nona object list: " << result.get()->error.message);
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Failed to call service NONA Object List");
  }

  return false;
}

bool ATRPathGenerator::getStaticData()
{
  // bool res = false;

  rclcpp::Rate loop_rate(2);

  // Wait until we get the nona objects
  while (!getNONA())
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the NONA Object List. Exiting.");
    }

    RCLCPP_INFO(get_logger(), "NONA Object List not available, waiting ...");
    loop_rate.sleep();
  }

  return true;
}

atr::math::VDouble ATRPathGenerator::get_min_max_distance(atr_path_msgs::msg::PathWithDTime& path)
{
  std::size_t poses_size = path.poses.size();

  atr::math::real min_distance = 10000.0;
  atr::math::real max_distance = 0.0;
  // We first look for the minimum and maximum distances between poses in each path
  int pose_id = 0;
  for (std::size_t i = 0; i < poses_size - 1; i++)
  {
    atr_path_msgs::msg::PoseWithDTime poses_current = path.poses.at(i);
    atr_path_msgs::msg::PoseWithDTime poses_next = path.poses.at(i + 1);

    // clang-format off
    // RCLCPP_INFO_STREAM(get_logger(), "Pose[" << pose_id << "]: (" 
    //                                              << poses_current.pose.position.x
    //                                      << ", " << poses_current.pose.position.y 
    //                                      << ", " << poses_current.pose.position.z<<")");

    // clang-format on

    Eigen::Vector2d p_current, p_next, Dp;

    p_current << poses_current.pose.position.x, poses_current.pose.position.y;
    p_next << poses_next.pose.position.x, poses_next.pose.position.y;

    atr::math::real nDp = (p_next - p_current).norm();

    // RCLCPP_INFO_STREAM(get_logger(), "dist: " << nDp);

    (nDp > max_distance) ? max_distance = nDp : max_distance = max_distance;
    (nDp < min_distance) ? min_distance = nDp : min_distance = min_distance;

    // clang-format on
    pose_id++;
  }

  return atr::math::VDouble{ min_distance, max_distance };
}

// atr_path_msgs::msg::PathWithDTime ATRPathGenerator::insert_path_segments(atr_path_msgs::msg::PathWithDTime& path)
// {
//   atr::math::VDouble min_max_dist = get_min_max_distance(path);

//   // RCLCPP_INFO_STREAM(get_logger(), "min: " << min_max_dist.at(0));
//   // RCLCPP_INFO_STREAM(get_logger(), "max: " << min_max_dist.at(1));

//   // We get the ratio between the distance between two poses and min_distance/2.0
//   int pose_id = 0;
//   std::size_t poses_size = path.poses.size();
//   atr_path_msgs::msg::PathWithDTime extended_path;
//   for (size_t i = 0; i < poses_size - 1; i++)
//   {
//     atr_path_msgs::msg::PoseWithDTime poses_current = path.poses.at(i);

//     // We push the current pose to the extended list
//     extended_path.poses.push_back(poses_current);

//     atr_path_msgs::msg::PoseWithDTime poses_next = path.poses.at(i + 1);

//     // clang-format off
//         // RCLCPP_INFO_STREAM(get_logger(), "Pose[" << pose_id << "]: (" 
//         //                                          << poses_current.pose.position.x
//         //                                  << ", " << poses_current.pose.position.y 
//         //                                  << ", " << poses_current.pose.position.z<<")");

//     // clang-format on

//     Eigen::Vector2d p_current, p_next, Dp;

//     p_current << poses_current.pose.position.x, poses_current.pose.position.y;
//     p_next << poses_next.pose.position.x, poses_next.pose.position.y;

//     // RCLCPP_INFO_STREAM(get_logger(), "p_current: " << p_current.transpose());
//     // RCLCPP_INFO_STREAM(get_logger(), "p_next: " << p_next.transpose());

//     // Pose difference
//     Dp = (p_next - p_current);

//     // slope
//     atr::math::real theta = atan2(Dp(1), Dp(0));

//     // distance
//     atr::math::real nDp = Dp.norm();

//     // RCLCPP_INFO_STREAM(get_logger(), "dist: " << nDp);

//     atr::math::real ratio = std::floor(nDp / (0.5 * min_max_dist.at(0)));

//     // RCLCPP_INFO_STREAM(get_logger(), "ratio: " << ratio);

//     // Segments

//     atr::math::real s = nDp / ratio;

//     Eigen::Vector2d x_s(s * cos(theta), s * sin(theta));

//     // RCLCPP_INFO_STREAM(get_logger(), "x_s: " << x_s.transpose());
//     // Calculate the intermediate equidistant poses between current and next
//     for (size_t j = 1; j < (size_t)ratio; j++)
//     {
//       Eigen::Vector2d aux = p_current + j * x_s;
//       // RCLCPP_INFO_STREAM(get_logger(), "aux: " << aux.transpose());
//       atr_path_msgs::msg::PoseWithDTime segment_pose;

//       segment_pose.pose.position.x = aux(0);
//       segment_pose.pose.position.y = aux(1);
//       extended_path.poses.push_back(segment_pose);
//     }

//     // Eigen::Vector2d r = p_current + ratio * x_s;

//     // RCLCPP_INFO_STREAM(get_logger(), "r: " << r.transpose());

//     // clang-format on
//     pose_id++;
//   }  // for poses

//   return extended_path;
// }

}  // namespace atr_path_generator
