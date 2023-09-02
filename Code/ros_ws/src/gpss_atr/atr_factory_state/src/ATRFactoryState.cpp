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

#include <atr_utils/convert_extended.h>
#include <atr_factory_state/ATRFactoryState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Standard
#include <fstream>
#include <iomanip>

// JSON
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using std::placeholders::_1;

namespace atr_factory_state
{
ATRFactoryState::ATRFactoryState()
  : Node("atr_state_list_publisher"), NodeDescriptions("NodeDescriptions"), tf_broadcaster_(this)
{
  // Initialize object, loading parameters.

  init();

  // Parse JSON file with the Nodes and Paths
  parse_json_file(nodes_filename_);

  // Get the node and path descriptions
  get_node_descriptions_json();

  // Publish ATRStates as a single ATRStateList message
  publisher_ = this->create_publisher<atr_state_msgs::msg::ATRStateListStamped>(list_topic_name_, 10);

  // Collects each published ATRState
  timer_ = this->create_wall_timer(std::chrono::milliseconds(atr_period_ms_),
                                   std::bind(&ATRFactoryState::timer_callback, this));

  // The ATRState message contains a full_state variable to define when both the odometry and photogrammetry
  // are available. This flag controls when the ATRs are publishing their state and the cameras are tracking the ATRs.
  // This function resets the full_state flag to false when the ATR odometry has not been published for some time.
  watchdog_timer_ = this->create_wall_timer(500ms, std::bind(&ATRFactoryState::watchdog_callback, this));

  // Defined from yaml file
  atr_states_msg_.header.frame_id = list_frame_id_;
  // We created a list for atr_number_ ATRs
  atr_states_msg_.atr_states.resize(atr_number_);

  // Also, we need to create the shared_data object to allocate the information from the different ATRState subscribers.
  // This vector should be populated using the maps atr_id->index.
  // It is important to allocate this vector before the initialization of the ATRState subscribers
  v_atr_states_.resize(atr_number_);

  // We create a mutex for each ATRState topic.
  std::vector<std::mutex> aux_list_mutex(atr_number_);
  // Mutex are not copyable, then we need to swap with the member variable
  v_atr_states_mutex_.swap(aux_list_mutex);

  // Create a vector of clocks, and initialize them.
  // This clocks are used to measure the elapsed time between odometry messages for the watchdog
  v_tc_.resize(atr_number_);
  for (auto&& clock : v_tc_)
  {
    clock = now();
  }

  // Create ATRState subscribers
  ATRStateSubscriber aux_subscriber;

  for (size_t i = 0; i < atr_number_; i++)
  {
    // define the name of the topic based on the ATR id
    // The subscriber message name must match the name defined in the publishers on each ATR.
    // For the moment, we define the message names "state_atr_<X>", where X is the ATR ID.
    std::string atr_topic_name;
    atr_topic_name = "atr_state_" + std::to_string(atr_id_list_.at(i));

    // Add the ATR id to the map id->index
    // In this way, we can map ATR IDs to index in the different vectors, e.g. ATR ID =4 -> vector[0] (4->0)
    map_id_index_[atr_id_list_.at(i)] = i;
    // and to the inverse map index->id, e.g. vector[0] -> ATR ID 4 (0->4)
    map_index_id_[i] = atr_id_list_.at(i);

    aux_subscriber = create_subscription<atr_state_msgs::msg::ATRStateStamped>(
        atr_topic_name, 10, std::bind(&ATRFactoryState::topic_callback, this, _1));

    // Since the function create_subscription returns a shared_pointer (non-copyable), we need to move it to our vector
    v_subscribers_.push_back(std::move(aux_subscriber));
  }
}

void ATRFactoryState::init()
{
  // Define parameters.

  declare_parameter<std::string>("topic_name");
  declare_parameter<std::string>("frame_id");
  declare_parameter<std::vector<int64_t>>("atr_id_list");
  declare_parameter<double>("watchdog_time");
  declare_parameter<int>("period_ms");
  declare_parameter<std::string>("nodes_filename");

  // Get parameters
  list_topic_name_ = get_parameter("topic_name").as_string();

  list_frame_id_ = get_parameter("frame_id").as_string();

  // This atr_ids should be defined by the visual tracker.
  atr_id_list_ = get_parameter("atr_id_list").as_integer_array();

  atr_number_ = atr_id_list_.size();

  // Time to wait fro the odometry message before reseting the full_state flag
  atr_watchdog_time = get_parameter("watchdog_time").as_double();

  // ATRStateList publisher period in [ms]
  atr_period_ms_ = get_parameter("period_ms").as_int();

  // Nodes and paths description file
  nodes_filename_ = get_parameter("nodes_filename").as_string();
}

void ATRFactoryState::timer_callback()
{
  // List of ATRStates
  atr_states_msg_.header.stamp = now();

  for (size_t i = 0; i < atr_states_msg_.atr_states.size(); i++)
  {
    // ATR id
    int64_t t_id = atr_id_list_.at(i);

    // ATR index
    int64_t t_idx = map_id_index_[t_id];

    atr_states_msg_.atr_states[i].atr_id = t_id;
    atr_states_msg_.atr_states[i].full_state = false;
    atr_states_msg_.atr_states[i].pose_source = atr_state_msgs::msg::ATRState::OPTOM;

    // Generate a dummy OPTOM pose using the initial pose from the pre-defined paths

    tf2::Transform tf;
    tf2::Quaternion q;
    geometry_msgs::msg::TransformStamped ts;

    // Get the first two nodes in the graph i (Path IDs start at 1, i+1)
    // int init_node_id = map_path_id_graph_[i + 1].at(0);
    // int next_node_id = map_path_id_graph_[i + 1].at(1);

    int init_node_id = v_paths_[map_path_id_idx_[i + 1]].graph.at(0);
    int next_node_id = v_paths_[map_path_id_idx_[i + 1]].graph.at(1);

    // Eigen::Vector3d init_pos = map_node_id_pos_[init_node_id];
    // Eigen::Vector3d next_pos = map_node_id_pos_[next_node_id];

    // Get positions of the init and next nodes
    Eigen::Vector3d init_pos = v_nodes_.at(map_node_id_idx_[init_node_id]).position;
    Eigen::Vector3d next_pos = v_nodes_.at(map_node_id_idx_[next_node_id]).position;

    // Set the initial position
    tf.setOrigin(tf2::Matrix3x3(q) * tf2::convert(init_pos));

    // Set the initial orientation
    Eigen::Vector3d DX = next_pos - init_pos;

    double theta = atan2(DX(1), DX(0));
    q.setRPY(0, 0, theta);  // Basic Rotation in z
    q.normalize();

    tf.setRotation(q);

    ts.transform = tf2::toMsg(tf);

    // Define the optom pose using Ts info
    tf2::convert(ts.transform, atr_states_msg_.atr_states[i].pose.optom);

    // Time derivative of the optom pose
    atr_states_msg_.atr_states[i].vel.optom.linear.x = 0;
    atr_states_msg_.atr_states[i].vel.optom.linear.y = 0;
    atr_states_msg_.atr_states[i].vel.optom.linear.z = 0;
    atr_states_msg_.atr_states[i].vel.optom.angular.x = 0;
    atr_states_msg_.atr_states[i].vel.optom.angular.y = 0;
    atr_states_msg_.atr_states[i].vel.optom.angular.z = 0;

    // Getting ODOM pose
    // If the ATR has already published data, we use its info to populate the atr_state_list (odom+fused_odom)
    // We have to guard the shared object during the writing/reading process

    v_atr_states_mutex_[t_idx].lock();
    if (v_atr_states_.at(t_idx).full_state)
    {
      atr_states_msg_.atr_states[i].pose.fused_odom = v_atr_states_.at(t_idx).pose.fused_odom;
      atr_states_msg_.atr_states[i].pose.odom = v_atr_states_.at(t_idx).pose.odom;
      atr_states_msg_.atr_states[i].pose_source = atr_state_msgs::msg::ATRState::FULL_POSE;
      atr_states_msg_.atr_states[i].full_state = v_atr_states_.at(t_idx).full_state;

      atr_states_msg_.atr_states[i].overall = v_atr_states_.at(t_idx).overall;
      atr_states_msg_.atr_states[i].mission = v_atr_states_.at(t_idx).mission;
      atr_states_msg_.atr_states[i].error = v_atr_states_.at(t_idx).error;

      atr_states_msg_.atr_states[i].vel.fused_odom = v_atr_states_.at(t_idx).vel.fused_odom;
    }
    v_atr_states_mutex_[t_idx].unlock();

    // The goal of each ATR must be defined by the Job Planner
    // This is only for debugging purposes
    // Generating GOAL pose
    q.setRPY(0, 0, M_PI);
    // Just for debugging, we use the atr_id as goal
    double goal = static_cast<double>(t_id);
    tf.setOrigin(tf2::Vector3(goal, goal, 0));
    tf.setRotation(q);
    ts.transform = tf2::toMsg(tf);
    tf2::convert(tf2::toMsg(tf), atr_states_msg_.atr_states[i].goal);
  }
  publisher_->publish(atr_states_msg_);
}

void ATRFactoryState::topic_callback(const atr_state_msgs::msg::ATRStateStamped::SharedPtr msg)
{
  int64_t t_idx = map_id_index_[msg->state.atr_id];

  v_tc_.at(t_idx) = now();

  // We protect the read/write of the atr_data. Everything after the mutex should be simple copy/assign functions. This
  // mutex will block the atr_state_list publisher
  std::lock_guard<std::mutex> guard(v_atr_states_mutex_[t_idx]);

  // We need to get the pose from the message, use the atr_id in the message and append the pose/vel odom + fused_odom
  // to the shared_data (v_atr_states_). This will be copied into the general message with all the atrs info (list of
  // atrs). We need to create a vector of atrs_states with the same size of the atr_number and use the maps to put the
  // data in the correct index of this vector.
  v_atr_states_[t_idx] = msg->state;

  // Just showing the sate of ATR 1
  // if (msg->state.atr_id == 1)
  // {
  //   // RCLCPP_INFO_STREAM(get_logger(), "ATR_IDX: " << t_idx);
  //   // RCLCPP_INFO_STREAM(get_logger(), "ATR_ID: " << (int)msg->state.atr_id);
  //   RCLCPP_INFO_STREAM(get_logger(), "Overall[" << (int)msg->state.atr_id << "]: " <<
  //   (int)msg->state.overall.status); RCLCPP_INFO_STREAM(get_logger(), "Mission[" << (int)msg->state.atr_id << "]: "
  //   << (int)msg->state.mission.status); RCLCPP_INFO_STREAM(get_logger(), "Error[" << (int)msg->state.atr_id << "]: "
  //   << (int)msg->state.error.id); RCLCPP_INFO_STREAM(get_logger(), "Error[" << (int)msg->state.atr_id << "]: " <<
  //   msg->state.error.message);
  // }

  // Once we have received the state information from the ATRS (odom + fused_odom), we set the full_state flag to true.
  // We can do it here because when this cb_function is called, then we will have both the optom info (generated by this
  // node) and the odom info. If the optom info comes from another node, we will follow the same procedure, once both
  // arrays of atr states have published data, then we set the fll_state flag to true
  v_atr_states_[t_idx].full_state = true;

  // If full_state=false, then we should not try to copy the info of the atr state to the atr list msg, because is not
  // properly populated
}

void ATRFactoryState::watchdog_callback()
{
  rclcpp::Time tc = now();

  for (size_t i = 0; i < atr_number_; i++)
  {
    double elapsed_time = (tc - v_tc_.at(i)).nanoseconds() * 1E-9;
    // RCLCPP_INFO_STREAM(get_logger(), "Elapsed Time: [" << i << "]: " << elapsed_time);

    if (elapsed_time > atr_watchdog_time)
    {
      int64_t t_idx = map_id_index_[atr_id_list_.at(i)];
      // RCLCPP_INFO_STREAM(get_logger(), "Reset full_state flag atr[" << atr_id_list_.at(i) << "]");
      v_atr_states_[t_idx].full_state = false;
    }
  }
}
}  // namespace atr_factory_state