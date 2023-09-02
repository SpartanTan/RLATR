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

// #include <stdio.h>

#include <atr_trajectory_generator/ATRTrajectoryGenerator.h>

// TF conversions from TF to Eigen and vice versa. It includes Eigen/Geometry.
#include <atr_utils/convert_extended.h>

namespace atr_trajectory_generator
{
ATRTrajectoryGenerator::ATRTrajectoryGenerator() : Node("atr_fleet_control"), atr_path_list_available(false)
{
  // Init parameters

  init();

  // Shared objects
  // atr_formation_list_ = std::make_shared<atr_formation_msgs::msg::ATRFormationList>();

  // Path from Path generator. This path is a discrete path, e.g. A* connecting factory nodes
  atr_path_list_in_ = std::make_shared<atr_path_msgs::msg::ATRPathList>();

  // This is the smoothed path for control
  atr_path_list_out_ = std::make_shared<atr_path_msgs::msg::ATRPathList>();

  // Aux object to create the path list
  atr_path_ = std::make_shared<atr_path_msgs::msg::PathWithDTime>();

  // atr_state info (from Factory State node)
  atr_state_list_ = std::make_shared<atr_state_msgs::msg::ATRStateListStamped>();

  // Formation List client (Static Info)
  formation_list_client_ = this->create_client<atr_srvs::srv::GetATRFormation>(atr_formation_service_name_);

  // Subscription to ATR list topic
  atr_list_subs_ = create_subscription<atr_state_msgs::msg::ATRStateListStamped>(
      atr_topic_name_, 10, std::bind(&ATRTrajectoryGenerator::atr_subs_callback, this, _1));

  // Service to receive the new ATRPath List (from PathGenerator)
  // TODO: we need to services, one for linear velocity trajectories, and the other for pure rotations
  // Currently, only the linear velocity trajectories is implemented
  atr_path_list_service_ = create_service<atr_srvs::srv::UpdateATRPathList>(
      service_name_, std::bind(&ATRTrajectoryGenerator::updateATRPathListCB, this, _1, _2));

  // ATR Path List publisher
  atr_path_list_pub_ = this->create_publisher<atr_path_msgs::msg::ATRPathList>(atr_path_topic_name_, 10);

  // We don't use formations in the MVP
  // if (!getStaticData())
  // {
  //   RCLCPP_ERROR(get_logger(), "Error getting static data");
  // }

  RCLCPP_INFO(get_logger(), "Ready to generate ATR trajectories!");
}

ATRTrajectoryGenerator::~ATRTrajectoryGenerator()
{
}

void ATRTrajectoryGenerator::init()
{
  // Get paramters from a list
  std::vector<std::string> param_names = { "atr_path_topic_name",
                                           "formation_list_service_name",
                                           "atr_list_topic_name",
                                           "path_list_service_name",
                                           "frame_id",
                                           "atr_path_publisher_prefix",
                                           "period_ms",
                                           "bspline_samples" };

  for (int i = 0; i < 6; i++)
    declare_parameter<std::string>(param_names.at(i));

  declare_parameter<int>(param_names.at(6));
  declare_parameter<int>(param_names.at(7));

  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  atr_path_topic_name_ = params.at(0).as_string();
  atr_formation_service_name_ = params.at(1).as_string();
  atr_topic_name_ = params.at(2).as_string();
  service_name_ = params.at(3).as_string();
  frame_id_ = params.at(4).as_string();
  pub_prefix_ = params.at(5).as_string();
  period_ms_ = params.at(6).as_int();
  bspline_samples_ = params.at(7).as_int();

  // BSpline

  // Time matrices
  T_ = Eigen::MatrixXd::Zero(3, bspline_samples_ + 1);

  Mi_ << 2, -4, 2, -3, 4, 0, 1, 0, 0;
  Mi_ = Mi_ * 0.5;

  Mm_ << 1, -2, 1, -2, 2, 1, 1, 0, 0;
  Mm_ = Mm_ * 0.5;

  Mf_ << 1, -2, 1, -3, 2, 1, 2, 0, 0;
  Mf_ = Mf_ * 0.5;

  Eigen::VectorXd t, t2;
  t.resize(bspline_samples_ + 1);
  auto begin = t.data();
  auto end = t.data() + t.size();

  t2.resize(bspline_samples_ + 1);

  int count = 0;
  for (auto i = begin; i != end; ++i)
  {
    *i = count / (float)bspline_samples_;
    t2(count) = *i * (*i);
    count++;
  }
  // T=[t²;t;1]'

  T_.row(0) << t2.transpose();

  T_.row(1) << t.transpose();

  Eigen::MatrixXd temp = Eigen::VectorXd::Ones(bspline_samples_ + 1);

  T_.row(2) << temp.transpose();

  // RCLCPP_INFO_STREAM(get_logger(), " t vector: " << t.transpose());
  // RCLCPP_INFO_STREAM(get_logger(), " t2 vector: " << t2.transpose());
  // RCLCPP_INFO_STREAM(get_logger(), " T: \n" << T_);
  // RCLCPP_INFO_STREAM(get_logger(), " Mi: \n" << Mi_);
  // RCLCPP_INFO_STREAM(get_logger(), " Mm: \n" << Mm_);
  // RCLCPP_INFO_STREAM(get_logger(), " Mf: \n" << Mf_);
}

void ATRTrajectoryGenerator::atr_subs_callback(const atr_state_msgs::msg::ATRStateListStamped::SharedPtr msg)
{
  size_t msg_size = msg->atr_states.size();

  if (msg_size > 0)
  {
    // TODO: verify that full_state = true or pose_source = FUSED_ODOM before making the local copy
    // This is to verify that the trajectory generator uses the fused_odom to compute the paths
    atr_state_mutex_.lock();
    atr_state_list_->atr_states = msg->atr_states;
    // Create map atr_ids to index

    size_t size_atr_list = atr_state_list_->atr_states.size();

    for (size_t i = 0; i < size_atr_list; i++)
    {
      // We populate the map between atr_ids and index in the atr_states vector.
      // We use this map to get the information of an atr using its atr_id.
      map_atr_id_index_[atr_state_list_->atr_states.at(i).atr_id] = i;
    }
    atr_state_mutex_.unlock();

    // Set the ATR data flag to true
    atr_state_list_flag_ = true;
  }
  else
  {
    std::string error_message = "The ATR List is empty [" + std::to_string(msg_size) + "]";
    RCLCPP_ERROR_STREAM(get_logger(), error_message);
  }
}

void ATRTrajectoryGenerator::updateATRPathListCB(
    const std::shared_ptr<atr_srvs::srv::UpdateATRPathList::Request> request,
    std::shared_ptr<atr_srvs::srv::UpdateATRPathList::Response> response)
{
  // This function creates Twist cmd publishers (TimerPub objects) depending on the path information
  // In the MVP case, it should create a thread to send the commanded Twist info to each ATR

  // int path_id = 0;
  // for (auto& paths : request->list.paths)
  // {
  //   int pose_id = 0;
  //   for (auto&& poses : paths.atr_path.path_w_time.poses)
  //   {
  //     RCLCPP_INFO_STREAM(get_logger(), "Pose{" << path_id << "}[" << pose_id << "]: (" << poses.pose.position.x << ",
  //     "
  //                                              << poses.pose.position.y << ", " << poses.pose.position.z << ")");
  //     pose_id++;
  //   }

  //   path_id++;
  // }

  if (updatePublisherList(request->list.paths))
  {
    // We will create two structures, one is a path list to publish the markers using the atr_path_list_subscriber node.
    // The second is the individual paths for each ATR. For this, we will split the input path list into single paths
    // one for each ATR.
    // We need to transform these paths into Twist commands and send them to the ATR controllers.

    // RCLCPP_INFO_STREAM(get_logger(), "Got [" << atr_path_list_out_->paths.size() << "] atr paths ");
    atr_path_list_available = true;

    // Once a Twist publisher has been created for each ATR (updatePublisherLsit()), we will use them to send the
    // target twist.
    // We need to verify that the first pose in the trajectory matches the pose of the target ATR.
    update_paths();

    response->success = true;
  }
  else
  {
    response->error.message = "The ATR Path List is empty";
    RCLCPP_WARN_STREAM(get_logger(), response->error.message);
    response->success = false;
    response->error.id = atr_error_msgs::msg::ATRError::DATA_NOT_AVAILABLE;
  }
}

bool ATRTrajectoryGenerator::updatePublisherList(v_ATRPathStamped& v_atr_paths)
{
  // Get the number of paths
  size_t paths_size = v_atr_paths.size();

  bool new_data = false;

  if (paths_size > 0)
  {
    // Copy to the shared object
    // This mutex is only needed with a threaded function
    // path_mutex_.lock();
    atr_path_list_in_->paths = v_atr_paths;
    // path_mutex_.unlock();

    // if there are new paths, we need to register them and create a client for each of them
    // TODO: change this, even if the new path list is the same size as the vector of clients, we need to see if
    // there are new paths. The path list can have the same size with completely new atr paths
    if (v_timer_obj_pub_.size() != paths_size)
    {
      // Get the node handler of this node. We needed as an argument for the TimerPub class
      auto node_ptr = shared_from_this();

      // find the new path
      for (auto&& i : v_atr_paths)
      {
        // Get the ATR ID
        int16_t atr_id = i.atr_path.atr_id;
        // Check if this atr_id is not already registered
        std::unordered_map<int, int>::iterator it = map_id_index_.find(atr_id);

        // if (it = end) this is a new atr path, and we need to create its corresponding client
        if (it == map_id_index_.end())
        {
          std::string s;
          s = pub_prefix_ + std::to_string(atr_id);

          // Create a shared_ptr of TimerPub
          // This class has a publisher and a timer. They will be used to send the twist command to each ATR
          // We need to pass a reference to the atr_state from atr_state_list_?

          std::shared_ptr<TimerPub> aux_timer_pub_obj =
              std::make_shared<TimerPub>(node_ptr, atr_id, s, period_ms_, frame_id_);

          v_timer_obj_pub_.push_back(std::move(aux_timer_pub_obj));

          // DEAN_NOTE: After the path for ATR X has been created, we must use the function:
          //  TimerPub::set_path(path);
          // and start the TimerPub thread
          // TimerPub::start();

          // v_timer_obj_pub_.back()->start();

          // Add its corresponding shared object. PathWithDTime is the msg used to command the path to the ATR
          v_atr_paths_.push_back(std::make_shared<atr_path_msgs::msg::PathWithDTime>());

          // populate map id -> index
          // we need to register the atr_id in the map. In this way, a new path for an already register atr_id will use
          // the same created publisher. New atr_ids are appended to the end of the list, and their corresponding
          // publishers will be created.
          map_id_index_[atr_id] = v_timer_obj_pub_.size() - 1;

          // clang-format off
          RCLCPP_WARN_STREAM(get_logger(),
                              "Created new Publisher : " << s <<
                              " for ATR[" << atr_id <<
                              "], Publisher("<< map_id_index_[atr_id] << ")");
          // clang-format on

        }  // if id not registered

      }  // for n paths

    }  // if new paths
    // RCLCPP_WARN_STREAM(get_logger(), "Got new atr path list");
    new_data = true;
  }  // if paths_size>0

  return new_data;
}

void ATRTrajectoryGenerator::update_paths()
{
  // RCLCPP_INFO(get_logger(), "Ready to process path list");
  // We need to get the current ATR state to calculate the trajectories from the robot pose
  // This flag is set to true when a atr_state_list msg has been received.
  if (atr_state_list_flag_)
  {
    // Get ATR states from subscriber thread
    // This info will be used to update the ATR paths
    atr_state_msgs::msg::ATRStateList local_atr_state_list;
    atr_state_mutex_.lock();
    local_atr_state_list.atr_states = atr_state_list_->atr_states;
    atr_state_mutex_.unlock();

    atr_path_msgs::msg::ATRPathList local_atr_path_list;
    // path_mutex_.lock();
    // We copy the input path list to initialize the atr_ids, header, etc.
    // We don't need the mutex since the W/R is serial in this case.
    local_atr_path_list.paths = atr_path_list_in_->paths;
    // path_mutex_.unlock();

    rclcpp::Time aux_time = now();
    int count = 0;
    // TODO: use a parameter connected with the delta time of the controller
    // TODO: Make sure that we use the initial state to compute the trajectories
    float delta_time = period_ms_ / 1000.0;
    for (auto& paths : local_atr_path_list.paths)
    {
      int atr_id = paths.atr_path.atr_id;

      // MD: make path equidistant, moved here from path generator
      paths.atr_path.path_w_time = insert_path_segments(paths.atr_path.path_w_time);

      atr_state_msgs::msg::ATRState atr_state = local_atr_state_list.atr_states[map_atr_id_index_[atr_id]];
      // Compute a non-uniform-quadratic BSpline using the input path points as the control points
      // auto m = compute_nuq_bspline(paths);
      Eigen::MatrixXd m = compute_nuq_bspline(paths, atr_state);
      // update the time stamp
      local_atr_path_list.paths[count].header.stamp = aux_time;
      local_atr_path_list.paths[count].header.frame_id = frame_id_;
      // define the new path
      local_atr_path_list.paths[count].atr_path.path_w_time.poses = getPosesWTime(m, delta_time);
      count++;
    }  // for paths

    // Divide the local atr path list in single atr paths
    for (auto& paths : local_atr_path_list.paths)
    {
      // get the atr_id to select the correct client
      int atr_id = paths.atr_path.atr_id;
      // get the index in the vector of clients corresponding to the atr_id
      int idx = map_id_index_[atr_id];

      // We set the vector of poses in each TimerPub object. This will also change the internal flag
      // to start sending the poses as Twist commands.
      v_timer_obj_pub_[idx]->set_poses_w_time(paths.atr_path.path_w_time.poses);
    }

    // Publish the modified ATR Path list to visualize it in Rviz
    atr_path_list_pub_->publish(local_atr_path_list);
  }  // if atr_state_list_flag_
  else
  {
    RCLCPP_WARN_STREAM(get_logger(), "Waiting for ATR State List");
  }
}

void ATRTrajectoryGenerator::timer_callback()
{
  if (atr_path_list_available)
  {
    RCLCPP_WARN(get_logger(), "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    update_paths();
  }
  else
  {
    RCLCPP_INFO(get_logger(), "NO path list available");
  }
}

bool ATRTrajectoryGenerator::getFormation()
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
      // v_data_flags[map_data_index_["formation"]] = true;
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

bool ATRTrajectoryGenerator::getStaticData()
{
  rclcpp::Rate loop_rate(2);

  // Wait until we get the Formation list
  while (!getFormation())
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the ATR Formation List. Exiting.");
    }

    RCLCPP_INFO(get_logger(), "ATR Formation List not available, waiting ...");
    loop_rate.sleep();
  }

  return true;
}

Eigen::MatrixXd ATRTrajectoryGenerator::compute_nuq_bspline(atr_path_msgs::msg::ATRPathStamped& paths)
{
  v_PoseWDTime poses = paths.atr_path.path_w_time.poses;

  std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> v_B;
  Eigen::MatrixXd Bi, Bm, Bf, Bt, Btempo;

  size_t poses_size = poses.size();
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2, poses_size);

  for (size_t i = 0; i < poses_size; i++)
  {
    P.col(i) << poses[i].pose.position.x, poses[i].pose.position.y;
  }

  // RCLCPP_INFO_STREAM(get_logger(), " P: \n" << P);

  Btempo = P.block(0, 0, 2, 3) * Mi_ * T_;
  v_B.push_back(Btempo);

  for (size_t i = 1; i < poses_size - 3; i++)
  {
    Btempo = P.block(0, i, 2, 3) * Mm_ * T_;
    v_B.push_back(Btempo);
  }

  Btempo = P.block(0, poses_size - 3, 2, 3) * Mf_ * T_;
  v_B.push_back(Btempo);

  Bt = v_B.at(0);
  for (auto it = v_B.begin() + 1; it != v_B.end(); ++it)
  {
    Btempo = Bt;
    int n_cols = it->cols() - 1;
    Bt.conservativeResize(Eigen::NoChange, Bt.cols() + n_cols);
    Bt << Btempo, it->block(0, 1, 2, n_cols);
  }

  return Bt;
}

Eigen::MatrixXd ATRTrajectoryGenerator::compute_nuq_bspline(atr_path_msgs::msg::ATRPathStamped& paths,
                                                            atr_state_msgs::msg::ATRState& atr_state)
{
  v_PoseWDTime poses = paths.atr_path.path_w_time.poses;

  std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> v_B;
  Eigen::MatrixXd Bi, Bm, Bf, Bt, Btempo;

  // The total P matrix is the number of poses extended by the current atr pose
  size_t poses_size = poses.size() + 1;
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2, poses_size);

  // We start the Pose matrix with the current atr position
  P.col(0) << atr_state.pose.fused_odom.position.x, atr_state.pose.fused_odom.position.y;

  // The next poses are defined by the path
  for (size_t i = 1; i < poses_size; i++)
  {
    P.col(i) << poses[i - 1].pose.position.x, poses[i - 1].pose.position.y;
  }

  // RCLCPP_WARN_STREAM(get_logger(), "atr_id: " << (int)atr_state.atr_id);
  // RCLCPP_WARN_STREAM(get_logger(), "atr pose (" << atr_state.pose.fused_odom.position.x << ", "
  //                                               << atr_state.pose.fused_odom.position.y << ")");
  // RCLCPP_INFO_STREAM(get_logger(), __FILE__ << ":" << __LINE__ << " P: \n" << P);

  // RCLCPP_INFO_STREAM(get_logger(), "Pblock: \n" << P.block(0, 0, 2, 3));

  Btempo = P.block(0, 0, 2, 3) * Mi_ * T_;

  // RCLCPP_INFO_STREAM(get_logger(), "Btempo: \n" << Btempo);

  v_B.push_back(Btempo);

  for (size_t i = 1; i < poses_size - 3; i++)
  {
    Btempo = P.block(0, i, 2, 3) * Mm_ * T_;
    v_B.push_back(Btempo);
  }

  Btempo = P.block(0, poses_size - 3, 2, 3) * Mf_ * T_;
  v_B.push_back(Btempo);

  // for (auto&& i : v_B)
  // {
  //   RCLCPP_INFO_STREAM(get_logger(), i);
  //   std::cout << i << std::endl;
  // }

  Bt = v_B.at(0);
  for (auto it = v_B.begin() + 1; it != v_B.end(); ++it)
  {
    Btempo = Bt;
    int n_cols = it->cols() - 1;
    Bt.conservativeResize(Eigen::NoChange, Bt.cols() + n_cols);
    Bt << Btempo, it->block(0, 1, 2, n_cols);
  }

  // RCLCPP_INFO_STREAM(get_logger(), "Bt: \n" << Bt);
  // std::cout << "Bt: \n" << Bt << std::endl;

  return Bt;
}

v_PoseWDTime ATRTrajectoryGenerator::getPosesWTime(Eigen::MatrixXd& m, float delta_time)
{
  v_PoseWDTime v_aux_poses;
  for (long int i = 0; i < m.cols(); i++)
  {
    atr_path_msgs::msg::PoseWithDTime aux_pose;
    aux_pose.delta_time = delta_time;
    aux_pose.pose.position.x = m(0, i);
    aux_pose.pose.position.y = m(1, i);
    aux_pose.pose.position.z = 0;

    // TODO: add the correct orientation, using position.x and position.y
    aux_pose.pose.orientation.x = 0.0;
    aux_pose.pose.orientation.y = 0.0;
    aux_pose.pose.orientation.z = 0.0;
    aux_pose.pose.orientation.w = 1.0;

    v_aux_poses.push_back(aux_pose);
  }

  return v_aux_poses;
}

atr::math::VDouble ATRTrajectoryGenerator::get_min_max_distance(atr_path_msgs::msg::PathWithDTime& path)
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

atr_path_msgs::msg::PathWithDTime ATRTrajectoryGenerator::insert_path_segments(atr_path_msgs::msg::PathWithDTime& path)
{
  atr::math::VDouble min_max_dist = get_min_max_distance(path);

  // RCLCPP_INFO_STREAM(get_logger(), "min: " << min_max_dist.at(0));
  // RCLCPP_INFO_STREAM(get_logger(), "max: " << min_max_dist.at(1));

  // We get the ratio between the distance between two poses and min_distance/2.0
  int pose_id = 0;
  std::size_t poses_size = path.poses.size();
  atr_path_msgs::msg::PathWithDTime extended_path;
  for (size_t i = 0; i < poses_size - 1; i++)
  {
    atr_path_msgs::msg::PoseWithDTime poses_current = path.poses.at(i);

    // We push the current pose to the extended list
    extended_path.poses.push_back(poses_current);

    atr_path_msgs::msg::PoseWithDTime poses_next = path.poses.at(i + 1);

    // clang-format off
        // RCLCPP_INFO_STREAM(get_logger(), "Pose[" << pose_id << "]: ("
        //                                          << poses_current.pose.position.x
        //                                  << ", " << poses_current.pose.position.y
        //                                  << ", " << poses_current.pose.position.z<<")");

    // clang-format on

    Eigen::Vector2d p_current, p_next, Dp;

    p_current << poses_current.pose.position.x, poses_current.pose.position.y;
    p_next << poses_next.pose.position.x, poses_next.pose.position.y;

    // RCLCPP_INFO_STREAM(get_logger(), "p_current: " << p_current.transpose());
    // RCLCPP_INFO_STREAM(get_logger(), "p_next: " << p_next.transpose());

    // Pose difference
    Dp = (p_next - p_current);

    // slope
    atr::math::real theta = atan2(Dp(1), Dp(0));

    // distance
    atr::math::real nDp = Dp.norm();

    // RCLCPP_INFO_STREAM(get_logger(), "dist: " << nDp);

    atr::math::real ratio = std::floor(nDp / (0.5 * min_max_dist.at(0)));

    // RCLCPP_INFO_STREAM(get_logger(), "ratio: " << ratio);

    // Segments

    atr::math::real s = nDp / ratio;

    Eigen::Vector2d x_s(s * cos(theta), s * sin(theta));

    // RCLCPP_INFO_STREAM(get_logger(), "x_s: " << x_s.transpose());
    // Calculate the intermediate equidistant poses between current and next
    for (size_t j = 1; j < (size_t)ratio; j++)
    {
      Eigen::Vector2d aux = p_current + j * x_s;
      // RCLCPP_INFO_STREAM(get_logger(), "aux: " << aux.transpose());
      atr_path_msgs::msg::PoseWithDTime segment_pose;

      segment_pose.pose.position.x = aux(0);
      segment_pose.pose.position.y = aux(1);
      extended_path.poses.push_back(segment_pose);
    }

    // Eigen::Vector2d r = p_current + ratio * x_s;

    // RCLCPP_INFO_STREAM(get_logger(), "r: " << r.transpose());

    // clang-format on
    pose_id++;
  }  // for poses

  return extended_path;
}
}  // namespace atr_trajectory_generator
