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

#include "atr_nona/NONAGenerator.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <atr_object_msgs/msg/object_class.hpp>
#include <atr_object_msgs/msg/object_type.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr_nona
{
NonaGenerator::NonaGenerator() : Node("nona_list_publisher"), error_flag_(false)
{
  // Parameter initialization
  init();

  // Get NONA descriptions (Polygon vertices)

  getNONADescriptionsJSON();

  // Object List publisher (Static and Dynamic Objects)
  publisher_ = this->create_publisher<atr_object_msgs::msg::ObjectListStamped>(topic_name_, 10);

  // This timer triggers the publisher of the ObjectList message
  timer_ = this->create_wall_timer(1s, std::bind(&NonaGenerator::timer_callback, this));

  // Marker publisher
  v_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_name_, 10);

  // Nona Object list server
  service_ = create_service<atr_srvs::srv::GetObjectListStamped>(
      get_o_list_srv_name_, std::bind(&NonaGenerator::getObjectListCB, this, _1, _2));
}

void NonaGenerator::init()
{
  // clang-format off
  std::vector<std::string> param_names = { "topic_name", 
                                           "marker_topic_name", 
                                           "frame_id", 
                                           "service_name",
                                           "nona_description_filename" };
  // clang-format on

  for (auto& i : param_names)
    declare_parameter<std::string>(i);

  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  topic_name_ = params.at(0).as_string();
  marker_topic_name_ = params.at(1).as_string();
  frame_id_ = params.at(2).as_string();
  get_o_list_srv_name_ = params.at(3).as_string();
  description_filename_ = params.at(4).as_string();
}

void NonaGenerator::getNONADescriptionsJSON()
{
  // Load NONA description from JSON file
  // For the moment, the yaml file has two descriptions
  // polygons: Each polygon is defined as a vector, i.e.
  // [x1,y1,z1,x2,y2,z2,...]. This is used with rapidJSON
  // polygonsM: Each polygon is represented as an array of vectors, i.e.
  // [x1,y1,z1]
  // [x2,y2,z2]
  //  ...
  // [xn,yn,zn]. This is used with mlohman JSON. This representation is the preferred
  std::string nona_path = (ament_index_cpp::get_package_share_directory("atr_nona") + "/config/").c_str();

  RCLCPP_INFO_STREAM(get_logger(), " Nona path: " << nona_path);

  std::string nona_file_path = nona_path + description_filename_;

  RCLCPP_INFO_STREAM(get_logger(), " Nona file path: " << nona_file_path);

  std::ifstream ifs(nona_file_path.c_str());

  if (ifs.fail())
  {
    error_message_.push_back("File " + nona_file_path + " not found!");
    RCLCPP_ERROR_STREAM(get_logger(), error_message_.back());
    error_flag_ = true;
    exit(EXIT_FAILURE);
  }
  json jff = json::parse(ifs);

  std::vector<std::vector<std::vector<double>>> list_polym =
      jff["polygons"].get<std::vector<std::vector<std::vector<double>>>>();

  size_t nPolyM = list_polym.size();

  RCLCPP_INFO_STREAM(get_logger(), "list_poly size: " << nPolyM);

  // Empty array
  if (nPolyM == 0)
  {
    error_message_.push_back("Nona description is an empty array");
    RCLCPP_ERROR_STREAM(get_logger(), error_message_.back());
    error_flag_ = true;
    exit(EXIT_FAILURE);
  }

  for (auto&& poly : list_polym)
  {
    // There will be always nRows vectors with 3 cols (x,y,z)
    int nRows = poly.size();
    int nCols = poly[0].size();  // for 3D this is always 3

    // Concatenate all the vectors of each poly. Each vector represents a vertex of the polygon
    std::vector<double> v_aux;

    v_aux.reserve(nRows * nCols);

    for (auto&& vec : poly)
    {
      v_aux.insert(v_aux.end(), vec.begin(), vec.end());
    }
    // Now, we map the long vector into a Matrix
    // The rows and cols are inverted because the mapping is column-major
    Eigen::Map<Eigen::MatrixXd> aux(v_aux.data(), nCols, nRows);
    nona_description_.v_Points.push_back(aux.transpose());
    nona_description_.v_classes.push_back(atr_object_msgs::msg::ObjectClass::WALL);
    nona_description_.v_types.push_back(atr_object_msgs::msg::ObjectType::NONAA);
  }

  // for (auto &&i : nona_description_.v_Points)
  // {
  //   RCLCPP_INFO_STREAM(get_logger(),"M: \n"<<i.matrix());
  // }

  ifs.close();
}

// void NonaGenerator::getNONADescriptionsRapidJSON()
// {
//   // Load NONA description from JSON file
//   std::string nona_path = (ament_index_cpp::get_package_share_directory("atr_nona") + "/config/").c_str();

//   RCLCPP_INFO_STREAM(get_logger(), " Nona path: " << nona_path);

//   std::string nona_file_path = nona_path + description_filename_;

//   RCLCPP_INFO_STREAM(get_logger(), " Nona file path: " << nona_file_path);
//   FILE* fp = fopen(nona_file_path.c_str(), "r");
//   if (fp == NULL)
//   {
//     error_message_.push_back("File " + nona_file_path + " not found!");
//     RCLCPP_ERROR_STREAM(get_logger(), error_message_.back());
//     error_flag_ = true;
//     exit(EXIT_FAILURE);
//   }
//   char readBuffer[65536];
//   rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
//   rapidjson::Document d;
//   d.ParseStream(is);
//   const rapidjson::Value& a = d["polygonsRapid"];  // Using a reference for consecutive access is handy and faster.
//   RCLCPP_INFO_STREAM(get_logger(), "a size: " << a.Size());

//   if (!a.IsArray())
//   {
//     error_message_.push_back("Nona description is not an array");
//     RCLCPP_ERROR_STREAM(get_logger(), error_message_.back());
//     error_flag_ = true;
//     exit(EXIT_FAILURE);
//   }

//   if (!error_flag_)
//   {
//     rapidjson::SizeType nPoly = a.Size();
//     for (rapidjson::SizeType i = 0; i < nPoly; i++)  // rapidjson uses SizeType instead of size_t.
//     {
//       rapidjson::SizeType nPoints = a[i].Size();

//       // Each polygon is a list of x,y,z points. Then, the total nPoints is divided by 3 to get the number of rows
//       const int r = nPoints / 3;
//       std::vector<double> points;
//       for (rapidjson::SizeType j = 0; j < nPoints; j++)
//       {
//         points.push_back(a[i][j].GetDouble());
//       }

//       // The rows and cols are inverted because the mapping is column-major
//       Eigen::Map<Eigen::MatrixXd> aux(points.data(), 3, r);

//       nona_description_.v_Points.push_back(aux.transpose());
//       nona_description_.v_classes.push_back(atr_object_msgs::msg::ObjectClass::WALL);
//       nona_description_.v_types.push_back(atr_object_msgs::msg::ObjectType::NONAA);

//       RCLCPP_INFO_STREAM(get_logger(), "M[" << i << "]: \n" << nona_description_.v_Points[i].matrix());
//     }
//   }
//   fclose(fp);
// }

void NonaGenerator::getObjectListCB(const std::shared_ptr<atr_srvs::srv::GetObjectListStamped::Request> request,
                                    std::shared_ptr<atr_srvs::srv::GetObjectListStamped::Response> response)
{
  auto local_message = atr_object_msgs::msg::ObjectListStamped();

  // copy the shared object locally
  data_mutex_.lock();
  local_message = nona_list_;
  data_mutex_.unlock();

  if (local_message.objects.size() > 0)
  {
    response->list = local_message;
    response->success = true;
  }
  else
  {
    std::string error_message = "No NONA objects available";
    response->success = false;
    response->error.id = atr_error_msgs::msg::ATRError::DATA_NOT_AVAILABLE;
    response->error.message = error_message;
  }

  // clang-format off
  if (request){} //dummy if to prevent warning
  // clang-format on
}

void NonaGenerator::timer_callback()
{
  auto aux_message = atr_object_msgs::msg::ObjectListStamped();
  visualization_msgs::msg::MarkerArray m_marker_msg;

  atr_object_msgs::msg::Object aux_obj;
  rclcpp::Time aux_time = now();

  aux_message.header.frame_id = frame_id_;
  aux_message.header.stamp = aux_time;

  visualization_msgs::msg::Marker aux_marker;
  aux_marker.header.frame_id = frame_id_;
  aux_marker.header.stamp = aux_time;
  aux_marker.ns = "points_and_lines";
  aux_marker.action = visualization_msgs::msg::Marker::ADD;
  aux_marker.pose.orientation.w = 1.0;
  aux_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  aux_marker.scale.x = 0.02;

  // Magenta color for NONA objects
  aux_marker.color.a = 1.0f;
  aux_marker.color.r = 1.0f;
  aux_marker.color.b = 1.0f;

  for (size_t i = 0; i < nona_description_.v_Points.size(); i++)
  {
    int64_t object_id = i + 1;
    aux_obj = set_object(nona_description_.v_classes[i], nona_description_.v_types[i], object_id, i,
                         nona_description_.v_Points[i]);

    // generate a closed polygon from the object points. This points will be used to create the Line markers
    get_polygon(aux_obj.polygon.points, aux_marker.points);

    // Use the same object ID for the marker ID
    aux_marker.id = object_id;

    aux_message.objects.push_back(aux_obj);
    m_marker_msg.markers.push_back(aux_marker);
  }

  // RCLCPP_INFO_STREAM(get_logger(), "MP- Publishing: " << now().nanoseconds());

  // TODO: add service to toggle the publishers
  publisher_->publish(aux_message);

  v_marker_publisher_->publish(m_marker_msg);

  // write the nona list to the share it with the server cb, we guard the write/read access
  std::lock_guard<std::mutex> guard(data_mutex_);
  nona_list_ = aux_message;

}  // timer_callback

}  // namespace atr_nona