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

#ifndef ATR_AUX_TOOLS_H
#define ATR_AUX_TOOLS_H

// #include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/StdVector>
// #include <atr_object_msgs/msg/object_stamped.hpp>
#include <atr_object_msgs/msg/object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <atr_utils/atr_utils.hpp>

#include "rclcpp/rclcpp.hpp"

/*! \def MAX_COLORS
 *  \brief A definition to set the maximum number of colors. This is connected to the max prediction horizon number.
 */
#define MAX_COLORS 20

namespace atr
{
namespace utils
{
class AuxTools
{
public:
  // TODO: move these declarations to a independent header
  using MarkerArrayPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
  using Point = geometry_msgs::msg::Point;
  using Point32 = geometry_msgs::msg::Point32;
  using v_Point = std::vector<Point>;
  using v_Point32 = std::vector<Point32>;

private:
  /* data */
protected:
  Eigen::MatrixXd Colors_;

public:
  AuxTools(/* args */);
  ~AuxTools();

protected:
  /**
   * @brief Create a Marker object
   *
   * @param frame_id reference frame for the marker
   * @param name_space namespace for the marker
   * @param id unique ID for this marker
   * @param shape shape type from visualization_msgs::Marker::<CYLINDER,CUBE,MESH,...>
   * @param x position in x relative to frame_id
   * @param y position in y relative to frame_id
   * @param z position in z relative to frame_id
   * @param q_w orientation w relative to frame_id
   * @param q_x orientation x relative to frame_id
   * @param q_y orientation y relative to frame_id
   * @param q_z orientation z relative to frame_id
   * @param s_x scaling factor x-axis
   * @param s_y scaling factor y-axis
   * @param s_z scaling factor z-axis
   * @param r marker color r-channel
   * @param g marker color g-channel
   * @param b marker color b-channel
   * @param a marker color a-channel
   * @param meshFile if shape == visualization_msgs::Marker::MESH, this is the pathfile name of the mesh
   * @return const visualization_msgs::msg::Marker Marker object
   */
  const visualization_msgs::msg::Marker createMarkerMesh(std::string frame_id, std::string name_space, int id,
                                                         int shape, double x, double y, double z, double q_w,
                                                         double q_x, double q_y, double q_z, double s_x, double s_y,
                                                         double s_z, float r, float g, float b, float a,
                                                         std::string meshFile = "") const;

  /**
   * @brief Generates a Point32 from a Eigen::MatrixXd
   *
   * @param Points Matrix with all the points, each row is a point, each column defines x, y, and z coordinates
   * @param idx row index
   * @return geometry_msgs::msg::Point32 position
   */
  geometry_msgs::msg::Point32 set_point(const Eigen::MatrixXd& Points, int64_t idx);

  /**
   * @brief Generates a Point32 from a Eigen::Vector3D
   *
   * @param point position
   * @return geometry_msgs::msg::Point32 position
   */
  geometry_msgs::msg::Point32 set_point(const Eigen::Vector3d& point);

  /**
   * @brief Generates a Point32 from three coordinates
   *
   * @param x position in x coordinate
   * @param y position in y coordinate
   * @param z position in z coordinate
   * @return geometry_msgs::msg::Point32 position
   */
  geometry_msgs::msg::Point32 set_point(const double x, const double y, const double z);

  // // clang-format off
  // /**
  //  * @brief Set the object header object
  //  *
  //  * @param frame reference frame name
  //  * @param c_time current time
  //  * @param o_class object class from atr_object_msgs::msg::ObjectClass::<UNKNOWN, HUMAN, ...>
  //  * @param o_type object type from atr_object_msgs::msg::ObjectType::<STATIC, DYNAMIC, ...>
  //  * @param o_id object id
  //  * @param o_idx object index, used for creating a id/index map
  //  * @param aux_object
  //  */
  // void set_object_header(const std::string frame, const rclcpp::Time& c_time, const int8_t o_class,
  //                        const int8_t o_type, const int64_t o_id, const int64_t o_idx,
  //                        atr_object_msgs::msg::ObjectStamped& aux_object);
  // // clang-format on

  /**
   * @brief Set the object object
   *
   * @param frame reference frame name
   * @param c_time current time
   * @param o_class o_class object class from atr_object_msgs::msg::ObjectClass::<UNKNOWN, HUMAN, ...>
   * @param o_type o_type object type from atr_object_msgs::msg::ObjectType::<STATIC, DYNAMIC, ...>
   * @param o_id object id
   * @param o_idx object index, used for creating a id/index map
   * @param v_points array of 3D vectors to define the polygon
   * @return atr_object_msgs::msg::ObjectStamped object message
   */
  atr_object_msgs::msg::Object set_object(const int8_t o_class, const int8_t o_type, const int64_t o_id,
                                          const int64_t o_idx, const std::vector<Eigen::Vector3d>& v_points);

  /**
   * @brief Set the object object
   *
   * @param frame reference frame name
   * @param c_time current time
   * @param o_class o_class object class from atr_object_msgs::msg::ObjectClass::<UNKNOWN, HUMAN, ...>
   * @param o_type o_type object type from atr_object_msgs::msg::ObjectType::<STATIC, DYNAMIC, ...>
   * @param o_id object id
   * @param o_idx object index, used for creating a id/index map
   * @param M_points 3D vectors in the form of a Matrix to define the polygon. The rows represent the 3D vectors, and
   * the columns the position in x, y, and z coordinates
   * @return atr_object_msgs::msg::ObjectStamped object message
   */
  atr_object_msgs::msg::Object set_object(const int8_t o_class, const int8_t o_type, const int64_t o_id,
                                          const int64_t o_idx, const Eigen::MatrixXd& M_points);

  /**
   * @brief transforms a vector of Points from type U to type T
   *
   * @tparam T output Point class, either geometry_msgs::msg::Point or geometry_msgs::msg::Point32
   * @tparam U input Point class, either geometry_msgs::msg::Point32 or geometry_msgs::msg::Point
   * @param in vector of points
   * @return std::vector<T> vector of points
   */
  template <typename T, typename U>
  std::vector<T> transform_point(const std::vector<U>& in) const
  {
    std::vector<T> out;
    T aux;
    for (auto&& i : in)
    {
      aux.x = i.x;
      aux.y = i.y;
      aux.z = i.z;
      out.push_back(aux);
    }
    return out;
  }

  /**
   * @brief Get the polygon points from a vector of geometry_msgs::msg::Point32
   *          This function generate a closed polygon from a list of Point32
   * @param obj_points vector of Point32 defining the polygon
   * @param marker_points output vector with the closed Polygon
   */
  void get_polygon(const v_Point32& obj_points, v_Point& marker_points) const;

  /**
   * @brief wait for service response callback function, used to create non-blocking calls
   *
   * @tparam T SharedFuture of client class
   * @param future service call object
   * @param n_logger node logger to output information
   */
  template <typename T>
  void wait_for_srv_response(T future)
  {
    auto result = future.get();

    if (!result.get()->success)
    {
      std::cout << "Something is wrong: " << result.get()->error.message << std::endl;
    }
  }

};  // atr_object_msgs class

}  // namespace utils
}  // namespace atr

#endif