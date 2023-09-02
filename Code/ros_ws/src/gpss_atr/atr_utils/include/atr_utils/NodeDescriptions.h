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

#ifndef NODE_DESCRIPTIONS_H
#define NODE_DESCRIPTIONS_H

/*! \file NodeDescriptions.h
 *  \brief Reads and parses a JSON file with the Node Descriptions.
 *
 *  Provides the following functionalities:
 *      - Function to parse a JSON file
 *      - Access to nodes' information as different types
 */
// Standard

// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// Standard
#include <fstream>
#include <iomanip>

// JSON
#include <nlohmann/json.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace atr
{
namespace utils
{
using json = nlohmann::json;

struct NodeDescription
{
  int id;                    ///< Node id
  Eigen::Vector3d position;  ///< Position of the node wrt wcf
  std::string name;          ///< Node name
  std::string description;   ///< Human-readable description of the node
};

struct PathDescription
{
  int id;                  ///< Path id
  std::vector<int> graph;  ///< Graph sequence of nodes to define a path
};

class NodeDescriptions
{
protected:
  std::string nodes_filename_;            ///< JSON file name with node descriptions
  std::vector<NodeDescription> v_nodes_;  ///< Vector of node descriptions
  std::map<int, int> map_node_id_idx_;    ///< Map to connect node's id with index in the vector
  std::map<int, int> map_node_idx_id_;    ///< Map to connect vector index with node's id

  std::vector<PathDescription> v_paths_;  ///< Vector of path descriptions
  std::map<int, int> map_path_id_idx_;    ///< Map to connect path's id with index in the vector
  std::map<int, int> map_path_idx_id_;    ///< Map to connect vector index with path's id

private:
  std::string package_path_;  ///< Path of the ros2 package
  std::string logger_name_;   ///< Name of the logger for printing out info

  std::string nodes_file_path_;  ///< JSon file name with path with the nodes descriptions

  json jnode_;  ///< json object

public:
  NodeDescriptions(std::string const& node_name);

  NodeDescriptions(std::string const& node_name, std::string const& json_filename);

  ~NodeDescriptions();

  /**
   * @brief Parse the JSON file with the node and path descriptions
   *
   * @param json_file JSON file name, e.g. my_file.json
   * @return true The file was found and parsed
   * @return false The file couldn't be found or there are errors in the json file
   */
  bool parse_json_file(std::string const& json_file);

  /**
   * @brief Get the Nodes Descriptions from JSON file
   *          It loads the node's description from a json file defined by the configuration yaml file, see launch file.
   *
   * @return true The nodes and paths were successfully obtained
   * @return false Error in the JSON file
   */
  bool get_node_descriptions_json();

  void distribute_nodes();
};
}  // namespace utils
}  // namespace atr

#endif  // NODE_DESCRIPTIONS_H