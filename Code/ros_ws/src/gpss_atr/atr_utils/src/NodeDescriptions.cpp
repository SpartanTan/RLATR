
// ROS2
#include "atr_utils/NodeDescriptions.h"

namespace atr
{
namespace utils
{
NodeDescriptions::NodeDescriptions(std::string const& node_name) : logger_name_(node_name)
{
}

NodeDescriptions::NodeDescriptions(std::string const& node_name, std::string const& json_filename)
  : logger_name_(node_name)
{
  parse_json_file(json_filename);
}

NodeDescriptions::~NodeDescriptions()
{
}

bool NodeDescriptions::parse_json_file(std::string const& json_file)
{
  nodes_filename_ = json_file;
  // Load and Parse JSON file with the paths
  // TODO: create a ros package with configuration files. Probably atr_demo package.
  std::string package_path_ = (ament_index_cpp::get_package_share_directory("atr_path_generator") + "/config/").c_str();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), " Package path: " << package_path_);

  RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), " Package path: " << package_path_);

  nodes_file_path_ = package_path_ + nodes_filename_;

  RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), " Nodes file path: " << nodes_file_path_);

  std::ifstream ifnode(nodes_file_path_.c_str());

  if (ifnode)
  {
    jnode_ = json::parse(ifnode);
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "File: " << nodes_file_path_ << " couldn't be opened");

    return EXIT_FAILURE;
  }
  //   RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), std::setw(4) << jnode_);

  ifnode.close();

  return EXIT_SUCCESS;
}

bool NodeDescriptions::get_node_descriptions_json()
{
  int idx = 0;
  for (const auto& item : jnode_["Nodes"])
  {
    if (item.find("id") != item.end() and item.find("position") != item.end())
    {
      NodeDescription aux_node;
      aux_node.id = item["id"].get<int>();

      std::vector<double> aux_pos;
      aux_pos = item["position"].get<std::vector<double> >();

      Eigen::Map<Eigen::Vector3d> aux_v(aux_pos.data(), 3);

      aux_node.position = aux_v;

      aux_node.name = item["name"].get<std::string>();
      aux_node.description = item["description"].get<std::string>();

      v_nodes_.push_back(aux_node);

      // Create a map between the Node ID and the index in the v_nodes_ vector
      map_node_id_idx_[v_nodes_.back().id] = idx;

      // Create a map between the index in the v_nodes_ vector and the Node ID
      map_node_idx_id_[idx] = v_nodes_.back().id;

      idx++;
    }
    else
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "Node's descriptions not found!");
      return EXIT_FAILURE;
    }
  }

  // Debug
  // for (auto&& nodes : v_nodes_)
  // {
  //   RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_),
  //                      "map VNode " << nodes.id << ": (" << map_node_id_pos_[nodes.id].transpose() << ")");
  // }
  idx = 0;
  for (const auto& item : jnode_["Paths"])
  {
    if (item.find("id") != item.end() and item.find("graph") != item.end())
    {
      PathDescription aux_path;

      // Get the path id
      aux_path.id = item["id"].get<int>();

      // Get the node ids of the path
      aux_path.graph = item["graph"].get<std::vector<int> >();

      // Create a map between path ids and index of the path_vector and viceversa
      map_path_id_idx_[aux_path.id] = idx;
      map_path_idx_id_[idx] = aux_path.id;

      // Populate the vector of paths
      v_paths_.push_back(aux_path);

      idx++;
    }
    else
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "Path's descriptions not found!");
      return EXIT_FAILURE;
    }
  }

  // Debug
  // auto paths = jnode_["Paths"];

  // RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), " N paths: " << v_paths_.size());
  // for (size_t i = 1; i < v_paths_.size()+1; ++i)
  // {
  //   std::stringstream stream;

  //   stream << "Path [" << i << "]: [";

  //   for (auto&& val : map_path_id_graph_[i])
  //   {
  //     stream << val << ", ";
  //   }

  //   stream <<"]"<<std::endl;

  //   RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), stream.str());
  //   stream.str("");
  // }

  return EXIT_SUCCESS;
}

void NodeDescriptions::distribute_nodes()
{
}

}  // namespace utils
}  // namespace atr