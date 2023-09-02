#include <string>
#include <vector>
#include <memory>

#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "kinematic_controller/ATRKinematicControllerNode.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace atr
{
namespace atr_kinematic_controller
{

ATRKinematicControllerNode::ATRKinematicControllerNode(const rclcpp::NodeOptions & options)
  : ATRKinematicControllerNode(
        "atr_controller_1", options)
{}

ATRKinematicControllerNode::ATRKinematicControllerNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode(node_name, options)
    , atr_id_(declare_parameter<int>("atr_id", 1))
    , state_topic_name_(declare_parameter<std::string>(
          "state_topic_name",
          "atr_joint_states") + "_" +
      std::to_string(atr_id_))
    , command_topic_name_(declare_parameter<std::string>(
          "command_topic_name",
          "joint_command") + "_" +
      std::to_string(atr_id_))
    , teleop_topic_name_(declare_parameter<std::string>("teleop_topic_name", "teleop") + "_" +
      std::to_string(atr_id_))
    , enable_topic_stats_(declare_parameter<bool>("enable_topic_stats", false))
    , topic_stats_topic_name_{declare_parameter<std::string>(
          "topic_stats_topic_name",
          "controller_stats") + "_" +
      std::to_string(atr_id_)}
    , topic_stats_publish_period_{std::chrono::milliseconds{
                declare_parameter<std::uint16_t>("topic_stats_publish_period_ms", 1000U)}}
    , deadline_duration_{std::chrono::milliseconds{declare_parameter<std::uint16_t>(
              "deadline_duration_ms", 0U)}}
    , controller_(ATRControllerConfig(declare_parameter<std::vector<double>>(
          "controller.feedback_matrix",
          {1.0, 1.0, 1.0, 0.01, 0.01, 0.0050})))
    , num_missed_deadlines_pub_{0U}
    , num_missed_deadlines_sub_{0U}
    , control_publish_period_(
        std::chrono::microseconds{declare_parameter<std::uint16_t>(
                "state_publish_period_us",
                1000U)})
    , got_state_flag(false)
    , set_init_time(true)
    , teleop_factors_(declare_parameter<std::vector<double>>(
          "controller.teleop_factors",
          {0.0, 0.0, 0.0}))
{
    RCLCPP_INFO_STREAM(get_logger(), "creating teleop_subs");
    create_teleoperation_subscription();
}

create_teleoperation_subscription()
{

}
}
}
