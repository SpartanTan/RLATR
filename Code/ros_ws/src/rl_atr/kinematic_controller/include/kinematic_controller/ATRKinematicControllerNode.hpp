#include <string>
#include <climits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "atr_controller/ATRController.hpp"
#include "atr_controller/visibility_control.hpp"
#include "atr_state_msgs/msg/atr_joint_command.hpp"
#include "atr_state_msgs/msg/atr_joint_state.hpp"
#include "atr_state_msgs/msg/atr_control_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "atr_srvs/srv/reload_gains.hpp"

#include "atr_utils/atr_utils.hpp"

namespace atr
{
namespace atr_kinematic_controller
{
class ATRKinematicControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    
    using PubLifeATRJointCmd =
      rclcpp_lifecycle::LifecyclePublisher<atr_state_msgs::msg::ATRJointCommand>::SharedPtr;

    // constructor without naming the node
    explicit ATRKinematicControllerNode(const rclcpp::NodeOptions & options);

    // constructor with node name parameter
    explicit ATRKinematicControllerNode(
        const std::string & node_name,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    const int atr_id_;                                      ///< atr's id
    const std::string state_topic_name_;                  ///< topic name to receive the current ATR state (form Driver)
    const std::string command_topic_name_;                ///< topic name to publish the wheel commanded velocities
    const std::string teleop_topic_name_;                 ///< topic name to receive the target Twist velocity
    bool enable_topic_stats_;                             ///< controls if the stats should be collected
    const std::string topic_stats_topic_name_;            ///< topic name to publish the stats
    std::chrono::milliseconds topic_stats_publish_period_; ///< period to publish the stats
    std::chrono::milliseconds deadline_duration_;         ///< deadline time to detect missed messages

    PubLifeATRJointCmd command_pub_;
};

}
}
