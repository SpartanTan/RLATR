#include <atr_joy/Joy2Twist.h>

using std::placeholders::_1;

namespace atr_joy
{
Joy2Twist::Joy2Twist() : Node("atr_joy2twist_publisher"), m_gains_(Vector3d::Zero()), m_status_(false)
{
  // Loading the gains from parameter server (yaml file).
  // Gets the topic names (joy and twist) and the gains to change joy data to twist data
  m_status_ = loadGains();

  f710TwistPublisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(m_cmdTwistName_, 10);
  f710Subs_ = create_subscription<sensor_msgs::msg::Joy>(m_joyTopicName_, 10,
                                                         std::bind(&Joy2Twist::f710SubsCallBack, this, _1));

  RCLCPP_INFO_STREAM(get_logger(), "Joy to Twist F710 started.");
}
Joy2Twist::~Joy2Twist()
{
}

void Joy2Twist::f710SubsCallBack(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  m_twMsg_.header.stamp = now();
  m_twMsg_.header.frame_id = "atr_" + std::to_string(atr_id_) + "_base_link";

  m_twMsg_.twist.linear.x = 0.0;
  m_twMsg_.twist.linear.y = 0.0;
  m_twMsg_.twist.angular.z = 0.0;

  // Dead-man switch, only when the user presses LB button the joy data is sent to the controller
  if (msg->buttons[F710_BUTTON_LB] == 1)
  {
    m_twMsg_.twist.linear.x = m_gains_(0) * msg->axes[F710_AXIS_STICK_LEFT_UPWARDS];
    m_twMsg_.twist.linear.y = m_gains_(1) * msg->axes[F710_AXIS_STICK_LEFT_LEFTWARDS];
    m_twMsg_.twist.angular.z = m_gains_(2) * msg->axes[F710_AXIS_STICK_RIGHT_LEFTWARDS];

    // RCLCPP_INFO_STREAM(get_logger(), "Twist: " << m_twMsg_.linear.x << ", " << m_twMsg_.linear.y << ", " <<
    // m_twMsg_.angular.z);
  }

  f710TwistPublisher_->publish(m_twMsg_);

  if (msg->buttons[F710_BUTTON_A] == 1 && msg->buttons[F710_BUTTON_B] == 1)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Restarting Robot");
  }
}

void Joy2Twist::setGains(const Vector3d& gains)
{
  m_gains_ = gains;
  RCLCPP_INFO_STREAM(get_logger(), "Joy2Twist gains: " << m_gains_.transpose());
}

bool Joy2Twist::loadGains()
{
  // clang-format off
  std::vector<std::string> param_names = { "joy_topic_name",
                                           "cmd_twist_name",
                                           "gains",
                                           "atr_id"
  };
  // clang-format on

  // for (auto&& i : param_names)
  //   declare_parameter(i);

  declare_parameter<std::string>(param_names.at(0));
  declare_parameter<std::string>(param_names.at(1));
  declare_parameter<std::vector<double>>(param_names.at(2));
  declare_parameter<int>(param_names.at(3));

  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  m_joyTopicName_ = params.at(0).as_string();
  m_cmdTwistName_ = params.at(1).as_string();
  std::vector<double> p = params.at(2).as_double_array();
  atr_id_ = params.at(3).as_int();

  int sp = p.size();

  if (sp < 3)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Joy2Twist loadGains(): Wrong number of parameters in /f710_gains/kp: " << sp);
    return false;
  }

  for (int i = 0; i < sp; i++)
  {
    m_gains_(i) = p[i];
  }

  RCLCPP_INFO_STREAM(get_logger(), "JoyName: " << m_joyTopicName_);
  RCLCPP_INFO_STREAM(get_logger(), "TwistName: " << m_cmdTwistName_);
  RCLCPP_WARN_STREAM(get_logger(), "JoyGains: " << m_gains_.transpose());

  return true;
}

bool Joy2Twist::getStatus()
{
  return m_status_;
}

}  // namespace atr_joy
