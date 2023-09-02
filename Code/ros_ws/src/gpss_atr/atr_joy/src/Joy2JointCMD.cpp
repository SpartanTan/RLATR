#include <atr_joy/Joy2JointCMD.h>

using std::placeholders::_1;

namespace atr_joy
{
Joy2JointCMD::Joy2JointCMD() : Node("atr_joy2twist_publisher"), m_gains_(Vector3d::Zero()), m_status_(false)
{
  // Loading the gains from parameter server (yaml file).
  // Gets the topic names (joy and twist) and the gains to change joy data to twist data
  m_status_ = loadGains();

  f710TwistPublisher_ = this->create_publisher<atr_state_msgs::msg::ATRJointCommand>(m_cmdJointName_, 10);
  f710Subs_ = create_subscription<sensor_msgs::msg::Joy>(m_joyTopicName_, 10,
                                                         std::bind(&Joy2JointCMD::f710SubsCallBack, this, _1));

  RCLCPP_INFO_STREAM(get_logger(), "Joy to Twist F710 started.");
}
Joy2JointCMD::~Joy2JointCMD()
{
}

void Joy2JointCMD::f710SubsCallBack(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  std::fill(m_JointCMDMsg_.wheel_velocity.begin(), m_JointCMDMsg_.wheel_velocity.end(), 0);

  // Dead-man switch, only when the user presses LB button the joy data is sent to the controller
  if (msg->buttons[F710_BUTTON_LB] == 1)
  {
    m_JointCMDMsg_.wheel_velocity.at(0) = m_gains_(0) * msg->axes[F710_AXIS_STICK_LEFT_UPWARDS];
    m_JointCMDMsg_.wheel_velocity.at(1) = m_gains_(0) * msg->axes[F710_AXIS_STICK_RIGHT_UPWARDS];

    // RCLCPP_INFO_STREAM(get_logger(), "Twist: " << m_twMsg_.linear.x << ", " << m_twMsg_.linear.y << ", " <<
    // m_twMsg_.angular.z);
  }

  f710TwistPublisher_->publish(m_JointCMDMsg_);

  if (msg->buttons[F710_BUTTON_A] == 1 && msg->buttons[F710_BUTTON_B] == 1)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Restarting Robot");
  }
}

void Joy2JointCMD::setGains(const Vector3d& gains)
{
  m_gains_ = gains;
  RCLCPP_INFO_STREAM(get_logger(), "Joy2JointCMD gains: " << m_gains_.transpose());
}

bool Joy2JointCMD::loadGains()
{
  // clang-format off
  std::vector<std::string> param_names = { "joy_topic_name",
                                           "cmd_twist_name",
                                           "gains"
  };
  // clang-format on

  // for (auto&& i : param_names)
  //   declare_parameter(i);

  declare_parameter<std::string>(param_names.at(0));
  declare_parameter<std::string>(param_names.at(1));
  declare_parameter<std::vector<double>>(param_names.at(2));

  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  m_joyTopicName_ = params.at(0).as_string();
  m_cmdJointName_ = params.at(1).as_string();
  std::vector<double> p = params.at(2).as_double_array();

  int sp = p.size();

  if (sp < 3)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Joy2JointCMD loadGains(): Wrong number of parameters in /f710_gains/kp: " << sp);
    return false;
  }

  for (int i = 0; i < sp; i++)
  {
    m_gains_(i) = p[i];
  }

  RCLCPP_INFO_STREAM(get_logger(), "JoyName: " << m_joyTopicName_);
  RCLCPP_INFO_STREAM(get_logger(), "TwistName: " << m_cmdJointName_);
  RCLCPP_WARN_STREAM(get_logger(), "JoyGains: " << m_gains_.transpose());

  return true;
}

bool Joy2JointCMD::getStatus()
{
  return m_status_;
}

}  // namespace atr_joy
