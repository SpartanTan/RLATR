import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch_ros.actions import PushRosNamespace

# How to run:
# Default parameters
# ros2 launch atr_joy joyCF710_launch.py

# Custom parameters
# ros2 launch atr_joy joyCF710_launch.py _autorepeat_rate:=100.0  _deadzone:=0.1 _dev:="/dev/input/Y"


def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription()

    # Get the launch directory
    bringup_dir = get_package_share_directory("atr_mac")

    # args that can be set from the command line or a default will be used
    dev_name_launch_arg = DeclareLaunchArgument("_dev", default_value="/dev/input/js0")
    ld.add_action(dev_name_launch_arg)
    deadzone_launch_arg = DeclareLaunchArgument("_deadzone", default_value=TextSubstitution(text="0.05"))
    ld.add_action(deadzone_launch_arg)
    autorepeat_launch_arg = DeclareLaunchArgument("_autorepeat_rate", default_value=TextSubstitution(text="30.0"))
    ld.add_action(autorepeat_launch_arg)

    # -------- NODES

    # Joy driver node

    joy_pub = Node(
        package="joy",
        name="joy_node_F710",
        executable="joy_node",
        parameters=[
            {
                "dev": LaunchConfiguration("_dev"),
                "deadzone": LaunchConfiguration("_deadzone"),
                "autorepeat_rate": LaunchConfiguration("_autorepeat_rate"),
            }
        ],
        output="screen",
    )
    ld.add_action(joy_pub)

    joy2joint_cmd_atr_yaml = os.path.join(get_package_share_directory("atr_joy"), "config", "atr_joy_jointcmd.yaml")

    joy2joint_cmd_pub = Node(
        package="atr_joy",
        name="joy2joint_cmd_publisher",
        executable="joy2joint_cmd_CF710",
        parameters=[joy2joint_cmd_atr_yaml],
        output="screen",
    )
    ld.add_action(joy2joint_cmd_pub)

    return ld
