#  \file
#
# \author Emmanuel Dean
#
# \version 0.1
# \date 13.06.2022
#
# \copyright Copyright 2022 Chalmers
#
### License
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml

import xacro


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()

    atr_description_path = FindPackageShare("atr_description").find("atr_description")
    default_model_path = atr_description_path + "/urdf/atr_robot_base.urdf.xacro"
    default_rviz_config_path = atr_description_path + "/rviz/atr_scenario.rviz"

    print("-------------------" + default_rviz_config_path)

    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher_gui",
    )
    ld.add_action(gui_arg)

    model_arg = DeclareLaunchArgument(
        name="model", default_value=str(default_model_path), description="Absolute path to robot urdf file"
    )
    ld.add_action(model_arg)

    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig", default_value=str(default_rviz_config_path), description="Absolute path to rviz config file"
    )
    ld.add_action(rviz_arg)

    with_rviz_param = DeclareLaunchArgument(
        "rviz", default_value="False", description="Launch RVIZ2 in addition to other nodes"
    )
    ld.add_action(with_rviz_param)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        emulate_tty=True,
    )
    ld.add_action(rviz_node)

    # Object List Publisher (NONA objects)
    nona_dir = FindPackageShare("atr_nona").find("atr_nona")
    nona_generator_yaml = os.path.join(nona_dir, "config", "nona_generator.yaml")
    nona_list_node = Node(
        package="atr_nona",
        name="nona_generator",
        executable="atr_nona_node",
        parameters=[nona_generator_yaml],
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(nona_list_node)

    # Create the launch configuration variables
    autostart_param = DeclareLaunchArgument(
        name="autostart", default_value="True", description="Automatically start lifecycle nodes"
    )
    priority_param = DeclareLaunchArgument(name="priority", default_value="0", description="Set process priority")
    ld.add_action(autostart_param)
    ld.add_action(priority_param)

    # CPU affinity defines the CPUs used to run a process (and its threads)
    # This parameter defines the CPU ids, after converting it into binary format.
    # For example,  cpu-affinity=0 --> uses all the available CPUs, cpu-affinity 4 = 0100 --> CPU 2, cpu-affinity 3 = 0011 --> CPUs 0,1
    cpu_affinity_param = DeclareLaunchArgument(
        name="cpu-affinity", default_value="0", description="Set process CPU affinity"
    )
    ld.add_action(cpu_affinity_param)

    with_lock_memory_param = DeclareLaunchArgument(
        name="lock-memory", default_value="False", description="Lock the process memory"
    )
    ld.add_action(with_lock_memory_param)

    lock_memory_size_param = DeclareLaunchArgument(
        name="lock-memory-size", default_value="0", description="Set lock memory size in MB"
    )
    ld.add_action(lock_memory_size_param)

    config_child_threads_param = DeclareLaunchArgument(
        name="config-child-threads",
        default_value="False",
        description="Configure process child threads (typically DDS threads)",
    )
    ld.add_action(config_child_threads_param)

    # Get the bringup directory
    atr_demo_dir = FindPackageShare("atr_demo").find("atr_demo")

    atr_id = 1

    print("Launching ATR: " + str(atr_id))

    atr_prefix = "atr_" + str(atr_id) + "_"

    doc = xacro.process_file(default_model_path, mappings={"prefix": atr_prefix, "type": "automower_base"})

    robot_descr_name = "robot_description_" + str(atr_id)
    joint_states_name = "joint_states_" + str(atr_id)
    robot_sate_pub = "robot_state_publisher_" + str(atr_id)

    robot_description = doc.toprettyxml(indent="  ")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=robot_sate_pub,
        parameters=[{"robot_description": robot_description}],
        remappings=[("robot_description", robot_descr_name), ("joint_states", joint_states_name)],
    )
    ld.add_action(robot_state_publisher_node)

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        remappings=[("robot_description", robot_descr_name)],
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    ld.add_action(joint_state_publisher_gui_node)

    # Set parameter file path
    param_file_path = os.path.join(atr_demo_dir, "config", "atr_demo.param.yaml")
    param_file = launch.substitutions.LaunchConfiguration("params", default=[param_file_path])
    # Node definitions
    # Runs:
    #  The atr driver:  Computes the state using integrator and the control input "joint_command"
    # (atr2_msgs::JointState) and publishes the "atr_joint_states" (atr2_msgs::JointState)
    #  The atr controller: Computes the control command "joint_command" based on the current state subscriber
    # "atr_joint_states"
    # With Debugger
    # https://answers.ros.org/question/385292/debugging-ros-2-cpp-node-with-vs-code-starting-the-node-with-a-launch-file/
    # atr_demo_runner = Node(
    #     package="atr_demo",
    #     executable="atr_demo",
    #     # prefix=["stdbuf -o L"],
    #     # prefix=["gdbserver localhost:3000"],
    #     # prefix=["xterm -e gdb -ex run --args"],
    #     output="screen",
    #     parameters=[param_file],
    #     arguments=[
    #         "--autostart",
    #         LaunchConfiguration("autostart"),
    #         "--priority",
    #         LaunchConfiguration("priority"),
    #         "--cpu-affinity",
    #         LaunchConfiguration("cpu-affinity"),
    #         "--lock-memory",
    #         LaunchConfiguration("lock-memory"),
    #         "--lock-memory-size",
    #         LaunchConfiguration("lock-memory-size"),
    #         "--config-child-threads",
    #         LaunchConfiguration("config-child-threads"),
    #         "--id",
    #         str(atr_id),
    #     ],
    #     emulate_tty=True,
    #     condition=UnlessCondition(LaunchConfiguration("gui")),
    # )
    # ld.add_action(atr_demo_runner)

    # args that can be set from the command line or a default will be used
    dev_name_launch_arg = DeclareLaunchArgument(name="_dev", default_value="/dev/input/js0")

    deadzone_launch_arg = DeclareLaunchArgument(name="_deadzone", default_value="0.05")

    autorepeat_launch_arg = DeclareLaunchArgument(name="_autorepeat_rate", default_value="100.0")

    ld.add_action(dev_name_launch_arg)
    ld.add_action(deadzone_launch_arg)
    ld.add_action(autorepeat_launch_arg)

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
        emulate_tty=True,
    )
    ld.add_action(joy_pub)

    joy_dir = FindPackageShare("atr_joy").find("atr_joy")

    joy2joint_cmd_atr_yaml = os.path.join(joy_dir, "config", "atr_joy_jointcmd.yaml")
    joy2twist_atr_yaml = os.path.join(joy_dir, "config", "atr_joy.yaml")

    joy_arg = DeclareLaunchArgument(
        name="joy_twist", default_value="False", description="Type of Joy output, True: Twist, False: Wheel velocities"
    )
    ld.add_action(joy_arg)

    joy_topic_arg = DeclareLaunchArgument(
        name="joy_topic",
        default_value="cmd_vel",
        description="Topic name to publish the joy command",
    )
    ld.add_action(joy_topic_arg)

    joy2joint_cmd_pub = Node(
        package="atr_joy",
        name="joy2joint_cmd_publisher",
        executable="joy2joint_cmd_CF710",
        parameters=[joy2joint_cmd_atr_yaml],
        remappings=[("cmd_vel", "joy_topic")],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("joy_twist")),
        emulate_tty=True,
    )
    ld.add_action(joy2joint_cmd_pub)

    joy2twist_pub = Node(
        package="atr_joy",
        name="joy2twist_publisher",
        executable="joy2twist_CF710",
        parameters=[joy2twist_atr_yaml],
        remappings=[("cmd_vel", LaunchConfiguration("joy_topic"))],
        output="screen",
        condition=IfCondition(LaunchConfiguration("joy_twist")),
        emulate_tty=True,
    )
    ld.add_action(joy2twist_pub)

    return ld
