from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():
    atr_description_path = get_package_share_path("atr_description")
    default_model_path = atr_description_path / "urdf/atr_robot.urdf.xacro"
    default_rviz_config_path = atr_description_path / "rviz/urdf.rviz"

    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher_gui",
    )
    model_arg = DeclareLaunchArgument(
        name="model", default_value=str(default_model_path), description="Absolute path to robot urdf file"
    )
    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig", default_value=str(default_rviz_config_path), description="Absolute path to rviz config file"
    )

    doc = xacro.process_file(default_model_path, mappings={"type": "automower"})

    robot_descr_name = "robot_description"

    robot_description = doc.toprettyxml(indent="  ")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        remappings=[("robot_description", robot_descr_name)],
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        remappings=[("robot_description", robot_descr_name)],
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        remappings=[("robot_description", robot_descr_name)],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            gui_arg,
            model_arg,
            rviz_arg,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
