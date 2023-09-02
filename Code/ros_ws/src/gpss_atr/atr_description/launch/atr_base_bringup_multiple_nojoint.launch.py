from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()

    atr_description_path = get_package_share_path("atr_description")
    # default_model_path = atr_description_path / "urdf/automower.urdf.xacro"
    default_model_path = atr_description_path / "urdf/atr_robot_base.urdf.xacro"
    default_rviz_config_path = atr_description_path / "rviz/urdf_multiple_4.rviz"

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

    atr_id_list = [1, 2, 3, 4]

    for atr_id in atr_id_list:
        # for atr_id in range(1, 50):

        print("Launching ATR: " + str(atr_id))

        # atr_id = item
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    ld.add_action(rviz_node)

    return ld
