import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription()

    # Get the launch directory
    bringup_dir = get_package_share_directory("atr_trajectory_generator")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")

    # Define arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "atrScenario.rviz"),
        description="Full path to the RVIZ config file to use",
    )
    ld.add_action(declare_rviz_config_file_cmd)

    declare_use_rviz_cmd = DeclareLaunchArgument("use_rviz", default_value="True", description="Whether to start RVIZ")
    ld.add_action(declare_use_rviz_cmd)

    # RQT
    rqt_graph_node = Node(package="rqt_graph", executable="rqt_graph", output="screen")
    ld.add_action(rqt_graph_node)

    # Rviz node
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )
    ld.add_action(rviz_cmd)

    # -------- NODES

    # Object List Publisher (NONA objects)
    nona_generator_yaml = os.path.join(get_package_share_directory("atr_nona"), "config", "nona_generator.yaml")
    nona_list_pub = Node(
        package="atr_nona",
        name="nona_generator",
        executable="nona_generator",
        parameters=[nona_generator_yaml],
        output="screen",
    )
    ld.add_action(nona_list_pub)

    # ATR Trajectory Generator
    atr_trajectory_generator_yaml = os.path.join(
        get_package_share_directory("atr_trajectory_generator"), "config", "atr_trajectory_generator.yaml"
    )
    atr_trajectory_generator_node = Node(
        package="atr_trajectory_generator",
        name="atr_trajectory_generator",
        executable="atr_trajectory_generator_node",
        parameters=[atr_trajectory_generator_yaml],
        output="screen",
    )
    ld.add_action(atr_trajectory_generator_node)

    return ld
