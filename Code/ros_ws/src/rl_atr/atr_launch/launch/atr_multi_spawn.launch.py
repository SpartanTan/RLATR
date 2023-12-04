import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro
import yaml
from launch.actions import LogInfo

# import matplotlib.pyplot as plt
# import numpy as np


def generate_launch_description():

    ld = LaunchDescription()

    # get atr description path
    atr_description_path = FindPackageShare(
        "atr_description").find("atr_description")
    factory_db_dir = FindPackageShare("factory_db").find("factory_db")

    # get the urdf and rviz config paths
    default_model_path = atr_description_path + "/urdf/atr_robot_base.urdf.xacro"
    default_rviz_config_path = atr_description_path + "/rviz/atr_scenario.rviz"
    factory_db_dir = FindPackageShare("factory_db").find("factory_db")
    atr_param_yaml = os.path.join(
        factory_db_dir, "config/atr", "atr.param.yaml")
    print("-------------------" + default_rviz_config_path)

    """
          _    ____   ____ _   _ __  __ _____ _   _ _____ ____  
         / \  |  _ \ / ___| | | |  \/  | ____| \ | |_   _/ ___| 
        / _ \ | |_) | |  _| | | | |\/| |  _| |  \| | | | \___ \ 
       / ___ \|  _ <| |_| | |_| | |  | | |___| |\  | | |  ___) |
      /_/   \_\_| \_\\____|\___/|_|  |_|_____|_| \_| |_| |____/ 
                                                                
 
    """
    # Create launch configuration variables
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher_gui"
    )

    ld.add_action(gui_arg)

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(default_model_path),
        description="Absolute path to robot urdf file"
    )
    ld.add_action(model_arg)

    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(default_rviz_config_path),
        description="Absolute path to rviz config file"
    )
    ld.add_action(rviz_arg)

    with_rviz_param = DeclareLaunchArgument(
        name="rviz",
        default_value="False",
        description="Launch RVIZ2 in addition to other nodes"
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

    """
    
        ____  ____  _____     _______ ____      ____  _____ __  __  ___       _    ____   ____ _   _ __  __ _____ _   _ _____ ____  
        |  _ \|  _ \|_ _\ \   / / ____|  _ \    |  _ \| ____|  \/  |/ _ \     / \  |  _ \ / ___| | | |  \/  | ____| \ | |_   _/ ___| 
        | | | | |_) || | \ \ / /|  _| | |_) |   | | | |  _| | |\/| | | | |   / _ \ | |_) | |  _| | | | |\/| |  _| |  \| | | | \___ \ 
        | |_| |  _ < | |  \ V / | |___|  _ <    | |_| | |___| |  | | |_| |  / ___ \|  _ <| |_| | |_| | |  | | |___| |\  | | |  ___) |
        |____/|_| \_\___|  \_/  |_____|_| \_\___|____/|_____|_|  |_|\___/  /_/   \_\_| \_\\____|\___/|_|  |_|_____|_| \_| |_| |____/ 
                                            |_____|                                                                                   
    
    """
    # Create the launch configuration variables
    # These variables will be used when launching atr_driver_demo
    autostart_param = DeclareLaunchArgument(
        name="autostart",
        default_value="True",
        description="Automatically start lifecycle nodes"
    )
    ld.add_action(autostart_param)

    priority_param = DeclareLaunchArgument(
        name="priority", default_value="0", description="Set process priority")
    ld.add_action(priority_param)

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

    """
 
       _   _  ___  _   _    _    
      | \ | |/ _ \| \ | |  / \   
      |  \| | | | |  \| | / _ \  
      | |\  | |_| | |\  |/ ___ \ 
      |_| \_|\___/|_| \_/_/   \_\
                                 
 
    """
    # Object List Publisher (NONA objects)
    nona_dir = FindPackageShare("atr_nona").find("atr_nona")
    nona_generator_yaml = os.path.join(
        nona_dir, "config", "nona_generator.yaml")
    nona_list_node = Node(
        package="atr_nona",
        name="nona_generator",
        executable="atr_nona_node",
        parameters=[nona_generator_yaml],
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(nona_list_node)
    """
 
       _____ _    ____ _____ ___  ______   __  ____  ____  
      |  ___/ \  / ___|_   _/ _ \|  _ \ \ / / |  _ \| __ ) 
      | |_ / _ \| |     | || | | | |_) \ V /  | | | |  _ \ 
      |  _/ ___ \ |___  | || |_| |  _ < | |   | |_| | |_) |
      |_|/_/   \_\____| |_| \___/|_| \_\|_|   |____/|____/ 
                                                           
 
    """
    with open(atr_param_yaml, "r") as stream:
        try:
            config_parameters = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    atr_id_list = config_parameters["atr_params"]["ros__parameters"]["atr_id_list"]

    """
 
       ____ _____  _  _____ _____   ____  _   _ ____  _     ___ ____  _   _ _____ ____  
      / ___|_   _|/ \|_   _| ____| |  _ \| | | | __ )| |   |_ _/ ___|| | | | ____|  _ \ 
      \___ \ | | / _ \ | | |  _|   | |_) | | | |  _ \| |    | |\___ \| |_| |  _| | |_) |
       ___) || |/ ___ \| | | |___  |  __/| |_| | |_) | |___ | | ___) |  _  | |___|  _ < 
      |____/ |_/_/   \_\_| |_____| |_|    \___/|____/|_____|___|____/|_| |_|_____|_| \_\
                                                                                        
 
    """
    # Get the atr_demo bringup directory, since the atr configures are there
    # Get the config file for bringing up an atr
    atr_demo_dir = FindPackageShare("atr_demo").find("atr_demo")

    for atr_id in atr_id_list:
        print("Launching ATR: " + str(atr_id))

        atr_prefix = "atr_" + str(atr_id) + "_"
        # mapping the "prefix" to 'atr_1_ and "type" to "automower_base"
        # automower_base will then be used to include urdf called 'automower_base.urdf.xacro'
        # this line dynamically generate a urdf file, with the two xacro:arg remapped
        doc = xacro.process_file(default_model_path, mappings={
            "prefix": atr_prefix, "type": "automower_base"})

        robot_description_name = "robot_description_" + str(atr_id)
        joint_states_name = "joint_states_" + str(atr_id)
        robot_state_pub = "robot_state_publisher_" + str(atr_id)

        # pass the newly generated urdf to robot_description parameter
        # it will be used by robot_state_publisher to publish robot state tf
        robot_description = doc.toprettyxml(indent="  ")
        joint_state_publisher_gui_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            remappings=[("robot_description", robot_description_name)],
            condition=IfCondition(LaunchConfiguration("gui"))
        )
        ld.add_action(joint_state_publisher_gui_node)

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=robot_state_pub,
            output="screen",
            parameters=[{"robot_description": robot_description}],
            remappings=[("robot_description", robot_description_name),
                        ("joint_states", joint_states_name)],
        )
        ld.add_action(robot_state_publisher_node)

        """
 
          _  _____ ____  ____  ____  _____     _______ ____  ____  _____ __  __  ___  
         / \|_   _|  _ \|  _ \|  _ \|_ _\ \   / / ____|  _ \|  _ \| ____|  \/  |/ _ \ 
        / _ \ | | | |_) | | | | |_) || | \ \ / /|  _| | |_) | | | |  _| | |\/| | | | |
       / ___ \| | |  _ <| |_| |  _ < | |  \ V / | |___|  _ <| |_| | |___| |  | | |_| |
      /_/   \_\_| |_| \_\____/|_| \_\___|  \_/  |_____|_| \_\____/|_____|_|  |_|\___/ 
                                                                                      
 
        """
        # Run atr_driver_demo node, do ros init, initialize a ATRDriverNode, then run this node
        # ATRDriverNode
        # subscribes to
        # - /joint_command_1: atr_state_msgs/msg/ATRJointCommand, [omega_right, omega_left]
        # - /disturbance_1: atr_state_msgs/msg/ATRJointCommand. Maybe add some noise onto the wheels

        # publishes
        # - lifecycle transition event
        # - /atr_drive_state_1: atr_state_msgs/msg/ATRJointState, float64[6] [x,y,\theta_z,v_x,v_y,\omega_z]
        param_file_path = os.path.join(
            atr_demo_dir, "config", "atr_demo.param.yaml")
        param_file = LaunchConfiguration("params", default=[param_file_path])

        atr_demo_runner = Node(
            package="atr_demo",
            executable="atr_driver_demo",
            output="screen",
            parameters=[param_file],
            arguments=[
                "--autostart",
                LaunchConfiguration("autostart"),
                "--priority",
                LaunchConfiguration("priority"),
                "--cpu-affinity",
                LaunchConfiguration("cpu-affinity"),
                "--lock-memory",
                LaunchConfiguration("lock-memory"),
                "--lock-memory-size",
                LaunchConfiguration("lock-memory-size"),
                "--config-child-threads",
                LaunchConfiguration("config-child-threads"),
                "--id",
                str(atr_id),
            ],
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration("gui")),
        )
        ld.add_action(atr_demo_runner)

    # args that can be set from the command line or a default will be used
    dev_name_launch_arg = DeclareLaunchArgument(
        name="_dev", default_value="/dev/input/js0")
    deadzone_launch_arg = DeclareLaunchArgument(
        name="_deadzone", default_value="0.05")
    autorepeat_launch_arg = DeclareLaunchArgument(
        name="_autorepeat_rate", default_value="100.0")
    ld.add_action(dev_name_launch_arg)
    ld.add_action(deadzone_launch_arg)
    ld.add_action(autorepeat_launch_arg)

    # ATR Factory State Publisher
    atr_factory_state_dir = FindPackageShare(
        "atr_factory_state").find("atr_factory_state")
    atr_factory_state_yaml = os.path.join(
        atr_factory_state_dir, "config", "atr_factory_state.yaml")
    atr_factory_state = Node(
        package="atr_factory_state",
        name="atr_factory_state",
        executable="atr_factory_state_node",
        parameters=[atr_factory_state_yaml, {"atr_id_list": atr_id_list}],
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(atr_factory_state)

    # ATR Path Generator
    path_gen_dir = FindPackageShare("atr_path_generator").find("atr_path_generator")
    atr_path_generator_yaml = os.path.join(path_gen_dir, "config", "atr_path_generator.yaml")

    atr_path_generator_node = Node(
        package="atr_path_generator",
        name="atr_path_generator",
        executable="atr_path_generator_node",
        parameters=[atr_path_generator_yaml],
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(atr_path_generator_node)
    # ATR Path List subscriber (transforms the ATRPathList topic to MarkerArray to visualize it in Rviz)

    atr_path_subs_yaml = os.path.join(path_gen_dir, "config", "atr_path_list_subs.yaml")
    # ATR Path List subscriber (visualize Path generator path)
    atr_path_list_subs_pg = Node(
        package="atr_path_generator",
        name="atr_path_list_subscriber_pg",
        executable="path_list_subscriber_node",
        parameters=[atr_path_subs_yaml],
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(atr_path_list_subs_pg)

    test_pub_node = Node(
        package="atr_rl_testpublisher",
        name="testPublisher",
        executable="cmdPub",
        output="screen",
    )
    ld.add_action(test_pub_node)

    # test_conda_node = Node(
    #     package="atr_launch",
    #     # name="testPublisher",
    #     executable="condatest",
    #     output="screen",
    # )
    # ld.add_action(test_conda_node)
    return ld
