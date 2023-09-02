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

import xacro


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()

    # ATR Trajectory Generator
    traj_gen_dir = FindPackageShare("atr_trajectory_generator").find("atr_trajectory_generator")
    atr_trajectory_generator_yaml = os.path.join(traj_gen_dir, "config", "atr_trajectory_generator.yaml")
    atr_trajectory_generator_node = Node(
        package="atr_trajectory_generator",
        name="atr_trajectory_generator",
        executable="atr_trajectory_generator_node",
        # prefix=["gdbserver localhost:3000"],
        parameters=[atr_trajectory_generator_yaml],
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(atr_trajectory_generator_node)

    return ld
