# Copyright 2020 Emmanuel Dean
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Adapted from https://github.com/ros2-realtime-demo/pendulum.git

import os
import signal

import unittest

import launch
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util

import pytest


@pytest.mark.rostest
def generate_test_description():
    package_dir = FindPackageShare("atr_demo").find("atr_demo")
    param_file_path = os.path.join(package_dir, "params", "atr_demo.param.yaml")
    param_file = launch.substitutions.LaunchConfiguration("params", default=[param_file_path])

    atr_demo = launch_ros.actions.Node(
        package="atr_demo",
        executable="atr_demo",
        output="screen",
        parameters=[param_file],
        arguments=["--autostart", "True"],
    )

    shutdown_timer = launch.actions.TimerAction(
        period=2.0,
        actions=[
            launch.actions.EmitEvent(
                event=launch.events.process.SignalProcess(
                    signal_number=signal.SIGINT, process_matcher=lambda proc: proc is atr_demo
                )
            )
        ],
    )

    return (
        launch.LaunchDescription(
            [
                atr_demo,
                shutdown_timer,
                launch_testing.util.KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "atr_demo": atr_demo,
        },
    )


class TestATRDemo(unittest.TestCase):
    def test_proc_starts(self, proc_info, atr_demo):
        proc_info.assertWaitForStartup(process=atr_demo, timeout=5)

    def test_proc_terminates(self, proc_info, atr_demo):
        proc_info.assertWaitForShutdown(atr_demo, timeout=10)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_node_graceful_shutdown(self, proc_info, atr_demo):
        """Test atr_demo graceful shutdown."""
        launch_testing.asserts.assertExitCodes(proc_info, process=atr_demo)
