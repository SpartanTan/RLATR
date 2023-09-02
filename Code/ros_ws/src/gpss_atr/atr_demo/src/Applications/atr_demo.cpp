// Copyright 2022 Emmanuel Dean
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Modified from https://github.com/ros2-realtime-demo/pendulum.git

#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "atr_driver/ATRDriverNode.hpp"
#include "atr_controller/ATRControllerNode.hpp"

#include "atr_utils/process_settings.hpp"
#include "atr_utils/lifecycle_autostart.hpp"

int main(int argc, char* argv[])
{
  atr::utils::ProcessSettings settings;

  // Loads input arguments and define the config flags
  if (!settings.init(argc, argv))
  {
    return EXIT_FAILURE;
  }

  int32_t ret = 0;

  std::string logger_name;

  try
  {
    // configure process real-time settings
    if (settings.configure_child_threads)
    {
      // process child threads created by ROS nodes will inherit the settings
      // sets priority, cpu_affinity, and lock_memory
      settings.configure_process();
    }

    rclcpp::init(argc, argv);

    std::string postfix("_");

    // Optional argument parsing
    if (rcutils_cli_option_exist(argv, argv + argc, "--id"))
    {
      std::string sid = rcutils_cli_get_option(argv, argv + argc, "--id");

      postfix = postfix + sid;

      logger_name = "atr_demo" + postfix;
    }

    // Create a static executor
    // With this executor, we can execute multiple nodes using the same spin
    rclcpp::executors::StaticSingleThreadedExecutor exec;

    // Create atr controller node
    using atr::atr_controller::ATRControllerNode;
    const auto controller_node_ptr = std::make_shared<ATRControllerNode>("atr_controller" + postfix);

    exec.add_node(controller_node_ptr->get_node_base_interface());

    // Create atr simulation
    using atr::atr_driver::ATRDriverNode;
    const auto driver_node_ptr = std::make_shared<ATRDriverNode>("atr_driver" + postfix);

    exec.add_node(driver_node_ptr->get_node_base_interface());

    // configure process real-time settings for this thread
    // We run this configuration after the ros nodes have been created. Then, the rt configuration will be limited to
    // this thread
    if (!settings.configure_child_threads)
    {
      // process child threads created by ROS nodes will NOT inherit the settings
      settings.configure_process();
    }

    if (settings.auto_start_nodes)
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "Auto-starting nodes");
      atr::utils::autostart(*controller_node_ptr);
      atr::utils::autostart(*driver_node_ptr);
    }

    exec.spin();
    rclcpp::shutdown();
  }
  catch (const std::exception& e)
  {
    RCLCPP_INFO(rclcpp::get_logger(logger_name), e.what());
    ret = 2;
  }
  catch (...)
  {
    RCLCPP_INFO(rclcpp::get_logger(logger_name),
                "Unknown exception caught. "
                "Exiting...");
    ret = -1;
  }
  return ret;
}
