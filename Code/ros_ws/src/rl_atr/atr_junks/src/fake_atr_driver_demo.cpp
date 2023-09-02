#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "atr_driver/ATRDriverNode.hpp"

#include "atr_utils/process_settings.hpp"
#include "atr_utils/lifecycle_autostart.hpp"

int main(int argc, char * argv[])
{
    atr::utils::ProcessSettings settings;

    // Loads input arguments and define the config flags
    if (!settings.init(argc, argv)) {
        return EXIT_FAILURE;
    }
    int32_t ret = 0;

    std::string logger_name;

    try {
        if (settings.configure_child_threads) {
            // process child threads created by ROS nodes will inherit the settings
            // sets priority, cpu_affinity, and lock_memory
            settings.configure_process();
        }
        rclcpp::init(argc, argv);
        std::string postfix("_");
        if (rcutils_cli_option_exist(argv, argv + argc, "--id")) {
            std::string sid = rcutils_cli_get_option(argv, argv + argc, "--id");

            postfix = postfix + sid;

            logger_name = "atr_demo" + postfix;
        }
        // Create a static executor
        // With this executor, we can execute multiple nodes using the same spin
        rclcpp::executors::StaticSingleThreadedExecutor exec;

        // bring the ATRDriverNode into scope
        using atr::atr_driver::ATRDriverNode;

        // Instantiate a node
        const auto driver_node_ptr = std::make_shared<ATRDriverNode>("atr_driver" + postfix);

        // Add the node to the executor
        exec.add_node(driver_node_ptr->get_node_base_interface());

        // configure process real-time settings for this specific node
        if (!settings.configure_child_threads) {
            // process child threads created by ROS nodes will NOT inherit the settings
            settings.configure_process();
        }

        // If sets the auto_start, then use autostart() function to make the lifecycle nodes active
        if (settings.auto_start_nodes) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "Auto-starting nodes");
            atr::utils::autostart(*driver_node_ptr);
        }
        exec.spin();
        rclcpp::shutdown();


    } catch (const std::exception & e) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name), e.what());
        ret = 2;
    } catch (...) {
        RCLCPP_INFO(
            rclcpp::get_logger(logger_name),
            "Unknown exception caught. "
            "Exiting...");
        ret = -1;
    }
    return ret;
}
