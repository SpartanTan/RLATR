cmake_minimum_required(VERSION 3.5)

project(atr_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcpputils REQUIRED)

find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ATR messages and serivces
find_package(atr_error_msgs REQUIRED)
find_package(atr_state_msgs REQUIRED)
find_package(atr_srvs REQUIRED)
# find_package(atr_srvs REQUIRED)

find_package(atr_utils REQUIRED)

# find standard add_dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# Include Cpp "include" directory
include_directories(include)

set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    lifecycle_msgs
    sensor_msgs
    atr_state_msgs
    atr_utils
    std_msgs
    atr_srvs)

set(ATR_CONTROLLER_LIB atr_controller)
add_library(
  ${ATR_CONTROLLER_LIB}
  SHARED
  #  src/atr_controller_node.cpp
  # src/atr_controller.cpp
  src/ATRControllerConfig.cpp
  src/ATRController.cpp
  src/ATRControllerNode.cpp
)

target_include_directories(
  ${ATR_CONTROLLER_LIB}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${ATR_CONTROLLER_LIB} ${dependencies})

ament_export_targets(export_${ATR_CONTROLLER_LIB} HAS_LIBRARY_TARGET)

# rclcpp_components_register_nodes(${ATR_CONTROLLER_LIB} "atr::Controller")

# Generate standalone node executable
# set(ATR_CONTROLLER_EXE "${ATR_CONTROLLER_LIB}_exe")
# add_executable(${ATR_CONTROLLER_EXE} src/atr_controller_node_main.cpp)
# target_link_libraries(${ATR_CONTROLLER_EXE} ${ATR_CONTROLLER_LIB})

# ament_export_targets(export_${ATR_CONTROLLER_LIB} HAS_LIBRARY_TARGET)
# ament_export_dependencies(${dependencies})

# if(BUILD_TESTING)
#   find_package(apex_test_tools REQUIRED)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   apex_test_tools_add_gtest(test_atr_controller test/test_atr_controller.cpp)
#   if(TARGET test_atr_controller)
#     target_link_libraries(test_atr_controller ${ATR_CONTROLLER_LIB})
#   endif()
#   apex_test_tools_add_gtest(test_atr_controller_node test/test_atr_controller_node.cpp)
#   if(TARGET test_atr_controller_node)
#     target_link_libraries(test_atr_controller_node ${ATR_CONTROLLER_LIB})
#   endif()

#   find_package(launch_testing_ament_cmake)
#   add_launch_test(
#       test/atr_controller_lifecycle.test.py
#       TARGET atr_controller_lifecycle.test
#       ENV
#       TIMEOUT 30
#   )
#   add_launch_test(
#       test/atr_controller_autostart.test.py
#       TARGET atr_controller_autostart.test
#       ENV
#       TIMEOUT 30
#   )
# endif()

install(
    DIRECTORY include/
    DESTINATION include
)


install(
    # TARGETS ${ATR_CONTROLLER_LIB} ${ATR_CONTROLLER_EXE}
    TARGETS ${ATR_CONTROLLER_LIB}
    EXPORT export_${ATR_CONTROLLER_LIB}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)


install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# Downstream exports
ament_export_include_directories(include)
ament_export_libraries(${ATR_CONTROLLER_LIB})

ament_export_dependencies(ament_cmake ${dependencies})

ament_package()
