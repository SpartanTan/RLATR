cmake_minimum_required(VERSION 3.8)
project(atr_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)

# Standard ros messages
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)



# TF pacakges. Needed to convert Eigen into TF, for example Rotation to Quaternion msg
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ATR messages and serivces
find_package(atr_error_msgs REQUIRED)
find_package(atr_state_msgs REQUIRED)
find_package(atr_srvs REQUIRED)
find_package(atr_utils REQUIRED)

# find standard add_dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# ## RapidJSON
# find_package(RapidJSON REQUIRED)
# include_directories(${RAPIDJSON_INCLUDE_DIRS})

# ## Nlohman JSON
# find_package(nlohmann_json)
# include_directories(${nlohmann_json_INCLUDE_DIRS})

# Include Cpp "include" directory
include_directories(include)

set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    lifecycle_msgs
    sensor_msgs
    atr_utils
    atr_error_msgs
    atr_state_msgs
    atr_srvs
    Eigen3
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    std_srvs
    std_msgs
    visualization_msgs
    sensor_msgs
    )

# Libraries
add_library(
  ${PROJECT_NAME}
  SHARED
  src/ATRDriverConfig.cpp
  src/ATRDriver.cpp
  src/ATRDriverNode.cpp
  src/RungeKutta.cpp
  ) # -> *.so 
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
# Target's especific compiler flags 
#target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# rclcpp_components_register_nodes(${PROJECT_NAME} "atr::Driver")


# # Executables
# set(APP_TARGETS
#   atr_demo
# )


# foreach(target ${APP_TARGETS})
#   add_executable(${target} src/Applications/${target}.cpp)
#   # Dependencies by package name
#   ament_target_dependencies(${target} ${dependencies})

#   target_include_directories(${target} PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

#   # target_compile_options(${target} PRIVATE -Wall)
#   # Other Libraries
#   # Dependencies by lib name
#   target_link_libraries(${target} ${PROJECT_NAME})
#   # target_include_directories(${target} PRIVATE
#   #     ${OTHER__INCLUDES}
#   # )
#   # NOTE: To link agains messages generated in the same package use: 
#   # rosidl_target_interfaces(${target}
#   # ${PROJECT_NAME} "rosidl_typesupport_cpp")
# endforeach()

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()


# include_directories(include)
install(DIRECTORY include/
  DESTINATION include)

# install Binaries (Libraries and exe)
install(
  TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# install(TARGETS
# ${APP_TARGETS}
# DESTINATION lib/${PROJECT_NAME}
# )


# Install the launch and config file so ROS2 can find it
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Downstream exports
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(ament_cmake ${dependencies})

ament_package()
