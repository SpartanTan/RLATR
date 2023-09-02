cmake_minimum_required(VERSION 3.5)
project(atr_joy)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Standard ros messages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(atr_state_msgs REQUIRED)

## EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

## LIBRARY
add_library(
  ${PROJECT_NAME}
  SHARED
  src/Joy2Twist.cpp
  src/Joy2JointCMD.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(dependencies
  rclcpp
  Eigen3
  std_msgs
  sensor_msgs
  geometry_msgs
  atr_state_msgs
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${dependencies}
)

# Target's especific compiler flags 
#target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


# Executables
set(APP_TARGETS
joy2twist_CF710
joy2joint_cmd_CF710
)

foreach(target ${APP_TARGETS})
  add_executable(${target} src/Applications/${target}.cpp)
  # Dependencies by package name
  ament_target_dependencies(${target} ${dependencies})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  # target_compile_options(${target} PRIVATE -Wall)
  # Other Libraries
  # Dependencies by lib name
  target_link_libraries(${target} ${PROJECT_NAME})
  # target_include_directories(${target} PRIVATE
  #     ${OTHER__INCLUDES}
  # )
  # NOTE: To link agains messages generated in the same package use: 
  # rosidl_target_interfaces(${target}
  # ${PROJECT_NAME} "rosidl_typesupport_cpp")
endforeach()


# include_directories(include)
install(DIRECTORY include/
  DESTINATION include)

install(
  TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
${APP_TARGETS}
DESTINATION lib/${PROJECT_NAME}
)

# Install the launch and config file so ROS2 can find it
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# Downstream exports
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
