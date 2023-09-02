cmake_minimum_required(VERSION 3.5)

project(atr_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

set(dependencies
    atr_object_msgs
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    std_msgs
    visualization_msgs
    geometry_msgs
    sensor_msgs
)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()


# find standard add_dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

set(ATR_UTILS_LIB atr_utils)

add_library(${ATR_UTILS_LIB} SHARED
  src/memory_lock.cpp
  src/rt_thread.cpp
  src/atr_utils.cpp
  src/AuxTools.cpp
  src/NodeDescriptions.cpp
  )

target_include_directories(${ATR_UTILS_LIB}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${ATR_UTILS_LIB} ${dependencies})

ament_export_targets(export_${ATR_UTILS_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${ATR_UTILS_LIB}
    EXPORT export_${ATR_UTILS_LIB}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include)

ament_package()
