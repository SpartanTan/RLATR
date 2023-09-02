cmake_minimum_required(VERSION 3.8)
project(atr_factory_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclpy REQUIRED)

# TF pacakges. Needed to convert Eigen into TF, for example Rotation to Quaternion msg
set(dependencies
  rclcpp
  std_msgs
  std_srvs
  visualization_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  atr_error_msgs
  atr_state_msgs
  atr_srvs
  atr_object_msgs
  atr_utils
)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

# find standard add_dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

## RapidJSON
# find_package(RapidJSON REQUIRED)
# include_directories(${RAPIDJSON_INCLUDE_DIRS})

## Nlohman JSON
find_package(nlohmann_json)
include_directories(${nlohmann_json_INCLUDE_DIRS})

# Include Cpp "include" directory
include_directories(include)

# Libraries
add_library(
  ${PROJECT_NAME}
  SHARED
  src/ATRFactoryState.cpp
) # -> *.so 
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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
atr_factory_state_node
)


foreach(target ${APP_TARGETS})
  add_executable(${target} src/Applications/${target}.cpp)
  # Dependencies by package name
  ament_target_dependencies(${target} 
  ${dependencies}
  Eigen3
  )
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# include_directories(include)
install(DIRECTORY include/
  DESTINATION include)

# install Libraries
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
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Downstream exports
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  ament_cmake 
  rclcpp
  rosidl_default_runtime 
  std_msgs
  std_srvs 
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen 
  tf2_geometry_msgs 
  atr_error_msgs
  atr_state_msgs
  atr_formation_msgs
  atr_object_msgs
  atr_predicted_object_msgs
  atr_path_msgs
  atr_job_msgs
  atr_signal_msgs
  atr_srvs)

ament_package()
