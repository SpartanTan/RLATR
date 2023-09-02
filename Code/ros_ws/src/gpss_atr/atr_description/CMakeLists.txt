cmake_minimum_required(VERSION 3.8)
project(atr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)

# Standard ros messages
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# TF pacakges. Needed to convert Eigen into TF, for example Rotation to Quaternion msg
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ATR messages and serivces
find_package(atr_error_msgs REQUIRED)
find_package(atr_state_msgs REQUIRED)
find_package(atr_srvs REQUIRED)




# find standard add_dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

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

# Install the launch and config file so ROS2 can find it
install(
  DIRECTORY launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  ament_cmake 
  rclcpp
  rosidl_default_runtime 
  std_msgs
  std_srvs 
  geometry_msgs
  visualization_msgs
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
  atr_srvs
  robot_state_publisher
  urdf
  xacro)

ament_package()
