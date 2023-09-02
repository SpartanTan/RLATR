cmake_minimum_required(VERSION 3.5)
project(atr_camera_state_msgs)

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

# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(msg_files
  "msg/ATRCameraState.msg"
  "msg/ATRCameraStateListStamped.msg"
)

# set(srv_files
#   "srv/GetObjectList.srv"
#   "srv/GetATRFormation.srv"
#   "srv/TriggerATRActuator.srv"
#   "srv/TriggerATRSignals.srv"
#   "srv/UpdateObjectList.srv"
#   "srv/UpdatePredictedObjectList.srv"
#   "srv/UpdateATRPath.srv"
#   "srv/UpdateATRPathList.srv"
#   "srv/UpdateJobList.srv"
# )

# set(action_files
# "action/+.action"
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  # ADD_LINTER_TESTS
)

# IMPORTANT NOTE: To build executables that use the interfaces generated in the same package
# add_executable(publish_address_book
#   src/publish_address_book.cpp
# )

# ament_target_dependencies(publish_address_book
#   "rclcpp"
# )

# NOTE: To link agains messages generated in the same package use: 
# rosidl_target_interfaces(publish_address_book
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install(TARGETS publish_address_book
#  DESTINATION lib/${PROJECT_NAME})


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
# install(DIRECTORY include/
#   DESTINATION include)

# Downstream exports
ament_export_include_directories(include)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs tf2_geometry_msgs)

ament_package()
