cmake_minimum_required(VERSION 3.5)
project(atr_srvs)

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


# ATR messages
find_package(atr_error_msgs REQUIRED)
find_package(atr_state_msgs REQUIRED)
find_package(atr_formation_msgs REQUIRED)
find_package(atr_object_msgs REQUIRED)
find_package(atr_predicted_object_msgs REQUIRED)
find_package(atr_path_msgs REQUIRED)
find_package(atr_job_msgs REQUIRED)
find_package(atr_signal_msgs REQUIRED)
find_package(atr_human_msgs REQUIRED)

set(srv_files
  "srv/GetATRFormation.srv"
  "srv/GetFactoryState.srv"
  "srv/GetObjectList.srv"
  "srv/GetObjectListStamped.srv"
  "srv/ReloadGains.srv"
  "srv/TriggerATRActuator.srv"
  "srv/TriggerATRSignals.srv"
  "srv/UpdateATRPath.srv"
  "srv/UpdateATRPathList.srv"
  "srv/UpdateJobList.srv"
  "srv/UpdateObjectList.srv"
  "srv/UpdatePredictedObjectList.srv"
  "srv/UpdateObstacleList.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES 
    builtin_interfaces 
    std_msgs 
    geometry_msgs 
    tf2_geometry_msgs 
    atr_error_msgs
    atr_state_msgs
    atr_formation_msgs
    atr_object_msgs
    atr_predicted_object_msgs
    atr_path_msgs
    atr_job_msgs
    atr_signal_msgs
    atr_human_msgs
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
  # DESTINATION include)

# Downstream exports
# ament_export_include_directories(include)

ament_export_dependencies(
  rosidl_default_runtime 
  std_msgs 
  geometry_msgs 
  tf2_geometry_msgs 
  atr_error_msgs
  atr_state_msgs
  atr_formation_msgs
  atr_object_msgs
  atr_predicted_object_msgs
  atr_path_msgs
  atr_job_msgs
  atr_signal_msgs
  atr_human_msgs
  )

ament_package()
