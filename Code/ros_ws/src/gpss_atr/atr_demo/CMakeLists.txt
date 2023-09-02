cmake_minimum_required(VERSION 3.5)

project(atr_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(atr_utils REQUIRED)
# find_package(atr_driver REQUIRED)
# find_package(atr_controller REQUIRED)
# find_package(atr_nona REQUIRED)

# Include Cpp "include" directory
# include_directories(include)

set(dependencies
    rclcpp
    atr_utils
    atr_driver
    atr_controller
    atr_nona
)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

# Executables
set(APP_TARGETS
  atr_driver_demo
  atr_demo
)

foreach(target ${APP_TARGETS})
  add_executable(${target} src/Applications/${target}.cpp)
  # Dependencies by package name
  ament_target_dependencies(${target} ${dependencies})

# #   target_include_directories(${target} PUBLIC
# #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# #     $<INSTALL_INTERFACE:include>)

#   # target_compile_options(${target} PRIVATE -Wall)
#   # Other Libraries
#   # Dependencies by lib name
#   target_link_libraries(${target} ${dependencies})
#   # target_include_directories(${target} PRIVATE
#   #     ${OTHER__INCLUDES}
#   # )
#   # NOTE: To link agains messages generated in the same package use: 
#   # rosidl_target_interfaces(${target}
#   # ${PROJECT_NAME} "rosidl_typesupport_cpp")
endforeach()


# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   find_package(launch_testing_ament_cmake)
#   add_launch_test(
#       test/atr_demo_autostart.test.py
#       TARGET atr_demo_autostart.test
#       ENV
#       TIMEOUT 30
#   )
#   add_launch_test(
#       test/atr_demo_lifecycle.test.py
#       TARGET atr_demo_lifecycle.test
#       ENV
#       TIMEOUT 30
#   )
# endif()


install(TARGETS
${APP_TARGETS}
DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

# Downstream exports
# ament_export_include_directories(include)
# ament_export_libraries(${ATR_DEMO_EXE})

ament_export_dependencies(ament_cmake ${dependencies})

ament_package()
