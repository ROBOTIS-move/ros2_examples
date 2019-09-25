################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(examples_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
  "msg/Count.msg"
)

set(srv_files
  "srv/Calculation.srv"
)

set(action_files
  "action/Led.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs action_msgs
  ADD_LINTER_TESTS
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()
