################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(examples_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rclcpp_action REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(CLIENT_NODE_NAME client)
set(PUBLISHER_NODE_NAME publisher)
set(SERVER_NODE_NAME server)
set(SUBSCRIBER_NODE_NAME subscriber)
set(ACTION_SERVER_NODE_NAME action_server)
set(ACTION_CLIENT_NODE_NAME action_client)

set(dependencies
  "examples_msgs"
  "rclcpp"
  "rclcpp_action"
  "example_interfaces"
)

add_executable(${CLIENT_NODE_NAME} src/client/main.cpp src/client/requestor.cpp)
ament_target_dependencies(${CLIENT_NODE_NAME} ${dependencies})

add_executable(${PUBLISHER_NODE_NAME} src/publisher/main.cpp src/publisher/counter.cpp)
ament_target_dependencies(${PUBLISHER_NODE_NAME} ${dependencies})

add_executable(${SERVER_NODE_NAME} src/server/main.cpp src/server/calculator.cpp)
ament_target_dependencies(${SERVER_NODE_NAME} ${dependencies})

add_executable(${SUBSCRIBER_NODE_NAME} src/subscriber/main.cpp src/subscriber/observer.cpp)
ament_target_dependencies(${SUBSCRIBER_NODE_NAME} ${dependencies})

add_executable(${ACTION_SERVER_NODE_NAME} src/action_server/main.cpp src/action_server/light.cpp)
ament_target_dependencies(${ACTION_SERVER_NODE_NAME} ${dependencies})

add_executable(${ACTION_CLIENT_NODE_NAME} src/action_client/main.cpp src/action_client/switch.cpp)
ament_target_dependencies(${ACTION_CLIENT_NODE_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS
  ${CLIENT_NODE_NAME}
  ${PUBLISHER_NODE_NAME}
  ${SERVER_NODE_NAME}
  ${SUBSCRIBER_NODE_NAME}
  ${ACTION_SERVER_NODE_NAME}
  ${ACTION_CLIENT_NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()